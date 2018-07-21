/*
 * Google Vision Bonnet Driver
 *
 * Author: Jonas Larsson <ljonas@google.com>
 *         Michael Brooks <mrbrooks@google.com>
 *         Alex Van Damme <atv@google.com>
 *         Leonid Lobachev <leonidl@google.com>
 *
 *         Copyright 2017
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include "aiy-vision.h"
#include <asm/uaccess.h>
#include <linux/atomic.h>
#include <linux/cdev.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/types.h>
#include <linux/vmalloc.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#define MYRIAD_FIRMWARE "myriad_fw.mvcmd"
#define POLL_INTERVAL_MS (1000 / 60)
#define SPI_BOOT_FREQ (13800 * 1000)
#define SPI_NORMAL_FREQ (SPI_BOOT_FREQ)

#define MAX_READ_ATTEMPTS 100
#define MAX_WRITE_ATTEMPTS 100

#define GPIO_SLAVE_READY_INDEX 0
#define GPIO_MASTER_ERROR_INDEX 1
#define GPIO_UNUSED_INDEX 2
#define GPIO_CHIP_SELECT_INDEX 3
#define AIYIO_GPIO_RESET_INDEX 0

#define TOKENCONCAT(str, num) str##num
#define MKVARNAME(str, num) TOKENCONCAT(str, num)
#define COMPILE_ASSERT_C(e, varname) \
  typedef char MKVARNAME(varname, __LINE__)[!!(e) ? 0 : -1]

typedef union {
  uint8_t as_byte;
  struct {
    // 0 - Acknowledge. Always 1 for master packets, 0/1 for slave.
    uint8_t ack : 1;
    // 1 - Supported (always 1 when reserved bits are 0).
    uint8_t is_supported : 1;
    // 2 - Transaction ID is valid (always 1 for master headers).
    uint8_t tid_valid : 1;
    // 3 - Indicates if data will follow this packet (in the same direction).
    uint8_t has_data : 1;
    // 4 - Master - 1, Slave - 0
    uint8_t is_master : 1;
    // 5 - Indicates if this packet completes transaction.
    uint8_t complete : 1;
    // 6-7 - Unused in current implementation. Should be 0.
    uint8_t reserved : 2;
  } bits;
} header_start_t;
COMPILE_ASSERT_C(sizeof(header_start_t) == 1, bad_header_start_size);

typedef struct __attribute__((packed)) {
  header_start_t start;
  uint8_t transaction_id;
  uint16_t crc;
  uint32_t size;
} header_t;
COMPILE_ASSERT_C(sizeof(header_t) == 8, bad_header_size);

typedef struct {
  struct spi_device *spidev;
  struct class *spicomm_class;
  struct device *spicomm_device;
  struct cdev spicomm_cdev;
  dev_t spicomm_region;

  struct gpio_descs *vision_gpios;
  struct gpio_descs *aiy_gpios;
  struct gpio_desc *me_gpio;
  struct gpio_desc *cs_gpio;
  struct gpio_desc *reset_gpio;

  wait_queue_head_t slave_ready_wait_queue;
  atomic_t slave_ready;
  bool booted;

  struct mutex lock;
  struct workqueue_struct *workqueue;
  wait_queue_head_t transaction_wait_queue;
  struct work_struct incoming_transaction_work;
  struct delayed_work ongoing_transaction_work;
  struct list_head incoming_transaction_queue;
  struct list_head ongoing_transaction_list;
  u8 used_tids[32];
} visionbonnet_t;

typedef struct {
  struct list_head list;

  uint8_t id;
  uint32_t flags;
  char *buffer;
  uint32_t buffer_len;
  uint32_t payload_len;
  struct mutex lock;
  atomic_t refs;
} transaction_t;

static int __attribute__((used)) debug = 0;
module_param(debug, int, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(debug, "Vision Bonnet debug");

static int __attribute__((used)) reset_on_failure = 1;
module_param(reset_on_failure, int, S_IWUSR | S_IRUGO);
MODULE_PARM_DESC(reset_on_failure, "Reset Myriad on fatal failure");

// Conditional debug. Use dev_info to avoid being compiled out.
#define cdebug(bonnet, format, ...)                          \
  do {                                                       \
    if (unlikely(debug)) {                                   \
      dev_info(&bonnet->spidev->dev, format, ##__VA_ARGS__); \
    }                                                        \
  } while (0)

static inline u32 compute_crc32(const uint8_t *data, size_t size) {
  return crc32(0xFFFFFFFF, data, size) ^ 0xFFFFFFFF;
}

static uint16_t xmodem_crc16_cumul(uint16_t crc, const uint8_t *data,
                                   size_t size) {
  const uint16_t kCrc16_CCIT_Poly_MSBF = 0x1021;
  for (size_t i = 0; i < size; ++i) {
    crc ^= (data[i] << 8);
    /* Compute the CRC one input bit at a time. See code fragment 4:
     * http://en.wikipedia.org/wiki/Computation_of_cyclic_redundancy_checks
     */
    for (unsigned bit = 0; bit < 8; ++bit) {
      if (crc & 0x8000) {
        crc = kCrc16_CCIT_Poly_MSBF ^ (crc << 1);
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

static inline uint16_t compute_header_crc16(header_t *header) {
  u16 crc = 0xFFFF;
  crc = xmodem_crc16_cumul(crc, (uint8_t *)&header->start.as_byte, 2);
  crc = xmodem_crc16_cumul(crc, (uint8_t *)&header->size, sizeof(header->size));
  return crc;
}

// bonnet->lock must already be held.
static void transaction_unref(visionbonnet_t *bonnet,
                              transaction_t *transaction) {
  if (!transaction) {
    return;
  }

  if (atomic_dec_and_test(&transaction->refs)) {
    if (transaction->id) {
      bonnet->used_tids[transaction->id / 8] &= ~BIT(transaction->id % 8);
      cdebug(bonnet, "Freeing tid %u\n", transaction->id);
    }
    vfree(transaction->buffer);
    kfree(transaction);
  }
}

// bonnet->lock must already be held.
static int transaction_alloc(visionbonnet_t *bonnet,
                             transaction_t **transaction, u32 buffer_len) {
  int i, j;

  *transaction = (transaction_t *)kzalloc(sizeof(transaction_t), GFP_KERNEL);
  if (!*transaction) {
    dev_err(&bonnet->spidev->dev, "Out of memory, transaction structure\n");
    return -ENOMEM;
  }

  atomic_set(&(*transaction)->refs, 1);
  mutex_init(&(*transaction)->lock);

  for (i = 0; i < sizeof(bonnet->used_tids); i++) {
    for (j = 0; j < 8; j++) {
      if (!(bonnet->used_tids[i] & BIT(j))) {
        bonnet->used_tids[i] |= BIT(j);
        (*transaction)->id = i * 8 + j;
        cdebug(bonnet, "Assigning tid %u\n", (*transaction)->id);
        break;
      }
    }
    if ((*transaction)->id) {
      break;
    }
  }

  if (!(*transaction)->id) {
    dev_err(&bonnet->spidev->dev, "No transaction id available\n");
    transaction_unref(bonnet, *transaction);
    *transaction = NULL;
    return -EBUSY;
  }

  (*transaction)->buffer = (char *)vmalloc(buffer_len);
  if (!(*transaction)->buffer) {
    dev_err(&bonnet->spidev->dev, "Out of memory, %u b buffer\n", buffer_len);
    transaction_unref(bonnet, *transaction);
    *transaction = NULL;
    return -ENOMEM;
  }
  (*transaction)->buffer_len = buffer_len;

  return 0;
}

static void transaction_set_flags(visionbonnet_t *bonnet,
                                  transaction_t *transaction, u32 flags) {
  if (!transaction) {
    return;
  }

  mutex_lock(&transaction->lock);
  transaction->flags |= flags;
  mutex_unlock(&transaction->lock);
  wake_up_interruptible(&bonnet->transaction_wait_queue);
}

static int transaction_done_waiting(transaction_t *transaction,
                                    u32 wait_flags) {
  int ret;
  mutex_lock(&transaction->lock);
  ret = ((transaction->flags & wait_flags) == wait_flags) ||
        (transaction->flags & FLAG_ERROR);
  mutex_unlock(&transaction->lock);
  return ret;
}

static void visionbonnet_alert_success(visionbonnet_t *bonnet) {
  gpiod_set_value(bonnet->cs_gpio, 1);
  gpiod_set_value(bonnet->cs_gpio, 0);
  gpiod_set_value(bonnet->cs_gpio, 1);
}

static void visionbonnet_alert_error(visionbonnet_t *bonnet) {
  gpiod_set_value(bonnet->me_gpio, 0);
  gpiod_set_value(bonnet->me_gpio, 1);
}

// Dumps pending transactions to debug output.
// Note: assumes that bonnet->lock is being held by the caller already.
static void visionbonnet_dump_transactions(visionbonnet_t *bonnet) {
  if (unlikely(debug)) {
    transaction_t *transaction = NULL, *t;
    dev_info(&bonnet->spidev->dev, "Pending tid(s) = ");
    list_for_each_entry_safe(transaction, t, &bonnet->ongoing_transaction_list,
                             list) {
      dev_info(&bonnet->spidev->dev, "%d ", transaction->id);
    }
    dev_info(&bonnet->spidev->dev, "\n");
  }
}

static int visionbonnet_validate_header(visionbonnet_t *bonnet,
                                        header_t *header) {
  int ret = 0;
  // The CRC16 of header+size must match what was provided in the packed.
  uint16_t validation_crc = compute_header_crc16(header);
  if (header->crc != validation_crc) {
    ret = -EBADMSG;
  }
  // If reserve values are set or supported is false, the header in invalid.
  else if (header->start.bits.reserved || !header->start.bits.is_supported) {
    ret = -ENOTSUPP;
  }
  // Check if the transaction ID is valid.
  else if (!header->start.bits.tid_valid) {
    ret = -EINVAL;
  } else if (!header->start.bits.ack) {
    ret = -EHOSTDOWN;
  }

  if (ret) {
    // If the slave NACKs or the CRC doesn't match, toggle error line and
    // retry the receive.
    if (ret == -EBADMSG) {
      visionbonnet_alert_error(bonnet);
      dev_err(&bonnet->spidev->dev, "CRC mismatch on response, re-reading.\n");
    } else if (ret == -EHOSTDOWN) {
      dev_err(&bonnet->spidev->dev,
              "Slave responded with a NACK, resending header.\n");
      visionbonnet_alert_success(bonnet);
    }
    // If not supported, transaction is complete. end transaction.
    else if (ret == -ENOTSUPP) {
      dev_err(&bonnet->spidev->dev, "Not supported.\n");
      visionbonnet_alert_success(bonnet);
    }
    // If the slave Transaction ID doesn't match the master, end transaction..
    else if (ret == -EINVAL) {
      dev_err(&bonnet->spidev->dev, "Transcation ID failure.\n");
      visionbonnet_alert_success(bonnet);
    }
  }
  return ret;
}

static bool visionbonnet_wait_slave_ready(visionbonnet_t *bonnet) {
  long timeout =
      bonnet->booted ? msecs_to_jiffies(1 * 1000) : msecs_to_jiffies(5 * 1000);
  int wait_ret = wait_event_interruptible_timeout(
      bonnet->slave_ready_wait_queue, atomic_read(&bonnet->slave_ready),
      timeout);
  if (wait_ret == -ERESTARTSYS) {
    dev_err(&bonnet->spidev->dev,
            "visionbonnet_wait_slave_ready interrupted by signal\n");
    return false;
  }

  // Consume slave_ready.
  int slave_ready = atomic_xchg(&bonnet->slave_ready, 0);
  bonnet->booted |= slave_ready;
  if (!slave_ready) {
    dev_err(&bonnet->spidev->dev, "Slave not ready after %u ms\n",
            jiffies_to_msecs(timeout));
  }
  return slave_ready;
}

static int visionbonnet_spi_read(visionbonnet_t *bonnet, uint8_t *data,
                                 size_t size) {
  int ret;
  size_t bytes_to_read = size;
  while (bytes_to_read) {
    cdebug(bonnet, "Waiting before read.\n");
    if (!visionbonnet_wait_slave_ready(bonnet)) {
      // Fatal error.
      ret = -ERESTART;
      break;
    }
    gpiod_set_value(bonnet->cs_gpio, 0);
    cdebug(bonnet, "Done waiting, reading.\n");
    size_t transfer_size = min(bytes_to_read, (size_t)4095);
    ret =
        spi_read(bonnet->spidev, (data + size - bytes_to_read), transfer_size);
    if (ret) {
      dev_err(&bonnet->spidev->dev, "Failed to read spi data ret=%d\n", ret);
      break;
    }
    bytes_to_read -= transfer_size;
    gpiod_set_value(bonnet->cs_gpio, 1);
  }
  return ret;
}

static int visionbonnet_spi_write(visionbonnet_t *bonnet, const uint8_t *data,
                                  size_t size) {
  int ret;
  size_t bytes_to_send = size;
  while (bytes_to_send) {
    cdebug(bonnet, "Waiting before write.\n");
    if (!visionbonnet_wait_slave_ready(bonnet)) {
      // Fatal error.
      ret = -ERESTART;
      break;
    }
    gpiod_set_value(bonnet->cs_gpio, 0);
    cdebug(bonnet, "Done waiting, writing.\n");
    size_t transfer_size = min(bytes_to_send, (size_t)4095);
    ret =
        spi_write(bonnet->spidev, (data + size - bytes_to_send), transfer_size);
    if (ret) {
      dev_err(&bonnet->spidev->dev, "Failed to write spi data ret=%d\n", ret);
      break;
    }
    bytes_to_send -= transfer_size;
    gpiod_set_value(bonnet->cs_gpio, 1);
  }
  cdebug(bonnet, "Spi write complete.\n");
  return ret;
}

static int visionbonnet_set_spi_freq(visionbonnet_t *bonnet, int freq) {
  bonnet->spidev->max_speed_hz = freq;
  return spi_setup(bonnet->spidev);
}

static int visionbonnet_write_firmware(visionbonnet_t *bonnet,
                                       const uint8_t *data, size_t size) {
  int ret;

  ret = visionbonnet_set_spi_freq(bonnet, SPI_BOOT_FREQ);
  if (ret) {
    dev_err(&bonnet->spidev->dev, "Failed to set spi freq: %d\n", ret);
    goto beach;
  }

  gpiod_set_value(bonnet->cs_gpio, 0);
  size_t bytes_to_send = size;
  while (bytes_to_send) {
    size_t transfer_size = min(bytes_to_send, (size_t)65535);
    ret =
        spi_write(bonnet->spidev, (data + size - bytes_to_send), transfer_size);
    if (ret) {
      dev_err(&bonnet->spidev->dev, "spi_write firmware: %d\n", ret);
      goto beach;
    }
    bytes_to_send -= transfer_size;
  }

  ret = visionbonnet_set_spi_freq(bonnet, SPI_NORMAL_FREQ);
  if (ret) {
    dev_err(&bonnet->spidev->dev, "Failed to set spi freq: %d\n", ret);
    goto beach;
  }

beach:
  gpiod_set_value(bonnet->cs_gpio, 1);
  return ret;
}

static int visionbonnet_myriad_reset(visionbonnet_t *bonnet) {
  int ret = 0;

  // Any and all transaction in flight must be aborted.
  mutex_lock(&bonnet->lock);

  transaction_t *transaction = NULL, *t;
  list_for_each_entry_safe(transaction, t, &bonnet->incoming_transaction_queue,
                           list) {
    list_del(&transaction->list);
    transaction->payload_len = 0;
    transaction_set_flags(bonnet, transaction, FLAG_ERROR);
    transaction_unref(bonnet, transaction);
  }
  list_for_each_entry_safe(transaction, t, &bonnet->ongoing_transaction_list,
                           list) {
    list_del(&transaction->list);
    transaction->payload_len = 0;
    transaction_set_flags(bonnet, transaction, FLAG_ERROR);
    transaction_unref(bonnet, transaction);
  }

  dev_notice(&bonnet->spidev->dev, "Resetting myriad\n");
  gpiod_set_value_cansleep(bonnet->reset_gpio, 1);
  msleep(20);
  gpiod_set_value_cansleep(bonnet->reset_gpio, 0);
  msleep(20);
  gpiod_set_value_cansleep(bonnet->reset_gpio, 1);
  // Give Myriad adequate time for boot ROM to execute.
  msleep(2000);

  atomic_set(&bonnet->slave_ready, 0);
  bonnet->booted = false;

  const struct firmware *firmware = NULL;
  cdebug(bonnet, "Requesting firmware %s\n", MYRIAD_FIRMWARE);
  ret = request_firmware(&firmware, MYRIAD_FIRMWARE, &bonnet->spidev->dev);
  if (ret) {
    dev_err(&bonnet->spidev->dev, "Failed to request firmware %s: %d\n",
            MYRIAD_FIRMWARE, ret);
    goto beach;
  }

  dev_notice(&bonnet->spidev->dev, "Writing myriad firmware\n");
  ret = visionbonnet_write_firmware(bonnet, firmware->data, firmware->size);
  if (ret) {
    dev_err(&bonnet->spidev->dev, "Failed to write firmware: %d\n", ret);
    goto beach;
  }
  dev_notice(&bonnet->spidev->dev, "Myriad booting\n");

  if (!visionbonnet_wait_slave_ready(bonnet)) {
    dev_err(&bonnet->spidev->dev, "Myriad did not boot in a timely fashion\n");
    ret = -EHOSTUNREACH;
    goto beach;
  }
  atomic_set(&bonnet->slave_ready, 1);
  dev_notice(&bonnet->spidev->dev, "Myriad ready\n");

beach:
  release_firmware(firmware);
  mutex_unlock(&bonnet->lock);
  return ret;
}

static void visionbonnet_fatal_error(visionbonnet_t *bonnet) {
  if (reset_on_failure) {
    dev_err(&bonnet->spidev->dev, "Fatal error, resetting\n");
    int ret = visionbonnet_myriad_reset(bonnet);
    if (ret) {
      // This is bad.
      dev_err(&bonnet->spidev->dev, "Failed to reset %d:\n", ret);
    }
  } else {
    dev_err(&bonnet->spidev->dev, "Fatal error, but reset skipped\n");
  }
}

static irqreturn_t visionbonnet_slave_ready_isr(int irq, void *dev) {
  visionbonnet_t *bonnet = (visionbonnet_t *)dev;

  atomic_xchg(&bonnet->slave_ready, 1);
  wake_up_interruptible(&bonnet->slave_ready_wait_queue);
  return IRQ_HANDLED;
}

static transaction_t *visionbonnet_dequeue_incoming_transaction(
    visionbonnet_t *bonnet) {
  transaction_t *transaction = NULL;
  mutex_lock(&bonnet->lock);
  if (!list_empty(&bonnet->incoming_transaction_queue)) {
    transaction = list_first_entry(&bonnet->incoming_transaction_queue,
                                   transaction_t, list);
    list_del(&transaction->list);
  }
  mutex_unlock(&bonnet->lock);
  return transaction;
}

static transaction_t *visionbonnet_find_pending_transaction(
    visionbonnet_t *bonnet, u8 tid) {
  transaction_t *transaction = NULL, *t;
  mutex_lock(&bonnet->lock);
  list_for_each_entry_safe(transaction, t, &bonnet->ongoing_transaction_list,
                           list) {
    if (transaction->id == tid) {
      list_del(&transaction->list);
      break;
    }
  }
  mutex_unlock(&bonnet->lock);
  return transaction;
}

static int visionbonnet_header_exchange(visionbonnet_t *bonnet,
                                        transaction_t *transaction,
                                        header_t *incoming_header,
                                        header_t *outgoing_header) {
  if (transaction) {
    outgoing_header->start.as_byte = 0b00011111;
    outgoing_header->transaction_id = transaction->id;
    outgoing_header->size = transaction->payload_len;
  } else {
    outgoing_header->start.as_byte = 0b00010111;
    outgoing_header->transaction_id = 0;
    outgoing_header->size = 0;
  }
  outgoing_header->crc = compute_header_crc16(outgoing_header);

  int ret = 0;
  int write_attempts = 0;
  int read_attempts = 0;

  do {
    write_attempts++;
    // Send initial header packet.
    cdebug(bonnet, "Sending initial header\n");
    ret = visionbonnet_spi_write(bonnet, (uint8_t *)outgoing_header,
                                 sizeof(header_t));
    if (ret) {
      dev_err(&bonnet->spidev->dev, "Failed to write header: %d\n", ret);
      return ret;
    }

    // SPI Write will appropriately toggle chip select and wait for the slave to
    // indicate it's ready. Begin a read event that will loop if there is a CRC
    // mismatch on the response.
    do {
      read_attempts++;
      ret = visionbonnet_spi_read(bonnet, (uint8_t *)incoming_header,
                                  sizeof(header_t));
      if (ret) {
        return ret;
      }
      cdebug(bonnet, "Recieved header: %02x size %u crc %04x tid %d\n",
             incoming_header->start.as_byte, incoming_header->size,
             incoming_header->crc, (int)incoming_header->transaction_id);

      // Validate the incoming packet.
      ret = visionbonnet_validate_header(bonnet, incoming_header);
      if (ret == -ENOTSUPP || ret == -EINVAL) {
        // If the response packet indicates TID mismatch or not supported, this
        // transaction is complete.
        return ret;
      }
      // Continue until succesful validation of header, or attempts exhausted.
    } while (ret == -EBADMSG && read_attempts < MAX_READ_ATTEMPTS);
    // Send transactions should continue until the slave validates CRC
    // (i.e. sends an ACK) or attempts exhausted.
  } while (ret == -EHOSTDOWN && write_attempts < MAX_WRITE_ATTEMPTS);

  if (!ret) {
    // With a clean master-slave exchange, toggle slave select to alert slave.
    cdebug(bonnet, "header_exchange succesful\n");
    visionbonnet_alert_success(bonnet);
  }
  return ret;
}

static int visionbonnet_receive_data_buffer(visionbonnet_t *bonnet,
                                            transaction_t *transaction,
                                            header_t *incoming_header,
                                            header_t *outgoing_header) {
  int ret;
  int read_attempts = 0;
  uint32_t slave_crc, computed_crc;
  do {
    bool overflow = false;
    ret = 0;
    read_attempts++;

    // We always need to read incoming_header->size bytes even if user space
    // supplied buffer is too small. To keep things simple we, for now,
    // allocate a temporary buffer for the incoming data in this case.
    // TODO(ljonas): Always read in to transaction->buffer without alloc.

    // Always set payload_len.
    mutex_lock(&transaction->lock);
    transaction->payload_len = incoming_header->size;
    mutex_unlock(&transaction->lock);

    cdebug(bonnet, "receive_data_buffer of size %u, buffer_len %u\n",
           incoming_header->size, transaction->buffer_len);
    if (incoming_header->size <= transaction->buffer_len) {
      // Transaction buffer is large enough.
      mutex_lock(&transaction->lock);
      ret = visionbonnet_spi_read(bonnet, transaction->buffer,
                                  incoming_header->size);
      mutex_unlock(&transaction->lock);
    } else {
      // Transaction buffer too small, allocate temporary storage.
      overflow = true;
      char *tmp = (char *)vmalloc(incoming_header->size);
      if (!tmp) {
        // This is fatal. We have no buffer to receive to and yet we must do so.
        dev_err(&bonnet->spidev->dev, "Failed to allocate %u b for spi_read\n",
                incoming_header->size);
        ret = -ERESTART;
      } else {
        ret = visionbonnet_spi_read(bonnet, tmp, incoming_header->size);
        vfree(tmp);
      }
    }
    if (ret) {
      return ret;
    }

    // After completing the data receive, read the CRC32 from the slave.
    ret =
        visionbonnet_spi_read(bonnet, (uint8_t *)&slave_crc, sizeof(slave_crc));
    if (ret) {
      dev_err(&bonnet->spidev->dev, "Failed on SPI read\n");
      return ret;
    }

    if (overflow) {
      // Don't check the crc, just signal error and break out.
      transaction_set_flags(bonnet, transaction, FLAG_OVERFLOW | FLAG_ERROR);
      break;
    }

    // Compare to calculated CRC32.
    computed_crc = compute_crc32(transaction->buffer, incoming_header->size);
    if (slave_crc == computed_crc) {
      transaction_set_flags(bonnet, transaction, FLAG_RESPONSE);
    } else {
      ret = -EBADMSG;
      dev_err(&bonnet->spidev->dev,
              "Incoming crc mismatch: slave %08x vs computed %08x\n", slave_crc,
              computed_crc);
      visionbonnet_alert_error(bonnet);
    }
    // Loop until CRC32s match, or attempts exhausted.
  } while (slave_crc != computed_crc && read_attempts < MAX_READ_ATTEMPTS);

  if (!ret) {
    cdebug(bonnet, "receive_data_buffer succesful\n");
    visionbonnet_alert_success(bonnet);
  }
  return ret;
}

static int vision_bonnet_send_data_buffer(visionbonnet_t *bonnet,
                                          transaction_t *transaction,
                                          header_t *incoming_header,
                                          header_t *outgoing_header) {
  int ret;
  int write_attempts = 0;
  int read_attempts = 0;
  cdebug(bonnet, "send_data_buffer of size %u\n", transaction->payload_len);
  do {
    write_attempts++;
    ret = visionbonnet_spi_write(bonnet, transaction->buffer,
                                 transaction->payload_len);
    if (ret) {
      dev_err(&bonnet->spidev->dev, "Failed on SPI write\n");
      return ret;
    }
    cdebug(bonnet, "Data sent, sending crc\n");
    // Succesful M->S send. Send CRC32
    uint32_t crc = compute_crc32(transaction->buffer, transaction->payload_len);
    ret = visionbonnet_spi_write(bonnet, (uint8_t *)&crc, sizeof(crc));
    if (ret) {
      dev_err(&bonnet->spidev->dev, "Failed to write CRC\n");
      return ret;
    }
    do {
      read_attempts++;
      // Validate crc32 via one last packet read. Expect incoming header to be
      // completed.
      cdebug(bonnet, "Reading crc packet\n");
      ret = visionbonnet_spi_read(bonnet, (uint8_t *)incoming_header,
                                  sizeof(header_t));
      if (ret) {
        dev_err(&bonnet->spidev->dev, "Failed on SPI read.\n");
        return ret;
      }
      ret = visionbonnet_validate_header(bonnet, incoming_header);
      if (ret == -ENOTSUPP || ret == -EINVAL) {
        // If the response packet indicates TID mismatch or not supported, this
        // transaction is complete.
        return ret;
      }
      // Loop reads until the header is verified, or attempts exhausted.
    } while (ret == -EBADMSG && read_attempts < MAX_READ_ATTEMPTS);
    // Loop writes until slave validates the sent CRC, or attempts exhausted.
  } while (ret == -EHOSTDOWN && write_attempts < MAX_WRITE_ATTEMPTS);

  if (!ret) {
    cdebug(bonnet, "send_data_buffer succesful\n");
    visionbonnet_alert_success(bonnet);
  }
  return ret;
}

static void vision_bonnet_incoming_transaction_work_handler(
    struct work_struct *work) {
  visionbonnet_t *bonnet =
      container_of(work, visionbonnet_t, incoming_transaction_work);
  transaction_t *transaction =
      visionbonnet_dequeue_incoming_transaction(bonnet);
  if (!transaction) {
    // Scheduled without any transactions to handle.
    return;
  }
  // We now own a ref to transaction.

  cdebug(bonnet, "processing tid %u\n", transaction->id);
  header_t outgoing_header = {{0}};
  header_t incoming_header = {{0}};

  // Exchange headers.
  int ret = visionbonnet_header_exchange(bonnet, transaction, &incoming_header,
                                         &outgoing_header);
  if (ret) {
    goto beach;
  }

  // Send request.
  ret = vision_bonnet_send_data_buffer(bonnet, transaction, &incoming_header,
                                       &outgoing_header);
  if (ret) {
    goto beach;
  }

  // We now consider the transaction acked, there may or may not be a response.
  transaction_set_flags(bonnet, transaction, FLAG_ACKED);

  cdebug(bonnet,
         "Data sent. tid %d complete %d is_supported %d has_data %d size %u\n",
         (int)incoming_header.transaction_id,
         (int)incoming_header.start.bits.complete,
         (int)incoming_header.start.bits.is_supported,
         (int)incoming_header.start.bits.has_data, incoming_header.size);

  // If the incoming header indicates it complete and the size is zero, this is
  // a write-only transaction. Return.
  if (incoming_header.start.bits.complete && !incoming_header.size) {
    cdebug(bonnet, "Completed write-only transaction.\n");
  }
  // If the incoming header indicates it's already done and has a non-zero size,
  // read it now.
  else if (incoming_header.start.bits.complete && incoming_header.size) {
    cdebug(bonnet, "Slave already has a response, reading.\n");
    ret = visionbonnet_receive_data_buffer(bonnet, transaction,
                                           &incoming_header, &outgoing_header);
  }
  // If the incoming message isn't complete, enqueue work to the ongoing
  // transaction queue.
  else if (!incoming_header.start.bits.complete) {
    cdebug(bonnet, "Slave has no response, deferring tid %d to ongoing queue\n",
           (int)incoming_header.transaction_id);
    mutex_lock(&bonnet->lock);
    list_add_tail(&transaction->list, &bonnet->ongoing_transaction_list);
    visionbonnet_dump_transactions(bonnet);
    queue_delayed_work(bonnet->workqueue, &bonnet->ongoing_transaction_work, 0);
    mutex_unlock(&bonnet->lock);

    // Transfer our ref to ongoing_transaction_list.
    transaction = NULL;
  }

beach:
  if (ret) {
    // A fatal error occurred. Flag the current transaction with error
    // and let the error handler deal with any others.
    transaction_set_flags(bonnet, transaction, FLAG_ERROR);
    visionbonnet_fatal_error(bonnet);
  }
  mutex_lock(&bonnet->lock);
  if (!list_empty(&bonnet->incoming_transaction_queue)) {
    cdebug(bonnet, "Scheduling more work on incoming transactions\n");
    queue_work(bonnet->workqueue, &bonnet->incoming_transaction_work);
  }
  transaction_unref(bonnet, transaction);
  mutex_unlock(&bonnet->lock);
}

static void vision_bonnet_ongoing_transaction_work_handler(
    struct work_struct *work) {
  visionbonnet_t *bonnet =
      container_of(work, visionbonnet_t, ongoing_transaction_work.work);

  // Do poll 0 exchange.
  header_t outgoing_header = {{0}};
  header_t incoming_header = {{0}};

  cdebug(bonnet, "Polling for completed transaction\n");

  transaction_t *transaction = NULL;

  // Exchange headers.
  int ret = visionbonnet_header_exchange(bonnet, NULL, &incoming_header,
                                         &outgoing_header);
  if (ret) {
    goto beach;
  }

  if (incoming_header.start.bits.complete) {
    cdebug(bonnet, "tid %d complete\n", (int)incoming_header.transaction_id);
    transaction = visionbonnet_find_pending_transaction(
        bonnet, incoming_header.transaction_id);
    if (!transaction) {
      dev_err(&bonnet->spidev->dev,
              "No transaction with tid %d in pending queue\n",
              (int)incoming_header.transaction_id);
      ret = -ERESTART;
      goto beach;
    }

    if (incoming_header.start.bits.has_data && incoming_header.size) {
      cdebug(bonnet, "Slave has a response for tid %d, reading.\n",
             (int)incoming_header.transaction_id);
      ret = visionbonnet_receive_data_buffer(
          bonnet, transaction, &incoming_header, &outgoing_header);
    } else {
      cdebug(bonnet, "tid %d complete, no data\n",
             (int)incoming_header.transaction_id);
      mutex_lock(&transaction->lock);
      transaction->payload_len = 0;
      mutex_unlock(&transaction->lock);
      transaction_set_flags(bonnet, transaction, FLAG_RESPONSE);
    }
  }

beach:
  if (ret) {
    // A fatal error occurred. Flag the current transaction with error
    // and let the error handler deal with any others.
    transaction_set_flags(bonnet, transaction, FLAG_ERROR);
    visionbonnet_fatal_error(bonnet);
  }
  mutex_lock(&bonnet->lock);
  if (!list_empty(&bonnet->ongoing_transaction_list)) {
    cdebug(bonnet, "Scheduling poll\n");
    visionbonnet_dump_transactions(bonnet);
    queue_delayed_work(bonnet->workqueue, &bonnet->ongoing_transaction_work,
                       msecs_to_jiffies(POLL_INTERVAL_MS));
  }
  transaction_unref(bonnet, transaction);
  mutex_unlock(&bonnet->lock);
}

static long visionbonnet_transact_ioctl(visionbonnet_t *bonnet,
                                        char __user *usr_arg) {
  int ret;
  usr_transaction_t usr_hdr;
  if (copy_from_user(&usr_hdr, usr_arg, sizeof(usr_transaction_t))) {
    dev_err(&bonnet->spidev->dev, "Invalid transaction header\n");
    return -EFAULT;
  }

  if (usr_hdr.payload_len == 0 || usr_hdr.payload_len > usr_hdr.buffer_len) {
    dev_err(&bonnet->spidev->dev, "Invalid transaction header: %u %u\n",
            usr_hdr.payload_len, usr_hdr.buffer_len);
    return -EINVAL;
  }

  transaction_t *transaction = NULL;

  mutex_lock(&bonnet->lock);
  ret = transaction_alloc(bonnet, &transaction, usr_hdr.buffer_len);
  if (ret) {
    mutex_unlock(&bonnet->lock);
    goto beach;
  }
  mutex_unlock(&bonnet->lock);

  if (copy_from_user(transaction->buffer, usr_arg + sizeof(usr_transaction_t),
                     usr_hdr.payload_len)) {
    dev_err(&bonnet->spidev->dev, "Failed to copy %u b payload\n",
            usr_hdr.payload_len);
    ret = -EFAULT;
    goto beach;
  }
  transaction->payload_len = usr_hdr.payload_len;

  // Queue transaction work, giving one ref to the queue.
  mutex_lock(&bonnet->lock);
  INIT_LIST_HEAD(&transaction->list);
  list_add_tail(&transaction->list, &bonnet->incoming_transaction_queue);
  atomic_inc(&transaction->refs);
  queue_work(bonnet->workqueue, &bonnet->incoming_transaction_work);
  mutex_unlock(&bonnet->lock);

  int wait_flags = FLAG_ACKED;
  if (!(usr_hdr.flags & USR_FLAG_ONEWAY)) {
    wait_flags |= FLAG_RESPONSE;
  }

  // Now wait for transaction complete, error or timeout.
  int wait_ret = wait_event_interruptible_timeout(
      bonnet->transaction_wait_queue,
      transaction_done_waiting(transaction, wait_flags),
      msecs_to_jiffies(usr_hdr.timeout_ms));
  mutex_lock(&transaction->lock);
  switch (wait_ret) {
    case -ERESTARTSYS:
      transaction->flags |= FLAG_ERROR;
      dev_notice(&bonnet->spidev->dev, "Transaction interrupted tid=%d\n",
                 transaction->id);
      ret = -EFAULT;
      break;
    case 0:
      // Timeout.
      transaction->flags |= (FLAG_ERROR | FLAG_TIMEOUT);
      dev_notice(&bonnet->spidev->dev, "Transaction timed out\n");
      ret = -ETIME;
      break;
    default:
      if (wait_ret < 0) {
        // Shouldn't happen.
        transaction->flags |= FLAG_ERROR;
        dev_err(&bonnet->spidev->dev, "Unknown wait_ret %d\n", wait_ret);
        ret = -EFAULT;
        break;
      } else {
        ret = transaction->flags & FLAG_ERROR ? -EFAULT : 0;
      }
      break;
  }
  mutex_unlock(&transaction->lock);

beach:
  if (transaction) {
    mutex_lock(&transaction->lock);

    // Only copy the buffer back if we have a response.
    usr_hdr.payload_len = transaction->payload_len;
    if (transaction->flags & FLAG_RESPONSE && transaction->payload_len) {
      if (copy_to_user(usr_arg + sizeof(usr_transaction_t), transaction->buffer,
                       transaction->payload_len)) {
        dev_err(&bonnet->spidev->dev, "Failed to copy buffer to user\n");
        transaction->flags |= FLAG_ERROR;
        ret = -EFAULT;
      }
    }

    // Lastly copy the updated header back.
    usr_hdr.flags = transaction->flags;
    if (copy_to_user(usr_arg, &usr_hdr, sizeof(usr_transaction_t))) {
      dev_err(&bonnet->spidev->dev,
              "Failed to copy transaction header to user\n");
      ret = -EFAULT;
    }
    transaction_unref(bonnet, transaction);
    mutex_unlock(&transaction->lock);
  }
  return ret;
}

static long visionbonnet_ioctl(struct file *filep, unsigned int cmd,
                               unsigned long arg) {
  visionbonnet_t *bonnet = (visionbonnet_t *)filep->private_data;
  cdebug(bonnet, "visionbonnet_ioctl cmd=%#.4x, arg=%lu", cmd, arg);

  switch (cmd) {
    case AIY_VISION_IOCTL_TRANSACT:
      return visionbonnet_transact_ioctl(bonnet, (char __user *)arg);
    case AIY_VISION_IOCTL_RESET:
      return visionbonnet_myriad_reset(bonnet);
    default:
      cdebug(bonnet, "Unknown IOCTL %#.8x", cmd);
  }
  return 0;
}

static int visionbonnet_open(struct inode *inode, struct file *filp) {
  filp->private_data =
      container_of(inode->i_cdev, visionbonnet_t, spicomm_cdev);
  return 0;
}

static struct file_operations visionbonnet_fops = {
    .open = visionbonnet_open,
    .unlocked_ioctl = visionbonnet_ioctl,
};

static int visionbonnet_uevent(struct device *dev,
                               struct kobj_uevent_env *env) {
  add_uevent_var(env, "DEVMODE=%#o", 0666);
  return 0;
}

static int visionbonnet_probe(struct spi_device *spi) {
  dev_notice(&spi->dev, "Initializing\n");

  if (!spi_busnum_to_master(0)) {
    dev_err(&spi->dev, "No spi master found\n");
    return -ENODEV;
  }

  int ret;
  bool vision_irq = false;
  visionbonnet_t *bonnet = NULL;

  spi->dev.platform_data = kzalloc(sizeof(visionbonnet_t), GFP_KERNEL);
  if (!spi->dev.platform_data) {
    ret = -ENOMEM;
    dev_err(&spi->dev, "Out of memory\n");
    goto beach;
  }
  bonnet = (visionbonnet_t *)spi->dev.platform_data;
  bonnet->spidev = spi;

  // Transaction id 0 is reserved, mark it as used.
  bonnet->used_tids[0] |= BIT(0);

  atomic_set(&bonnet->slave_ready, 0);

  // As all operations are blocking and should be executed serially, use a
  // dedicated driver workqueue.
  bonnet->workqueue = create_singlethread_workqueue("vision_wq");
  mutex_init(&bonnet->lock);
  INIT_LIST_HEAD(&bonnet->incoming_transaction_queue);
  INIT_LIST_HEAD(&bonnet->ongoing_transaction_list);
  init_waitqueue_head(&bonnet->transaction_wait_queue);
  init_waitqueue_head(&bonnet->slave_ready_wait_queue);
  INIT_WORK(&bonnet->incoming_transaction_work,
            vision_bonnet_incoming_transaction_work_handler);
  INIT_DELAYED_WORK(&bonnet->ongoing_transaction_work,
                    vision_bonnet_ongoing_transaction_work_handler);

  cdev_init(&bonnet->spicomm_cdev, &visionbonnet_fops);
  bonnet->spicomm_cdev.owner = THIS_MODULE;
  alloc_chrdev_region(&bonnet->spicomm_region, 0, 1, "vision_spicomm");
  cdev_add(&bonnet->spicomm_cdev, MKDEV(MAJOR(bonnet->spicomm_region), 0), 1);
  bonnet->spicomm_class = class_create(THIS_MODULE, "spicomm");
  bonnet->spicomm_class->dev_uevent = visionbonnet_uevent;
  bonnet->spicomm_device = device_create(
      bonnet->spicomm_class, NULL, MKDEV(MAJOR(bonnet->spicomm_region), 0),
      NULL, "vision_spicomm");

  bonnet->vision_gpios = devm_gpiod_get_array(&spi->dev, "vision", GPIOD_ASIS);
  if (IS_ERR(bonnet->vision_gpios)) {
    ret = -ENODEV;
    bonnet->vision_gpios = NULL;
    dev_err(&spi->dev, "Failed to bind vision GPIOs\n");
    goto beach;
  }
  cdebug(bonnet, "Bound %d vision GPIOs\n", bonnet->vision_gpios->ndescs);

  bonnet->aiy_gpios = devm_gpiod_get_array(&spi->dev, "aiy", GPIOD_ASIS);
  if (IS_ERR(bonnet->aiy_gpios)) {
    ret = -EPROBE_DEFER;
    bonnet->aiy_gpios = NULL;
    dev_err(&spi->dev, "Failed to bind reset GPIO\n");
    goto beach;
  }
  cdebug(bonnet, "Bound reset GPIO\n");

  ret = request_irq(
      gpiod_to_irq(bonnet->vision_gpios->desc[GPIO_SLAVE_READY_INDEX]),
      visionbonnet_slave_ready_isr, IRQF_TRIGGER_FALLING, "slave_ready_isr",
      bonnet);
  if (ret) {
    dev_err(&spi->dev, "Failed to claim slave ready IRQ: %d\n", ret);
    goto beach;
  }
  vision_irq = true;

  ret =
      gpiod_direction_output(bonnet->vision_gpios->desc[GPIO_UNUSED_INDEX], 1);
  if (ret) {
    dev_err(&spi->dev, "Failed to set unused GPIO direction: %d\n", ret);
    goto beach;
  }

  bonnet->me_gpio = bonnet->vision_gpios->desc[GPIO_MASTER_ERROR_INDEX];
  ret = gpiod_direction_output(bonnet->me_gpio, 1);
  if (ret) {
    dev_err(&spi->dev, "Failed to set master error GPIO direction: %d\n", ret);
    goto beach;
  }

  bonnet->cs_gpio = bonnet->vision_gpios->desc[GPIO_CHIP_SELECT_INDEX];
  ret = gpiod_direction_output(bonnet->cs_gpio, 1);
  if (ret) {
    dev_err(&spi->dev, "Failed to set chip select GPIO direction: %d\n", ret);
    goto beach;
  }

  bonnet->reset_gpio = bonnet->aiy_gpios->desc[AIYIO_GPIO_RESET_INDEX];
  ret = gpiod_direction_output(bonnet->reset_gpio, 1);
  if (ret) {
    dev_err(&spi->dev, "Failed to set reset GPIO direction: %d\n", ret);
    goto beach;
  }

  // Re-initialize spi without automatic control of CS.
  // This facilitates large transfers with a single toggling of CS, to
  // make things like booting the bonnet happy.
  spi->mode |= SPI_NO_CS;
  spi->cs_gpio = -1;
  ret = visionbonnet_set_spi_freq(bonnet, SPI_NORMAL_FREQ);
  if (ret) {
    dev_err(&spi->dev, "spi_setup failed: %d\n", ret);
    goto beach;
  }

  // Reset and load the bonnet.
  dev_notice(&spi->dev, "Resetting myriad on probe");
  ret = visionbonnet_myriad_reset(bonnet);
  if (ret) {
    dev_err(&spi->dev, "Inital bonnet boot failed: %d\n", ret);
    goto beach;
  }

beach:
  if (ret) {
    if (bonnet) {
      device_destroy(bonnet->spicomm_class,
                     MKDEV(MAJOR(bonnet->spicomm_region), 0));
      class_unregister(bonnet->spicomm_class);
      class_destroy(bonnet->spicomm_class);
      cdev_del(&bonnet->spicomm_cdev);
      unregister_chrdev_region(bonnet->spicomm_region, 1);

      destroy_workqueue(bonnet->workqueue);
      if (bonnet->vision_gpios) {
        if (vision_irq) {
          free_irq(
              gpiod_to_irq(bonnet->vision_gpios->desc[GPIO_SLAVE_READY_INDEX]),
              bonnet);
        }
        devm_gpiod_put_array(&spi->dev, bonnet->vision_gpios);
      }
      if (bonnet->aiy_gpios) {
        devm_gpiod_put_array(&spi->dev, bonnet->aiy_gpios);
      }
    }
    kfree(spi->dev.platform_data);
  }
  return ret;
}

static int visionbonnet_remove(struct spi_device *spi) {
  visionbonnet_t *bonnet = (visionbonnet_t *)spi->dev.platform_data;
  mutex_lock(&bonnet->lock);
  transaction_t *transaction = NULL, *t;
  list_for_each_entry_safe(transaction, t, &bonnet->incoming_transaction_queue,
                           list) {
    list_del(&transaction->list);
    transaction->payload_len = 0;
    transaction_set_flags(bonnet, transaction, FLAG_ERROR);
    transaction_unref(bonnet, transaction);
  }
  list_for_each_entry_safe(transaction, t, &bonnet->ongoing_transaction_list,
                           list) {
    list_del(&transaction->list);
    transaction->payload_len = 0;
    transaction_set_flags(bonnet, transaction, FLAG_ERROR);
    transaction_unref(bonnet, transaction);
  }
  mutex_unlock(&bonnet->lock);
  drain_workqueue(bonnet->workqueue);
  destroy_workqueue(bonnet->workqueue);

  device_destroy(bonnet->spicomm_class,
                 MKDEV(MAJOR(bonnet->spicomm_region), 0));
  class_unregister(bonnet->spicomm_class);
  class_destroy(bonnet->spicomm_class);
  cdev_del(&bonnet->spicomm_cdev);
  unregister_chrdev_region(bonnet->spicomm_region, 1);
  free_irq(gpiod_to_irq(bonnet->vision_gpios->desc[GPIO_SLAVE_READY_INDEX]),
           bonnet);
  devm_gpiod_put_array(&spi->dev, bonnet->vision_gpios);
  devm_gpiod_put_array(&spi->dev, bonnet->aiy_gpios);
  kfree(spi->dev.platform_data);
  return 0;
}

static const struct of_device_id visionbonnet_of_match[] = {
    {
        .compatible = "google,visionbonnet",
    },
    {},
};
MODULE_DEVICE_TABLE(of, visionbonnet_of_match);

static const struct spi_device_id gvb_id[] = {{"visionbonnet", 0}, {}};
MODULE_DEVICE_TABLE(spi, gvb_id);

static struct spi_driver visionbonnet_driver = {
    .driver =
        {
            .name = "aiy-vision",
            .owner = THIS_MODULE,
            .of_match_table = of_match_ptr(visionbonnet_of_match),
        },
    .probe = visionbonnet_probe,
    .remove = visionbonnet_remove,
};
module_spi_driver(visionbonnet_driver);

MODULE_AUTHOR("Jonas Larsson <ljonas@google.com>");
MODULE_AUTHOR("Michael Brooks <mrbrooks@google.com>");
MODULE_AUTHOR("Alex Van Damme <atv@google.com>");
MODULE_AUTHOR("Leonid Lobachev <leonidl@google.com>");
MODULE_ALIAS("spi:visionbonnet");
MODULE_DESCRIPTION("Driver for Google AIY Vision Bonnet");
MODULE_LICENSE("GPL v2");
