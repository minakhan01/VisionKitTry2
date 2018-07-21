/*
 * LED driver : leds-ktd202x.c
 *
 * Copyright (C) 2017 Google, Inc.
 * June Tate-Gans <jtgans@google.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/string.h>

enum ktd202x_registers {
	REG0_RESERVED = 0,
	REG0_RISEFALL_SCALE,
	REG0_ENABLE,
	REG0_RESET,
	REG0_TSLOT_CONTROL,
	REG1_LOG_SCURVE,
	REG1_TFLASH,
	REG2_PWM1,
	REG3_PWM2,

	REG4_CH1_ENABLE,
	REG4_CH2_ENABLE,
	REG4_CH3_ENABLE,
	REG4_CH4_ENABLE,

	REG5_TFALL,
	REG5_TRISE,

	REG6_LED1,
	REG7_LED2,
	REG8_LED3,
	REG9_LED4,

	REG_MAX,
};

#define REG_LED_BASE REG6_LED1
#define REG_CHANNEL_BASE REG4_CH1_ENABLE

static const struct {
	uint8_t addr;
	uint8_t mask;
	uint8_t shift;
	uint8_t max;
	const char* name;
} ktd202x_regs[REG_MAX] = {
	/* REG0 - Reset/Control                                          */
	/* [7]  - Reserved                                               */
	/* [6:5] - Rise/fall scaling                                     */
	/*         (1x normal, 2x slower, 4x slower, 8x fast)            */
	/* [4:3] - Enable control, 11 means always on.                   */
	/*         See p13 of the aug 2016 datasheet.                    */
	/* [2] - Reset / Offset cancel, with [2] set, then [1:0] set for */
	/*       reset mode                                              */
	/* [1:0] - Timer slot control with [2] clear                     */
	[REG0_RESERVED]       = {0x00, 0x80, 7, 0x00, "reserved"},
	[REG0_RISEFALL_SCALE] = {0x00, 0x60, 5, 0x03, "risefall_scale"},
	[REG0_ENABLE]         = {0x00, 0x18, 3, 0x03, "enable"},
	[REG0_RESET]          = {0x00, 0x07, 0, 0x00, "reset" },
	[REG0_TSLOT_CONTROL]  = {0x00, 0x07, 0, 0x04, "tslot_control" },

	/* REG1 - Flash period (MSB toggles log/s-curve)                   */
	/* [7]   - Log / S-curve                                           */
	/* [6:0] - T_flash period (1111111 is one shot) see p14 of the aug */
	/* 2016 datasheet.                                                 */
	[REG1_LOG_SCURVE] = {0x01, 0x80, 7, 0x01, "ramp"},
	[REG1_TFLASH]     = {0x01, 0x7F, 0, 0x7F, "tflash"},

	[REG2_PWM1] = {0x02, 0xFF, 0, 0xFF, "pwm1"},
	[REG3_PWM2] = {0x03, 0xFF, 0, 0xFF, "pwm2"},

	/* REG4 - Channel enable                             */
	/* [7:6] - LED4 enable (0 off, 1 on, 2 PWM1, 3 PWM2) */
	/* [5:4] - LED3 enable                               */
	/* [3:2] - LED2 enable                               */
	/* [1:0] - LED1 enable                               */
	[REG4_CH1_ENABLE] = {0x04, 0x03, 0, 0x04, "ch1_enable"},
	[REG4_CH2_ENABLE] = {0x04, 0x0C, 2, 0x04, "ch2_enable"},
	[REG4_CH3_ENABLE] = {0x04, 0x30, 4, 0x04, "ch3_enable"},
	[REG4_CH4_ENABLE] = {0x04, 0xC0, 6, 0x04, "ch4_enable"},

	/* REG5 - Ramp Register */
	/* [7:4] - T_fall       */
	/* [3:0] - T_rise       */
	[REG5_TFALL] = {0x05, 0xF0, 4, 0x0F, "trise"},
	[REG5_TRISE] = {0x05, 0x0F, 0, 0x0F, "tfall"},

	[REG6_LED1] = {0x06, 0xFF, 0, 0xFF, "led1"},
	[REG7_LED2] = {0x07, 0xFF, 0, 0xFF, "led2"},
	[REG8_LED3] = {0x08, 0xFF, 0, 0xFF, "led3"},
	[REG9_LED4] = {0x09, 0xFF, 0, 0xFF, "led4"},
};

static int ktd202x_get_register_by_name(const char* name)
{
	int i;
	for (i = 0; i < REG_MAX; ++i) {
		if (strcmp(name, ktd202x_regs[i].name) == 0)
			return i;
	}
	return -1;
}

/*
 * The KTD202x series of chips do not handle i2c reads correctly. This driver
 * caches internal register states instead.
 */
bool ktd202x_check_readable_register(struct device *dev, unsigned int reg) {
	return false;
}

static const struct reg_default ktd202x_register_defaults[] = {
	{ 0x00, 0x00 },  /* Reset/Control */
	{ 0x01, 0x00 },  /* Flash Period */
	{ 0x02, 0x01 },  /* PWM1 Timer */
	{ 0x03, 0x01 },  /* PWM2 Timer */
	{ 0x04, 0x00 },  /* Channel Enable */
	{ 0x05, 0x00 },  /* Ramp Rate */
	{ 0x06, 0x4f },  /* LED1 Brightness */
	{ 0x07, 0x4f },  /* LED2 Brightness */
	{ 0x08, 0x4f },  /* LED3 Brightness */
	{ 0x09, 0x4f },  /* LED4 Brightness */
};

static const struct regmap_config ktd202x_i2c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x09,
	.cache_type = REGCACHE_FLAT,
	.can_multi_write = true,
	.readable_reg = ktd202x_check_readable_register,
	.reg_defaults = ktd202x_register_defaults,
	.num_reg_defaults = ARRAY_SIZE(ktd202x_register_defaults),
};

/* REG0[6:5] */
enum Reg0RiseFallScalingModes {
	KTD202X_RISEFALL_NORMAL  = 0x0,
	KTD202X_RISEFALL_2X_SLOW = 0x1,
	KTD202X_RISEFALL_4X_SLOW = 0x2,
	KTD202X_RISEFALL_8X_FAST = 0x3,
};

/* REG0[4:3] */
enum Reg0EnableControl {
	KTD202X_ENABLE_SCL_SDA_HIGH = 0x0,
	KTD202X_ENABLE_SDA_TOGGLING = 0x1,
	KTD202X_ENABLE_SCL_HIGH     = 0x2,
	KTD202X_ENABLE_ALWAYS_ON    = 0x3,
};

/* REG0[2:0] becomes reset control when MSB is high */
enum Reg0TimerSlotControl {
	KTD202X_TCTRL_TSLOT1 = 0x0,
	KTD202X_TCTRL_TSLOT2 = 0x1,
	KTD202X_TCTRL_TSLOT3 = 0x2,
	KTD202X_TCTRL_TSLOT4 = 0x3,
};

// REG2 - PWM1 Flash On Timer 1 (percent of T_flash period)
// REG3 - PWM2 Flash On Timer 2 (percent of T_flash period)

enum Reg4ChannelEnable {
	KTD202X_CHANNEL_OFF  = 0x0,
	KTD202X_CHANNEL_ON   = 0x1,
	KTD202X_CHANNEL_PWM1 = 0x2,
	KTD202X_CHANNEL_PWM2 = 0x3,
};

// REG6 - LED1 Output, 8-bit, 0.125mA (0) to 24mA (255), 0.125mA steps
// REG7 - LED2 Output, 8-bit, 0.125mA (0) to 24mA (255), 0.125mA steps
// REG8 - LED3 Output, 8-bit, 0.125mA (0) to 24mA (255), 0.125mA steps
// REG9 - LED4 Output, 8-bit, 0.125mA (0) to 24mA (255), 0.125mA steps

#define KTD202X_LED_MAX_BRIGHTNESS  255

struct ktd202x_channel {
	struct led_classdev cdev;
	uint8_t channel_no;
};

struct ktd202x_context {
	struct i2c_client *client;
	struct regmap *regmap;
	struct ktd202x_channel led_channel[4];
};

static struct ktd202x_channel *
to_ktd202x_channel(struct led_classdev* led_cdev) {
	struct ktd202x_channel *ch = container_of(led_cdev,
						  struct ktd202x_channel, cdev);
	return ch;
}

static struct ktd202x_context *
to_ktd202x_context(struct led_classdev* led_cdev) {
	struct i2c_client* client = to_i2c_client(led_cdev->dev->parent);
	struct ktd202x_context* context = i2c_get_clientdata(client);
	return context;
}

static inline int ktd202x_update_bits(struct regmap *regmap,
				unsigned int reg,
				unsigned int data)
{
	uint8_t output = data << ktd202x_regs[reg].shift;
	int result = 0;

	result = regmap_update_bits(regmap,
				ktd202x_regs[reg].addr,
				ktd202x_regs[reg].mask,
				output);
	if (result < 0) {
		pr_err("ktd202x: regmap_update_bits failed: %d.", result);
	}

	return result;
}

static inline int ktd202x_get_bits(struct regmap *regmap,
				int reg,
				unsigned int *out)
{
	int read_result = 0;
	int result = regmap_read(regmap, ktd202x_regs[reg].addr, &read_result);

	if (result < 0) {
		pr_err("ktd202x: regmap_read failed %d.", result);
		return result;
	}

	*out = (read_result & ktd202x_regs[reg].mask) >> ktd202x_regs[reg].shift;

	return 0;
}

static int ktd202x_reset(struct ktd202x_context *ctx)
{
	int ret;

	/* Reset must occur by writing Reg0[2:0] = 0b111 on init. need a 200us
	   delay before the next command. Note that during reset, the chip NAKs
	   the request, so we throw away the result here. */
	regmap_write(ctx->regmap, ktd202x_regs[REG0_RESET].addr, 0x07);
	udelay(300);

	ret = regmap_reinit_cache(ctx->regmap, &ktd202x_i2c_regmap_config);
	if (ret < 0) {
		dev_err(&ctx->client->dev, "Could not reinit cache: %d", ret);
		return ret;
	}

	/* Ensure we always listen to i2c by forcing the device to always be
	   on */
	ret = regmap_write(ctx->regmap, ktd202x_regs[REG0_ENABLE].addr, 0x03);
	if (ret < 0) {
		dev_err(&ctx->client->dev,
			"Could not set chip power management: %d", ret);
		return ret;
	}

	return 0;
}

static void ktd202x_brightness_set(struct led_classdev *led_cdev,
				   enum led_brightness brightness)
{
	enum Reg4ChannelEnable state =
		(brightness == LED_OFF) ? KTD202X_CHANNEL_OFF : KTD202X_CHANNEL_ON;
	struct ktd202x_channel *ch = to_ktd202x_channel(led_cdev);
	struct ktd202x_context *context = to_ktd202x_context(led_cdev);
	int ret = 0;

	ret = ktd202x_update_bits(context->regmap,
				  REG_LED_BASE + ch->channel_no, brightness);
	if (ret < 0) {
		dev_err(led_cdev->dev,
			"failed to update brightness for channel %d: 0x%x\n",
			ch->channel_no,
			ret);
		return;
	}

	ret = ktd202x_update_bits(context->regmap,
				  REG_CHANNEL_BASE + ch->channel_no, state);
	if (ret < 0) {
		dev_err(led_cdev->dev,
			"failed to set channel enable to 0x%x for channel %d\n",
			state,
			ch->channel_no);
	}
}

static enum led_brightness ktd202x_brightness_get(struct led_classdev *led_cdev)
{
	struct ktd202x_channel *ch = to_ktd202x_channel(led_cdev);
	struct ktd202x_context *context = to_ktd202x_context(led_cdev);
	int ret = 0;
	unsigned int result = 0;

	ret = ktd202x_get_bits(context->regmap,
			REG_CHANNEL_BASE + ch->channel_no,
			&result);
	if (ret < 0) {
		dev_err(led_cdev->dev,
			"failed to get bits for channel %d -- returning 0\n",
			ch->channel_no);
		return 0;
	}

	/* Channel isn't enabled, so effectively has a brightness of 0. */
	if (result == 0) return 0;

	/* Channel is enabled, so let's actually get the real brightness. */
	ret = ktd202x_get_bits(context->regmap,
			REG_LED_BASE + ch->channel_no,
			&result);
	if (ret < 0) {
		dev_err(led_cdev->dev,
			"failed to get bits for channel %d -- returning 0\n",
			ch->channel_no);
		return 0;
	}

	return result;
}

static const char *const channel_names[] = {
	"ktd202x:led1",
	"ktd202x:led2",
	"ktd202x:led3",
	"ktd202x:led4",
};

static int ktd202x_init(struct ktd202x_context *ctx)
{
	struct ktd202x_channel *ch;
	int i, err;

	/* Reset before the LED classdev driver gets a chance to query for
	   brightness values. */
	err = ktd202x_reset(ctx);
	if (err < 0) return err;

	for (i = 0; i < ARRAY_SIZE(channel_names); ++i) {
		ch = &ctx->led_channel[i];
		ch->channel_no = i;
		ch->cdev.name = channel_names[i];
		ch->cdev.brightness = LED_OFF;
		ch->cdev.max_brightness = KTD202X_LED_MAX_BRIGHTNESS;
		ch->cdev.brightness_set = ktd202x_brightness_set;
		ch->cdev.brightness_get = ktd202x_brightness_get;

		err = devm_led_classdev_register(&ctx->client->dev, &ch->cdev);
		if (err < 0) {
			dev_err(&ctx->client->dev,
				"Led register failed for channel %d: %d\n",
				i, err);
			return err;
		}
	}

	return 0;
}

static ssize_t ktd202x_show_register(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	struct ktd202x_context *ctx = dev_get_drvdata(dev);
	int reg = ktd202x_get_register_by_name(attr->attr.name);
	unsigned int value = 0;
	int res;

	if (!ctx || reg == -1) return -ENOENT;

	res = ktd202x_get_bits(ctx->regmap, reg, &value);
	if (res < 0) return res;

	return sprintf(buf, "%d\n", value);
}

static ssize_t ktd202x_store_register(struct device *dev,
				      struct device_attribute *attr,
				      const char* buf, size_t count)
{
	u8 value = 0;
	struct ktd202x_context *ctx = dev_get_drvdata(dev);
	int reg = ktd202x_get_register_by_name(attr->attr.name);
	int result = kstrtou8(buf, 10, &value);

	if (!ctx || reg == -1) return -ENOENT;
	if (result != 0) return -ERANGE;
	if (value > ktd202x_regs[reg].max) return -ERANGE;

	result = ktd202x_update_bits(ctx->regmap, reg, value);
	if (result < 0) return result;

	return count;
}

static ssize_t ktd202x_reset_store(struct device *dev,
				   struct device_attribute *attr,
				   const char* buf, size_t count)
{
	struct ktd202x_context *ctx = dev_get_drvdata(dev);
	int err = ktd202x_reset(ctx);
	if (err < 0) return err;
	return count;
}

/*
 * Parses "reg=value;" string where reg is a register name and value is in
 * range 0-255.
 */
static int ktd202x_parse_reg_and_value(char **str, struct reg_default *parsed)
{
	char *reg_name, *reg_value;
	int reg;
	u8 value;

	reg_name = strsep(str, "=");
	if (!reg_name)
		return -EINVAL;

	reg = ktd202x_get_register_by_name(reg_name);
	if (reg == -1)
		return -EINVAL;

	reg_value = strsep(str, ";");
	if (!reg_value)
		return -EINVAL;

	if (kstrtou8(reg_value, 10, &value) < 0)
		return -EINVAL;

	if (value > ktd202x_regs[reg].max)
		return -EINVAL;

	parsed->reg = reg;
	parsed->def = value;
	return 0;
}

/*
 * Parses "reg=val;reg=val;...;reg=val;" command to update register values.
 * For example, "led1=230;led2=100;trise=5;" sets led1 to 230, led2 to 100,
 * and trise to 5.
 */
static ssize_t ktd202x_store_registers(struct device *dev,
				       struct device_attribute *attr,
				       const char* buf, size_t count)
{
	struct ktd202x_context *ctx = dev_get_drvdata(dev);
	struct reg_default parsed[128] = {0};
	char config[128 + 1] = {0};
	char *str = config;
	int result, i, n = 0;

	sscanf(buf, "%128s\n", config);
	while (*str && n < ARRAY_SIZE(parsed)) {
		result = ktd202x_parse_reg_and_value(&str, &parsed[n++]);
		if (result < 0)
			return result;
		if (!str)
			return -EINVAL;
		dev_dbg(dev, "reg=%d, value=%d",
			parsed[n - 1].reg, parsed[n - 1].def);
	}

	for (i = 0; i < n; ++i) {
		result = ktd202x_update_bits(ctx->regmap,
					     parsed[i].reg, parsed[i].def);
		if (result < 0)
			return result;
	}

	return count;
}

static DEVICE_ATTR(reset, 0200, NULL, ktd202x_reset_store);

static DEVICE_ATTR(tflash, 0644, ktd202x_show_register, ktd202x_store_register);
static DEVICE_ATTR(ramp, 0644, ktd202x_show_register, ktd202x_store_register);

static DEVICE_ATTR(pwm1, 0644, ktd202x_show_register, ktd202x_store_register);
static DEVICE_ATTR(pwm2, 0644, ktd202x_show_register, ktd202x_store_register);

static DEVICE_ATTR(trise, 0644, ktd202x_show_register, ktd202x_store_register);
static DEVICE_ATTR(tfall, 0644, ktd202x_show_register, ktd202x_store_register);

static DEVICE_ATTR(ch1_enable, 0644, ktd202x_show_register,
		   ktd202x_store_register);
static DEVICE_ATTR(ch2_enable, 0644, ktd202x_show_register,
		   ktd202x_store_register);
static DEVICE_ATTR(ch3_enable, 0644, ktd202x_show_register,
		   ktd202x_store_register);
static DEVICE_ATTR(ch4_enable, 0644, ktd202x_show_register,
		   ktd202x_store_register);

static DEVICE_ATTR(led1, 0644, ktd202x_show_register, ktd202x_store_register);
static DEVICE_ATTR(led2, 0644, ktd202x_show_register, ktd202x_store_register);
static DEVICE_ATTR(led3, 0644, ktd202x_show_register, ktd202x_store_register);
static DEVICE_ATTR(led4, 0644, ktd202x_show_register, ktd202x_store_register);

static DEVICE_ATTR(registers, 0644, NULL, ktd202x_store_registers);


static struct attribute *ktd202x_dev_attrs[] = {
	&dev_attr_reset.attr,
	&dev_attr_tflash.attr,
	&dev_attr_ramp.attr,
	&dev_attr_pwm1.attr,
	&dev_attr_pwm2.attr,
	&dev_attr_trise.attr,
	&dev_attr_tfall.attr,
	&dev_attr_ch1_enable.attr,
	&dev_attr_ch2_enable.attr,
	&dev_attr_ch3_enable.attr,
	&dev_attr_ch4_enable.attr,
	&dev_attr_led1.attr,
	&dev_attr_led2.attr,
	&dev_attr_led3.attr,
	&dev_attr_led4.attr,
	&dev_attr_registers.attr,
	NULL
};

static struct attribute_group ktd202x_dev_attr_group = {
	.attrs = ktd202x_dev_attrs,
};

static int ktd202x_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct ktd202x_context *ctx = NULL;
	int err;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx) return -ENOMEM;

	ctx->client = client;
	ctx->regmap = devm_regmap_init_i2c(client, &ktd202x_i2c_regmap_config);
	if (IS_ERR(ctx->regmap)) {
		err = PTR_ERR(ctx->regmap);
		dev_err(dev, "Failed to allocate register map: %d\n", err);
		return err;
	}

	err = sysfs_create_group(&dev->kobj, &ktd202x_dev_attr_group);
	if (err) {
		dev_err(dev, "Failed to register attribute groups: %d\n", err);
		return err;
	}

	i2c_set_clientdata(client, ctx);
	ktd202x_init(ctx);

	dev_info(dev, "Driver loaded for a %s.\n", id->name);

	return 0;
}

static int ktd202x_remove(struct i2c_client *client)
{
	struct ktd202x_context *ctx = i2c_get_clientdata(client);
	ktd202x_reset(ctx);

	sysfs_remove_group(&client->dev.kobj, &ktd202x_dev_attr_group);

	return 0;
}

static const struct of_device_id ktd202x_of_match[] = {
	{ .compatible = "kinetic,ktd2026" },
	{ .compatible = "kinetic,ktd2027" },
	{},
};
MODULE_DEVICE_TABLE(of, ktd202x_of_match);

static const struct i2c_device_id ktd202x_idtable[] = {
	{ "ktd2026", 0 },
	{ "ktd2027", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, ktd202x_idtable);

static struct i2c_driver ktd202x_driver = {
	.driver = {
		.name = "ktd202x",
		.of_match_table = of_match_ptr(ktd202x_of_match),
	},
	.probe = ktd202x_probe,
	.remove = ktd202x_remove,
	.id_table = ktd202x_idtable,
};

module_i2c_driver(ktd202x_driver);

MODULE_AUTHOR("June Tate-Gans <jtgans@google.com>");
MODULE_DESCRIPTION("Kinetic KTD2026/7 LED driver");
MODULE_LICENSE("GPL v2");
