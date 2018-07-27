# generate graphs for pb and visualize 
import tensorflow as tf
from tensorflow.python.platform import gfile
with tf.Session() as sess:
	# chaange this
    model_filename ='mobilenet_ssd_256res_0.125_person_cat_dog.pb'
    with gfile.FastGFile(model_filename, 'rb') as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())
        g_in = tf.import_graph_def(graph_def)
# directory path        
LOGDIR='./logs/tests/1/'
train_writer = tf.summary.FileWriter(LOGDIR)
train_writer.add_graph(sess.graph)
# tensorboard --logdir .
