#!/usr/bin/env python3
#!/home/sdhm/.virtualenvs/keras_tf_p3/bin/python3
import tensorflow as tf

hello = tf.constant('Hello, tensorflow')
sess = tf.Session()
print(sess.run(hello))
