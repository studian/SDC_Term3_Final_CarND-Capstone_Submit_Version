from styx_msgs.msg import TrafficLight
import scipy.misc
import tensorflow as tf
import numpy as np

# TLC_GRAPH_PATH = 'optimized_graph.pb'
TLC_INPUT_SHAPE = (300, 400)
TL_STATES = [TrafficLight.RED, TrafficLight.YELLOW, TrafficLight.GREEN, TrafficLight.UNKNOWN]

class TLClassifier(object):
    def __init__(self, path):
        #TODO load classifier
        self.tlc_graph = tf.Graph()

        with self.tlc_graph.as_default():
            gd = tf.GraphDef()
            with tf.gfile.GFile(path, 'rb') as f:
                serialized_graph = f.read()
                gd.ParseFromString(serialized_graph)
                tf.import_graph_def(gd, name='')

            self.input_tensor = self.tlc_graph.get_tensor_by_name('input_1:0')
            self.predictions_tensor = self.tlc_graph.get_tensor_by_name('predictions/Softmax:0')
            self.sess = tf.Session(graph=self.tlc_graph)

        self.img_count = 0



    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
        image = scipy.misc.imresize(image, TLC_INPUT_SHAPE)
        predictions = self.sess.run(self.predictions_tensor, {self.input_tensor: [image]})

        tl_state_id = np.argmax(predictions)

        return TL_STATES[tl_state_id]