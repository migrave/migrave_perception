# from migrave_common import vision_utils
import numpy as np

class FaceFeatures(object):
    """
    Extract face feature
    """

    def __init__(self, config):
        self._config = config

    def get_face_features(self, image):
        """
        Extract face feature given image

        :param image:       The input rgb image to be preprocessed
        :type name:         numpy.array

        :return:            preprocessed image

        """
        # image = vision_utils.preprocess_image(image)

        #ToDo:
        #Face feature extraction with openface
        #it has 55 features
        face_landmarks = np.random.random((55))

        return face_landmarks

