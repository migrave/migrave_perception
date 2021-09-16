# from migrave_common.utils import vision_utils

class FaceRecognition(object):
    """
    Extract face feature
    """

    def __init__(self, config):
        self._config = config

    def recognize_face(self, image):
        """
        Extract face feature given image

        :param image:    The input rgb image to be preprocessed
        :type name:         numpy.array

        :return:    preprocessed image

        """
        # image = vision_utils.preprocess_image(image)

        #ToDo:
        #Face feature extraction with openface
        #it has 55 features
        recognized_face = "unknown"

        return recofnized_face

