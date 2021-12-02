import numpy as np
import parselmouth as pm

from mas_tools.file_utils import load_yaml_file

class AudioFeatures(object):
    pitch_frequency = None
    var_pitch_frequency = None
    pitch_strength = None
    var_pitch_strength = None
    mean_harmonicity = None
    var_harmonicity = None
    mean_intensity = None
    var_intensity = None
    mfccs = None
    var_mfccs = None

class AudioFeatureDetector(object):
    sampling_rate = -1

    def __init__(self, config_file: str):
        config = load_yaml_file(config_file)
        self.sampling_rate = config['sampling_rate']

    def get_audio_features(self, sound_frame: np.float64):
        sound = pm.Sound(sound_frame, self.sampling_rate)

        features = AudioFeatures()

        pitch = sound.to_pitch()
        features.pitch_frequency = np.mean(pitch.selected_array['frequency'])
        features.var_pitch_frequency = np.var(pitch.selected_array['frequency'])
        features.pitch_strength = np.mean(pitch.selected_array['strength'])
        features.var_pitch_strength = np.var(pitch.selected_array['strength'])

        harmonicity = sound.to_harmonicity()
        features.mean_harmonicity = np.mean(harmonicity.values)
        features.var_harmonicity = np.var(harmonicity.values)

        intensity = sound.to_intensity()
        features.mean_intensity = np.mean(intensity.values)
        features.var_intensity = np.var(intensity.values)

        mfccs = sound.to_mfcc(number_of_coefficients=2).to_array()
        features.mfccs = np.array([np.mean(mfccs[1]), np.mean(mfccs[2])])
        features.var_mfccs = np.array([np.var(mfccs[1]), np.var(mfccs[2])])
        return features
