"""One-dimensional odometry prediction and fixed-gain camera correction.

No Kalman filter, covariance propagation, or probabilistic gain estimation.
"""


class FixedGainEstimator:
    def __init__(self, initial_distance: float, gain: float):
        if initial_distance <= 0 or not 0 <= gain <= 1:
            raise ValueError("initial_distance must be positive and gain in [0, 1]")
        self.distance = float(initial_distance)
        self.gain = float(gain)
        self.prediction = self.distance
        self.innovation = float("nan")
        self.corrections = 0

    def predict(self, forward_delta_m: float):
        self.distance -= forward_delta_m
        self.prediction = self.distance
        return self.distance

    def correct(self, camera_distance_m: float):
        self.innovation = camera_distance_m - self.distance
        self.distance += self.gain * self.innovation
        self.corrections += 1
        return self.distance
