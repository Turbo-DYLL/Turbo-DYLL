class LatPIDResult:
    def __init__(self, steering: float, error: float, wide_error: float, sharp_error: float):
        self.steering = steering
        self.error = abs(round(error, 3))
        self.wide_error = abs(round(wide_error, 3))
        self.sharp_error = abs(round(sharp_error, 3))
