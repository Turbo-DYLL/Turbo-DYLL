


def sharp_turn(current_sharp_error, current_max_speed, max_sharp_error, max_max_speed, brake, throttle):
    """
    returns the desired throttle if the max speed is within the parameter
    """
    return brake,throttle if current_sharp_error < max_sharp_error and current_max_speed <= max_max_speed else None

def wide_turn(current_wide_error, current_max_speed, max_wide_error, max_max_speed, brake, throttle):
    return brake,throttle if current_wide_error < max_wide_error and current_max_speed > max_max_speed else None

def brake(current_wide_error, current_max_speed, max_wide_error, max_max_speed, brake, throttle):
    return brake, throttle