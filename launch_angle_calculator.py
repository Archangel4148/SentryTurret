import math

DEFAULT_PAN_POSITION = 0
DEFAULT_TILT_POSITION = 0


# ========== Required inputs: ==========
#  - Camera Spacing (From gun to camera)
#  - Horizontal angles (theta + phi)
#  - One of the vertical angles (psi)
# ======================================

def find_camera_distance(camera_spacing: float, angle_a: float, angle_b: float) -> float:
    """
    Finds the horizontal distance from a reference camera to the target point
    :param camera_spacing:
    :param angle_a: The horizontal angle from arbitrary camera A to the target
    :param angle_b: The horizontal angle from arbitrary camera B to the target
    :return: The horizontal distance from camera A to the target
    """
    camera_distance = 2 * camera_spacing * (math.sin(angle_a) / math.sin(180 - angle_a - angle_b))
    return camera_distance


def find_distance_x(camera_spacing: float, camera_distance: float, camera_pan_angle: float) -> float:
    """
    Finds the horizontal distance from the gun to the target point
    :param camera_distance: The distance from the reference camera to the target point (c)
    :param camera_spacing: The distance from the gun to each camera (camera to camera = double this value)
    :param camera_pan_angle: The angle from the reference camera to the target point
    :return: The horizontal distance from the gun to the target point
    """

    x_distance = math.sqrt(
        camera_spacing ** 2 + camera_distance ** 2 - 2 * camera_spacing * camera_distance * math.cos(camera_pan_angle))
    return x_distance


def find_distance_y(camera_distance: float, camera_tilt_angle: float) -> float:
    """
    Finds the vertical distance from the gun to the target point
    :param camera_distance: The distance from the reference camera to the target point (c)
    :param camera_tilt_angle: The vertical angle from the reference camera to the target point
    :return: The vertical distance from the gun to the target
    """
    y_distance = camera_distance * math.tan(camera_tilt_angle)

    return y_distance


def find_servo_pan_angle(x_distance: float, camera_distance: float, camera_pan_angle: float) -> int:
    """
    Finds the required pan angle from the gun to the target point
    :param x_distance: The horizontal distance from the gun to the target point
    :param camera_distance: The distance from the reference camera to the target point (c)
    :param camera_pan_angle: The horizontal angle from the reference camera to the target point
    :return: The pan angle from the gun to the target point
    """

    pan_angle = math.asin(camera_distance / x_distance * math.sin(camera_pan_angle))

    # Round the angle to be handled by the servo
    servo_pan_angle = int(round(pan_angle))

    return servo_pan_angle


def find_servo_tilt_angle(x_distance: float, y_distance: float, initial_velocity: float, g: float = 9.8) -> int:
    """
    Finds the required tilt angle from the gun to the target point
    :param x_distance: The horizontal distance from the gun to the target point
    :param y_distance: The vertical distance from the gun to the target point
    :param initial_velocity: The initial velocity of the projectile (in meters/second)
    :param g: The gravitational acceleration constant (9.8 m/s^2 by default)
    :return: The tilt angle from the gun to the target point (accounting for gravity)
    """

    # Calculate the discriminant
    discriminant = x_distance ** 2 - ((2 * g * x_distance ** 2) / initial_velocity ** 2) * (
            y_distance + ((g * x_distance ** 2) / (2 * initial_velocity ** 2)))

    # If the target is out of range, return to default position
    if discriminant < 0:
        return DEFAULT_TILT_POSITION

    # Find the two solutions for the angle
    angle_1 = (x_distance + math.sqrt(discriminant)) / ((g * x ** 2) / (initial_velocity ** 2))
    angle_2 = (x_distance - math.sqrt(discriminant)) / ((g * x ** 2) / (initial_velocity ** 2))

    # Round the values to be handled by the servo
    servo_angle = int(round(min(angle_1, angle_2)))

    return servo_angle
