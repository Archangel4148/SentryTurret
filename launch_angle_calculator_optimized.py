import math

DEFAULT_PAN_POSITION = 0
DEFAULT_TILT_POSITION = 0


def find_camera_distance(camera_spacing: float, angle_a: float, angle_b: float) -> float:
    """Calculate horizontal distance from the camera to the target."""
    angle_a_rad = math.radians(angle_a)
    angle_b_rad = math.radians(angle_b)
    camera_distance = (
        2 * camera_spacing * math.sin(angle_a_rad) / math.sin(math.pi - angle_a_rad - angle_b_rad)
    )
    return camera_distance


def find_distance_x(camera_spacing: float, camera_distance: float, camera_pan_angle: float) -> float:
    """Calculate horizontal distance from the gun to the target."""
    pan_angle_rad = math.radians(camera_pan_angle)
    x_distance = math.sqrt(
        camera_spacing**2 + camera_distance**2 - 2 * camera_spacing * camera_distance * math.cos(pan_angle_rad)
    )
    return x_distance


def find_distance_y(camera_distance: float, camera_tilt_angle: float) -> float:
    """Calculate vertical distance from the gun to the target."""
    return camera_distance * math.tan(math.radians(camera_tilt_angle - 90))


def find_servo_pan_angle(x_distance: float, camera_distance: float, camera_pan_angle: float) -> tuple[int, float]:
    """Calculate required pan angle for the servo."""
    pan_angle_rad = math.radians(camera_pan_angle)
    sin_pan = math.sin(pan_angle_rad)
    pan_angle = math.degrees(math.asin((camera_distance / x_distance) * sin_pan))
    return int(round(pan_angle)), pan_angle


def find_servo_tilt_angle(
    x_distance: float, y_distance: float, initial_velocity: float, g: float = 9.8
) -> tuple[int, str | float]:
    """Calculate required tilt angle for the servo."""
    v_squared = initial_velocity**2
    g_x_squared = g * x_distance**2
    discriminant = x_distance**2 - (2 * g_x_squared / v_squared) * (
        y_distance + g_x_squared / (2 * v_squared)
    )

    if discriminant < 0:
        return DEFAULT_TILT_POSITION, "Out of Range"

    sqrt_discriminant = math.sqrt(discriminant)
    angle_1 = math.degrees(math.atan((x_distance + sqrt_discriminant) / (g_x_squared / v_squared)))
    angle_2 = math.degrees(math.atan((x_distance - sqrt_discriminant) / (g_x_squared / v_squared)))

    actual_angle = min(angle_1, angle_2)
    return int(round(actual_angle)), actual_angle


if __name__ == '__main__':
    camera_spacing = 1.5
    theta, phi = 34, 50
    tilt_angle = 10
    initial_velocity = 22

    camera_distance = find_camera_distance(camera_spacing, theta, phi)
    distance_x = find_distance_x(camera_spacing, camera_distance, phi)
    distance_y = find_distance_y(camera_distance, tilt_angle)

    servo_pan_angle, actual_pan_angle = find_servo_pan_angle(distance_x, camera_distance, phi)
    servo_tilt_angle, actual_tilt_angle = find_servo_tilt_angle(distance_x, distance_y, initial_velocity)

    print(f"Camera Distance: {camera_distance:.3f}m")
    print(f"Gun Distance - X: {distance_x:.3f}m, Y: {distance_y:.3f}m")
    print(f"Actual Pan Angle: {actual_pan_angle:.3f}°")
    print(f"Servo Pan Angle: {servo_pan_angle}°")
    print(f"Actual Tilt Angle: {actual_tilt_angle:.3f}°")
    print(f"Servo Tilt Angle: {servo_tilt_angle}°")
