import math

DEFAULT_PAN_POSITION = 0
DEFAULT_TILT_POSITION = 0


def find_camera_distance(camera_to_gun_dist: float, angle_a: float, angle_b: float) -> float:
    """Calculate horizontal distance from the camera to the target."""
    angle_a_rad = math.radians(angle_a)
    angle_b_rad = math.radians(angle_b)
    camera_distance_x = (
            2 * camera_to_gun_dist * math.sin(angle_a_rad) / math.sin(angle_a_rad + angle_b_rad)
    )
    return camera_distance_x


def find_distance_x(camera_to_gun_dist: float, camera_distance_x: float, camera_pan_angle: float) -> float:
    """Calculate horizontal distance from the gun to the target."""
    pan_angle_rad = math.radians(camera_pan_angle)
    cos_pan = math.cos(pan_angle_rad)
    x_distance = math.sqrt(
        camera_to_gun_dist ** 2 + camera_distance_x ** 2 - 2 * camera_to_gun_dist * camera_distance_x * cos_pan
    )
    return x_distance


def find_distance_y(camera_distance_x: float, camera_tilt_angle: float) -> float:
    """Calculate vertical distance from the gun to the target."""
    return camera_distance_x * math.tan(math.radians(camera_tilt_angle))


def find_servo_pan_angle(x_distance: float, camera_distance_x: float, camera_pan_angle: float) -> tuple[int, float]:
    """Calculate required pan angle for the servo."""
    pan_angle_rad = math.radians(camera_pan_angle)
    sin_pan = math.sin(pan_angle_rad)
    ratio = (camera_distance_x / x_distance) * sin_pan
    clamped_ratio = max(-1, min(1, ratio))
    pan_angle = math.degrees(math.asin(clamped_ratio))
    return int(round(pan_angle)), pan_angle


def find_servo_tilt_angle(
        x_distance: float, y_distance: float, launch_velocity: float, use_laser: bool = False, g: float = 9.8
) -> tuple[int, str | float]:
    """Calculate required tilt angle for the servo."""
    if x_distance <= 0:
        return DEFAULT_TILT_POSITION, "Out of Range"
    if use_laser:
        laser_angle = math.degrees(math.atan(y_distance / x_distance))
        return int(round(laser_angle)), laser_angle

    v_squared = launch_velocity ** 2
    g_x_squared = g * x_distance ** 2
    discriminant = x_distance ** 2 - (2 * g_x_squared / v_squared) * (
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
    theta, phi = 33, 75
    tilt_angle = 40
    initial_velocity = 22

    camera_distance = find_camera_distance(camera_spacing, theta, phi)
    distance_x = find_distance_x(camera_spacing, camera_distance, phi)
    distance_y = find_distance_y(camera_distance, tilt_angle)

    servo_pan_angle, actual_pan_angle = find_servo_pan_angle(distance_x, camera_distance, phi)
    laser_servo_tilt_angle, actual_laser_tilt_angle = find_servo_tilt_angle(distance_x, distance_y, initial_velocity,
                                                                            True)
    servo_tilt_angle, actual_tilt_angle = find_servo_tilt_angle(distance_x, distance_y, initial_velocity)

    print(f"Camera Distance: {camera_distance:.3f}m")
    print(f"Gun Distance - X: {distance_x:.3f}m, Y: {distance_y:.3f}m")
    print(f"Actual Pan Angle: {actual_pan_angle:.3f}°")
    print(f"Servo Pan Angle: {servo_pan_angle}°")
    print(f"Actual Tilt Angle: {actual_tilt_angle:.3f}°" if isinstance(actual_tilt_angle,
                                                                       float) else f"Actual Tilt Angle: {actual_tilt_angle}")
    print(f"Servo Tilt Angle: {servo_tilt_angle}°")
    print(f"Actual Tilt Angle (Laser): {actual_laser_tilt_angle:.3f}°")
    print(f"Servo Tilt Angle (Laser): {laser_servo_tilt_angle}°")
