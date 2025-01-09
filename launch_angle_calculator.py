import math


def calculate_launch_angle(x, y, v0):
    """
    Calculate the launch angles to hit a target at a given distance and height.

    Parameters:
        x (float): Horizontal distance to the target (m).
        y (float): Height difference to the target (m).
        v0 (float): Initial velocity of the projectile (m/s).

    Returns:
        tuple: A tuple of possible launch angles in degrees (lower angle, higher angle).
               Returns None if the target is unreachable.
    """
    g = 9.8  # Acceleration due to gravity (m/s^2)

    # Compute the discriminant of the quadratic equation
    discriminant = (v0 ** 4) - g * (g * x ** 2 + 2 * y * v0 ** 2)

    if discriminant < 0:
        return None  # Target is unreachable

    # Calculate the two possible angles (in radians)
    angle1 = math.atan((v0 ** 2 + math.sqrt(discriminant)) / (g * x))
    angle2 = math.atan((v0 ** 2 - math.sqrt(discriminant)) / (g * x))

    # Convert angles to degrees
    angle1_deg = math.degrees(angle1)
    angle2_deg = math.degrees(angle2)

    return min(angle1_deg, angle2_deg), max(angle1_deg, angle2_deg)


def calculate_laser_angle(x, y):
    """
    Calculate the angle for a laser to hit the target directly.

    Parameters:
        x (float): Horizontal distance to the target (m).
        y (float): Height difference to the target (m).

    Returns:
        float: The angle in degrees for a direct laser shot.
    """
    return math.degrees(math.atan2(y, x))


# Example usage
x = 4  # Horizontal distance to the target (m)
y = 1  # Height difference to the target (m)
v0 = 21  # Initial velocity (m/s)

angles = calculate_launch_angle(x, y, v0)
laser_angle = calculate_laser_angle(x, y)

if angles:
    print(f"Possible launch angles: {angles[0]:.2f} degrees and {angles[1]:.2f} degrees")
else:
    print("The target is unreachable with the given initial velocity.")

print(f"Laser angle: {laser_angle:.2f} degrees")
