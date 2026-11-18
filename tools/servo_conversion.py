from math import radians, pi

TICKS = 1024                       # servo positions go from 0 to 1023
RAD_PER_TICK = radians(300) / TICKS  # size of one tick, in radians


def angle_to_ticks(angle, neutral):
    """Convert a joint angle (radians) to a servo tick number."""
    ticks = neutral + angle / RAD_PER_TICK
    if ticks >= TICKS:
        return TICKS - 1
    if ticks < 0:
        return 0
    return int(ticks)


def ticks_to_angle(ticks, neutral):
    """Convert a servo tick reading to a joint angle (radians)."""
    return (ticks - neutral) * RAD_PER_TICK


if __name__ == "__main__":
    print(ticks_to_angle(496, 506))        # expect about -0.0511
    print(angle_to_ticks(0.0, 506))        # expect 506
    print(angle_to_ticks(pi / 2, 506))     # expect 813
    print(angle_to_ticks(10.0, 506))       # expect 1023 (clamped)
    print(angle_to_ticks(-10.0, 506))      # expect 0 (clamped)
