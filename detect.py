from hokuyolx import HokuyoLX
import numpy as np


track_width = 2.0  # wheel center to center distance of car
forward_constant = 1.0 # multiplier for speed of car, adjust for proper braking distance
car_length = 6.0 # length of car from front to back wheels, center to center


def in_path(points, speed, angle):

    r_center = car_length * np.sin(90 - angle)

    # transform points to match new origin at turn center
    points[0, :] += r_center
    points[1, :] += car_length

    r_cf = np.hypot(r_center, car_length)  # front center radius

    max_y = forward_constant * speed

    x_min = -track_width / \
        2 if angle > 0 else np.sqrt(
            np.pow(r_cf - track_width/2, 2) - np.pow(max_y, 2))
    x_max = -track_width / \
        2 if angle < 0 else np.sqrt(
            np.pow(r_cf - track_width/2, 2) - np.pow(max_y, 2))

    # filter points to x range
    points = points[points[0, :] <= x_max and points[0, :] >= x_min]

    # check for points inside predicted car path
    return np.any(points[
        points[1, :] >= np.sqrt(
            np.pow(points[0, :], 2) + np.pow(r_cf - track_width/2, 2))
        and
        points[1, :] <= np.sqrt(
            np.pow(points[0, :], 2) + np.pow(r_cf + track_width/2, 2))
    ])


