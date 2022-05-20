from hokuyolx import HokuyoLX
import numpy as np


track_width = 2.0  # wheel center to center distance of car
forward_constant = 1.0 # multiplier for speed of car, adjust for proper braking distance
car_length = 6.0 # length of car from front to back wheels, center to center

graph = True

if graph:
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle


def in_path(points, speed, angle):

    r_center = car_length * np.sin(90 - angle)

    # transform points to match new origin at turn center
    points[:, 0] += r_center
    points[:, 1] += car_length

    r_cf = np.hypot(r_center, car_length)  # front center radius

    y_max = forward_constant * speed

    x_min = r_center - track_width / \
        2 if angle < 0 else np.sqrt(
            np.pow(r_cf - track_width/2, 2) - np.pow(max_y, 2))
    x_max = r_center + track_width / \
        2 if angle > 0 else np.sqrt(
            np.pow(r_cf + track_width/2, 2) - np.pow(max_y, 2))

    # filter points to x range
    points = points[points[:, 0] <= x_max and points[:, 0] >= x_min]

    if graph:
        fig, ax = plt.subplots()
        ax.plot(0, 0, label='Turn Center')
        ax.add_patch(Rectangle(r_center - track_width/2, 0), track_width, car_length)
        x1 = np.linspace(r_center - track_width / 2, np.sqrt(np.pow(r_cf - track_width/2, 2) - np.pow(max_y, 2)), 100) # left boundary
        x2 = np.linspace(r_center + track_width / 2, np.sqrt(np.pow(r_cf + track_width/2, 2) - np.pow(max_y, 2)), 100) # right boundary
        y1 = np.sqrt(np.pow(r_center - track_width / 2, 2) - np.pow(x1, 2))
        y2 = np.sqrt(np.pow(r_center + track_width / 2, 2) - np.pow(x2, 2))
        ax.plot(x1, y1)
        ax.plot(x2, y2)
        plt.show()

    # check for points inside predicted car path
    return np.any(points[
        points[:, 1] >= np.sqrt(
            np.pow(points[:, 0], 2) + np.pow(r_cf - track_width/2, 2))
        and
        points[:, 1] <= np.sqrt(
            np.pow(points[:, 0], 2) + np.pow(r_cf + track_width/2, 2))
        and
        points[:, 1] >= car_length
        and
        points[:, 1] <= y_max
    ])


in_path()