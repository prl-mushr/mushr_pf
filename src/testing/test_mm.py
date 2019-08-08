# Testing Code for MotionModel.py
# Matt Schmittle
# DO NOT EDIT

import math
from threading import Lock

import matplotlib.pyplot as plt
import numpy as np

from MotionModel import KinematicMotionModel

# Radius to count particles around ground truth
radius = 0.1


def euclidean_distance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def model_error(model_particles, true_pose):
    r_disc = []
    for particle in model_particles:
        distance = euclidean_distance(true_pose, particle)
        if distance <= radius:
            r_disc.append(distance)
    return len(r_disc)


def test_mm_1(motion_model, counts):
    motion_model.apply_motion_model(motion_model.particles, [1.0, 0.34, 0.1])
    counts.append(
        model_error(motion_model.particles, [0.09937757, 0.0033936, 0.10634897])
    )
    return counts


def test_mm_2(motion_model, counts):
    motion_model.apply_motion_model(motion_model.particles, [3.0, 0.4, 0.5])
    counts.append(
        model_error(motion_model.particles, [0.76003705, 0.99738116, 1.86917909])
    )
    return counts


def test_mm_3(motion_model, counts):
    motion_model.apply_motion_model(motion_model.particles, [3.0, 0.4, 0.5])
    counts.append(
        model_error(motion_model.particles, [0.7661687, 0.97692261, 1.85254576])
    )
    return counts


def pass_fail(test_num, mean):
    if test_num == 1:
        if mean <= 90:
            return "FAIL"
        else:
            return "PASS"
    if test_num == 2:
        if mean <= 15:
            return "FAIL"
        else:
            return "PASS"
    if test_num == 3:
        if mean <= 10:
            return "FAIL"
        else:
            return "PASS"


if __name__ == "__main__":
    num_particles = 100
    counts = []
    counts1 = []
    counts2 = []

    # 99 tests. Hint: set me to 100 to have insight on the question about resampling
    for i in range(0, 99):
        start_particles = np.zeros(
            (num_particles, 3)
        )  # Numpy matrix of dimension N_PARTICLES x 3
        state_lock = (
            Lock()
        )  # A lock used to prevent concurrency issues. You do not need to worry about this
        # Instatiate Your Motion Model
        motion_model = KinematicMotionModel(
            "/fake_topic1",
            "/fake_topic2",
            0.0,
            4350,
            0.5,
            -1.2135,
            0.33,
            start_particles,
            state_lock,
        )

        # Apply 3 different controls
        if i % 3 == 0:
            counts = test_mm_1(motion_model, counts)
        if i % 3 == 1:
            counts1 = test_mm_2(motion_model, counts1)
        if i % 3 == 2:
            counts2 = test_mm_3(motion_model, counts2)

    print(
        "Average Number of Particles within " + str(radius) + "cm of the Ground Truth:"
    )
    print("Test 1:", np.mean(counts), pass_fail(1, np.mean(counts)))
    print("Test 2:", np.mean(counts1), pass_fail(2, np.mean(counts)))
    print("Test 3:", np.mean(counts2), pass_fail(3, np.mean(counts)))

    # Plot
    arrow_thickness = 0.03
    plt.xlabel("x")
    plt.ylabel("y")
    plt.ylim(bottom=-0.1)
    plt.xlim(left=-0.1)
    plt.ylim(top=1.5)
    delta = 0
    dx = arrow_thickness * math.cos(delta)
    dy = arrow_thickness * math.sin(delta)
    plt.quiver(0, 0, dx, dy, color="r")
    plt.quiver(
        motion_model.particles[:, 0],
        motion_model.particles[:, 1],
        arrow_thickness * np.cos(motion_model.particles[:, 2]),
        arrow_thickness * np.sin(motion_model.particles[:, 2]),
        color="b",
    )
    plt.show()
