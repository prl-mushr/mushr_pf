# Testing Code for MotionModel.py
# Matt Schmittle
# DO NOT EDIT

import math
import unittest
from threading import Lock

import matplotlib.pyplot as plt
import numpy as np

from mushr_pf.motion_model import KinematicMotionModel

# Radius to count particles around ground truth
RADIUS = 0.25
SHOW_PLOT = False


def euclidean_distance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def model_error(model_particles, true_pose):
    r_disc = []
    for particle in model_particles:
        distance = euclidean_distance(true_pose, particle)
        if distance <= RADIUS:
            r_disc.append(distance)
    return len(r_disc)


class TestMotionModel(unittest.TestCase):
    def __init__(self, *args, **kwargs):
        super(TestMotionModel, self).__init__(*args, **kwargs)

        self.num_particles = 100
        self.ctrls = [[1.0, 0.34, 0.1], [3.0, 0.4, 0.5], [3.0, 0.4, 0.5]]
        self.truths = [
            [0.09937757, 0.0033936, 0.10634897],
            [0.76003705, 0.99738116, 1.86917909],
            [0.7661687, 0.97692261, 1.85254576],
        ]

    def mm_1(self, mm):
        mm.apply_motion_model(mm.particles, self.ctrls[0])
        return model_error(mm.particles, self.truths[0])

    def mm_2(self, mm):
        mm.apply_motion_model(mm.particles, self.ctrls[1])
        return model_error(mm.particles, self.truths[1])

    def mm_3(self, mm):
        mm.apply_motion_model(mm.particles, self.ctrls[2])
        return model_error(mm.particles, self.truths[2])

    def test_motion_model(self):
        counts = [[] for _ in range(3)]
        mms = [self.mm_1, self.mm_2, self.mm_3]

        # 99 tests. Hint: set me to 100 to have insight on the question about resampling
        for i in range(0, 99):
            start_particles = np.zeros((self.num_particles, 3))
            # Instatiate Your Motion Model
            motion_model = self.new_mm(start_particles)
            # Apply 3 different controls
            counts[i % 3].append(mms[i % 3](motion_model))

        self.assertTrue(np.mean(counts[0]) > 90)
        self.assertTrue(
            np.mean(counts[1]) > 15,
            msg="Mean: %.3f There was a bug in the source code, the particle filter is functional,"
            "but the RADIUS is either too small, or the parameters chosen are way too"
            "large (even though the PF is really good)." % np.mean(counts[1]),
        )
        self.assertTrue(
            np.mean(counts[2]) > 10,
            msg="Mean: %f There was a bug in the source code, the particle filter is functional,"
            "but the RADIUS is either too small, or the parameters chosen are way too"
            "large (even though the PF is really good)." % np.mean(counts[2]),
        )

        if SHOW_PLOT:
            for i in range(3):
                # Plot
                plt.figure()
                arrow_thickness = 0.03
                plt.xlabel("x")
                plt.ylabel("y")
                plt.ylim(bottom=-0.1)
                plt.xlim(left=-0.1)
                plt.ylim(top=1.5)

                start_particles = np.zeros((self.num_particles, 3))
                mm = self.new_mm(start_particles)
                mms[i](mm)
                plt.quiver(
                    mm.particles[:, 0],
                    mm.particles[:, 1],
                    arrow_thickness * np.cos(mm.particles[:, 2]),
                    arrow_thickness * np.sin(mm.particles[:, 2]),
                    color="b",
                )

                # Plot start
                delta = 0
                dx = arrow_thickness * math.cos(delta)
                dy = arrow_thickness * math.sin(delta)
                plt.quiver(0, 0, dx, dy, color="r")

                # Plot finish ground truth
                dx = arrow_thickness * math.cos(self.truths[i][2])
                dy = arrow_thickness * math.sin(self.truths[i][2])
                plt.quiver(self.truths[i][0], self.truths[i][1], dx, dy, color="g")

                # Plot radius of acceptance around ground truth
                ax = plt.gca()
                ax.add_artist(
                    plt.Circle(self.truths[i][:2], RADIUS, color="g", fill=False)
                )
            plt.show()

    def new_mm(self, start_particles):
        state_lock = Lock()
        # Instatiate Your Motion Model
        mm = KinematicMotionModel(
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
        return mm


if __name__ == "__main__":
    unittest.main()
