#!/usr/bin/env python

# Copyright (c) 2019, The Personal Robotics Lab, The MuSHR Team, The Contributors of MuSHR
# License: BSD 3-Clause. See LICENSE.md file in root directory.

"""
  Provides methods for re-sampling from a distribution represented by weighted samples
"""

from threading import Lock

import numpy as np


class ReSampler:
    """
    Initializes the resampler
    particles: The particles to sample from
    weights: The weights of each particle
    state_lock: Controls access to particles and weights
    """

    def __init__(self, particles, weights, state_lock=None):
        self.particles = particles
        self.weights = weights

        # Indices for each of the M particles
        self.particle_indices = np.arange(self.particles.shape[0])

        # Bins for partitioning of weight values
        self.step_array = (1.0 / self.particles.shape[0]) * np.arange(
            self.particles.shape[0], dtype=np.float32
        )

        if state_lock is None:
            self.state_lock = Lock()
        else:
            self.state_lock = state_lock

    def resample_low_variance(self):
        """
        Performs in-place, lower variance sampling of particles
        returns: nothing
        """
        self.state_lock.acquire()

        # YOUR CODE HERE

        # resolve issue of 0 weights in circle.bag
        if abs(self.weights.sum() - 1.0) > 1e-6:
            self.weights.fill(1.0 / self.weights.shape[0])
            self.state_lock.release()
            return

        M = self.particles.shape[0]
        avg = 1.0 / M
        cdf = np.cumsum(self.weights)
        r = np.random.uniform(0, avg)
        bins = np.arange(M) * avg + r
        particles = np.zeros_like(self.particles)
        cnt, idx = 0, 0
        while idx < M:
            if bins[idx] < cdf[cnt]:
                particles[idx] = self.particles[cnt]  # assign the particle
                idx += 1
            else:
                cnt += 1
        self.particles[:] = particles[:]
        self.weights[:] = avg
        self.state_lock.release()
