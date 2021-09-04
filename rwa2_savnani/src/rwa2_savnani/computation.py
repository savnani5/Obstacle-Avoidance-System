#!/usr/bin/env python

# generate random floating point values
import math
from random import seed
from random import random
# seed random number generator
seed(1)


def generate_random():
    """Generate a random value between 1 and 2

    Return
    ----------
    The random value.
    """
    # generate random numbers between 0-1
    value = random()
    scaled_value = 1 + (value * (2 - 1))
    return scaled_value


def compute_distance(x1, y1, x2, y2):
    """Compute the distance between 2 points.

    Parameters
    ----------
    x1 : float
        x coordinate of the first point.
    y1 : float
        y coordinate of the first point.
    x2 : float
        x coordinate of the second point.
    y2 : float
        y coordinate of the second point.

    Return
    ----------
    The distance between between a point (x1,y1) and another point (x2,y2).
    """
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


