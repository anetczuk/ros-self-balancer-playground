#!/usr/bin/env python3

import math
# import random
import numpy as np
import quaternion


def v_angle(v1, v2):
    scalar = np.dot( v1, v2 )
    arad = math.acos( scalar )
    adeg = math.degrees( arad )
    print( v1, v2, "angle:", arad, adeg )


def oz_angle( v1 ):
    oz = np.array( [0, 0, 1] )
    v_angle( v1, oz )


def qoz_angle( q1 ):
    oz = np.array( [0, 0, 1] )
    vecsprime = quaternion.rotate_vectors(q1, oz)
    v_angle( vecsprime, oz )


if __name__ == '__main__':
#     vecs = np.random.rand(3)
#     vecs = np.array( [0, 0, 1] )
    quats = quaternion.x
#     vecs = np.array( [1, 0, 0] )
#     vecsprime = quaternion.rotate_vectors(quats, vecs)
#     print( quats, vecs, vecsprime )
#
#     oz_angle( vecs )

    qoz_angle( quats )
