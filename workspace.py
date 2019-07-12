from shapely.geometry import Polygon, Point
import pyvisgraph as vg
import numpy as np
from random import uniform


def get_label(x, regions, obs):
    """
    generating the label of position state
    """
    point = Point(x)
    # whether x lies within obstacle
    for (obs, boundary) in iter(obs.items()):
        if point.within(boundary):
            return obs

    # whether x lies within regions
    for (region, boundary) in iter(regions.items()):
        if point.within(boundary):
            return region
    # x lies within unlabeled region
    return ''


class Workspace(object):
    """
    define the workspace where robots reside
    """
    def __init__(self):
        # dimension of the workspace
        self.length = 1
        self.width = 1
        self.dim = (self.length, self.width)
        # define regions (eg, isosceles right triangle)
        length_of_side = 0.2
        # coordinates of the right angle point
        center = [(0.1, 0.7), (0.7, 0.7), (0.7, 0.3), (0.3, 0.3), (0, 0.1), (0, 0.4)]
        self.regions = {'l1': Polygon([(center[0][0], center[0][1]), (center[0][0] + length_of_side, center[0][1]),
                                       (center[0][0], center[0][1] + length_of_side)]),
                        'l2': Polygon([(center[1][0], center[1][1]), (center[1][0] + length_of_side, center[1][1]),
                                       (center[1][0], center[1][1] + length_of_side)]),
                        'l3': Polygon([(center[2][0], center[2][1]), (center[2][0] + length_of_side, center[2][1]),
                                       (center[2][0], center[2][1] + length_of_side)]),
                        'l4': Polygon([(center[3][0], center[3][1]), (center[3][0] + length_of_side, center[3][1]),
                                       (center[3][0], center[3][1] + length_of_side)]),
                        'l5': Polygon([(center[4][0], center[4][1]), (center[4][0] + length_of_side, center[4][1]),
                                       (center[4][0], center[4][1] + length_of_side)]),
                        'l6': Polygon([(center[5][0], center[5][1]), (center[5][0] + length_of_side, center[5][1]),
                                       (center[5][0], center[5][1] + length_of_side)]),
                        }

        # define obstacles (e.g., rectangle)
        self.obs = {'o1': Polygon([(0.3, 0.0), (0.7, 0.0), (0.7, 0.2), (0.3, 0.2)]),
                    'o2': Polygon([(0.4, 0.7), (0.6, 0.7), (0.6, 1.0), (0.4, 1.0)])
                    }

        self.number_of_robots = 1
        # randomly generate initial locations of robots with empty atomic propositions
        self.init = []
        for i in range(self.number_of_robots):
            while 1:
                ini = [round(uniform(0, self.dim[i]), 3) for i in range(len(self.dim))]
                ap = get_label(ini, self.regions, self.obs)
                if 'l' not in ap and 'o' not in ap:
                    break
            self.init.append(tuple(ini))
        self.init = tuple(self.init)
        self.init_label = ['']*self.number_of_robots


    def shortest_path_between_regions(self):
        """
        calculate shoresr path between any two labeled regions
        :param regions: regions
        :return: dict (region, region) : length
        """
        polys = [[vg.Point(0.4, 1.0), vg.Point(0.4, 0.7), vg.Point(0.6, 0.7), vg.Point(0.6, 1.0)],
                 [vg.Point(0.3, 0.2), vg.Point(0.3, 0.0), vg.Point(0.7, 0.0), vg.Point(0.7, 0.2)]]
        g = vg.VisGraph()
        g.build(polys, status=False)

        min_len_region = dict()
        for key1, value1 in self.regions.items():
            for key2, value2 in self.regions.items():
                init = value1[:2]
                tg = value2[:2]
                # shorest path between init and tg point
                shortest = g.shortest_path(vg.Point(init[0], init[1]), vg.Point(tg[0], tg[1]))
                # (key2, key1) is already checked
                if (key2, key1) in min_len_region.keys():
                    min_len_region[(key1, key2)] = min_len_region[(key2, key1)]
                else:
                    # different regions
                    if key1 != key2:
                        dis = 0
                        for i in range(len(shortest)-1):
                            dis = dis + np.linalg.norm(np.subtract((shortest[i].x, shortest[i].y), (shortest[i+1].x, shortest[i+1].y)))

                        min_len_region[(key1, key2)] = dis
                    # same region
                    else:
                        min_len_region[(key1, key2)] = 0

        return min_len_region