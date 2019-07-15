# -*- coding: utf-8 -*-

from shapely.geometry import Polygon, Point


def get_label(x, workspace):
    """
    generating the label of position component
    """
    point = Point(x)
    # whether x lies within obstacle
    for (obs, boundary) in iter(workspace.obs.items()):
        if point.within(boundary):
            return obs

    # whether x lies within regions
    for (region, boundary) in iter(workspace.regions.items()):
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
        self.workspace = (self.length, self.width)
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
