from shapely.geometry import Point, Polygon, LineString
import pyvisgraph as vg


obs = {'o1': Polygon([(0.3, 0.0), (0.7, 0.0), (0.7, 0.2), (0.3, 0.2)]),
                    'o2': Polygon([(0.4, 0.7), (0.6, 0.7), (0.6, 1.0), (0.4, 1.0)])
                    }

polys = []
for poly in obs.values():
    polys.append([vg.Point(x[0], x[1]) for x in list(poly.exterior.coords)[:-1]])
print(polys)