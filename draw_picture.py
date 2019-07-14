import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection


def region_plot(workspace, object, flag, ax):
    """
    plot the workspace
    :param regions: regions
    :param flag: regions or obstacle
    :param ax: figure axis
    :param d: 2D or 3D
    :return: none
    """

    ax.set_xlim((0, workspace[0]))
    ax.set_ylim((0, workspace[1]))
    plt.rc('text', usetex=True)
    plt.rc('font', family='serif')
    plt.gca().set_aspect('equal', adjustable='box')
    plt.grid(b=True, which='major', color='k', linestyle='--')
    for key in object.keys():
        color = '0.75' if flag != 'region' else 'c'
        x = []
        y = []
        patches = []
        for point in list(object[key].exterior.coords)[:-1]:
            x.append(point[0])
            y.append(point[1])
        polygon = Polygon(np.column_stack((x, y)), True)
        patches.append(polygon)
        p = PatchCollection(patches, facecolors=color, edgecolors=color)
        ax.add_collection(p)
        ax.text(np.mean(x), np.mean(y), r'${}_{}$'.format(key[0], key[1:]), fontsize=16)


def path_plot(path, workspace, number_of_robots):
    """
    plot the optimal path in the 2D and 3D
    :param path: ([pre_path], [suf_path])
    :param workspace:
    :return: none
    """

    for n in range(number_of_robots):
        ax = plt.figure(n).gca()
        region_plot(workspace.workspace, workspace.regions, 'region', ax)
        region_plot(workspace.workspace, workspace.obs, 'obs', ax)

        # prefix path
        x_pre = np.asarray([point[0][n][0] for point in path[0]])
        y_pre = np.asarray([point[0][n][1] for point in path[0]])
        pre = plt.quiver(x_pre[:-1], y_pre[:-1], x_pre[1:] - x_pre[:-1], y_pre[1:] - y_pre[:-1], color='r',
                         scale_units='xy', angles='xy', scale=1, label='prefix path')

        # suffix path
        x = [point[0][n][0] for point in path[1]]
        y = [point[0][n][1] for point in path[1]]
        x_suf = np.asarray(x + [x_pre[-1]])
        y_suf = np.asarray(y + [y_pre[-1]])
        suf = plt.quiver(x_suf[:-1], y_suf[:-1], x_suf[1:] - x_suf[:-1], y_suf[1:] - y_suf[:-1], color='g',
                         scale_units='xy', angles='xy', scale=1, label='suffix path')

        plt.legend(handles=[pre, suf])
        plt.savefig('path{0}.png'.format(n), bbox_inches='tight', dpi=600)
