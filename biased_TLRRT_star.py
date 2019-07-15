# -*- coding: utf-8 -*-

from task import Task
from buchi_parse import Buchi
from workspace import Workspace
import datetime
from collections import OrderedDict
import numpy as np
from biased_tree import BiasedTree
from construct_biased_tree import construction_biased_tree, path_via_visibility
from draw_picture import path_plot, path_print
import matplotlib.pyplot as plt
import pyvisgraph as vg

# task
task = Task()
buchi = Buchi(task)
buchi.construct_buchi_graph()
buchi.get_minimal_length()
buchi.get_feasible_accepting_state()
buchi_graph = buchi.buchi_graph
# workspace
workspace = Workspace()

n_max = 40000
para = dict()
# lite version, excluding extending and rewiring
para['is_lite'] = True
# step_size used in function near
para['step_size'] = 0.25 * buchi.number_of_robots
# probability
para['p_closest'] = 0.9
# target point
para['y_rand'] = 0.99
# threshold for collision avoidance
para['threshold'] = 0.005

cost_path = OrderedDict()

start = datetime.datetime.now()

for b_init in buchi_graph.graph['init']:
    # initialization
    opt_cost = (np.inf, 0)
    opt_path_pre = []
    opt_path_suf = []
    """
     #----------------------------------------------#
     |                Prefix Path                   |
     #----------------------------------------------#
    """
    init_state = (task.init, b_init)
    init_label = task.init_label
    tree_pre = BiasedTree(workspace, buchi, init_state, init_label, 'prefix', para)
    print('--------------prefix path---------------------')
    # prefix path with cost
    cost_path_pre = construction_biased_tree(tree_pre, n_max)
    if len(tree_pre.goals):
        pre_time = (datetime.datetime.now() - start).total_seconds()
        print('Time for prefix path: {0}'.format(pre_time))
        print('{0} accepting goals found'.format(len(tree_pre.goals)))
    else:
        print('Couldn\'t find the path within predetermined iteration')
        break

    """
     #----------------------------------------------#
     |                Suffix Path                   |
     #----------------------------------------------#
    """
    # treat all regions as obstacles
    polys_suf = []
    for poly in workspace.obs.values():
        polys_suf.append([vg.Point(x[0], x[1]) for x in list(poly.exterior.coords)[:-1]])

    for poly in workspace.regions.values():
        polys_suf.append([vg.Point(x[0], x[1]) for x in list(poly.exterior.coords)[:-1]])

    start = datetime.datetime.now()
    # # each initial state <=> multiple accepting states
    for i in range(len(tree_pre.goals)):
    # for i in range(1):
        # goal product state as the initial state
        init_state = tree_pre.goals[i]
        init_label = tree_pre.biased_tree.nodes[init_state]['label']
        buchi_graph.graph['accept'] = init_state[1]
        tree_suf = BiasedTree(workspace, buchi, init_state, init_label, 'suffix', para)
        # update accepting buchi state

        # construct suffix tree
        cost_path_suf_cand = construction_biased_tree(tree_suf, n_max)

        print('-----------suffix path for {0}-th goal (of {1} in total)--------------'.format(i, len(tree_pre.goals)))
        print('{0}-th goal: {1} accepting goals found'.format(i, len(tree_suf.goals)))

        # couldn't find the path
        if not cost_path_suf_cand:
            del cost_path_pre[i]
            print('delete {0}-th item in cost_path_pre, {1} left'.format(i, len(cost_path_pre)))
            continue
        # not trivial suffix path
        if cost_path_suf_cand[0][1]:
            # find the remaining path to close the loop
            tree_suf.g = vg.VisGraph()
            tree_suf.g.build(polys_suf, status=False)
            for k in range(len(cost_path_suf_cand)):
                cost, path_free = path_via_visibility(tree_suf, cost_path_suf_cand[k][1])
                cost_path_suf_cand[k][0] += cost
                cost_path_suf_cand[k][1] += path_free[1:]

        # order according to cost
        cost_path_suf_cand = OrderedDict(sorted(cost_path_suf_cand.items(), key=lambda x: x[1][0]))
        min_cost = list(cost_path_suf_cand.keys())[0]
        cost_path_suf = cost_path_suf_cand[min_cost]
        # update the best path so far
        if cost_path_pre[i][0] + cost_path_suf[0] < opt_cost[0] + opt_cost[1]:
            opt_path_pre = cost_path_pre[i][1]  # plan of [(position, buchi)]
            opt_path_suf = cost_path_suf[1]
            opt_cost = [cost_path_pre[i][0], cost_path_suf[0]]  # optimal cost (pre_cost, suf_cost)

    suf_time = (datetime.datetime.now() - start).total_seconds()
    # first pre + suf path

    """
     #----------------------------------------------#
     |                  Pre + Suf                   |
     #----------------------------------------------#
    """

    print('Time to find the surfix path: {0}'.format(suf_time))
    print(pre_time, suf_time, opt_cost[0], opt_cost[1], pre_time + suf_time,
          (opt_cost[0] + opt_cost[1])/2)
    # path_plot((opt_path_pre, opt_path_suf), workspace, buchi.number_of_robots)
    path_print((opt_path_pre, opt_path_suf), workspace, buchi.number_of_robots)
    # plt.show()
