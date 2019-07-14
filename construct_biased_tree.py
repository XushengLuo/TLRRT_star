import numpy as np
import pyvisgraph as vg


def construction_biased_tree(tree, n_max):
    final = True
    # trivial suffix path
    if tree.segment == 'suffix' and tree.check_transition_b(tree.init[1], tree.biased_tree.nodes[tree.init]['label'],
                                                            tree.init[1]):
        tree.goals.append(tree.init)
        return {0: [0, []]}

    for n in range(n_max):
        # biased sample
        x_new, q_p_closest = tree.sample()
        # couldn't find x_new
        if not x_new:
            continue

        label = []
        for i in range(tree.robot):
            ap = tree.get_label(x_new[i])
            if ap != '':
                ap = ap + '_' + str(i+1)
            label.append(ap)
        # near state
        # near_nodes = tree.near(tree.mulp2single(x_new))
        # if q_p_closest not in near_nodes:
        #     near_nodes = near_nodes + [q_p_closest]
        near_nodes = [q_p_closest]

        # check the line is obstacle-free
        obs_check = tree.obstacle_check(near_nodes, x_new, label)
        # not obstacle-free
        if not list(obs_check.items())[0][1]:
            continue

        # iterate over each buchi state
        for b_state in tree.buchi.buchi_graph.nodes:
            # new product state
            q_new = (x_new, b_state)
            # extend
            added = tree.extend(q_new, near_nodes, label, obs_check) and final
            # rewire
            if added == 1:
                pass
            #     tree.rewire(q_new, near_v, obs_check)

        # first accepting state
        if len(tree.goals)>2:
            break

    cost_path = tree.find_path(tree.goals)
    return cost_path


def path_via_visibility(tree, path):

    paths = []
    max_len = 0
    # find a path for each robot
    for i in range(tree.robot):
        init = path[-1][0][i]
        goal = path[0][0][i]
        shortest = tree.g.shortest_path(vg.Point(init[0], init[1]), vg.Point(goal[0], goal[1]))
        max_len = len(shortest) if len(shortest) > max_len else max_len
        paths.append([(point.x, point.y) for point in shortest])
    # append to the same length
    for i in range(tree.robot):
        paths[i] = paths[i] + [paths[i][-1]]*(max_len-len(paths[i]))
    # one path of product state
    path_free = []
    for i in range(max_len):
        path_free.append((tuple([p[i] for p in paths]), ''))  # second component serves as buchi state
    # calculate cost
    cost = 0
    for i in range(1, max_len):
        cost = cost + np.linalg.norm(np.subtract(tree.mulp2single(path_free[i][0]), tree.mulp2single(path_free[i-1][0])))

    return cost, path_free
