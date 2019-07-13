import numpy as np
from random import uniform
from uniform_geometry import sample_uniform_geometry


def sample(tree):
    """
    sample point from the workspace
    :return: sampled point, tuple
    """
    # sample random nodes from two sets
    p_rand = np.random.uniform(0, 1, 1)
    if (p_rand <= tree.p_closest and len(tree.q_min2final) > 0) or not tree.not_q_min2final:
        q_p_closest = sample_uniform_geometry(tree.q_min2final)
    elif p_rand > tree.p_closest or not tree.q_min2final:
        q_p_closest = sample_uniform_geometry(tree.not_q_min2final)

    # find feasible successors of buchi state in q_rand
    reachable_q_b_closest = []
    for b_state in tree.buchi.buchi_graph.succ[q_p_closest[1]]:
        # if tree.t_satisfy_b(x_label, buchi_graph.edges[(q_rand[1], b_state)]['label']):
        if tree.check_transition_b_helper(tree.biased_tree.nodes[q_p_closest]['label'],
                                          tree.buchi.buchi_graph.edges[(q_p_closest[1], b_state)]['truth']):
            reachable_q_b_closest.append(b_state)
    # if empty
    if not reachable_q_b_closest:
        return [], []

    # collects the buchi state in the reachable set of qb_rand with minimum distance to the final state
    b_min_from_q_b_closest = tree.get_min2final_from_subset(reachable_q_b_closest)

    # collects the buchi state in the reachable set of b_min with distance
    # to the final state equal to that of b_min - 1
    reachable_decr = dict()
    m_q_b_closest = []
    for b_state in b_min_from_q_b_closest:
        candidate = []
        for succ in tree.buchi.buchi_graph.succ[b_state]:
            if tree.buchi.min_length[(b_state, tree.b_final)] - 1 == tree.buchi.min_length[(succ, tree.b_final)] \
                    or succ in tree.buchi.buchi_graph.graph['accept']:
                candidate.append(succ)
        if candidate:
            reachable_decr[b_state] = candidate
            m_q_b_closest.append(b_state)
    # if empty
    if not m_q_b_closest:
        return [], []
    # sample q_b_min and b_decr
    q_b_min = sample_uniform_geometry(m_q_b_closest)
    q_b_decr = sample_uniform_geometry(reachable_decr[q_b_min])

    truth = tree.buchi.buchi_graph.edges[(q_b_min, q_b_decr)]['truth']
    x_rand = list(q_p_closest[0])
    return tree.buchi_guided_sample_by_truthvalue(truth, x_rand, q_p_closest,
                                                  tree.biased_tree.nodes[q_p_closest]['label'])


def buchi_guided_sample_by_truthvalue(tree, truth, x_rand, q_p_closest, x_label):
    """
    sample guided by truth value
    :param truth: the value making transition occur
    :param x_rand: random selected node
    :param x_label: label of x_rand
    :param regions: regions
    :return: new sampled point
    """
    if truth == '1':
        return [], []
    # not or be in some place
    else:
        for key in truth:
            pair = key.split('_')  # region-robot pair
            robot_index = int(pair[1]) - 1
            orig_x_rand = x_rand[robot_index]  # save for further recover
            while True:
                x_rand[robot_index] = orig_x_rand  # recover
                # move towards target position
                if key not in x_label:
                    if np.random.uniform(0, 1, 1) <= tree.y_rand:
                        target = tree.get_target(orig_x_rand, pair[0])
                        x_rand[robot_index] = tree.gaussian_guided_towards_target(orig_x_rand, target)
                    else:
                        x_rand_i = []
                        for i in range(tree.dim):
                            x_rand_i.append(uniform(0, tree.workspace[i]))
                        x_rand[robot_index] = tuple(x_rand_i)
                # collision avoidance
                if tree.collision_avoidance(x_rand, robot_index):
                    break

    return tree.mulp2single(x_rand), q_p_closest
