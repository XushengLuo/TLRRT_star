def construction_tree(tree, n_max):
    final = True
    # trivial suffix path
    if tree.segment == 'suffix' and tree.check_transition_b(tree.init[1], tree.biased_tree.nodes[tree.init]['label'], tree.init[1]):
        return {0 : [0, []]}, True

    for n in range(n_max):
        # biased sample
        x_new, q_rand = tree.sample()
        # couldn't find x_new
        if not x_new:
            continue

        x_new = tree.single2mulp(x_new)
        label = []
        flag = True
        for i in range(tree.robot):
            ap = tree.get_label(x_new[i])
            # exists one sampled point lies within obstacles
            if 'o' in ap:
                flag = False
                break
            if ap != '':
                ap = ap + '_' + str(i+1)
            label.append(ap)
        if not flag:
            continue
        # # near state
        # near_v = tree.near(tree.mulp2sglp(x_new))
        # # add q_rand
        # if q_rand not in near_v:
        #     near_v = near_v + [q_rand]
        near_v = [q_rand]
        # check obstacle free
        obs_check = tree.obs_check(near_v, x_new, label, 'reg')

        if not list(obs_check.items())[0][1]:
            continue
        # iterate over each buchi state
        for b_state in tree.buchi.buchi_graph.nodes:

            # new product state
            q_new = (x_new, b_state)

            # extend
            final = tree.extend(q_new, near_v, label, obs_check) and final
            # rewire
            # if added == 1:
                # tree.rewire(q_new, near_v, obs_check)

        # first accepting state
        if len(tree.goals):
            break

    cost_path = tree.findpath(tree.goals)
    return cost_path, final