"""
__author__ = chrislaw
__project__ = RRT*_LTL
__date__ = 8/30/18
"""
"""
construct trees for biased sampling optimal task planning for multi-robots
"""

from random import uniform
from networkx.classes.digraph import DiGraph
from networkx.algorithms import dfs_labeled_edges
import math
import numpy as np
from collections import OrderedDict
import pyvisgraph as vg
from shapely.geometry import Point, Polygon, LineString
from uniform_geometry import sample_uniform_geometry


class biased_tree(object):
    """
    construction of prefix and suffix trees
    """
    def __init__(self, workspace, buchi, init_b, step_size, segment):
        """
        :param acpt:  accepting state
        :param ts: transition system
        :param buchi_graph:  Buchi graph
        :param init: product initial state
        """
        # parameters regarding workspace
        self.workspace = workspace.workspace
        self.dim = len(self.workspace)
        self.regions = workspace.regions
        self.obstacles = workspace.obs
        self.robot = workspace.number_of_robots
        # parameters regarding task
        self.buchi = buchi
        self.accept = self.buchi.buchi_graph.graph['accept']
        self.init = (workspace.init, init_b)
        
        # initlizing the tree
        self.biased_tree = DiGraph(type='PBA', init=self.init)
        self.biased_tree.add_node(self.init, cost=0, label=workspace.init_label)
        
        # parameters regarding TL-RRT* algorithm
        self.goals = []
        self.step_size = step_size
        self.segment = segment
        # size of the ball used in function near
        uni_v = np.power(np.pi, self.robot*self.dim/2) / math.gamma(self.robot*self.dim/2+1)
        self.gamma = np.ceil(4 * np.power(1/uni_v, 1./(self.dim*self.robot)))   # unit workspace
        # parameters regarding biased sampling
        # group the nodes in the tree by the buchi state
        self.group = dict()
        self.add_group(self.init)
        # select final buchi states
        if self.segment == 'prefix':
            self.b_final = self.buchi.buchi_graph.graph['accept'][0]
        else:
            self.b_final = self.buchi.buchi_graph.graph['accept']
        self.min_dis = np.inf
        self.b_min = set()
        self.q_min2final = []
        self.not_q_min2final = []

        # probability
        self.p_closest = 0.9
        # target point
        self.y_rand = 0.99
        # threshold for collision avoidance
        self.threshold = 0.005
        # polygon obstacle
        polys = []
        for poly in self.obstacles.values():
            polys.append([vg.Point(x[0], x[1]) for x in list(poly.exterior.coords)[:-1]])
        self.g = vg.VisGraph()
        self.g.build(polys, status=False)

        # biased sampling
        # self.acp = np.random.randint(0, len(buchi_graph.graph['accept']))

    def sample(self):
        """
        sample point from the workspace
        :return: sampled point, tuple
        """
        # sample random nodes from two sets
        p_rand = np.random.uniform(0, 1, 1)
        if (p_rand <= self.p_closest and len(self.q_min2final) > 0) or not self.not_q_min2final:
            q_p_closest = sample_uniform_geometry(self.q_min2final)
        elif p_rand > self.p_closest or not self.q_min2final:
            q_p_closest = sample_uniform_geometry(self.not_q_min2final)

        # find feasible successors of buchi state in q_rand
        reachable_q_b_closest = []
        for b_state in self.buchi.buchi_graph.succ[q_p_closest[1]]:
            # if self.t_satisfy_b(x_label, buchi_graph.edges[(q_rand[1], b_state)]['label']):
            if self.check_transition_b_helper(self.biased_tree.nodes[q_p_closest]['label'],
                                              self.buchi.buchi_graph.edges[(q_p_closest[1], b_state)]['truth']):
                reachable_q_b_closest.append(b_state)
        # if empty
        if not reachable_q_b_closest:
            return [], []

        # collects the buchi state in the reachable set of qb_rand with minimum distance to the final state
        b_min_from_q_b_closest = self.get_min2final_from_subset(reachable_q_b_closest)

        # collects the buchi state in the reachable set of b_min with distance
        # to the final state equal to that of b_min - 1
        reachable_decr = dict()
        m_q_b_closest = []
        for b_state in b_min_from_q_b_closest:
            candidate = []
            for succ in self.buchi.buchi_graph.succ[b_state]:
                if self.buchi.min_length[(b_state, self.b_final)] - 1 == self.buchi.min_length[(succ, self.b_final)] \
                        or succ in self.buchi.buchi_graph.graph['accept']:
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

        truth = self.buchi.buchi_graph.edges[(q_b_min, q_b_decr)]['truth']
        x_rand = list(q_p_closest[0])
        return self.buchi_guided_sample_by_truthvalue(truth, x_rand, q_p_closest,
                                                      self.biased_tree.nodes[q_p_closest]['label'])

    def buchi_guided_sample_by_truthvalue(self, truth, x_rand, q_p_closest, x_label):
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
                        if np.random.uniform(0, 1, 1) <= self.y_rand:
                            target = self.get_target(orig_x_rand, pair[0])
                            x_rand[robot_index] = self.gaussian_guided_towards_target(orig_x_rand, target)
                        else:
                            x_rand_i = []
                            for i in range(self.dim):
                                x_rand_i.append(uniform(0, self.workspace[i]))
                            x_rand[robot_index] = tuple(x_rand_i)

                    # sampled point lies within obstacles
                    ap = self.get_label(x_rand[robot_index])
                    if 'o' in ap:
                        continue
                    # collision avoidance
                    if self.collision_avoidance(x_rand, robot_index):
                        break
        return x_rand, q_p_closest

    def add_group(self, q_p):
        """
        group nodes with same buchi state
        """
        try:
            self.group[q_p[1]].append(q_p)
        except KeyError:
            self.group[q_p[1]] = [q_p]

    def get_min2final_from_subset(self, subset):
        """
         collects the buchi state in the tree with minimum distance to the final state
        :param min_qb_dict: dict
        :param b_final: feasible final state
        :return: list of buchi states in the tree with minimum distance to the final state
        """
        l_min = np.inf
        b_min = []
        for b_state in subset:
            if self.buchi.min_length[(b_state, self.b_final)] < l_min:
                l_min = self.buchi.min_length[(b_state, self.b_final)]
                b_min = [b_state]
            elif self.buchi.min_length[(b_state, self.b_final)] == l_min:
                b_min.append(b_state)
        return b_min

    def update_min_dis2final_and_partition(self, q_p_new):
        """
         update the buchi state in the tree with minimum distance to the final state
        :param q_p_new:
        :return: list of buchi states in the tree with minimum distance to the final state
        """
        # smaller than the current nodes with minimum distance
        if self.buchi.min_length[(q_p_new[1], self.b_final)] < self.min_dis:
            self.min_dis = self.buchi.min_length[(q_p_new[1], self.b_final)]
            self.b_min = set(q_p_new[1])
            self.not_q_min2final = self.not_q_min2final + self.q_min2final
            self.q_min2final = [q_p_new]
        # equivalent to 
        elif self.buchi.min_length[(q_p_new[1], self.b_final)] == self.min_dis:
            self.b_min.add(q_p_new[1])
            self.q_min2final = self.q_min2final + [q_p_new]
        # larger than
        else:
            self.not_q_min2final = self.not_q_min2final + [q_p_new]

    def get_target(self, init, target):
        """
        find the closest vertex in the short path from init to target
        :param init: inital point
        :param target: target labeled region
        :param regions: regions
        :return: closest vertex
        """
        tg = self.regions[target].centroid.coords[0]
        shortest = self.g.shortest_path(vg.Point(init[0], init[1]), vg.Point(tg[0], tg[1]))
        return shortest[1].x, shortest[1].y

    def gaussian_guided_towards_target(self, x, target):
        """
        calculate new point following gaussian dist guided by the target
        :param x: mean point
        :param target: target point
        :return: new point
        """
        # d = self.get_truncated_normal(0, 1/3, 0, np.inf)
        # d = d.rvs() + np.linalg.norm(np.subtract(x, target))
        d = np.random.normal(np.linalg.norm(np.subtract(x, target)), 1/3/3)
        angle = np.random.normal(0, np.pi/12/3/3, 1) + np.arctan2(target[1] - x[1], target[0] - x[0])
        x_rand = np.add(x, (d*np.cos(angle), d*np.sin(angle)))
        x_rand = [self.trunc(x_rand_i) for x_rand_i in x_rand]
        return tuple(x_rand)

    def collision_avoidance(self, x, robot_index):
        """
        check whether any robots are collision-free from index-th robot
        :param x: all robots
        :param index: index-th robot
        :return: true collision free
        """
        for i in range(self.robot):
            if i != robot_index and np.fabs(x[i][0]-x[robot_index][0]) <= self.threshold \
                    and np.fabs(x[i][1]-x[robot_index][1]) <= self.threshold:
                return False
        return True

    def nearest(self, x_rand):
        """
        find the nearest class of vertices in the tree
        :param: x_rand randomly sampled point form: single point ()
        :return: nearest class of vertices form: single point ()
        """
        min_dis = math.inf
        q_nearest = []
        for vertex in self.biased_tree.nodes:
            x_vertex = self.mulp2sglp(vertex[0])
            dis = np.linalg.norm(np.subtract(x_rand, x_vertex))
            if dis < min_dis:
                q_nearest = list()
                q_nearest.append(vertex)
                min_dis = dis
            elif dis == min_dis:
                q_nearest.append(vertex)
        return q_nearest

    def steer(self, x_rand, x_nearest):
        """
        steer
        :param: x_rand randomly sampled point form: single point ()
        :param: x_nearest nearest point in the tree form: single point ()
        :return: new point single point ()
        """
        if np.linalg.norm(np.subtract(x_rand, x_nearest)) <= self.step_size:
            return x_rand
        else:
            return tuple(np.asarray(x_nearest) + self.step_size * (np.subtract(x_rand, x_nearest))/np.linalg.norm(np.subtract(x_rand, x_nearest)))

    def extend(self, q_new, near_nodes, label, obs_check):
        """
        :param: q_new: new state form: tuple (mulp, buchi)
        :param: near_v: near state form: tuple (mulp, buchi)
        :param: obs_check: check obstacle free  form: dict { (mulp, mulp): True }
        :return: extending the tree
        """
        added = 0
        cost = np.inf
        q_min = ()
        # loop over all nodes in near_noodes
        for node in near_nodes:
            if q_new != node and obs_check[(q_new[0], node[0])] and \
                    self.check_transition_b(node[1], self.biased_tree.nodes[node]['label'], q_new[1]):
                c = self.biased_tree.nodes[node]['cost'] \
                    + np.linalg.norm(np.subtract(self.mulp2single(q_new[0]), self.mulp2single(node[0])))
                if c < cost:
                    added = 1
                    q_min = node
                    cost = c
        if added == 1:
            self.biased_tree.add_node(q_new, cost=cost, label=label)
            self.biased_tree.add_edge(q_min, q_new)
            self.add_group(q_new)
            self.update_min_dis2final_and_partition(q_new)
            if self.segment == 'prefix' and q_new[1] in self.accept:
                q_n = list(list(self.biased_tree.pred[q_new].keys())[0])
                cost = self.biased_tree.nodes[tuple(q_n)]['cost']
                label = self.biased_tree.nodes[tuple(q_n)]['label']
                q_n[1] = q_new[1]
                q_n = tuple(q_n)
                self.biased_tree.add_node(q_n, cost=cost, label=label)
                self.biased_tree.add_edge(q_min, q_n)
                self.add_group(q_n)
                self.update_min_dis2final_and_partition(q_n)
                self.goals.append(q_n)
            if self.segment == 'suf' and self.obstacle_check([self.init], q_new[0], label)[(q_new[0], self.init[0])] \
                    and self.check_transition_b(q_new[1], label, self.init[1]):
                self.goals.append(q_new)
            # same buchi state but untransitionable x
            if self.segment == 'suf' and self.init[1] == q_new[1]:
                self.goals.append(q_new)
                return False
        return True

    def rewire(self, q_new, near_v, obs_check):
        """
        :param: q_new: new state form: tuple (mul, buchi)
        :param: near_v: near state form: tuple (mul, buchi)
        :param: obs_check: check obstacle free form: dict { (mulp, mulp): True }
        :return: rewiring the tree
        """
        for node in near_v:
            if obs_check[(q_new[0], node[0])] and self.checkTranB(q_new[1], self.biased_tree.nodes[q_new]['label'], node[1]):
                c = self.biased_tree.nodes[q_new]['cost'] + np.linalg.norm(np.subtract(self.mulp2sglp(q_new[0]), self.mulp2sglp(node[0])))      # without considering control
                delta_c = self.biased_tree.nodes[node]['cost'] - c
                # update the cost of node in the subtree rooted at node
                if delta_c > 0:
                    # self.biased_tree.nodes[node]['cost'] = c
                    if not list(self.biased_tree.pred[node].keys()):
                        print('empty')
                    self.biased_tree.remove_edge(list(self.biased_tree.pred[node].keys())[0], node)
                    self.biased_tree.add_edge(q_new, node)
                    edges = dfs_labeled_edges(self.biased_tree, source=node)
                    for _, v, d in edges:
                        if d == 'forward':
                            self.biased_tree.nodes[v]['cost'] = self.biased_tree.nodes[v]['cost'] - delta_c

    def near(self, x_new):
        """
        find the states in the near ball
        :param x_new: new point form: single point
        :return: p_near: near state, form: tuple (mulp, buchi)
        """
        near_nodes = []
        radius = min(self.gamma * np.power(np.log(self.biased_tree.number_of_nodes()+1)/self.biased_tree.number_of_nodes(),
                    1./(self.dim*self.robot)), self.step_size)
        for node in self.biased_tree.nodes:
            if np.linalg.norm(np.subtract(x_new, self.mulp2single(node[0]))) <= radius:
                near_nodes.append(node)
        return near_nodes

    def obstacle_check(self, near_node, x_new, label):
        """
        check whether obstacle free along the line from x_near to x_new
        :param q_near: states in the near ball, tuple (mulp, buchi)
        :param x_new: new state form: multiple point
        :param label: label of x_new
        :param stage: regular stage or final stage, deciding whether it's goal state
        :return: dict (x_near, x_new): true (obs_free)
        """

        obs_check = {}
        checked = set()

        for node in near_node:
            # check whether the position component of node is checked
            if node[0] in checked:
                continue
            checked.add(node[0])
            obs_check[(x_new, node[0])] = True
            flag = True       # indicate whether break and jump to outer loop
            for r in range(self.robot):
                # the line connecting two points crosses an obstacle
                for (obs, boundary) in iter(self.obstacles.items()):
                    if LineString([Point(node[0][r]), Point(x_new[r])]).intersects(boundary):
                        obs_check[(x_new, node[0])] = False
                        flag = False
                        break
                # no need to check further
                if not flag:
                    break

                for (region, boundary) in iter(self.regions.items()):
                    if LineString([Point(node[0][r]), Point(x_new[r])]).intersects(boundary) \
                        and region + '_' + str(r + 1) != label[r] \
                            and region + '_' + str(r + 1) != self.biased_tree.nodes[node]['label'][r]:
                        obs_check[(x_new, node[0])] = False
                        flag = False
                        break
                # no need to check further
                if not flag:
                    break

        return obs_check

    def get_label(self, x):
        """
        generating the label of position state
        :param x: position
        :return: label
        """
        point = Point(x)
        # whether x lies within obstacle
        for (obs, boundary) in iter(self.ts['obs'].items()):
            if point.within(boundary):
                return obs

        # whether x lies within regions
        for (region, boundary) in iter(self.ts['region'].items()):
            if point.within(boundary):
                return region
        # x lies within unlabeled region
        return ''

    def check_transition_b(self, q_b, x_label, q_b_new):
        """ decide valid transition, whether b_state --L(x)---> q_b_new
             Algorithm2 in Chapter 2 Motion and Task Planning
             :param q_b: buchi state
             :param x_label: label of x
             :param q_b_new buchi state
             :return True satisfied
        """
        b_state_succ = self.buchi.buchi_graph.succ[q_b]
        # q_b_new is not the successor of b_state
        if q_b_new not in b_state_succ:
            return False
        # check whether label of x enables the transition
        truth = self.buchi.buchi_graph.edges[(q_b, q_b_new)]['truth']
        if self.check_transition_b_helper(x_label, truth):
            return True

        return False

    def check_transition_b_helper(self, x_label, truth):
        """
        check whether transition enabled under current label
        :param x_label: current label
        :param truth: truth value making transition enabled
        :return: true or false
        """
        if truth == '1':
            return True
        # all true propositions should be satisdied
        true_label = [true_label for true_label in truth.keys() if truth[true_label]]
        for label in true_label:
            if label not in x_label:
                return False
        #  all fasle propositions should not be satisfied
        false_label = [false_label for false_label in truth.keys() if not truth[false_label]]
        for label in false_label:
            if label in x_label:
                return False

        return True

    def findpath(self, goals):
        """
        find the path backwards
        :param goal: goal state
        :return: dict path : cost
        """
        paths = OrderedDict()
        for i in range(len(goals)):
            goal = goals[i]
            path = [goal]
            s = goal
            while s != self.init:
                s = list(self.biased_tree.pred[s].keys())[0]
                if s == path[0]:
                    print("loop")
                path.insert(0, s)

            if self.segment == 'pre':
                paths[i] = [self.biased_tree.nodes[goal]['cost'], path]
            elif self.segment == 'suf':
                # path.append(self.init)
                paths[i] = [self.biased_tree.nodes[goal]['cost'] + np.linalg.norm(np.subtract(goal[0], self.init[0])), path]
        return paths

    def mulp2single(self, point):
        """
        convert multiple form point ((),(),(),...) to single form point ()
        :param point: multiple points ((),(),(),...)
        :return: signle point ()
        """
        sp = []
        for p in point:
            sp = sp + list(p)
        return tuple(sp)

    def single2mulp(self, point):
        """
        convert single form point () to multiple form point ((), (), (), ...)
        :param point: single form point ()
        :return:  multiple form point ((), (), (), ...)
        """
        mp = []
        for i in range(self.robot):
            mp.append(point[i*self.dim:(i+1)*self.dim])
        return tuple(mp)
