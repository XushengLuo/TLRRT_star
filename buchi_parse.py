# -*- coding: utf-8 -*-

import subprocess
import os.path
import re
import networkx as nx
import numpy as np
from networkx.classes.digraph import DiGraph
from sympy import satisfiable
from itertools import combinations


class Buchi(object):
    """ construct buchi automaton graph
    """

    def __init__(self, task):
        # task specified in LTL
        self.formula = task.formula
        self.subformula = task.subformula
        self.number_of_robots = task.number_of_robots
        # graph of buchi automaton
        """
        Format:
            buchi_graph.node = NodeView(('T0_init', 'T1_S1', 'accept_S1'))
            buchi_graph.edges = OutEdgeView([('T0_init', 'T0_init'), ('T0_init', 'T1_S1'),....])
            buchi_graph.succ = AdjacencyView({'T0_init': {'T0_init': {'label': '1'}, 'T1_S1': {'label': 'r3'}}})
        """
        self.buchi_graph = DiGraph(type='buchi', init=[], accept=[])

        # minimal length (in terms of number of transitions) between a pair of nodes
        self.min_length = dict()
                
    def construct_buchi_graph(self):
        """parse the output of the program ltl2ba and build the buchi automaton
        """
        # directory of the ltl2ba
        dirname = os.path.dirname(__file__)
        # output string of program ltl2ba
        output = subprocess.check_output(dirname + "/./ltl2ba -f \"" + self.formula + "\"", shell=True).decode(
            "utf-8")
        
        # find all states/nodes in the buchi automaton
        state_re = re.compile(r'\n(\w+):\n\t')
        state_group = re.findall(state_re, output)

        # find initial and accepting states
        init = [s for s in state_group if 'init' in s]
        accept = [s for s in state_group if 'accept' in s]
        # finish the inilization of the graph of the buchi automaton
        self.buchi_graph.graph['init'] = init
        self.buchi_graph.graph['accept'] = accept
        
        order_key = list(self.subformula.keys())
        order_key.sort(reverse=True)
        # for each state/node, find it transition relations
        for state in state_group:
            # add node
            self.buchi_graph.add_node(state)
            # loop over all transitions starting from current state
            state_if_fi = re.findall(state + r':\n\tif(.*?)fi', output, re.DOTALL)
            if state_if_fi:
                relation_group = re.findall(r':: (\(.*?\)) -> goto (\w+)\n\t', state_if_fi[0])
                for symbol, next_state in relation_group:
                    # whether the edge is feasible in terms of atomic propositions
                    for k in order_key:
                        symbol = symbol.replace('e{0}'.format(k), self.subformula[k])
                    # get the trurh assignment
                    truth_table = self.get_truth_assignment(symbol)
                    # infeasible transition
                    if not truth_table:
                        continue
                    # add edge
                    self.buchi_graph.add_edge(state, next_state, truth=truth_table)
            else:
                state_skip = re.findall(state + r':\n\tskip\n', output, re.DOTALL)
                if state_skip:
                    self.buchi_graph.add_edge(state, state, truth='1')

    def get_truth_assignment(self, symbol):
        """
            get one set of truth assignment that makes the symbol true
        """
        # non-empty symbol
        if symbol != '(1)':
            exp = symbol.replace('||', '|').replace('&&', '&').replace('!', '~')
            # add extra constraints: a single robot can reside in at most one region
            robot_region = self.robot2region(exp)
            for robot, region in robot_region.items():
                mutual_exclution = list(combinations(region, 2))
                # single label in the symbol
                if not mutual_exclution:
                    continue
                for i in range(len(mutual_exclution)):
                    mutual_exclution[i] = '(~(' + ' & '.join(list(mutual_exclution[i])) + '))'
                exp = exp + '&' + ' & '.join(mutual_exclution)
            # find one truth assignment that makes symbol true
            truth = satisfiable(exp, algorithm="dpll")
            try:
                truth_table = dict()
                for key, value in truth.items():
                    truth_table[key.name] = value
            except AttributeError:
                return False
            else:
                return truth_table
        # empty symbol
        else:
            return '1'
    
    def get_minimal_length(self):
        """
        search the shorest path from a node to another, weight = 1, i.e. # of state in the path
        """

        # loop over pairs of buchi states
        for node1 in self.buchi_graph.nodes():
            for node2 in self.buchi_graph.nodes():
                # node1 = node2, and node2 is an accepting state
                if node1 != node2 and 'accept' in node2:
                    try:
                        length, _ = nx.algorithms.single_source_dijkstra(self.buchi_graph, source=node1, target=node2)
                    except nx.exception.NetworkXNoPath:
                        length = np.inf
                    self.min_length[(node1, node2)] = length
                # node1 != node2 and node 2 is an accepting state
                # move 1 step forward to all reachable states of node1 then calculate the minimal length
                elif node1 == node2 and 'accept' in node2:
                    length = np.inf
                    for suc in self.buchi_graph.succ[node1]:
                        try:
                            len0, _ = nx.algorithms.single_source_dijkstra(self.buchi_graph, source=suc, target=node1)
                        except nx.exception.NetworkXNoPath:
                            len0 = np.inf
                        if len0 < length:
                            length = len0 + 1
                    self.min_length[(node1, node2)] = length

    def get_feasible_accepting_state(self):
        """
        delete infeasible accepting/final state
        """
        accept = self.buchi_graph.graph['accept']
        self.buchi_graph.graph['accept'] = []
        for ac in accept:
            for init in self.buchi_graph.graph['init']:
                if self.min_length[(init, ac)] < np.inf and self.min_length[(ac, ac)] < np.inf:
                    self.buchi_graph.graph['accept'].append(ac)
                    break

    def robot2region(self, symbol):
        """
        pair of robot and corresponding regions in the expression
        :param symbol: logical expression
        :param robot: # of robots
        :return: dic of robot index : regions
        exp = 'l1_1 & l3_1 & l4_1 & l4_6 | l3_4 & l5_6'
        {1: ['l1_1', 'l3_1', 'l4_1'], 4: ['l3_4'], 6: ['l4_6', 'l5_6']}
        """

        robot_region = dict()
        for r in range(self.number_of_robots):
            findall = re.findall(r'(l\d+?_{0})[^0-9]'.format(r + 1), symbol)
            if findall:
                robot_region[str(r + 1)] = findall

        return robot_region
