# -*- coding: utf-8 -*-

from workspace import Workspace, get_label
from random import uniform
import numpy as np


class Task(object):
    """
    define the task specified in LTL
    """
    def __init__(self):
        """
        +----------------------------+
        |   Propositonal Symbols:    |
        |       true, false         |
        |	    any lowercase string |
        |                            |
        |   Boolean operators:       |
        |       !   (negation)       |
        |       ->  (implication)    |
        |	    <-> (equivalence)    |
        |       &&  (and)            |
        |       ||  (or)             |
        |                            |
        |   Temporal operators:      |
        |       []  (always)         |
        |       <>  (eventually)     |
        |       U   (until)          |
        |       V   (release)        |
        |       X   (next)           |
        +----------------------------+
        """
        workspace = Workspace()

        # task specification, e1-e5 are subfomulas
        # Task 1 ----------------------------------------------------------
        # self.formula = '<> e1 && []<> (e2 && <> e3) && (!e3 U e4) && []!e5'
        # self.subformula = {1: '(l1_1)',
        #                    2: '(l2_1)',
        #                    3: '(l3_1)',
        #                    4: '(l4_1)',
        #                    5: '(l5_1)',
        #                    }

        # Task 2 ------------------------------------------------------------
        # task specification, e1-e5 are subfomulas
        self.formula = '[]<> e1 && []<> e3 && !e1 U e2'
        self.subformula = {1: '(l1_1)',
                           2: '(l6_1)',
                           3: '(l5_2)'
                        }

        # Task 3 -------------------------------------------------------------
        # self.formula = '[]<> e1 && []<> e2 && []<> e3 && []<>(e4 && <>(e5 && <> e6))'
        # self.subformula = {
        #     1: '(l1_1 && l1_2)',
        #     2: '(l2_2 && l2_3)',
        #     3: '(l3_3 && l3_4)',
        #     4: '(l4_1)',
        #     5: '(l5_4)',
        #     6: '(l6_3)'}

        # Task 4 -------------------------------------------------------------
        # randomly generate tasks
        # num_of_robot_in_one_group = 8
        # group = np.array(range(1, self.number_of_robots + 1))
        # np.random.shuffle(group)
        # group = group.reshape(8, num_of_robot_in_one_group)
        #
        # formula = []
        # for i in range(8):
        #     subformula = []
        #     for j in range(num_of_robot_in_one_group):
        #         subformula.append('l' + str(np.random.randint(1, 7)) + '_' + str(group[i][j]))
        #     for j in range(num_of_robot_in_one_group//2):
        #         while True:
        #             g = np.random.randint(7)
        #             robot = group[g][np.random.randint(num_of_robot_in_one_group)]
        #             if robot not in group[i]:
        #                 break
        #         subformula.append('l' + str(np.random.randint(1, 7)) + '_' + str(robot))
        #
        #     formula.append('(' + ' && '.join(subformula) + ')')
        #
        # self.subformula = {i: formula[i-1] for i in range(1, 9)}
        # print(self.subformula)
        # self.formula = '[]<> e1 && []<> e2 && []<> e3 && []<>(e4 && <>(e5 && <> e6)) && <> e7 && []<>e8 && (!e7 U e8)'

        self.number_of_robots = 2   # num_of_robot_in_one_group * 8 for task 4
        # randomly generate initial locations of robots with empty atomic propositions
        self.init = []
        self.init_label = []
        for i in range(self.number_of_robots):
            while True:
                ini = [round(uniform(0, workspace.workspace[k]), 3) for k in range(len(workspace.workspace))]
                ap = get_label(ini, workspace)
                if 'o' not in ap:
                    break
            self.init.append(tuple(ini))
            # label
            ap = ap + '_' + str(i + 1) if 'l' in ap else ''
            self.init_label.append(ap)
        self.init = tuple(self.init)
