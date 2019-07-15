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
        sz = 8
        self.number_of_robots = sz * 8
        # randomly generate initial locations of robots with empty atomic propositions
        self.init = []
        for i in range(self.number_of_robots):
            while True:
                ini = [round(uniform(0, workspace.workspace[k]), 3) for k in range(len(workspace.workspace))]
                ap = get_label(ini, workspace)
                if 'l' not in ap and 'o' not in ap:
                    break
            self.init.append(tuple(ini))
        self.init = tuple(self.init)
        self.init_label = [''] * self.number_of_robots

        # Task 1
        # task specification, e1-e5 are subfomulas
        # self.formula = '<> e1 && []<> (e2 && <> e3) && (!e3 U e4) && []!e5'
        #
        # self.subformula = {1: '(l1_1)',
        #                    2: '(l2_1)',
        #                    3: '(l3_1)',
        #                    4: '(l4_1)',
        #                    5: '(l5_1)',
        #                    }

        # Task 2
        # task specification, e1-e5 are subfomulas
        # self.formula = '[]<> e1 && []<> e3 && !e1 U e2'
        #
        # self.subformula = {1: '(l1_1)',
        #                    2: '(l6_1)',
        #                    3: '(l5_2)'
        #                 }
        # Task 3
        # self.formula = '[]<> e1 && []<> e2 && []<> e3 && []<>(e4 && <>(e5 && <> e6))'
        # self.subformula = {
        #     1: '(l1_1 && l1_2)',
        #     2: '(l2_2 && l2_3)',
        #     3: '(l3_3 && l3_4)',
        #     4: '(l4_1)',
        #     5: '(l5_4)',
        #     6: '(l6_3)'}

        # group = np.array(range(1, self.number_of_robots + 1))
        # np.random.shuffle(group)
        # group = group.reshape(8, sz)
        #
        # formula = []
        # for i in range(8):
        #     subformula = []
        #     for j in range(sz):
        #         subformula.append('l' + str(np.random.randint(1, 7)) + '_' + str(group[i][j]))
        #     for j in range(sz//2):
        #         while True:
        #             g = np.random.randint(7)
        #             robot = group[g][np.random.randint(sz)]
        #             if robot not in group[i]:
        #                 break
        #         subformula.append('l' + str(np.random.randint(1, 7)) + '_' + str(robot))
        #
        #     formula.append('(' + ' && '.join(subformula) + ')')
        #
        # self.subformula = {i: formula[i-1] for i in range(1, 9)}
        # print(self.subformula)
        self.formula = '[]<> e1 && []<> e2 && []<> e3 && []<>(e4 && <>(e5 && <> e6)) && <> e7 && []<>e8 && (!e7 U e8)'
        self.subformula = {1: '(l1_51 && l4_55 && l2_33 && l4_11 && l3_34 && l4_38 && l2_26 && l4_31 && l5_8 && l2_39)',
                2: '(l4_1 && l5_44 && l6_41 && l4_40 && l5_36 && l1_9 && l6_39 && l1_56 && l3_49 && l5_6)',
                3: '(l3_15 && l5_6 && l5_24 && l3_52 && l1_2 && l2_32 && l3_56 && l1_23 && l3_41 && l4_14)',
                4: '(l6_16 && l3_27 && l6_54 && l6_25 && l3_8 && l5_49 && l1_35 && l2_24 && l2_39 && l1_51)',
                5: '(l5_31 && l6_37 && l2_10 && l4_28 && l4_50 && l2_23 && l2_21 && l2_46 && l1_13 && l3_52)',
                6: '(l5_48 && l5_17 && l3_18 && l4_30 && l1_42 && l1_7 && l4_20 && l2_13 && l2_46 && l1_15)',
                7: '(l5_29 && l3_19 && l2_3 && l2_46 && l2_12 && l4_14 && l5_13 && l5_42 && l5_31 && l3_17)',
                8: '(l6_4 && l2_53 && l3_47 && l1_45 && l1_43 && l5_5 && l4_22 && l3_52 && l5_48 && l5_36)'}
