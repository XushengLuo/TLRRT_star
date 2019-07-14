from workspace import Workspace, get_label
from random import uniform


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
        self.number_of_robots = 4
        # randomly generate initial locations of robots with empty atomic propositions
        self.init = []
        for i in range(self.number_of_robots):
            while True:
                ini = [round(uniform(0, workspace.workspace[k]), 3) for k in range(len(workspace.workspace))]
                ap = get_label(ini, workspace.regions, workspace.obs)
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

        # Task 1
        # task specification, e1-e5 are subfomulas
        # self.formula = '[]<> e1 && []<> e3 && !e1 U e2'
        #
        # self.subformula = {1: '(l1_1)',
        #                    2: '(l6_1)',
        #                    3: '(l5_2)'
        #                 }
        self.formula = '[]<> e1 && []<> e2 && []<> e3 && []<>(e4 && <>(e5 && <> e6))'
        self.subformula = {
            1: '(l1_1 && l1_2)',
            2: '(l2_2 && l2_3)',
            3: '(l3_3 && l3_4)',
            4: '(l4_1)',
            5: '(l5_4)',
            6: '(l6_3)'}

        # Task 2
        # self.number_of_robots = 40
        # # task specification (subfomulas such as e1 are defined by self.subformula)
        # self.formula = '[]<> e1 && []<> e2 && []<> e3 && []<>(e4 && <>(e5 && <> e6)) && []!e7'
        # # subformulas
        # self.subformula = {
        #     1: '(l1_1 && l1_2 && l1_4 && l1_6 && l4_21 && l5_24 && l3_30 && l2_33 && l6_36)',
        #     2: '(l2_6 && l2_7 && l2_9 && l2_11 && l5_22 && l6_25 && l4_31 && l3_34 && l1_37)',
        #     3: '(l3_11 && l3_12 && l3_14 && l3_16 && l6_23 && l4_26 && l5_32 && l2_35 && l1_38)',
        #     4: '(l4_1 && l4_27 && l4_30 && l4_33 && l4_37)',
        #     5: '(l5_6 && l5_28 && l5_31 && l5_34 && l5_38)',
        #     6: '(l6_11 && l6_29 && l6_32 && l6_35 && l6_36)',
        #     7: '(l3_4 || l3_9)',
        #     8: '(l4_3 && l5_8 && l6_13 && l1_21 && l2_22 && l3_23 && l3_39)',
        #     9: '(l2_17 && l2_18 && l4_19 && l4_20 && l1_24 && l2_25 && l3_26 && l1_40)'
        # }
        # # pairs of subformulas that cannot be enabled at the same time
        # self.exclusion = [('e1', 'e2'), ('e1', 'e4'), ('e1', 'e5'), ('e2', 'e3'), ('e2', 'e5'), ('e2', 'e6'),
        #                   ('e3', 'e6'), ('e1', 'e8'), ('e2', 'e8'), ('e3', 'e8'), ('e1', 'e9'), ('e2', 'e9'),
        #                   ('e3', 'e9')]