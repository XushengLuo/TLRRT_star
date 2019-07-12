from itertools import combinations
import re


def robot2region(symbol):
    """
    pair of robot and corresponding regions in the expression
    :param symbol: logical expression
    :param robot: # of robots
    :return: dic of robot index : regions
    exp = 'l1_1 & l3_1 & l4_1 & l4_6 | l3_4 & l5_6'
    {1: ['l1_1', 'l3_1', 'l4_1'], 4: ['l3_4'], 6: ['l4_6', 'l5_6']}
    """

    robot_region = dict()
    for r in range(1):
        findall = re.findall(r'(l\d+?_{0})[^0-9]'.format(r + 1), symbol)
        if findall:
            robot_region[str(r + 1)] = findall

    return robot_region


symbol = 'l1_1 & l3_1 & l4_1 '
exp = symbol.replace('||', '|').replace('&&', '&').replace('!', '~')
robot_region = robot2region(exp)

for robot, region in robot_region.items():
    mutual_exclution = list(combinations(region, 2))
    for i in range(len(mutual_exclution)):
        mutual_exclution[i] = '(~(' + ' & '.join(list(mutual_exclution[i])) + '))'
    exp = exp + '&' + ' & '.join(mutual_exclution)

print(exp)
