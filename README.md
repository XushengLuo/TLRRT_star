# TLRRT*

An Abstraction-Free Method for Multi-Robot Temporal Logic Optimal Control Synthesis

Tasks specified by Linear Temporal Logic can capture more complex missions compared to traditional point-to-point navigation.
The majority of existing Linear Temporal Logic (LTL) planning methods rely on the construction of a discrete product 
automaton, that combines a discrete abstraction of robot mobility and a Bu ̈chi automaton that captures the LTL specification
We propose a new sampling-based LTL planning algorithm that does not require any discrete abstraction of robot mobility.
Instead, it builds incrementally trees that explore the product state-space, until a maximum number of iterations is
reached or a feasible plan is found. The use of trees makes data storing and manipulation tractable, which significantly
increases the scalability of our algorithm. To accelerate the construction of feasible plans, we introduce bias in the
sampling process which is guided by transitions in the Buchi automaton that belong to the shortest path to the accepting
states.

# Usage

First, specify the LTL task in the file [task.py](/task.py), which mainly involves the assigned task, the number of robots
, the initial locations of robots and the minimum distance between any pair of robots, and workspace in the file [workspace.py](/workspace.py) that contains the information 
about the size of the workspace, the layout of regions and obstacles. Second, set the parameters used in the TLRRT* in the
file [construct_biased_tree.py](/construct_biased_tree.py), such as the maximum number of iterations, the step size, whether the lite version that does not use function `near`, `extend` and `rewire` in 
[biased_tree.py](/biased_tree.py) is used. Finally, after the TLRRT* terminates, the runtime and the cost of the solution is 
presented. What's more, the path composed of prefix and suffix parts for each robot is drawn with workspace layout when the number of robots is relatively small, otherwise, the path for each
robot is printed onto the screen when the number of robots is large. 

# Example




