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

# Requirements
* [Python >=3.6](https://www.python.org/downloads/)
* [sympy](https://www.sympy.org/en/index.html)
* [re]()
* [Pyvisgraph](https://github.com/TaipanRex/pyvisgraph)
* [NetworkX](https://networkx.github.io)
* [Shapely](https://github.com/Toblerity/Shapely)
* [scipy](https://www.scipy.org)
* [matplotlib](https://matplotlib.org)
* [termcolor](https://pypi.org/project/termcolor/)

# Usage

* First, specify the LTL task in the file [task.py](/task.py), which mainly involves the assigned task, the number of robots
, the initial locations of robots and the minimum distance between any pair of robots, and workspace in the file [workspace.py](/workspace.py) that contains the information 
about the size of the workspace, the layout of regions and obstacles. 
* Second, set the parameters used in the TLRRT* in the
file [construct_biased_tree.py](/construct_biased_tree.py), such as the maximum number of iterations, the step size, whether the lite version that does not use function `near`, `extend` and `rewire` in 
[biased_tree.py](/biased_tree.py) is used. 
* Finally, after the TLRRT* terminates, the runtime and the cost of the solution is 
presented. What's more, the path composed of prefix and suffix parts for each robot is drawn with workspace layout when the number of robots is relatively small, otherwise, the path for each
robot is printed onto the screen when the number of robots is large. 

# Example

## Workspace
The workspace of size `1-by-1` is shown below, with `l_1`-`l_6` being regions and `o_1`-`o_2` being obstacles
<p align="center">
<img src="workspace.png"  width="750" height="500">
</p>

## Test Case
For all the following test cases, the same set of parameters are used.
```python
# parameters
# maximum number of iterations
n_max = 10000
para = dict()
# lite version, excluding extending and rewiring
para['is_lite'] = True
# step_size used in function near
para['step_size'] = 0.25 * buchi.number_of_robots
# probability used when choosing node q_p_closest
para['p_closest'] = 0.9
# probability used when deciding the target point 
para['y_rand'] = 0.99
# minimum distance between any pair of robots  
para['threshold'] = 0.005
```
Furthermore, the construction of the tree terminates once an accepting node is detected, which is controlled in [construct_biased_tree.py](construct_biased_tree.py) by line
```python
if len(tree.goals): break
```
### Case 1
The task is specified by 
```python
self.formula = '<> e1 && []<> (e2 && <> e3) && (!e3 U e4) && []!e5'
self.subformula = { 1: '(l1_1)',
                    2: '(l2_1)',
                    3: '(l3_1)',
                    4: '(l4_1)',
                    5: '(l5_1)',
                    }     
self.init = ((0.8, 0.1), )  # in the form of ((x,y), (x,y), ...)    
```
The output result during the process is 
```
Time for constructing the NBA: 0.1343 s
------------------------------ prefix path --------------------------------
Time for the prefix path: 0.0516 s
1 accepting goals found
-------------- suffix path for 1-th pre-goal (of 1 in total) --------------
1-th pre-goals: 1 accepting goals found
Time for the suffix path: 0.0561 s
------------------------ prefix + suffix path -----------------------------
t_pre  | t_suf  | t_total | cost
0.0516 | 0.0561 | 0.2421  | 1.7245
------------------------- print the path path -----------------------------
(. for empty label, || ... || for the suffix path)
robot 1 :  . -->  . -->  . -->  . -->  . -->  . -->  . --> l4 --> l1 -->  . -->  . --> l2 --> l2 --> || l2 --> l3 --> l3 -->  . --> l2 --> l2 --> l2 --> || 
```
## Case 2
```python
self.formula = '[]<> e1 && []<> e3 && !e1 U e2'
self.subformula = { 1: '(l1_1)',
                    2: '(l6_1)',
                    3: '(l5_2)'
                    }
self.init = ((0.8, 0.1), (0.8, 0.1))  # in the form of ((x,y), (x,y), ...)    
```
The output result during the process is 
```
Time for constructing the NBA: 0.0225 s
------------------------------ prefix path --------------------------------
Time for the prefix path: 0.1162 s
1 accepting goals found
-------------- suffix path for 1-th pre-goal (of 1 in total) --------------
1-th pre-goals: 1 accepting goals found
Time for the suffix path: 0.0273 s
------------------------ prefix + suffix path -----------------------------
t_pre  | t_suf  | t_total | cost
0.1162 | 0.0273 | 0.1660  | 0.8407
------------------------- print the path path -----------------------------
(. for empty label, || ... || for the suffix path)
robot 1 :  . -->  . -->  . --> l4 --> l6 -->  . -->  . -->  . --> l1 --> l1 --> || l1 --> || 
robot 2 :  . -->  . -->  . -->  . -->  . -->  . -->  . -->  . --> l5 --> l5 --> || l5 --> ||
```
## Case 3
```python
self.formula = '[]<> e1 && []<> e2 && []<> e3 && []<>(e4 && <>(e5 && <> e6)) && <> e7 && []<>e8 && (!e7 U e8)'
```
where each subformula is randomly generated, so is the initial location of each robot. It took less time when the following code in [buchi_parse.py](buchi_parse.py) is uncommented.
```python
if ' && ' in symbol: continue
```

