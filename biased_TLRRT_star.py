from task import Task
from buchi_parse import Buchi

task = Task()
buchi = Buchi(task)
buchi.construct_buchi_graph()
buchi.get_minimal_length()
buchi.get_feasible_accepting_state()
buchi_graph = buchi.buchi_graph
print(buchi_graph.number_of_nodes(), buchi_graph.number_of_edges())
for edge in buchi_graph.edges():
    print(edge, buchi_graph.edges[edge]['truth'])

print(buchi.min_length)