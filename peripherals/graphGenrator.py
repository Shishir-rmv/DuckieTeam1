import networkx as nx
import matplotlib.pyplot as plt
import pprint, json

# initialize graph
DG = nx.DiGraph()

nodes = [1,2,3,4,5,6,7,8,9,10,11,12]

# weights are rough estimates, change to introduce bias later
wEdges = [(1,12,2, ("fast", False)),
        (1, 4, 2, ("fast", False)), #to prefer going straight
        (2, 4, 1.5, ("fast", False)),
        (2, 8, 2, ("fast", False)),
        (3, 8, 1.5, ("fast", False)),
        (3, 12, 2, ("fast", False)),
        (4, 7, 4, ("fast", False)),
        (4, 11, 3, ("fast", False)),
        (5, 3, 2, ("fast", False)),
        (5, 7, 4, ("fast", False)),
        (6, 3, 1.5, ("fast", False)),
        (6, 11, 3.5, ("fast", False)),
        (7, 10, 8, ("fast", True)),
        (7, 1, 2, ("fast", False)),
        (8, 10, 8, ("fast", True)),
        (8, 6, 3, ("fast", False)),
        (9, 1, 1.5, ("fast", False)),
        (9, 6, 3.5, ("fast", False)),
        (10, 2, 2, ("fast", False)),
        (10, 5, 4, ("fast", False)),
        (11, 2, 1.5, ("fast", False)),
        (11, 9, 7, ("fast", True)),
        (12, 9, 6.5, ("fast", True)),
        (12, 5, 4, ("fast", False))]



# construct graph
DG.add_nodes_from(nodes)
DG.add_weighted_edges_from(wEdges)

# justEdges = []
# for edge in wEdges:
#     justEdges.append((edge[0], edge[1]))
# labels = nx.get_edge_attributes(DG, 'all')
# nx.draw_networkx_edge_labels(DG, justEdges)

# convert the graph to an exportable format
dic = nx.node_link_data(DG)

# dump and read
with open('graph.json', 'w') as fp:
    json.dump(dic, fp, indent=4)
with open("graph.json", 'r') as f:
    read = json.load(f)

# re-convert to graph
RG = nx.node_link_graph(read, directed=True, multigraph=False, attrs=None)

pprint.pprint(read)
print(type(read))


# draw graph
nx.draw_shell (RG, with_labels=True, font_weight='bold')
# plt.show()