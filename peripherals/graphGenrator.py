import networkx as nx
import matplotlib.pyplot as plt
import pprint, json

def planner(path, DG):
    for spot in range(len(path)):
        if (spot + 1 != len(path)):
            print("%i to %i:  \t %s" % (path[spot], path[spot+1], nx.dijkstra_path(DG, path[spot], path[spot+1])))

# initialize graph
DG = nx.DiGraph()

nodes = [1,2,3,4,5,6,7,8,9,10,11,12]

# weights are rough estimates, change to introduce bias later
edges = {"1,12," : {"weight": 2, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "SS"}},
        "1,4," : {"weight": 2, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "LS"}}, #to prefer going straight
        "2,4," : {"weight": 1.5, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "RS"}},
        "2,8," : {"weight": 2, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "SS"}},
        "3,8," : {"weight": 1.5, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "RS"}},
        "3,12" : {"weight": 2, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "LS"}},
        "4,7," : {"weight": 4, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "LSLS"}},
        "4,11" : {"weight": 3, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "RSRS"}},
        "5,3," : {"weight": 2, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "LS"}},
        "5,7," : {"weight": 4, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "SLS"}},
        "6,3," : {"weight": 1.5, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "RS"}},
        "6,11" : {"weight": 3.5, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "SRS"}},
        "7,10" : {"weight": 8, "attrs" : {"fast", True, "map":{"stopAt": ["SL"], "actions": "SLFLS"}},
        "7,1," : {"weight": 2, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "LS"}},
        "8,10" : {"weight": 8, "attrs" : {"fast", True, "map":{"stopAt": ["SL"], "actions": "LSLFLS"}},
        "8,6," : {"weight": 3, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "RSRS"}},
        "9,1," : {"weight": 1.5, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "RS"}},
        "9,6," : {"weight": 3.5, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "SRS"}},
        "10,2" : {"weight": 2, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "LS"}},
        "10,5" : {"weight": 4, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "SLS"}},
        "11,2" : {"weight": 1.5, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "RS"}},
        "11,9" : {"weight": 7, "attrs" : {"fast", True, "map":{"stopAt": ["SL"], "actions": "SRFRS"}},
        "12,9" : {"weight": 6.5, "attrs" : {"fast", True, "map":{"stopAt": ["SL"], "actions": "RSRFRS"}},
        "12,5" : {"weight": 4, "attrs" : {"fast", False, "map":{"stopAt": ["SL"], "actions": "LSLS"}}}

wEdges = []
for edge in edges:
    wEdges.append((edge[0], edge[1], edge[2]))


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

path = [1,5,7,2,9,3,12,6,8,10,1]
print("path given:" + str(path))
print("planned path: ")
planner(path, DG)

# draw graph
nx.draw_shell (RG, with_labels=True, font_weight='bold')
# plt.show()