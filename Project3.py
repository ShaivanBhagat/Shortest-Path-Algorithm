# CODE OBTAINED FROM GITHUB 
# CODE MODIFIED ONLY TO INCLUDE MIDDELBURG COORDINATES AND ACCORDING VARIABLES AND VALUES

import networkx as nx
import osmnx as ox
import time

# download/model a street network for some city then visualize it
G = ox.graph_from_place("Middelburg, Zeeland, Netherlands", network_type="walk")
fig, ax = ox.plot_graph(G)

# convert your MultiDiGraph to an undirected MultiGraph
M = ox.utils_graph.get_undirected(G)

# convert your MultiDiGraph to a DiGraph without parallel edges
D = ox.utils_graph.get_digraph(G)

# you can convert your graph to node and edge GeoPandas GeoDataFrames
gdf_nodes, gdf_edges = ox.graph_to_gdfs(G)
gdf_nodes.head()

# convert node/edge GeoPandas GeoDataFrames to a NetworkX MultiDiGraph
G2 = ox.graph_from_gdfs(gdf_nodes, gdf_edges, graph_attrs=G.graph)

# what sized area does our network cover in square meters?
G_proj = ox.project_graph(G)
nodes_proj = ox.graph_to_gdfs(G_proj, edges=False)
graph_area_m = nodes_proj.unary_union.convex_hull.area
graph_area_m

# show some basic stats about the network
ox.basic_stats(G_proj, area=graph_area_m, clean_int_tol=15)

# save graph to disk as geopackage (for GIS) or graphml file (for gephi etc)
ox.save_graph_geopackage(G, filepath="./data/mynetwork.gpkg")
ox.save_graphml(G, filepath="./data/mynetwork.graphml")

# convert graph to line graph so edges become nodes and vice versa
edge_centrality = nx.closeness_centrality(nx.line_graph(G))
nx.set_edge_attributes(G, edge_centrality, "edge_centrality")

# color edges in original graph with closeness centralities from line graph
ec = ox.plot.get_edge_colors_by_attr(G, "edge_centrality", cmap="inferno")
fig, ax = ox.plot_graph(G, edge_color=ec, edge_linewidth=2, node_size=0)

# impute missing edge speeds and calculate edge travel times with the speed module
G = ox.speed.add_edge_speeds(G)
G = ox.speed.add_edge_travel_times(G)

# get the nearest network nodes to two lat/lng points with the distance module
orig = ox.distance.nearest_nodes(G, X=51.495446, Y=3.618127)
dest = ox.distance.nearest_nodes(G, X=51.499318,  Y=3.610658)

# Implementation of Djikstra's algorithm
def has_path(G, source, target):
    try:
        nx.shortest_path(G, orig, dest)
    except nx.NetworkXNoPath:
        return False
    return True

def shortest_path(G, source=None, target=None, weight=None, method="dijkstra"):
    if method not in ("dijkstra", "bellman-ford"):
        # so we don't need to check in each branch later
        raise ValueError(f"method not supported: {method}")
    method = "unweighted" if weight is None else method
    if source is None:
        if target is None:
            # Find paths between all pairs.
            if method == "unweighted":
                paths = dict(nx.all_pairs_shortest_path(G))
            elif method == "dijkstra":
                paths = dict(nx.all_pairs_dijkstra_path(G, weight=weight))
            else:  # method == 'bellman-ford':
                paths = dict(nx.all_pairs_bellman_ford_path(G, weight=weight))
        else:
            # Find paths from all nodes co-accessible to the target.
            if G.is_directed():
                G = G.reverse(copy=False)
            if method == "unweighted":
                paths = nx.single_source_shortest_path(G, target)
            elif method == "dijkstra":
                paths = nx.single_source_dijkstra_path(G, target, weight=weight)
            else:  # method == 'bellman-ford':
                paths = nx.single_source_bellman_ford_path(G, target, weight=weight)
            # Now flip the paths so they go from a source to the target.
            for target in paths:
                paths[target] = list(reversed(paths[target]))
    else:
        if target is None:
            # Find paths to all nodes accessible from the source.
            if method == "unweighted":
                paths = nx.single_source_shortest_path(G, source)
            elif method == "dijkstra":
                paths = nx.single_source_dijkstra_path(G, source, weight=weight)
            else:  # method == 'bellman-ford':
                paths = nx.single_source_bellman_ford_path(G, source, weight=weight)
        else:
            # Find shortest source-target path.
            if method == "unweighted":
                paths = nx.bidirectional_shortest_path(G, source, target)
            elif method == "dijkstra":
                _, paths = nx.bidirectional_dijkstra(G, source, target, weight=weight)
            else:  # method == 'bellman-ford':
                paths = nx.bellman_ford_path(G, source, target, weight=weight)
    return paths

# find the shortest path between nodes, minimizing travel time, then plot it
route = ox.shortest_path(G, orig, dest, weight="travel_time")
fig, ax = ox.plot_graph_route(G, route, node_size=0)

# find the shortest path between nodes, minimizing travel time, then plot it
i = 0
while i < 10:
    start_time = time.time()
    routeLength = ox.shortest_path(G, orig, dest, weight="travel_time")
    print((time.time() - start_time))
    i += 1



# Configure OSMnx settings
ox.config(use_cache=True, log_console=True)

# Download street network data from OSM and construct a MultiDiGraph model
G = ox.graph_from_address('Middelburg railway station, Netherlands', dist=750, network_type='walk')

# Impute edge (walking) speeds and calculate edge traversal times
G = ox.add_edge_speeds(G)
G = ox.add_edge_travel_times(G)

# Coordinates for University College Roosevelt
ucr_latitude, ucr_longitude = 51.501795, 3.610837

# Coordinates for Middelburg railway station
station_latitude, station_longitude = 51.498240, 3.618964

# Get the nearest network nodes to University College Roosevelt
orig = ox.distance.nearest_nodes(G, X=ucr_latitude, Y=ucr_longitude)

# Get the nearest network nodes to Middelburg railway station
dest = ox.distance.nearest_nodes(G, X=station_latitude, Y=station_longitude)

# Implementation of Djikstra's algorithm
def dijkstra_shortest_path(G, source, target, weight=None):
    return nx.shortest_path(G, source=source, target=target, weight=weight, method='dijkstra')


# Timing the shortest path calculation using Dijkstra's algorithm
start_time_dijkstra = time.time()
dijkstra_path = dijkstra_shortest_path(G, source=orig, target=dest, weight='travel_time')
elapsed_time_dijkstra = (time.time() - start_time_dijkstra) * 1e6
print(f"Dijkstra's Shortest path time: {elapsed_time_dijkstra} seconds")
print("Dijkstra's Shortest path:", dijkstra_path)

# Implementation of Bellman-Ford algorithm
def bellman_ford_shortest_path(G, source, target, weight=None):
    return nx.shortest_path(G, source=source, target=target, weight=weight, method='bellman-ford')

# Timing the shortest path calculation using Bellman-Ford algorithm
start_time_bellman_ford = time.time()
bellman_ford_path = bellman_ford_shortest_path(G, source=orig, target=dest, weight='travel_time')
elapsed_time_bellman_ford = (time.time() - start_time_bellman_ford) * 1e6
print(f"Bellman-Ford Shortest path time: {elapsed_time_bellman_ford} seconds")
print("Bellman-Ford Shortest path:", bellman_ford_path)
