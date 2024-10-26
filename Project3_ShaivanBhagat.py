# CODE OBTAINED FROM GITHUB 
# CODE MODIFIED ONLY TO INCLUDE MIDDELBURG COORDINATES AND ACCORDING VARIABLES AND VALUES

import networkx as nx
import osmnx as ox
import time

# Configure OSMnx settings
ox.config(use_cache=True, log_console=True)

# Download street network data from OSM and construct a MultiDiGraph model
G = ox.graph_from_point((51.495446, 3.618127), dist=750, network_type="walk")

# Impute edge (walking) speeds and calculate edge traversal times
G = ox.add_edge_speeds(G)
G = ox.add_edge_travel_times(G)

# Convert MultiDiGraph to GeoDataFrames
gdf_nodes, gdf_edges = ox.graph_to_gdfs(G)
G = ox.graph_from_gdfs(gdf_nodes, gdf_edges, graph_attrs=G.graph)

# Convert MultiDiGraph to DiGraph to use nx.betweenness_centrality function
D = ox.utils_graph.get_digraph(G, weight="travel_time")

# Calculate node betweenness centrality, weighted by travel time
bc = nx.betweenness_centrality(D, weight="travel_time", normalized=True)
nx.set_node_attributes(G, values=bc, name="bc")

# Plot the graph, coloring nodes by betweenness centrality
nc = ox.plot.get_node_colors_by_attr(G, "bc", cmap="plasma")
fig, ax = ox.plot_graph(
    G, bgcolor="k", node_color=nc, node_size=50, edge_linewidth=2, edge_color="#333333"
)

# Get the nearest network nodes to two lat/lng points
orig = ox.distance.nearest_nodes(G, X=51.495446, Y=3.618127)
dest = ox.distance.nearest_nodes(G, X=51.499318, Y=3.610658)

# Save graph to shapefile, geopackage, or graphml
ox.save_graph_shapefile(G, filepath="./graph_shapefile/")
ox.save_graph_geopackage(G, filepath="./graph.gpkg")
ox.save_graphml(G, filepath="./graph.graphml")

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
                _, paths = nx.bidirectional_dijkstra(G, source, target, weight)
            else:  # method == 'bellman-ford':
                paths = nx.bellman_ford_path(G, source, target, weight)
    return paths



# Timing the shortest path calculation
start_time = time.time()
shortest_path(G, source=orig, target=dest, weight="travel_time", method="dijkstra")
elapsed_time = (time.time() - start_time) * 1e6  # convert to microseconds
print(f"Shortest path time Djikstra: {elapsed_time} microseconds")


# CODE FOR BELLMAN FORD ADDED MANUALLY 
start_time = time.time()
shortest_path(G, source=orig, target=dest, weight="travel_time", method="bellman-ford")
elapsed_time = (time.time() - start_time) * 1e6
print(f"Shortest path time Bellman-Ford: {elapsed_time} seconds")




