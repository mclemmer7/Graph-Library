from graph import Graph

# This file holds some tests used to show the graph library from graph.py

# graph is a Graph with weighted edges
graph = Graph()
graph.InsertVertex(1)
graph.InsertVertex(2)
graph.InsertVertex(3)
graph.InsertVertex(4)
graph.InsertVertex(5)
graph.InsertVertex(6)

graph.WeightedEdge(1,2,3)
graph.WeightedEdge(1,5,1)
graph.WeightedEdge(5,4,3)
graph.WeightedEdge(2,3,10)
graph.WeightedEdge(2,4,2)
graph.WeightedEdge(4,3,5)
graph.WeightedEdge(4,6,3)
graph.WeightedEdge(1,6,8)

print("Edges of 1 in graph: " + str(graph.GetEdges(1)))
print("Edges of 2 in graph: " + str(graph.GetEdges(2)))

# TESTS WITH WEIGHTED EDGES ON THE GRAPH
print("Path lengths from Dijkstra starting at node 1: " + str(graph.Dijkstra(1)))
print("BFS for graph from root node 1: " + str(graph.BFS(1)))
print("DFS for graph from root node 1: " + str(graph.DFS(1)))
print("The path between 1 and 6 is: " + str(graph.FindPath(1,6)))
print("The path between 5 and 3 is: " + str(graph.FindPath(5,3)))

# Returns None since there is no path from 4 to 2 since it is directed
print("The path between 4 and 2 is: " + str(graph.FindPath(4,2)))


# Graph g2 has no edge weights
g2 = Graph()
g2.InsertVertex(1)
g2.InsertVertex(2)
g2.InsertVertex(3)
g2.InsertVertex(4)
g2.InsertVertex(5)
g2.InsertVertex(6)
g2.InsertVertex(7)
g2.InsertVertex(8)
g2.InsertVertex(9)

g2.InsertEdge(1,2)
g2.InsertEdge(1,3)
g2.InsertEdge(1,4)
g2.InsertEdge(2,5)
g2.InsertEdge(2,3)
g2.InsertEdge(2,6)
g2.InsertEdge(3,6)
g2.InsertEdge(3,7)
g2.InsertEdge(7,8)
g2.InsertEdge(7,9)
g2.InsertEdge(4,9)

print("BFS for g2 from the root node 1: " + str(g2.BFS(1)))
print("DFS for g2 from the root node 1: " + str(g2.DFS(1)))
print("DFS for g2 from 3, which is not a root: " + str(g2.DFS(3)))
print("The path between 1 and 8 is: " + str(g2.FindPath(1,8)))

