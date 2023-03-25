# Graph Library
This is a graph library with functions for inserting a vertex, inserting an edge (weighted or unweighted), DFS, BFS, Dijkstra, and finding a path between two vertices.

This graph is implemented with an adjacency list, created with a 2D dictionary that has vertex keys and holds a dictionary with all of the adjacent vertices to the given vertex along with their edge weights.
This graph uses directed edges. You can make the edges point both directions and use DFS and BFS, but you can only use one-direction edges for Dijkstra's algorithm.

Must run ***pip install HeapDict*** to install the heapdict priority queue for this graph library to work. These files were created and run in the PyCharm IDE.

**graph.py** is the graph library.

**main.py** contains tests to show how this graph library works.

### Graph Library Functions:
- InsertVertex(vertex)
- InsertEdge(u, v)
- WeightedEdge(u, v, weight)
- GetEdges(vertex)
- GetGraph()
- DFS(start)
- BFS(start)
- FindPath(start, end)
- Dijkstra(start)
