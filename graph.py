import heapdict
from queue import Queue

# A Graph class, with insert edge, insert weighted edge, insert vertex, DFS, BFS, Dijkstra, and
# Find Path between two nodes.
# This graph is implemented with an adjacency list, created with a dictionary that has node keys
# with a value of another dictionary with node keys and edge weight values for each adjacent node.


class Graph:
    # Constructor to create the graph's dictionary for the adjacency list
    def __init__(self):
        self.myDictionary = {}

    # Inserts a vertex with no edges connected to it into the graph
    def InsertVertex(self, vertex):
        self.myDictionary[vertex] = {}

    # Insert a directed edge from u to v with a weight of 0.
    def InsertEdge(self, u, v):
        self.myDictionary[u][v] = 0

    # Adds a weighted edge to the graph from u to v
    # Weights should not be negative. Dijkstra won't work with negative weights
    def WeightedEdge(self, u, v, weight):
        if weight >= 0:
            self.myDictionary[u][v] = weight
        else:
            print("Error: No negative weights allowed")

    # Returns the edges for the given vertex
    def GetEdges(self, vertex):
        return self.myDictionary[vertex]

    # Returns the dictionary for the graph
    def GetGraph(self):
        return self.myDictionary

    # This is the recursive Depth First Search given the starting node, visited set, and dfsTree list
    # It adds the edges that build the DFS tree to the dfsTree list
    def DFS_Recursive(self, start, visited, dfsTree):
        visited.add(start)
        for w in self.myDictionary[start]:
            if w not in visited:
                dfsTree.append((start, w))
                self.DFS_Recursive(w, visited, dfsTree)
        return dfsTree

    # This function creates the visited set and dfsTree list then calls DFS_Recursive so that the
    # user won't have to make the set and list when calling DFS.
    def DFS(self, start):
        visited = set()
        dfsTree = []
        return self.DFS_Recursive(start, visited, dfsTree)

    # This is the Breadth First Search given a starting node.
    def BFS(self, start):
        # This is the set of visited edges
        visited = {start}
        q = Queue()
        edgeList = []
        q.put(start)
        while not q.empty():
            v = q.get()
            for w in self.myDictionary[v]:
                if w not in visited:
                    visited.add(w)
                    q.put(w)
                    edgeList.append((v,w))
        return edgeList

    # This is a modified version of DFS where it is given a starting node and an ending node and
    # will keep going through until it reaches the ending node. Once the desired node is reached,
    # a path will be created from stop to start with each return that this function has.
    # This function returns true if there is a path from start to stop.
    def DFS_Search(self, start, stop, visited):
        visited.add(start)
        for w in self.myDictionary[start]:
            if w == stop:
                # node w is the same as stop, so add it as the first node in the path, then return true.
                self.path.append(w)
                return True
            elif w not in visited:
                if self.DFS_Search(w, stop, visited):
                    # if this function call returned true, add node w to the path
                    self.path.append(w)
                    return True
        return False

    # This function will find a path between start and end using DFS_Search.
    def FindPath(self, start, end):
        # This is the list that holds the path between start and end
        self.path = []
        visited = set()
        # DFS_Search returns true if there is a path between start and end
        if self.DFS_Search(start, end, visited):
            # Add the first node to the path
            self.path.append(start)
            # Reverse the path so that it goes from start to end
            self.path.reverse()
            return self.path
        else:
            return None

    # Dijkstra algorithm to find the shortest path to every node from the given node.
    # This function uses the heapdict priority queue to pick the next node with the next smallest estimate.
    # This function returns the weights dictionary that holds the shortest path length for each node in the graph.
    def Dijkstra(self, start):
        weights = {start: 0}
        for v in self.myDictionary:
            # Add values to the dictionary of node : path length
            # Add a very large value so that the comparisons will always reduce the value
            if v != start:
                weights[v] = 100000

        # Create the priority queue with heapdict, which was imported
        q = heapdict.heapdict()
        for i in self.myDictionary[start]:
            # insert (weight, node)
            q[i] = self.myDictionary[start][i]
            weights[i] = self.myDictionary[start][i]
        # Explored is the set of explored nodes
        explored = {start}
        # This loop continues until there are no items left in the queue
        while len(q.items()) != 0:
            # dequeue the cheapest vertex from priorityQueue
            current = q.popitem()
            # current[0] is the node and current[1] is the weight
            explored.add(current[0])
            for i in self.myDictionary[current[0]]:
                # Add the node with its total value to the priority queue
                # Total value is the edge weight + value of current
                q[i] = self.myDictionary[current[0]][i] + weights[current[0]]
                # if the current path length is greater than the new path length, replace it
                if weights[i] > weights[current[0]] + self.myDictionary[current[0]][i]:
                    weights[i] = weights[current[0]] + self.myDictionary[current[0]][i]
        return weights

