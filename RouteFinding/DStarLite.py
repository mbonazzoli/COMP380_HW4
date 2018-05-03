###############################################
# D* Lite
# Susan Fox
# Spring 2016

"""This file contains an implementation of D* Lite. Because of the different\
pieces of this algorithm, and their need to access a lot of shared data, and\
the way in which the algorithm must be restarted when new information comes in, it
made sense to make a class to contain the information. I have separated the outer loop, where
it computes shortest path and then waits for new information, into external functions. The purpose of
the class is to contain the information that is generated, including the priority queue,
and to respond with methods that implement the key features of the algorithm."""

import random
from FoxQueue import Queue, PriorityQueue
import time
import numpy as np
# import RouteFinding

class DStarAlgorithm:


    def __init__(self, graph, startVert, goalVert):
        """Takes in a graph, start vertex and goal vertex, and sets up the D* Lite
        search, initializing all the data structures and the priority queue."""
        self.graph = graph
        self.startVert = startVert
        self.goalVert = goalVert
        self.maxVal = 1000

        # PriorityQueue with values [fEst, gEst]
        self.U = PriorityQueue()
        # estimated cost from g and c for a vertex
        self.rhs = {}
        # estimated cost from start vertex to other vertex
        self.g = {}

        # print(self.startVert, self.goalVert)

        self.initialize()
        
        
        
    def initialize(self):
        """The Initialize algorithm from the pseudocode."""
        for s in range(0, self.graph._numVerts):
            # s = self.graph.getData(i)
            self.rhs.update({s: self.maxVal})
            self.g.update({s: self.maxVal})

        self.rhs[self.startVert] = 0
        # print(self.g, self.rhs)
        # print(self.g.values(),self.rhs.values())

        self.U.insert(self.calculateKey(self.startVert), self.startVert)

        
    def computeShortestPath(self):
        """The ComputeShortestPath algorithm from the pseudocode."""
        # print(self.rhs[self.goalVert])
        # print(self.g[self.goalVert])
        # print((self.rhs[self.goalVert] != self.g[self.goalVert]))
  
        while (self.U.firstElement() is not None) and ((self.U.firstElement()[0][0] < self.calculateKey(self.goalVert)[0]) or
                                        (self.rhs[self.goalVert] != self.g[self.goalVert])):
            u = self.U.firstElement()[1]
            # print(u)
            self.U.delete()
            if self.g.get(u) > self.rhs.get(u):
                self.g[u] = self.rhs[u]
            else:
                self.g[u] = np.inf
                self.updateVertex(u)
            for [s, cost] in self.graph.getNeighbors(u):
                self.updateVertex(s)
        
        return self.reconstructPath()
                        
            # if (self.U.firstElement() is not None)
            #     break
                
    def updateVertex(self, vert):
        """The UpdateVertex algorithm from the pseudocode."""
        if vert != self.startVert:
            self.rhs[vert] = self.minNeighCost(vert)
        if self.U.contains(vert):
            self.U.removeValue(vert)
        if self.g[vert] != self.rhs[vert]:
            self.U.insert(self.calculateKey(vert), vert)

            
    def minNeighCost(self, vert):
        """A helper to compute the new rhs value, by finding the minimum cost among
        all the neighbors of a vertex. The cost is computed as the g cost of the
        neighbor plus the edge cost between the neighbor and the vertex."""
        minNCost = self.maxVal
        minVert = -1
        for neighInfo in self.graph.getNeighbors(vert):
            neigh = neighInfo[0]
            edgeCost = neighInfo[1]
            newCost = self.g[neigh] + edgeCost
            if newCost < minNCost:
                minNCost = newCost
                minVert = neigh
        return minNCost
               
        
    def calculateKey(self, vert):
        """The CalculateKey algorithm from the pseudocode."""
        minG = min(self.g.get(vert), self.rhs.get(vert))
        heurCost = self.graph.heuristicDist(vert, self.goalVert)
        return [minG + heurCost, minG]


    def compareKeys(self, key1, key2):
        """Takes in two keys, each of which is a list containing f cost
        and g cost. It prefers the lower f cost, but for equal f costs
        it chooses the lower g cost."""
        [f1, g1] = key1
        [f2, g2] = key2
        return (f1 < f2) or ( (f1 == f2) and (g1 < g2) )
    
    
    def correctInformation(self, newInfo):
        """Takes in a dictionary whose keys are (r, c) tuples, and the value
        is the newly corrected cell weight. Updates the graph, and then updates
        the search information appropriately."""
        for (r, c) in newInfo:
            self.graph.setCellValue(r, c, newInfo[r, c])
        self.graph.graphFromGrid()
        for (r, c) in newInfo:
            nodeNum = r * self.graph.getWidth() + c
            #print("(", r, c, ")", nodeNum)
            self.updateVertex(nodeNum)
            neighs = self.graph.getNeighbors(nodeNum)
            for (nextNeigh, wgt) in neighs:
                self.updateVertex(nextNeigh)
        
    
    
  

    def reconstructPath(self):
        """ Given the start vertex and goal vertex, and the table of
        predecessors found during the search, this will reconstruct the path
        from start to goal"""

        path = [self.goalVert]   
        currVert = self.goalVert
        while currVert != self.startVert:
            currVert = self._pickMinNeighbor(currVert)
            path.insert(0, currVert)
        return path

    
    def _pickMinNeighbor(self, vert):
        """A helper to path-reconstruction that finds the neighbor of a vertex
        that has the minimum g cost."""
        neighs = self.graph.getNeighbors(vert)
        minVal = self.maxVal
        minNeigh = None
        for [neigh, cost] in neighs:
            if self.g[neigh] < minVal:
                minVal = self.g[neigh]
                minNeigh = neigh
        return minNeigh
                

# ---------------------------------------------------------------
def DStarRoute(graph, startVert, goalVert):
    """ This algorithm searches a graph using D* Lite looking for a path from
    some start vertex to some goal vertex It uses a queue to store the
    indices of vertices that it still needs to examine. This version of D*
    Lite is equivalent to A*, because the information used in the search is
    all accurate."""

    dStarRunner = DStarAlgorithm(graph, startVert, goalVert)
    route = dStarRunner.computeShortestPath()
    return route




def DStarGlobal(graph, startVert, goalVert, percWrong = 20):
    """This algorithm search a graph using D* Lite for
    a path from some start to some goal. It simulates incorrect
    information about the world by modifying percWrong percent
    of the cells in the graph to have a different value. Then,
    after the first route is generated, the correct data is
    provided to the DStarLite object, and it updates its route."""
    if graph.__class__.__name__ != "GridGraph":
        print("DStarGlobal only works on Grid Graphs, try again.")
        return
    correctInfo, incorrectGraph = corruptGraph(graph, percWrong)
    print("CORRUPTED GRAPH:")
    # incorrectGraph.printGrid()
    dStarRunner = DStarAlgorithm(incorrectGraph, startVert, goalVert)
    t1 = time.time()
    route1 = dStarRunner.computeShortestPath()
    # t2 = time.time()
    # print ("First route found is:")
    # print (route1)
    # print ("Time elapsed:", t2-t1)
    # graph.printWithRoute(route1)

    print("--------")
    print("CORRECT MAP:")
    # graph.printGrid()
    print ("Correcting information...")
    dStarRunner.correctInformation(correctInfo)

    # t1 = time.time()
    route2 = dStarRunner.computeShortestPath()
    t2 = time.time()
    if route1 == route2:
        print("SAME ROUTE")
    print ("Fixed route found is:")
    # print (route2)
    print ("Time elapsed:", t2-t1)
    # graph.printWithRoute(route2)
    return route2
    
    
    


def DStarLocal(graph, startVert, goalVert, percWrong = 50):
    """This algorithm search a graph using D* Lite for
    a path from some start to some goal. It simulates incorrect
    information about the world by modifying percWrong percent
    of the cells in the graph to have a different value. Then,
    after the first route is generated, the function determines when
    the route comes close to an incorrect value, and it feeds a small number of corrections at a time to DStarLite.."""
    if graph.__class__.__name__ != "GridGraph":
        print("DStarGlobal only works on Grid Graphs, try again.")
        return
    correctInfo, incorrectGraph = corruptGraph(graph, percWrong)
    print("CORRUPTED GRAPH:")
    dStarRunner = DStarAlgorithm(incorrectGraph, startVert, goalVert)
    print("First pass...")
    # RouteFinding.testAndRun(dStarRunner)
    t1 = time.time()
    route1 = dStarRunner.computeShortestPath()
    # t2 = time.time()
    print ("First route found is:")
    # print (route1)
    # print ("Time elapsed:", t2-t1)
    # incorrectGraph.printWithRoute(route1)
    mapWid = graph.getWidth()
    mapHgt = graph.getHeight()
    for cell in route1:
        (r, c) = graph.getData(cell)
        badNeighbors = findIncorrectNeighbors(correctInfo, r, c, mapWid, mapHgt)
        if len(badNeighbors) > 0:
            print("Incorrect neighbors:", badNeighbors)
            print ("Correcting information...")
            dStarRunner.correctInformation(badNeighbors)
            # t1 = time.time()
            nextRoute = dStarRunner.computeShortestPath()
            t2 = time.time()
            print ("Fixed route found is:")
            # print (nextRoute)
            # print ("Time elapsed:", t2-t1)
            # graph.printWithRoute(nextRoute)
    t2 = time.time()
    print ("Time elapsed:", t2-t1)

    return nextRoute



def findIncorrectNeighbors(correctInfo, row, col, wid, hgt):
    """Takes in the dictionary of correct information, along
    with a row and column of a cell in the occupancy grid, and the size
    of the occupancy grid. It checks if any of the nine cells centered
    on row and col are incorrect. If so, they, along with the correct
    information, are added to the dictionary. That dictionary is returned."""
    bads = {}
    for r in [row - 1, row, row + 1]:
        for c in [col -1, col, col + 1]:
            if (r >= 0) and (r < wid) and (c >= 0) and (c < hgt):
                if (r, c) in correctInfo:
                    bads[r, c] = correctInfo[r, c]
                    del correctInfo[r, c]
    return bads

    
    
def corruptGraph(oldGraph, percWrong):    
    """Takes in an old grid graph and the percentage that should
    be modified, and returns a dictionary containing the correct 
    values, so they can be fixed later, and a new graph with
    modified values."""
    graph = oldGraph.copy()
    graphSize = graph.getSize()
    minC = oldGraph.getMinCost()
    maxC = oldGraph.getMaxCost()
    #print("minC =", minC, "maxC =", maxC)
    correctInfo = {}
    for i in range(percWrong * graphSize // 100):
        randNode = random.randrange(graphSize)
        (randRow, randCol) = graph.getData(randNode)
        randVal = random.randint(minC, maxC)
        correctInfo[randRow, randCol] = graph.getCellValue(randRow, randCol)
        graph.setCellValue(randRow, randCol, randVal)
    graph.graphFromGrid()
    return correctInfo, graph
        
        
        