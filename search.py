# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()

def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    # create the start node
    startNode = node(problem.getStartState(), None, 0, None, 0)
    # if the start node is the goal state, we are done (return startNode)
    if problem.isGoalState(problem.getStartState):
        return startNode
    # create a stack d/s to hold nodes
    dfsStack = util.Stack()
    # push start node onto stack
    dfsStack.push(startNode)
    # create a list to hold the states of explored nodes
    exploredSet = []
    a = True
    # big while loop that runs until either pacman fails or a solution is found
    while(a):
        # if the stack is empty, then we have examined all nodes and not found a solution, so return failure
        if dfsStack.isEmpty():
            a = False
        # pop the most recently pushed item from the stack, and call it the current node
        currentNode = dfsStack.pop()
         # if the child node has a state that is the goal state
        if problem.isGoalState(currentNode.getState()):
            returnList = []
            returnList.append(currentNode.getDirection())
            while currentNode.getParent():
                tempNode = currentNode.getParent()
                returnList.append(tempNode.getDirection())
                currentNode = currentNode.getParent()
            returnList.remove(None)
            returnList.reverse()
            return returnList
        # add the state of the current node to the explored set
        exploredSet.append(currentNode.getState())
        # find all the successors (child nodes) of the current node
        for i in problem.getSuccessors(currentNode.getState()):
            # construct an instance of each child node
            childNode = node(i[0], i[1], i[2], currentNode, 0)
            # if the child node has a state that has not been explored yet
            if childNode.getState() not in exploredSet:
                # if the child node has a state that is not the goal state, push onto the stack
                dfsStack.push(childNode)
    util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    # create the start node
    startNode = node(problem.getStartState(), None, 0, None, 0)
    # if the start node is the goal state, we are done (return startNode)
    if problem.isGoalState(problem.getStartState()):
        return startNode
    # create a queue d/s to hold nodes
    bfsQueue = util.Queue()
    # creates a list of states already in the queue
    inQueue = []
    # push start node onto queue
    bfsQueue.push(startNode)
    # add start node onto list of states already in the queue
    inQueue.append(startNode.getState())
    # create a list to hold the states of explored nodes
    exploredSet = []
    a = True
    # big while loop that runs until either pacman fails or a solution is found
    while(a):
        # if the stack is empty, then we have examined all nodes and not found a solution, so return failure
        if bfsQueue.isEmpty():
            a = False
        # pop the most recently pushed item from the stack, and call it the current node
        currentNode = bfsQueue.pop()
        # if the child node has a state that is the goal state
        if problem.isGoalState(currentNode.getState()):
            # create empty list of actions to return, and append direction from goal state node
            returnList = []
            returnList.append(currentNode.getDirection())
            # while each node still has a parent node
            while currentNode.getParent():
                # get the current node's parent
                tempNode = currentNode.getParent()
                # add the direction of the parent node to the return list of actions
                returnList.append(tempNode.getDirection())
                # change current node reference to parent node
                currentNode = currentNode.getParent()
            # remove direction from initial start node
            returnList.remove(None)
            # reverse directions in return list of actions
            returnList.reverse()
            # return list of actions
            return returnList
        # add the state of the current node into the explored set
        exploredSet.append(currentNode.getState())
        # find all the successors (child nodes) of the current node
        for i in problem.getSuccessors(currentNode.getState()):
            # construct an instance of each child node
            childNode = node(i[0],i[1],i[2],currentNode,0)
            # if the child node has a state that has not been explored yet
            if childNode.getState() not in exploredSet and childNode.getState()  not in inQueue:
                # if the child node has a state that is not the goal state, push onto the stack
                bfsQueue.push(childNode)
                inQueue.append(childNode.getState())
    util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    # creates a start node
    startNode = node(problem.getStartState(), None, 0, None, 0)
    # create a dictionary to store nodes & path costs in priority queue d/s
    inPQ = {}
    # creates a priority queue d/s for the frontier
    ucsPQ = util.PriorityQueue()
    # push start node and path cost to priority queue
    ucsPQ.push(startNode, startNode.getPathCost())
    # add start node state and path cost to dictionary
    inPQ[startNode.getState()] = startNode.getPathCost()
    # create empty list to store explored states
    exploredSet = []
    a = True
    # create empty list to return list of actions
    returnList = []
    while(a):
        # if priority queue is empty, then we have examined all nodes and not found the goal state, so return failure
        if ucsPQ.isEmpty():
            a = False
        # pop highest priority (least total cost) node from priority queue, and call it current node
        currentNode = ucsPQ.pop()
        # if the state of the current node is the goal state
        if problem.isGoalState(currentNode.getState()):
            # add direction of current node to return list of actions
            returnList.append(currentNode.getDirection())
            # while current node has a parent node
            while currentNode.getParent():
                # get the parent node of the current node
                tempNode = currentNode.getParent()
                # add direction of parent node to return list of actions
                returnList.append(tempNode.getDirection())
                # rename current name reference from current node to parent of current node
                currentNode = currentNode.getParent()
            # remove direction of start node from return list of actions
            returnList.remove(None)
            # reverse return list of actions
            returnList.reverse()
            # return list of actions
            return returnList
        # if state of current node is not goal state, add to explored set
        exploredSet.append(currentNode.getState())
        # for all successor nodes of current node
        for i in problem.getSuccessors(currentNode.getState()):
            # create child node using node class
            childNode = node(i[0], i[1], i[2]+currentNode.getPathCost(), currentNode, 0)
            # if state of child node is not in explored set or in dictionary
            if childNode.getState() not in exploredSet and childNode.getState() not in inPQ.keys():
                # push child node and path cost onto priority queue
                ucsPQ.push(childNode, childNode.getPathCost())
                # add child node and path cost to dictionary
                inPQ[childNode.getState()] = childNode.getPathCost()
            # else if state of child node is in dictionary and path cost is less than path cost in dictionary
            elif childNode.getState() in inPQ.keys() and childNode.getPathCost() < inPQ[childNode.getState()]:
                # update priority queue with updated path cost
                ucsPQ.update(childNode, childNode.getPathCost())
    util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    # creates a start node
    startNode = node(problem.getStartState(), None, 0, None, heuristic(problem.getStartState(), problem))
    # create dictionary to store states and path costs of nodes in the priority queue
    inPQ = {}
    # creates a priority queue for the frontier
    astarPQ = util.PriorityQueue()
    # push start node onto priority queue
    astarPQ.push(startNode, (startNode.getPathCost() + startNode.getHeuristic()))
    # track what's in the priority queue by adding the path cost of the start node
    inPQ[startNode.stateToString()] = startNode.getPathCost() + startNode.getHeuristic()
    # create empty list to store visited states
    exploredSet = []
    a = True
    # create empty list to store return list of actions
    returnList = []
    # while goal state has not been found and there are still nodes to explore
    while (a):
        # if the priority queue is empty, then pacman has not found a solution
        if astarPQ.isEmpty():
            a = False
        # pop top priority node from priority queue, i.e. with lowest f(n)
        currentNode = astarPQ.pop()
        # if the current node is the goal state
        if problem.isGoalState(currentNode.getState()):
            # add direction of current node to return list of actions
            returnList.append(currentNode.getDirection())
            # while current node has a parent node
            while currentNode.getParent():
                # get parent node of current node and call it temp node
                tempNode = currentNode.getParent()
                # add direction of parent node to return list of actions
                returnList.append(tempNode.getDirection())
                # rename current node reference to parent node from current node
                currentNode = currentNode.getParent()
            # remove direction of start node from return list of actions
            returnList.remove(None)
            # reverse return list of actions
            returnList.reverse()
            # return list of actions
            return returnList
        # add state of current node to explored set of node states
        exploredSet.append(currentNode.getState())
        # for each child node of current node
        for i in problem.getSuccessors(currentNode.getState()):
            # create child node using node class
            childNode = node(i[0], i[1], i[2] + currentNode.getPathCost(), currentNode, heuristic(i[0], problem))
            # if state of child node is not in the explored set of states and dictionary
            if childNode.getState() not in exploredSet and childNode.stateToString() not in inPQ.keys():
                # push child node with path cost + heuristic cost onto priority queue
                astarPQ.push(childNode, (childNode.getPathCost() + childNode.getHeuristic()))
                # add child node with path cost + heuristic cost to dictionary
                inPQ[childNode.stateToString()] = childNode.getPathCost() + childNode.getHeuristic()
            # if state of child node is in the dictionary and path cost + heuristic cost is less than current function
            elif childNode.stateToString() in inPQ.keys() and (childNode.getPathCost() +  childNode.getHeuristic()) < \
                    inPQ[childNode.stateToString()]:
                # update child node in priority queue with updated path cost + heuristic cost
                astarPQ.update(childNode, (childNode.getPathCost() + childNode.getHeuristic()))
    util.raiseNotDefined()

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch

# node class
class node:
    # initialize node with state, direction, path cost, parent node pointer, heuristic function
    def __init__(self, state, direction, pathcost, parentnode, heuristic):
        self.state = state
        self.direction = direction
        self.pathcost = pathcost
        self.parentnode = parentnode
        self.heuristic = heuristic

    # print node
    def printNode(self):
        print self.state, self.direction, self.pathcost, self.parentnode, self.heuristic

    # get state
    def getState(self):
        return self.state

    # print state
    def printState(self):
        print self.state

    # get parent
    def getParent(self):
        return self.parentnode

    # get direction
    def getDirection(self):
        return self.direction

    # print direction
    def printDirection(self):
        print self.direction

    # set path cost
    def setPathCost(self, pathcost):
        self.pathcost = pathcost

    # get path cost
    def getPathCost(self):
        return self.pathcost

    # get heuristic
    def getHeuristic(self):
        return self.heuristic

    # get state as string
    def stateToString(self):
        return str(self.state)