# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for 
# educational purposes provided that (1) you do not distribute or publish 
# solutions, (2) you retain this notice, and (3) you provide clear 
# attribution to UC Berkeley, including a link to 
# http://inst.eecs.berkeley.edu/~cs188/pacman/pacman.html
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero 
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and 
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called
by Pacman agents (in searchAgents.py).
"""

import util
# from searchAgents import manhattanHeuristic

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples,
        (successor, action, stepCost), where 'successor' is a
        successor to the current state, 'action' is the action
        required to get there, and 'stepCost' is the incremental
        cost of expanding to that successor
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.  The sequence must
        be composed of legal moves
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other
    maze, the sequence of moves will be incorrect, so only use this for tinyMaze
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first

    Your search algorithm needs to return a list of actions that reaches
    the goal.  Make sure to implement a graph search algorithm

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:
"""
    # print("Start:", problem.getStartState())
    # print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    # print("Start's successors:", problem.getSuccessors(problem.getStartState()))

    "*** YOUR CODE HERE ***"
    # create an epmpty list to store visited locations
    visited = []
    # create the frontier using a stack for LIFO function used by DFS
    frontier = util.Stack()
    # get the start node from start state
    startNode = (problem.getStartState(), [])
    # push that node to the frontier
    frontier.push(startNode)
    # get the next node in the frontier
    chosenNode = frontier.pop()
    # extract location
    chosenNodeLocation = chosenNode[0]
    # because it is the first node, add it to visited without check
    visited.append(chosenNodeLocation)
    # get the path
    chosenNodePath = chosenNode[1]
    # while the current location is not a goal state
    while not problem.isGoalState(chosenNodeLocation):
        # get all successor nodes and store in a list
        successors = problem.getSuccessors(chosenNodeLocation)
        # for each successor node
        for state in successors:
            # extract location
            successorLocation = state[0]
            # extract path
            successorPath = chosenNodePath + [state[1]]
            # push the successor to the frontier
            frontier.push((successorLocation, successorPath))
        # get the next node from the frontier
        chosenNode = frontier.pop()
        # extract its location
        chosenNodeLocation = chosenNode[0]
        # while the current location is in the visited locations list
        while chosenNodeLocation in visited:
            # get a new node from the frontier to avoid duplicate expansions
            chosenNode = frontier.pop()
            # extract its location to check at top of while loop
            chosenNodeLocation = chosenNode[0]
        # once we have found an unvisited location, add it to the list of visited locations
        visited.append(chosenNodeLocation)
        # get its path to return and return to top of while to check if its a goal state
        chosenNodePath = chosenNode[1]
    # Once we have found a goal state, return the path to it
    return chosenNodePath


def breadthFirstSearch(problem):
    """
    Search the shallowest nodes in the search tree first.
    """
    "*** YOUR CODE HERE ***"
    # create an empty list to hold previously visited locations
    visited = []
    # create the frontier using a queue for FIFO structure used by BFS
    frontier = util.Queue()
    # create the start node from start state
    startNode = (problem.getStartState(), [])
    # push start node to frontier
    frontier.push(startNode)
    # grab next node from frontier
    chosenNode = frontier.pop()
    # extract location
    chosenNodeLocation = chosenNode[0]
    # add to visited locations list
    visited.append(chosenNodeLocation)
    # extract path
    chosenNodePath = chosenNode[1]
    # while current state is not a goal state
    while not problem.isGoalState(chosenNodeLocation):
        # expand the current node
        successors = problem.getSuccessors(chosenNodeLocation)
        # for each successor node
        for state in successors:
            # extract location
            successorLocation = state[0]
            # extract path
            successorPath = chosenNodePath + [state[1]]
            # print(successorLocation, successorPath)
            # push node to the frontier
            frontier.push((successorLocation, successorPath))
        # get the next node in the frontier
        chosenNode = frontier.pop()
        # extract location
        chosenNodeLocation = chosenNode[0]
        # while the current location is in the visited list
        while chosenNodeLocation in visited:
            # get another node
            chosenNode = frontier.pop()
            # extract location
            chosenNodeLocation = chosenNode[0]
        # once we find an unvisited location, add it to the visited list
        visited.append(chosenNodeLocation)
        # extract the path
        chosenNodePath = chosenNode[1]
    # once we find a solution, return it
    return chosenNodePath


def uniformCostSearch(problem):
    """
    Search the node with the least total cost first.
    """
    "*** YOUR CODE HERE ***"
    # create an empty list to store visited locations
    visited = []
    # create the frontier using a priority queue to consider cost
    frontier = util.PriorityQueue()
    # get the start state with a list containing no actions
    startNode = (problem.getStartState(), [])
    # push the start state to the frontier with a cost of 0
    frontier.push(startNode, 0)
    # get the start node
    chosenNode = frontier.pop()
    # extract start node location
    chosenNodeLocation = chosenNode[0]
    # add location to visited list
    visited.append(chosenNodeLocation)
    # extract location path, should be empty
    chosenNodePath = chosenNode[1]

    # while the current position is not a goal state
    while not problem.isGoalState(chosenNodeLocation):
        # input("Next?")
        # print("chosenNodeLocation: ", chosenNodeLocation)
        # print("chosenNodePath: ", chosenNodePath)
        # print("Cost: ", problem.getCostOfActions(chosenNodePath))
        # expand the current node to see options
        successors = problem.getSuccessors(chosenNodeLocation)
        # for each of our options
        for state in successors:
            # extract successor location
            successorLocation = state[0]
            # print("HERE IS THE SUCCESSOR LOCATION: ", successorLocation)
            # extract successor location path
            successorPath = chosenNodePath + [state[1]]
            # extract current location cost
            successorCost = state[2]
            # get the cost of the successor's path
            successorCost = problem.getCostOfActions(successorPath)
            # print("Here is the cost: ", successorCost)
            # print(successorLocation, successorPath)
            # push the successor onto the frontier with its location, path, and cost
            frontier.push((successorLocation, successorPath), successorCost)
        # get the next node in the frontier
        chosenNode = frontier.pop()
        # extract location
        chosenNodeLocation = chosenNode[0]
        # while the current location has been visited before
        while chosenNodeLocation in visited:
            # get another node from the frontier to eliminate duplicate visits
            chosenNode = frontier.pop()
            chosenNodeLocation = chosenNode[0]
        # add the newly visited location to the visited list
        visited.append(chosenNodeLocation)
        # extract current node's path
        chosenNodePath = chosenNode[1]
        # print(problem.getCostOfActions(chosenNodePath))
    # print(chosenNode[1])
    # return the current node's path only if it is a goal state
    return chosenNodePath


def nullHeuristic(position, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    """
        The Manhattan distance heuristic for a PositionSearchProblem
        """
    # xy1 = position
    # print ("HERE IS THE POSITION", position)
    # xy2 = [1, 1]
    # return abs(xy1[0] - xy2[0]) + abs(xy1[1] - xy2[1])
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """
    Search the node that has the lowest combined cost and heuristic first.
    """
    "*** YOUR CODE HERE ***"
    # list of visited nodes
    visited = []
    # using a priority queue to consider cost for the frontier
    frontier = util.PriorityQueue()
    # get start node containing the start state amd an empty list to hold the position's path
    startNode = (problem.getStartState(), [])
    # add the initial node to the frontier
    frontier.push(startNode, 0)

    # get the next node in the frontier based on cost
    chosenNode = frontier.pop()
    # extract location
    chosenNodeLocation = chosenNode[0]
    # add location to the visited locations list
    visited.append(chosenNodeLocation)
    # extract the path to that position from the start node
    chosenNodePath = chosenNode[1]
    # print(heuristic)
    # while the current location is not a goal state
    while not problem.isGoalState(chosenNodeLocation):
        # input("Next?")
        # print("chosenNodeLocation: ", chosenNodeLocation)
        # print("chosenNodePath: ", chosenNodePath)
        # print("Cost: ", problem.getCostOfActions(chosenNodePath))
        # print(chosenNodePath)
        # expand that node to get all the successors
        successors = problem.getSuccessors(chosenNodeLocation)
        # cycle through all of our options
        for state in successors:
            # extract successor location
            successorLocation = state[0]
            # print("HERE IS THE SUCCESSOR LOCATION: ", successorLocation)
            # extract successor location path
            successorPath = chosenNodePath + [state[1]]
            # extract the cost of that path
            successorCost = state[2]
            successorCost = problem.getCostOfActions(successorPath)

            # if heuristic is not None:
                # AStarCost += heuristic(successorLocation, problem)
                # print("HERE IS THE SUCCESSOR COST: ", AStarCost)
            # add estimated cost to goal(using manhattan distance) to total cost
            successorCost += heuristic(successorLocation, problem)
                # print("HERE IS THE SUCCESSOR COST: ", successorCost)
            # print("Here is the cost: ", successorCost)
            # print(successorLocation, successorPath, successorCost)
            # add the successor to the frontier
            frontier.push((successorLocation, successorPath), successorCost)
        # get the next node based on total cost
        chosenNode = frontier.pop()
        chosenNodeLocation = chosenNode[0]
        # while the location has been visited already, pull from the frontier again to eliminate duplicate visits
        while chosenNodeLocation in visited:
            chosenNode = frontier.pop()
            chosenNodeLocation = chosenNode[0]
        # once we visit a "new" node, add it to the list of visited nodes and store it
        # to check if it is a goal state back at the top of the while
        visited.append(chosenNodeLocation)
        chosenNodePath = chosenNode[1]
        # print(problem.getCostOfActions(chosenNodePath))

    # print(chosenNode[1])
    # print(successorCost)
    # once we have found the optimal path, return it
    return chosenNodePath


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
