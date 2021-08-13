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
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.
    """
#     # BASED ON https://www.youtube.com/watch?v=MMCUepjSoLQ&list=LL&index=2&t=2s adapted to:

    # DFS :
    # https://en.wikipedia.org/wiki/Depth-first_search#Pseudocode
    # procedure DFS_iterative(G, v) is
    #     let S be a stack
    #     S.push(v)
    #     while S is not empty do
    #         v = S.pop()
    #         if v is not labeled as discovered then
    #             label v as discovered
    #             for all edges from v to w in G.adjacentEdges(v) do
    #                 S.push(w)

    # inicial position:
    inicio = problem.getStartState()
    # discovered:
    vertices_descobertos = {}
    # graph storing directions to go to start
    parents_graph = {}
    # action solutions
    acoes = []
    pilha_s = util.Stack()

    pilha_s.push((inicio, 'inicio'))
    # DFS ITERATIVE:
    #     while S is not empty do
    while not pilha_s.isEmpty():
        v = pilha_s.pop()
        # v[0] : current node
        # v[1] : action
        vertices_descobertos[v[0]] = v[1]
        if problem.isGoalState(v[0]):
            final_state = v[0]
            break
        for vertice_adjacente in problem.getSuccessors(v[0]):
            if vertice_adjacente[0] not in vertices_descobertos.keys():
                parents_graph[vertice_adjacente[0]] = v[0]
                pilha_s.push(vertice_adjacente)

   # montando a lista de acoes:
    try:
        while(final_state in parents_graph.keys()):
            # traveling through last way u reached each parents and pushing into solution
            final_state_prev = parents_graph[final_state]
            acoes.insert(0, vertices_descobertos[final_state])
            final_state = final_state_prev
    except NameError:
        print "Caminho nao encontrado."
    return acoes


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""

    inicio = problem.getStartState()
    vertices_descobertos = []
    fila = util.Queue()
    fila.push([inicio, []])

    while not fila.isEmpty():
        node = fila.pop()
        # node[0]: state/name
        # node[1]: actions to take to node[0]
        if node[0] not in vertices_descobertos:
            if problem.isGoalState(node[0]):
                return node[1]
            vertices_descobertos.append(node[0])
            for vertice_adjacente in problem.getSuccessors(node[0]):
                actions_to_vertice_adjacente = node[1] + [vertice_adjacente[1]]
                fila.push([vertice_adjacente[0], actions_to_vertice_adjacente])


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "https://www.educative.io/edpresso/what-is-uniform-cost-search*** and https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm#Practical_optimizations_and_infinite_graphs"

    # priority = cost + parent cost
    start = problem.getStartState()
    frontier = util.PriorityQueue()
    # using actions inside queue instead of parents_graph to backtrack goal. Each node stores the actions to take to him, so when the goal is reached, you return this value)
    actions = []
    frontier.push([start, actions, 0], 0)
    explored = []

    while not frontier.isEmpty():
        # node[0] = State/Name
        # node[1] = Actions to go to node[0] from start
        # node[2] = Cost
        node = frontier.pop()
        if node[0] not in explored:
            explored.append(node[0])
            if problem.isGoalState(node[0]):
                return node[1]
            for neighbor in problem.getSuccessors(node[0]):
                priority = node[2] + neighbor[2]
                actions_to_neighbor = node[1] + [neighbor[1]]
                frontier.push(
                    (neighbor[0], actions_to_neighbor, priority), priority)


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    # priority = (cost + parent cost) + heuristic
    # using actions inside queue instead of parents_graph to backtrack goal. Each node stores the actions to take to him, so when the goal is reached, you return this value):

    start = problem.getStartState()
    frontier = util.PriorityQueue()
    frontier.push([start, [], 0], 0)
    explored = []

    while not frontier.isEmpty():
        # node[0] = State/Name
        # node[1] = List of Actions from Start to Node ['South', 'West', .... ]
        # node[2] = Cost
        node = frontier.pop()
        if node[0] not in explored:
            explored.append(node[0])
            if problem.isGoalState(node[0]):
                return node[1]

            for neighbor in problem.getSuccessors(node[0]):
                priority = (node[2] + neighbor[2]) + \
                    heuristic(neighbor[0], problem)
                actions_to_neighbor = node[1] + [neighbor[1]]
                frontier.push(
                    (neighbor[0], actions_to_neighbor, priority), priority)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
