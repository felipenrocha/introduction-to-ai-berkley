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

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"

    # DFS :
    # print "Start:", problem.getStartState()
    # print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    #     # print "Start's successors:", problem.getSuccessors(problem.getStartState())

    # BASED ON: https://en.wikipedia.org/wiki/Depth-first_search#Pseudocode
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
    parents_graph = {}
    # action solutions
    acoes = []
    # bool value to check if goal is reached
    goal = False
    #  let S be a stack
    pilha_s = util.Stack()

    pilha_s.push((inicio, 'inicio'))
    # DFS ITERATIVE:
    #     while S is not empty do
    while not pilha_s.isEmpty() and goal == False:
        #  v = S.pop()
        v = pilha_s.pop()

        vertices_descobertos[v[0]] = v[1]

        if problem.isGoalState(v[0]):
            goal = True
            final_state = v[0]
            break
        for vertice_adjacente in problem.getSuccessors(v[0]):
            if vertice_adjacente[0] not in vertices_descobertos.keys():
                parents_graph[vertice_adjacente[0]] = v[0]
                pilha_s.push(vertice_adjacente)

    # montando a lista de acoes:
    while(final_state in parents_graph.keys()):
        final_state_prev = parents_graph[final_state]
        acoes.insert(0, vertices_descobertos[final_state])
        final_state = final_state_prev
    return acoes


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
#     # BASED ON https://www.youtube.com/watch?v=MMCUepjSoLQ&list=LL&index=2&t=2s adapted to:
#        escolha uma raiz s de G
#    marque s
#    insira s em F
#    enquanto F nao esta vazia faca
#       seja v o primeiro vertice de F
#       para cada w pertence a listaDeAdjacencia de v faca
#          se w nao esta marcado entao
#             visite aresta entre v e w
#             marque w
#             insira w em F
#          senao se w pertence a F entao
#             visite aresta entre v e w
#          fim se
#       fim para
#       retira v de F
#    fim enquanto

    inicio = problem.getStartState()
    vertices_descobertos = {}
    parents_graph = {}
    acoes = []
    goal = False
    fila = util.Queue()

    fila.push((inicio, 'inicio'))
    # DFS ITERATIVE:
    #     while S is not empty do
    while not fila.isEmpty() and goal == False:
        v = fila.pop()
        if problem.isGoalState(v[0]):
            goal = True
            final_state = v[0]
            vertices_descobertos[v[0]] = v[1]
            break
        vertices_descobertos[v[0]] = v[1]

        for vertice_adjacente in problem.getSuccessors(v[0]):
            if vertice_adjacente[0] not in vertices_descobertos.keys():
                parents_graph[vertice_adjacente[0]] = v[0]
                fila.push(vertice_adjacente)


    # montando a lista de acoes:
    try:
        while(final_state in parents_graph.keys()):
            final_state_prev = parents_graph[final_state]
            acoes.insert(0, vertices_descobertos[final_state])
            final_state = final_state_prev
    except NameError:
        print "Caminho nao encontrado."
    return acoes


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
