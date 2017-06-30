#!/usr/bin/python
import sys

# +--------------------------------------+
# | 4puzzle_bfs.py                       |
# | Developed with Python 2.7            |
# | Example code to demonstrate the      |
# | Depth First Search (DFS) algorithm   |
# + -------------------------------------+

# General variables and constants
log = []   # Historic states
steps = 0  # Number of steps

MOVE_LEFT = "L"
MOVE_RIGHT = "R"
MOVE_UP = "U"
MOVE_DOWN = "D"


# Class: Node
# Properties:
#    - parent: Parent node
#    - state: State of the node
#    - move: Movement from the parent to this ('L', 'R', 'U', 'D')
#    - depth: Depth level in the nodes tree
#    - cost: Value cost
# Note: This class stores the values of each node of the tree
class Node:
    def __init__(self, parent, state, move, depth, cost):
        self.parent = parent
        self.state = state
        self.move = move
        self.depth = depth
        self.cost = cost


# Method: paint_board()
# Goal: Paint the board with a given state
#       This method is used for visual trace only
# Params:
#    - state: State to be represented
def paint_board(state):
    print("")
    print("+---+---+")
    print("| {} | {} |".format(state[0], state[1]))
    print("+---+---+")
    print("| {} | {} |".format(state[2], state[3]))
    print("+---+---+")


# Method: get_next_state()
# Goal: Given a state and a movement, get the next state.
#       The next state is filtered if this one is already present in the log
#       This method helps to build the children nodes
# Params:
#    - state: State form which get the next state(for children node)
#    - move: 'L' (left), 'R' (right), 'U' (up) or 'D' (down)
# Returns:
#    - "" (empty string) if no state is possible
#    - String with the next state from "state" param and "move" direction
def get_next_state(state, move):
    tmp = list(state)
    idx = state.find('0', 0)
    pos = 123   # default 'dummie' position

    # Get the position for swap
    if move == MOVE_LEFT:
        if not idx in [0, 2]:
            pos = idx - 1
    elif move == MOVE_RIGHT:
        if not idx in [1, 3]:
            pos = idx + 1
    elif move == MOVE_UP:
        if not idx in [0,1]:
            pos = idx - 2
    else:
        if not idx in [2, 3]:
            pos = idx + 2

    # If no position (is still 'dummie')
    if pos == 123:
        return ""  # In this case, returns an empty string

    # Swap characters between idx and pos positions
    tmp[idx] = tmp[pos]
    tmp[pos] = '0'

    # Get the provisional state
    next_state = "".join(tmp)

    # Check if the provisional state has appeared in the historic states
    if next_state in log:
        next_state = ""  # In this case, next_state is an empty string

    return next_state


# Method: get_children_nodes()
# Goal: Obtain the children nodes from the current node
#       The children nodes are filtered:
#          - No possible nodes (no moves are possible (limits))
#          - No repeated nodes (already present in the log (historic states))
# Params:
#    - node: Node from which obtain the new children nodes
# Returns:
#    - Array with the valid children nodes
def get_children_nodes(node):
    children = []
    children.append(Node(node, get_next_state(node.state, MOVE_LEFT), MOVE_LEFT, node.depth + 1, 0))
    children.append(Node(node, get_next_state(node.state, MOVE_RIGHT), MOVE_RIGHT, node.depth + 1, 0))
    children.append(Node(node, get_next_state(node.state, MOVE_UP), MOVE_UP, node.depth + 1, 0))
    children.append(Node(node, get_next_state(node.state, MOVE_DOWN), MOVE_DOWN, node.depth + 1, 0))

    # Filter the children nodes with no result or limited
    children = [node for node in children if node.state != ""]

    # Append the new children to the log (historic states) in order to avoid repeat them
    for child in children:
        log.append(child.state)

    return children


# Method: dfs_agent()
# Goal: Apply the Depth First Search (DFS) algorithm
#       From an initial state, generates a tree of possible movements
#       until to reach the goal state, avoiding to repeat the states
#       This algorithm expand nodes in depth first (vertical), and after breath (horizontal)
# Params:
#    - start: Initial state (string format)
#    - goal: Goal state (string format)
# Returns:
#    - None: The search can't find the goal
#    - [None]: The initial state and goal state are identical
#    - [moves]: List of the moves with the path for find the goal
def dfs_agent(start, goal, depth_limit=25):
    global steps, log
    # Stack for the nodes.
    stack_nodes = []

    # Create the stack initializing it with root node
    stack_nodes.append(Node(None, start, None, 0, 0))

    # Initialize the log (historic states) with the initial state
    log.append(start)


    while True:
        # If queue of nodes is empty, no solution found
        if len(stack_nodes) == 0:
            return None

        # Get the next node from the queue
        current_node = stack_nodes.pop(0)
        steps += 1

        # Uncomment the next two lines for trace purposes
        paint_board(current_node.state)
        print("Step: {} >>> depth: {} - move: {} - state: {}"
              .format(steps, current_node.depth, current_node.move, current_node.state))

        # If goal is reached, generate the moves list and exit
        if current_node.state == goal:
            moves = []
            tmp = current_node

            while True:
                moves.insert(0, tmp.move)
                if tmp.depth <= 1:
                    break

                tmp = tmp.parent

            return moves

        # Add all the expansions to the beginning of the stack if we are under the depth limit
        if current_node.depth <= depth_limit:
            extended_nodes = get_children_nodes(current_node)
            extended_nodes.extend(stack_nodes)
            stack_nodes = extended_nodes


# Main method
def main():
    # Configure the search
    # Example:
    #    - start: 0231, 0312, 1023, 1302, 1320, 2031, 2103, 2130, 3012, 3201, 3210 for
    #      goal=0123
    start = "2310"
    goal = "0123"

    print("Executing BFS agent. This take a long time. Please wait...")

    result = dfs_agent(start, goal)

    print("----------")

    if result == None:
        print "The puzzle can't be resolved"
    elif result == [None]:
        print "Start node and goal node are identical"
    else:
        print("Total steps: {}".format(steps))
        print("Moves: {} - Total: {} moves".format(result, len(result)))


# Start automatically executing the main() method
if __name__ == "__main__":
    main()