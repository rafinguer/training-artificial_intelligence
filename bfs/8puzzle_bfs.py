#!/usr/bin/python
import sys
import datetime
#import resource

# +--------------------------------------+
# | 8puzzle_bfs.py                       |
# | Developed with Python 2.7            |
# | Example code to demonstrate the      |
# | Breadth First Search (BFS) algorithm |
# + -------------------------------------+

# General variables and constants
log = []   # Historic states
nodes_expanded = 0  # Number of nodes expanded (or steps)
cost_of_path = 0  # Number of nodes for path
max_search_depth = 0   # Maximum search depth level
search_depth = 0  # Depth level in which the goal is present
max_ram_usage = 0.0

MOVE_LEFT = "L"
MOVE_RIGHT = "R"
MOVE_UP = "U"
MOVE_DOWN = "D"

movements = {"L":"Left", "R":"Right", "U":"Up", "D":"Down"}


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
    print("+---+---+---+")
    print("| {} | {} | {} |".format(state[0], state[1], state[2]))
    print("+---+---+---+")
    print("| {} | {} | {} |".format(state[3], state[4], state[5]))
    print("+---+---+---+")
    print("| {} | {} | {} |".format(state[6], state[7], state[8]))
    print("+---+---+---+")


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
        if not idx in [0, 3, 6]:
            pos = idx - 1
    elif move == MOVE_RIGHT:
        if not idx in [2, 5, 8]:
            pos = idx + 1
    elif move == MOVE_UP:
        if not idx in [0, 1, 2]:
            pos = idx - 3
    else:
        if not idx in [6, 7, 8]:
            pos = idx + 3

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


# Method: bfs_agent()
# Goal: Apply the Breadth First Search (BFS) algorithm
#       From an initial state, generates a tree of possible movements
#       until to reach the goal state, avoiding to repeat the states
#       This algorithm expand nodes in breadth first (horizontal), and after depth (vertical)
# Params:
#    - start: Initial state (string format)
#    - goal: Goal state (string format)
# Returns:
#    - None: The search can't find the goal
#    - [None]: The initial state and goal state are identical
#    - [moves]: List of the moves with the path for find the goal
def bfs_agent(start, goal):
    global nodes_expanded, search_depth, max_search_depth, max_ram_usage, log

    # Queue of nodes
    queue_nodes = []

    # Create the queue initializing it with root node
    queue_nodes.append(Node(None, start, None, 0, 0))

    # Initialize the log (historic states) with the initial state
    log.append(start)

    while True:
        # If queue of nodes is empty, no solution found
        if len(queue_nodes) == 0:
            return None

        # Calculate RAM usage
        ram_usage = sys.getsizeof(queue_nodes)
        # ram_usage = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss

        if ram_usage > max_ram_usage:
           max_ram_usage = ram_usage

        # Get a node from the queue
        current_node = queue_nodes.pop(0)
        nodes_expanded += 1

        if current_node.depth > max_search_depth:
            max_search_depth = current_node.depth

        # Uncomment the next two lines for trace purposes
        # paint_board(current_node.state)
        # print("Step: {} >>> depth: {} - move: {} - state: {}"
        #      .format(nodes_expanded, current_node.depth, current_node.move, current_node.state))

        # If goal is reached, generate the moves list and exit
        if current_node.state == goal:
            path_to_goal = []
            tmp = current_node
            search_depth = current_node.depth

            # Go across the path in order inverse, taking the node parent
            # from the last node to the root one
            while True:
                path_to_goal.insert(0, movements[tmp.move])

                if tmp.depth <= 1:
                    break

                tmp = tmp.parent

            # Exit with the path or moves list
            return path_to_goal

        # Extends the node queue with the children of the current node
        queue_nodes.extend(get_children_nodes(current_node))


# Main method
def main():
    # Configure the search
    # Example: Start: 120534867 - goal=012345678
    start = "120534867"
    # start = "520134678"
    # start = "120345678"
    # start = "123054678"
    goal = "012345678"

    print("Executing BFS agent. This operation could take a long time. Please wait...")

    starting_time = datetime.datetime.now()

    path_to_goal = bfs_agent(start, goal)

    end_time = datetime.datetime.now()
    running_time = float((end_time - starting_time).seconds)

    mb_ram = float(0)
    cost_of_path = 0

    print("----------")

    if path_to_goal is None:
        print "The puzzle can't be resolved"
    elif path_to_goal == [None]:
        print "Start node and goal node are identical"
    else:
        cost_of_path = len(path_to_goal)
        mb_ram = (float(max_ram_usage)/1024)/1024

    print("path_to_goal: {}".format(path_to_goal))
    print("cost_of_path: {}".format(cost_of_path))
    print("nodes_expanded: {}".format(nodes_expanded))
    print("search_depth: {}".format(search_depth))
    print("max_search_depth: {}".format(max_search_depth))
    print("running_time: {} seconds ({}-{})".
          format(running_time,
                 starting_time.strftime('%H:%M:%S'),
                 end_time.strftime('%H:%M:%S')))
    print("max_ram_usage: {0:.8}MB".format(mb_ram))



# Start automatically executing the main() method
if __name__ == "__main__":
    main()