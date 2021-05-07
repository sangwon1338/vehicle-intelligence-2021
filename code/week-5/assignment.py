import numpy as np
import itertools

# Given map
grid = np.array([
    [1, 1, 1, 0, 0, 0],
    [1, 1, 1, 0, 1, 0],
    [0, 0, 0, 0, 0, 0],
    [1, 1, 1, 0, 1, 1],
    [1, 1, 1, 0, 1, 1]
])

# List of possible actions defined in terms of changes in
# the coordinates (y, x)
forward = [
    (-1,  0),   # Up
    ( 0, -1),   # Left
    ( 1,  0),   # Down
    ( 0,  1),   # Right
]

# Three actions are defined:
# - right turn & move forward
# - straight forward
# - left turn & move forward
# Note that each action transforms the orientation along the
# forward array defined above.
action = [-1, 0, 1]
action_name = ['R', '#', 'L']

init = (4, 3, 0)    # Representing (y, x, o), where
                    # o denotes the orientation as follows:
                    # 0: up
                    # 1: left
                    # 2: down
                    # 3: right
                    # Note that this order corresponds to forward above.
goal = (2, 0)
cost = (2, 1, 20)   # Cost for each action (right, straight, left)

# EXAMPLE OUTPUT:
# calling optimum_policy_2D with the given parameters should return
# [[' ', ' ', ' ', 'R', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', '#'],
#  ['*', '#', '#', '#', '#', 'R'],
#  [' ', ' ', ' ', '#', ' ', ' '],
#  [' ', ' ', ' ', '#', ' ', ' ']]

def optimum_policy_2D(grid, init, goal, cost):
    # Initialize the value function with (infeasibly) high costs.
    value = np.full((4, ) + grid.shape, 999, dtype=np.int32)
    # Initialize the policy function with negative (unused) values.
    policy = np.full((4,) + grid.shape, -1, dtype=np.int32)
    # Final path policy will be in 2D, instead of 3D.
    policy2D = np.full(grid.shape, ' ')

    # Apply dynamic programming with the flag change.
    change = True
    while change:
        change = False
        # This will provide a useful iterator for the state space.
        p = itertools.product(
            range(grid.shape[0]),
            range(grid.shape[1]),
            range(len(forward))
        )
        # Compute the value function for each state and
        # update policy function accordingly.
        for y, x, t in p:
            #print(y, ' ', x, ' ', t)
            # Mark the final state with a special value that we will
            # use in generating the final path policy.
            if (y, x) == goal and value[(t, y, x)] > 0:
       
                value[(t,y,x)] = 0
                # Final Marker --> -444
                policy[(t,y,x)] = -444
                change = True
            # Try to use simple arithmetic to capture state transitions.
            elif grid[(y, x)] == 0:

                for f_idx in range(len(forward)):
                    # get post position
                    x_tmp = x + forward[f_idx][1]
                    y_tmp = y + forward[f_idx][0]

                    # boundary check
                    # Be Careful -> len_x < len(grid[0])    --> 1 hour source bug ㅡㅡ
                    if x_tmp >= 0 and x_tmp < len(grid[0]) and y_tmp >= 0 and y_tmp < len(grid) and grid[y_tmp][x_tmp] == 0:
                        post_tmp = value[(f_idx, y_tmp, x_tmp)]
                        #print(post_tmp)

                        for act_idx in range(len(action)):
                            if (t + action[act_idx]) % len(forward) == f_idx:
                                v_tmp = post_tmp + cost[act_idx]

                                # cost coparison & change
                                if v_tmp < value[(t,y,x)]:
                                    value[(t,y,x)] = v_tmp
                                    policy[(t,y,x)] = action[act_idx]
                                    change = True

    # Now navigate through the policy table to generate a
    # sequence of actions to take to follow the optimal path.
   

  

    # init position & orientation
    y = init[0]
    x = init[1]
    f = init[2]

    if policy[(f,y,x)] == -1:
        policy2D[(y,x)] = action_name[0]
    elif policy[(f,y,x)] == 0:
        policy2D[(y,x)] = action_name[1]
    elif policy[(f,y,x)] == 1:
        policy2D[(y,x)] = action_name[2]
    else:
        policy2D[(y,x)] = "*"

    # visualization
    while policy[(f,y,x)] != -444:
        if policy[(f,y,x)] == -1:
            f = (f - 1)%4
        elif policy[(f,y,x)] == 1:
            f = (f + 1)%4

        x += forward[f][1]
        y += forward[f][0]

        if policy[(f,y,x)] == -1:
            policy2D[(y,x)] = action_name[0]
        elif policy[(f,y,x)] == 0:
            policy2D[(y,x)] = action_name[1]
        elif policy[(f,y,x)] == 1:
            policy2D[(y,x)] = action_name[2]
        else:
            # final state is visualized as "*" star marker
            policy2D[(y,x)] = "*"

    # Return the optimum policy generated above.
    return policy2D

print(optimum_policy_2D(grid, init, goal, cost))
