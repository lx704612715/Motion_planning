import pickle
from racetracks import *
import time
from graph_node import Node
import matplotlib.pyplot as plt
from shutil import copyfile

seed = np.random.seed(1234)
graph = {}

def build_up_graph(grid, save_path):
    max_vel = 5

    # velocity dimension
    vel_list = []
    for i_vel in range(-max_vel+1, max_vel):
        for j_vel in range(-max_vel+1, max_vel):
            vel_list.append([i_vel, j_vel])

    # position dimension
    x_idx, y_idx = np.where(grid == FREE)
    coord = np.stack([x_idx, y_idx], axis=1)
    for p_idx in range(coord.shape[0]):
        pnt = coord[p_idx]
        for vel in vel_list:
            state = Node(pnt[0], pnt[1], vel[0], vel[1])
            state.connect_to_graph(grid)
            graph[state.key] = state

    for pnt in START_LINE:
        state = Node(pnt[0], pnt[1], 0, 0)
        state.connect_to_graph(grid)
        graph[state.key] = state

    for pnt in FINISH_LINE:
        state = Node(pnt[0], pnt[1], 0, 0)
        state.is_goal = True
        graph[state.key] = state

    output = open(save_path, 'wb')
    pickle.dump(graph, output)

def check_graph(grid):
    plt.figure(figsize=(4.5, 16))
    plt.pcolor(grid, edgecolors='k', linewidths=1)
    for key in graph.keys():
        for child_idx, child_key in enumerate(graph[key].next_prob_1): # or next_prob_1
            ux, uy = ACTION_SPACE[child_idx]
            vx, vy = graph[key].vx + ux,  graph[key].vy + uy
            child = graph[child_key]
            # check a specific connection
            # plt.title(str(vy) + '_' + str(vx))
            # plt.show()
            if [child.px, child.py] in START_LINE:
                print('found')
                continue
            plt.arrow(graph[key].py + 0.5, graph[key].px + 0.5,
                      child.py - graph[key].py, child.px - graph[key].px,
                      color='r', head_width=0.3, head_length=0.1)
            print(key, child_idx)
        # end for
    # end for
    plt.show()

def track_the_best_plan(idx = 1):
    start_node = Node(START_LINE[idx][0], START_LINE[idx][1], 0, 0)
    start_key = start_node.key
    state = graph[start_key]
    trajectory = [state]
    # for i in range(grid.shape[0]+grid.shape[1]) a safer condition
    while not state.is_goal:
        value_uk = []
        for child_idx in range(len(ACTION_SPACE)):
            child_key_9 = state.next_prob_9[child_idx]
            child_9 = graph[child_key_9]
            value_uk.append(child_9.g_value)
        child_key = state.next_prob_9[np.argmin(value_uk)]
        state = graph[child_key]
        trajectory.append(state)
        print(state.px, state.py)
    return trajectory

def visualize_the_best_plan(plan, grid_para):
    assert isinstance(plan, list)
    plt.figure(figsize=(4.5, 16))
    plt.pcolor(grid_para, edgecolors='k', linewidths=1)
    plan_len = len(plan)
    plan.append(plan[-1])
    for i in range(plan_len):
        plt.arrow(plan[i].py + 0.5, plan[i].px + 0.5,
                  plan[i+1].py - plan[i].py, plan[i+1].px - plan[i].px,
                  color='r', head_width=0.3, head_length=0.1)
    plt.show()

def initialize_graph_value(goal_position, current_node, dist_thre=15, dist_factor=7, vel_factor=0.005):
    current_position = np.asarray([current_node.px, current_node.py])
    distance = np.linalg.norm(goal_position-current_position)
    velocity = np.linalg.norm(current_node.vx+current_node.vy)
    if distance >= dist_thre:
        value = distance/dist_factor - vel_factor*velocity
    else:
        value = distance/dist_factor - vel_factor*velocity/10
    # value = distance/dist_factor
    return value

def generate_trajectory(iter_num, idx=1):
    start_node = Node(START_LINE[idx][0], START_LINE[idx][1], 0, 0)
    start_key = start_node.key
    state = graph[start_key]
    goal_position = np.asarray(FINISH_LINE[idx])
    trajectory = [state]

    while not state.is_goal:
        value_uk = []
        for child_idx in range(len(ACTION_SPACE)):
            child_key_9 = state.next_prob_9[child_idx]
            child_9 = graph[child_key_9]
            if child_9.g_value == 0:
                child_9.g_value = initialize_graph_value(goal_position, child_9)
            value_uk.append(child_9.g_value)
        random_prob = 0.4*np.exp(-iter_num*0.01)
        if np.random.choice([0,1],p=[1-random_prob,random_prob]):
            child_key = state.next_prob_9[np.random.choice(len(value_uk))]
        else:
            child_key = state.next_prob_9[np.argmin(value_uk)]
        state = graph[child_key]
        trajectory.append(state)

    trajectory.reverse()
    return trajectory

# def Real_time_dynamic_programming():
    """ update value without concern about action space
    """
#     itr_num = 0
#     bellman_error = np.inf
#     bellman_error_list = []
#
#     while bellman_error > 0.0001:
#         itr_num += 1
#         bellman_error = 0.0
#         print("wanne a new trajectory")
#         trajectory = generate_trajectory(itr_num)
#         print("created a trajectory")
#         goal_state = trajectory[0]
#         goal_state.g_value = 0
#         for i in range(len(trajectory)-1):
#             state = trajectory[i+1]
#             next_state = trajectory[i]
#             expected_cost_uk = 0.9*(1+next_state.g_value) + 0.1*(1+state.g_value)
#             bellman_error += np.linalg.norm(state.g_value - expected_cost_uk)
#             state.g_value = expected_cost_uk
#         bellman_error_list.append(bellman_error)
#         print("{}th iteration: {}".format(itr_num, bellman_error))
#
#     plt.figure()
#     x_axis = range(len(bellman_error_list))
#     plt.plot(x_axis, bellman_error_list)
#     plt.show()

def Real_time_dynamic_programming():
    """ choose min value while update states along trajectories
    """
    itr_num = 0
    bellman_error = np.inf
    bellman_error_list = []

    while bellman_error > 0.0001:
        itr_num += 1
        bellman_error = 0.0
        trajectory = generate_trajectory(itr_num)
        trajectory.reverse()
        goal_position = np.asarray(FINISH_LINE[1])
        for i in range(len(trajectory)):
            state = trajectory[i]
            if state.is_goal == True:
                state.g_value = 0
            else:
                value_uk = []
                for child_idx in range(len(ACTION_SPACE)):
                    child_key_9 = state.next_prob_9[child_idx]
                    child_9 = graph[child_key_9]
                    if child_9.g_value == 0:
                        child_9.g_value = initialize_graph_value(goal_position, child_9)
                    expected_cost_uk = 0.9 * (1 + child_9.g_value) + 0.1 * (1 + state.g_value)
                    value_uk.append(expected_cost_uk)
                current_value = min(value_uk)
                bellman_error += np.linalg.norm(state.g_value - current_value)
                state.g_value = min(value_uk)
        bellman_error_list.append(bellman_error)
        print("{}th iteration: {}".format(itr_num, bellman_error))

    plt.figure()
    x_axis = range(len(bellman_error_list))
    plt.plot(x_axis, bellman_error_list)
    plt.show()

def dynamic_programming():
    itr_num = 0
    bellman_error = np.inf
    bellman_error_list = []
    while bellman_error > 0.0001:
        itr_num += 1
        bellman_error = 0.0
        for key in graph.keys():
            state = graph[key]
            if state.is_goal:
                state.g_value = 0
            else:
                value_uk = []
                for child_idx in range(len(ACTION_SPACE)):
                    child_key_9 = state.next_prob_9[child_idx]
                    child_9 = graph[child_key_9]
                    child_key_1 = state.next_prob_1[child_idx]
                    child_1 = graph[child_key_1]
                    expected_cost_uk = 0.9 * (1 + child_9.g_value) + 0.1 * (1 + child_1.g_value)
                    value_uk.append(expected_cost_uk)
                current_value = min(value_uk)
                bellman_error += np.linalg.norm(state.g_value - current_value)
                state.g_value = min(value_uk)
            # end if
        # end for
        bellman_error_list.append(bellman_error)
        print("{}th iteration: {}".format(itr_num, bellman_error))
    # end while

    plt.figure()
    x_axis = range(len(bellman_error_list))
    plt.plot(x_axis, bellman_error_list)
    plt.show()



if __name__ == '__main__':
    path = './solution/graph_dp.dat'
    track_map = race_track
    build_up_graph(track_map, path)
    graph = pickle.load(open(path, 'rb'))

    # solve
    start = time.time()
    Real_time_dynamic_programming()
    # dynamic_programming()
    end = time.time()
    print("running time is ", end - start)
    plan = track_the_best_plan()
    visualize_the_best_plan(plan, track_map)
