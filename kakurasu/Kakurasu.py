from functools import reduce
import numpy as np
from queue import LifoQueue
import time
import Heap as hp
class KakurasuProblem:
    def __init__(self, initial_state, row_constraints, col_constraints, dimention):
        self.initial_state = initial_state
        self.row_constraints = row_constraints
        self.col_constraints = col_constraints
        self.dimention = dimention

    def subset_sum(self, numbers, target, list, partial=[]):
        s = sum(partial)

        if s == target:
            list.append(partial)
        if s >= target:
            return

        for i in range(len(numbers)):
            n = numbers[i]
            remaining = numbers[i+1:]
            self.subset_sum(remaining, target, list, partial + [n])

    def PossibleArray(self, floor):
        Possible_Pos_Arr = []
        Possible_combinations = []
        self.subset_sum(list(i for i in range(1, self.dimention+1)), self.col_constraints[floor][0],
                        Possible_combinations)
        for i in range(0, len(Possible_combinations)):
            new_arr = np.zeros(self.dimention)
            for j in range(0, len(Possible_combinations[i])):
                new_arr[Possible_combinations[i][j]-1] = 1
            Possible_Pos_Arr.append(new_arr)
        return Possible_Pos_Arr

    def Goal_test(self, state):
        flag = 0
        for i in range(0, self.dimention):
            check_sum = 0
            col_check = list(state[:, i])
            for j in range(0, self.dimention):
                if (col_check[j] == 1):
                    check_sum += j+1
            if (check_sum == self.row_constraints[i][0]):
                flag += 1
        if (flag == self.dimention):
            return True
        else:
            return False

    def Change_state(self, state, floor, array_insert):
        new_state = state.copy()
        new_state[floor] = array_insert
        return new_state

    def Heuristic_function_cost(self, state):
        total_cost = 0
        for i in range(0, self.dimention):
            sum = 0
            for j in range(0, self.dimention):
                if state[j][i] == 1:
                    total_cost += j+1
                    sum += j+1
            if sum > self.row_constraints[i][0]:
                total_cost -= 100
        return total_cost
import tracemalloc

class Node:
    def __init__(self, state, total_cost=0, parent=None, depth=0):
        self.state = state
        self.parent = parent
        self.depth = depth
        self.total_cost = total_cost

    def Child_node(self, array_insert, problem):
        child_state = problem.Change_state(
            self.state, self.depth, array_insert)
        child_depth = self.depth + 1
        result = problem.Heuristic_function_cost(child_state)
        child_total_cost = child_depth + result
        return Node(child_state, child_total_cost, self, self.depth + 1)

    def Expand(self, problem):
        lst_successor = ()
        if self.depth == problem.dimention:
            return lst_successor
        PossibleArrays_List = problem.PossibleArray(self.depth)
        for possible_array in PossibleArrays_List:
            lst_successor = (self.Child_node(possible_array, problem),) + lst_successor
        return lst_successor

    def Solution(self):
        node, solution = self, []
        while (node.parent):
            solution.append(node.state)
            node = node.parent
        return list(reversed(solution))

class KakurasuSolver:
    def __init__(self, problem):

        self.mem_start_1 = tracemalloc.start()
        self.history_DFS = []
        self.start_time_1 = time.time()
        self.goal_node_DFS = self.Depth_first_graph_search(problem)
        self.end_time_1 = time.time()
        self.mem_peak_1 = tracemalloc.get_traced_memory()
        tracemalloc.clear_traces()

        self.history_A = []
        # self.mem_start_2 = tracemalloc.start()
        self.start_time_2 = time.time()
        self.goal_node_A = self.A_star(problem)
        self.end_time_2 = time.time()
        self.mem_peak_2 = tracemalloc.get_traced_memory()

    def Depth_first_graph_search(self, problem):
        stack_frontier = LifoQueue()
        stack_frontier.put(Node(problem.initial_state))
        self.explored1 = ()
        self.step1 = 0
        while stack_frontier:
            node = stack_frontier.get()
            self.history_DFS.append(node.state)
            self.step1 += 1
            self.explored1 += (node.state,)
            if problem.Goal_test(node.state):
                return node
            Childs = node.Expand(problem)
            for child in Childs:
                stack_frontier.put(child)
        return None

    def A_star(self, problem):
        priority_frontier = hp.Max_Heap()
        priority_frontier.push(Node(problem.initial_state))
        self.explored2 = ()
        self.step2 = 0
        while priority_frontier:
            node = priority_frontier.pop()
            self.history_A.append(node.state)
            self.step2 += 1
            self.explored2 += (node.state,)
            if problem.Goal_test(node.state):
                return node
            Childs = node.Expand(problem)
            for child in Childs:
                priority_frontier.push(child)
        return None

    def TimeExe_DFS(self):
        return self.end_time_1 - self.start_time_1

    def TimeExe_A_star(self):
        return self.end_time_2 - self.start_time_2

    def MemoryLoss_DFS(self):
        return self.mem_peak_1[1]

    def MemoryLoss_A_star(self):
        return self.mem_peak_2[1]

    def Show(self):
        if self.goal_node_A and self.goal_node_DFS:
            print(self.goal_node_A.state)
            print(self.goal_node_DFS.state)
            print("DFS: ", self.step1)
            print("A_star:", self.step2)
            print("Time DFS: ", self.TimeExe_DFS())
            print("Time A*: ", self.TimeExe_A_star())
            print("Memory DFS: ", self.MemoryLoss_DFS())
            print("Memory A*: ", self.MemoryLoss_A_star())
        else:
            print("No Solution!!!!")