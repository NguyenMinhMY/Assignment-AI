from functools import reduce
import numpy as np
from queue import LifoQueue
import Heap as hp
import time
import tracemalloc

################################################################################################
# This function finds a number in a given list, if that number does not exist, it will return -1 
################################################################################################
def Find(list, number):
    for i in range (0,len(list)):
        if list[i] == number:
            return i
    return -1
class NonogramProblem:
    def __init__(self,initial_state, row_constraints, col_constraints):
        self.initial = initial_state
        self.shape = self.initial.shape
        self.row_constraints = row_constraints
        self.col_constraints  = col_constraints
    def Result(self,state,floor,array_insert):
        new_state = state.copy()
        new_state[floor] = array_insert
        return new_state
    def PossibleArrays(self,floor):
        PossibleArrays_List = []
        if len(self.col_constraints[floor]) == 1:
            num_check = self.col_constraints[floor][0]
            for i in range(0, (self.shape[1] - num_check + 1)):
                new_arr = np.zeros(self.shape[1])
                for j in range (0,num_check):
                    new_arr[i+j] = 1
                PossibleArrays_List.append(new_arr)
        else:
            num1_check, num2_check = (num for num in self.col_constraints[floor])
            # Fix point_end and translate point_start to right
            new_arr = np.zeros(self.shape[1])
            n = self.shape[1] - (num1_check+num2_check+1)
            for i in range (0,n+1):
                temp1_arr = new_arr.copy()
                for j in range (0, num1_check):
                    temp1_arr[i+j] = 1
                for j in range (i + num1_check+1, self.shape[1] - num2_check + 1):
                    temp2_arr = temp1_arr.copy()
                    for k in range (0,num2_check):
                        temp2_arr[j+k] = 1
                    PossibleArrays_List.append(temp2_arr)

        return PossibleArrays_List
    def Goal_test(self,state):
        for i in range (0,self.shape[1]):
            col_check = list(state[:,i])
            nums_constraints = self.row_constraints[i]
            for num in nums_constraints:
                if (col_check == []) and num > 0:
                    return False
                first_idx = Find(col_check,1)
                if first_idx != -1 and first_idx + num <= len(col_check):
                    if reduce(lambda a,b: a+b,col_check[first_idx:first_idx+num]) == num :
                        if first_idx + num <len(col_check):
                            if col_check[first_idx+num] == 1:
                                return False
                            col_check = col_check[first_idx+num + 1:]
                        else:
                            col_check = []
                    else: 
                        return False
                else:
                    return False
        return True
    def Heuristic_function_cost(self,state):
        result = 0
        for i in range (0,self.shape[1]):
            nums_constraints = self.row_constraints[i]
            col_check = list(state[:,i])
            sum1 = reduce(lambda a,b: a+b , col_check)
            sum2 = reduce(lambda a,b: a+b,nums_constraints) 
            if sum1 > sum2:
                result -= 1000
            elif sum1 == sum2:
                flag = False
                for num in nums_constraints:
                    if (col_check == []) and num > 0:
                        result -= 1000
                        flag = True
                        break
                    first_idx = Find(col_check,1)
                    if first_idx != -1 and first_idx + num <= len(col_check):
                        if reduce(lambda a,b: a+b,col_check[first_idx:first_idx+num]) == num :
                            if first_idx + num <len(col_check):
                                if col_check[first_idx+num] == 1:
                                    result -= 1000
                                    flag = True
                                    break
                                col_check = col_check[first_idx+num + 1:]
                            else:
                                col_check = []
                        else: 
                            result -= 1000
                            flag = True
                            break
                    else:
                        result -= 1000
                        flag = True
                        break
                if flag == False:
                    result +=2
            else:
                if sum1 > 0:
                    for num in nums_constraints:
                        if col_check == [] :
                            break
                        first_idx  = Find(col_check,1)
                        if first_idx != -1 and first_idx + num <= len(col_check):
                            if reduce(lambda a,b: a+b,col_check[first_idx:first_idx+num]) == num :
                                if first_idx + num <len(col_check):
                                    if col_check[first_idx+num] == 1:
                                        result -= 1000
                                        col_check = col_check[first_idx+num + 1:] 
                                        break
                                    else:
                                        result += 1
                                        col_check = col_check[first_idx+num + 1:] 
                                       
                                else:
                                    col_check = []
                            else:
                                col_check = col_check[first_idx+num + 1:] 
                        else:
                            col_check = []
                            break
                    if col_check != []:
                        if reduce(lambda a,b: a+b, col_check) > 0:
                            result -= 1000
        return result

class Node:
    def __init__(self,state,total_cost = 0,parent = None,depth = 0):
        self.state = state
        self.parent = parent
        self.depth = depth
        self.total_cost = total_cost
    def Child_node(self,array_insert,problem):
        child_state = problem.Result(self.state, (self.depth + 1) -1,array_insert )
        child_depth = self.depth + 1
        child_total_cost = child_depth + problem.Heuristic_function_cost(child_state)
        return Node(child_state,child_total_cost,self,self.depth + 1)
    def Expand(self,problem):
        lst_successor = ()
        if self.depth == problem.shape[0]:
            return lst_successor
        PossibleArrays_List = problem.PossibleArrays(self.depth)
        for possible_array in PossibleArrays_List:
            lst_successor =(self.Child_node(possible_array,problem),) + lst_successor
        return lst_successor
    def Solution(self):
        node, solution = self, []
        while (node.parent):
            solution.append(node.state)
            node = node.parent
        return list(reversed(solution))

class NonogramSolver:
    def __init__(self, problem):
        self.history_DFS = ()
        self.history_A_star = ()

        # Start trace memory and calculator time for 2 algorithm
        self.mem_start_1 = tracemalloc.start()

        # DFS
        self.start_time_1 = time.time()
        self.goal_DFS = self.Depth_first_graph_search(problem)
        self.end_time_1 = time.time()
        self.mem_peak_1  = tracemalloc.get_traced_memory() 
        
        tracemalloc.clear_traces()
        # A*
        self.start_time_2 = time.time()
        self.goal_A_star = self.A_star(problem)
        self.end_time_2 = time.time()
        self.mem_peak_2 = tracemalloc.get_traced_memory()

    def Depth_first_graph_search(self,problem):
        stack_frontier = LifoQueue()
        stack_frontier.put(Node(problem.initial))
        while stack_frontier:
            node = stack_frontier.get()
            self.history_DFS+=(node.state,)
            if problem.Goal_test(node.state):
                return node
            Childs = node.Expand(problem)
            for child in Childs:
                stack_frontier.put(child)
        return None

    def A_star(self,problem):
        priority_frontier = hp.Max_Heap()
        priority_frontier.push(Node(problem.initial))
        while priority_frontier:
            node = priority_frontier.pop()
            self.history_A_star+=(node.state,)
            if problem.Goal_test(node.state):
                return node
            Childs = node.Expand(problem)
            for child in Childs:
                priority_frontier.push(child)
        return None

    def Steps_DFS(self):
        return len(self.history_DFS)

    def TimeExe_DFS(self):
        return self.end_time_1 - self.start_time_1
    
    def MemoryLoss_DFS(self):
        return self.mem_peak_1[1]

    def Steps_A_star(self):
        return len(self.history_A_star)

    def TimeExe_A_star(self):
        return self.end_time_2 - self.start_time_2

    def MemoryLoss_A_star(self):
        return self.mem_peak_2[1]

    def Show(self):
        if self.goal_DFS:
            print("Depth First Search Algorithm:\nSteps: ", self.Steps_DFS())
            print("Goal state:\n ",self.goal_DFS.state)
            print("Time DFS:",self.TimeExe_DFS())
            print("Memory: ", self.MemoryLoss_DFS())
        if self.goal_A_star:
            print("A* Algorithm:\nSteps: ", self.Steps_A_star())
            print("Goal state:\n",self.goal_A_star.state)
            print("Time A*:",self.TimeExe_A_star())
            print("Memory: ", self.MemoryLoss_A_star())