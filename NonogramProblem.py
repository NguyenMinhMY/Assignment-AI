from functools import reduce
import numpy as np
import sys
from collections import deque
def compare(matrix_A, matrix_B):
    if(matrix_A.shape == matrix_B.shape):
        shape = matrix_A.shape
        for i in range (0,shape[0]):
            for j in range (0,shape[1]):
                if matrix_A[i][j] != matrix_B[i][j]:
                    return False 
        return True
    return False
class NonogramProblem:
    def __init__(self,initial_state, horizontal_list, vertical_list):
        self.initial = initial_state
        self.hor_lst = horizontal_list
        self.ver_lst  = vertical_list
        self.step_cost = 1
    def Possible_coordinates(self,state):
        lst_coordinates = []
        for i in range (0,4):
            for j in range (0,4):
                nums_x = reduce(lambda a,b: a+b,list(state[i]))
                nums_y = reduce(lambda a,b: a+b,list(state[:,j]))
                if nums_x < reduce(lambda a,b: a+b,self.ver_lst[i]) and nums_y < reduce(lambda a,b: a+b, self.hor_lst[j] or state[i][j] == 1):
                    lst_coordinates.append((i,j))
        return lst_coordinates    
    def Result(self,state,coordinate):
        temp_state = state.copy()
        x,y = coordinate[0],coordinate[1]
        temp_state[x][y] = 1 - temp_state[x][y]
        return temp_state  
    def Step_cost(self, cur_state, next_state):
        return self.step_cost
    def Path_cost(self, cur_state, cur_cost, next_state):
        return cur_cost + self.Step_cost(cur_state,next_state)
    def Goal_test(self,state):
        goal = [[1,1,0,0],
                [1,1,0,0],
                [0,0,1,0],
                [0,0,0,1]]
        
        return (goal==list(list(state[i]) for i in range (0,4)))

class Node:
    def __init__(self,state,parent = None,coordinate = None,cost = 0):
        self.state = state
        self.parent = parent
        self.coordinate = coordinate
        self.cost = cost
        if (parent):
            self.depth = parent.depth
        else : 
            self.depth = 0
    def Child_node(self,coordinate,problem):
        child_state = problem.Result(self.state,coordinate)
        child_parent = self
        child_cost = self.cost + problem.Step_cost(self.state, child_state)

        return Node(child_state,self,coordinate,child_cost)
    def Expand(self,problem):
        lst_successor = []
        possible_coordinate = problem.Possible_coordinates(self.state)
        for coordinate in possible_coordinate:
            lst_successor.append(self.Child_node(coordinate,problem))
        return lst_successor
    def Solution(self):
        node, solution = self, []
        while (node.parent):
            solution.append(node.coordinate)
            node = node.parent
        return list(reversed(solution))
class NonogramSolver:
    def __init__(self, problem):
        # self.solution = self.breadth_first_graph_search(problem).Solution()
        self.goal_state = self.breadth_first_graph_search(problem).state
    def breadth_first_graph_search(self,problem):
        node = Node(problem.initial)
        if problem.Goal_test(node.state):
            return node
        frontier = deque([node])
        explored = ()
        while frontier:
            node = frontier.popleft()
            explored+= (node.state,)
            Childs = node.Expand(problem)
            for child in Childs:
                flag = False
                for ex_state in explored:
                    if compare(child.state,ex_state):
                        flag =True
                        break
                if flag == False and child not in frontier:
                    if problem.Goal_test(child.state):
                        return child
                    frontier.append(child)
        return None
    def ShowBoard(self):
        print(self.goal_state)
if __name__ == '__main__':
    board = np.zeros((4,4))
    hr = [[2],[2],[1],[1]]
    vr = [[2],[2],[1],[1]]
    problem = NonogramProblem(board,hr,vr)
    solving = NonogramSolver(problem)
    solving.ShowBoard()





