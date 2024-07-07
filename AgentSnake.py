
class Agent(object):
	def SearchSolution(self, state):

		return []
		
# class AgentSnake(Agent):    
# 	def SearchSolution(self, state):
# 		FoodX = state.FoodPosition.X
# 		FoodY = state.FoodPosition.Y

# 		HeadX = state.snake.HeadPosition.X #L
# 		HeadY = state.snake.HeadPosition.Y #T
		
# 		DR = FoodY - HeadY
# 		DC = FoodX - HeadX
		
# 		plan = []
		
# 		F = -1
# 		if(DR == 0 and state.snake.HeadDirection.X*DC < 0):    # if snake and food row is same, and snake's diretion x*column < 0 
# 			plan.append(0)
# 			F = 6
			
# 		if(state.snake.HeadDirection.Y*DR < 0):
# 			plan.append(3)
# 			if(DC == 0):
# 				F = 9
# 			else:
# 				DC = DC - 1
# 		Di = 6
# 		if(DR < 0):
# 			Di = 0
# 			DR = -DR
# 		for i in range(0,int(DR)):
# 			plan.append(Di)
# 		Di = 3
# 		if(DC < 0):
# 			Di = 9
# 			DC = -DC
# 		for i in range(0,int(DC)):
# 			plan.append(Di)
# 		if(F > 0):
# 			plan.append(F)
# 			F = -1

# 		return plan
	
# 	def showAgent():
# 		print("A Snake Solver By MB")
		
import heapq

'''
This agent class will return path using greedy best first search algorithm, which only use h(n).
'''
class AgentSnakeGFS(Agent):
    def SearchSolution(self, state):
        open_list = []
        closed_set = set()
        start_node = Node((state.snake.HeadPosition.X, state.snake.HeadPosition.Y), 0, 0)
        food_position = (state.FoodPosition.X, state.FoodPosition.Y)

        heapq.heappush(open_list, (self.heuristic(start_node.position, food_position), start_node))

        while open_list:
            current_node = heapq.heappop(open_list)[1]

            if current_node.position == food_position:
                return self.reconstruct_path(current_node)

            closed_set.add(current_node.position)

            for neighbor in self.get_neighbors(current_node.position, state):
                if neighbor in closed_set:
                    continue

                h_n_cost = self.heuristic(neighbor, food_position)
                child_node = Node(neighbor, 0, h_n_cost, current_node)

                heapq.heappush(open_list, (h_n_cost, child_node))

        return []

    def get_neighbors(self, position, state):
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        neighbors = []
        for direction in directions:
            new_position = (position[0] + direction[0], position[1] + direction[1])
            if self.is_valid_position(new_position, state):
                neighbors.append(new_position)
        return neighbors

    def is_valid_position(self, position, state):
        if 0 <= position[0] < state.maze.WIDTH and 0 <= position[1] < state.maze.HEIGHT:
            if state.maze.MAP[position[1]][position[0]] != -1:
                return True
        return False

    def heuristic(self, current, goal):
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

    def reconstruct_path(self, node):
        path = []
        while node.parent:
            prev_pos = node.parent.position
            curr_pos = node.position
            if prev_pos[0] - curr_pos[0] == 1:
                path.append(9)  # Left
            elif prev_pos[0] - curr_pos[0] == -1:
                path.append(3)  # Right
            elif prev_pos[1] - curr_pos[1] == 1:
                path.append(0)  # Up
            elif prev_pos[1] - curr_pos[1] == -1:
                path.append(6)  # Down
            node = node.parent
        path.reverse()
        return path

class Node:
    def __init__(self, position, g_n_cost, h_n_cost, parent=None):
        self.position = position
        self.g_n_cost = g_n_cost
        self.h_n_cost = h_n_cost
        self.parent = parent

    def f_cost(self):
        return self.g_n_cost + self.h_n_cost

    def __lt__(self, other):
        return self.f_cost() < other.f_cost()


'''
This agent class will return path using A* star algorithm based on g(n) + h(n).
'''
class AgentSnakeAStar(Agent):
    def SearchSolution(self, state):
        start_node = Node((state.snake.HeadPosition.X, state.snake.HeadPosition.Y), 0, 0)
        food_position = (state.FoodPosition.X, state.FoodPosition.Y)

        visited_nodes = []
        closed_set = set()

        heapq.heappush(visited_nodes, (start_node.f_cost(), start_node))

        while visited_nodes:
            current_node = heapq.heappop(visited_nodes)[1]

            if current_node.position == food_position:
                return self.reconstruct_path(current_node)

            closed_set.add(current_node.position)

            for neighbor in self.get_neighbors(current_node.position, state):
                if neighbor in closed_set:
                    continue

                g_n_cost = current_node.g_n_cost + 1
                h_n_cost = self.heuristic(neighbor, food_position)
                child_node = Node(neighbor, g_n_cost, h_n_cost, current_node)

                heapq.heappush(visited_nodes, (child_node.f_cost(), child_node))

        return []

    def get_neighbors(self, position, state):
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        neighbors = []
        for direction in directions:
            new_position = (position[0] + direction[0], position[1] + direction[1])
            if self.is_valid_position(new_position, state):
                neighbors.append(new_position)
        return neighbors

    def is_valid_position(self, position, state):
        if 0 <= position[0] < state.maze.WIDTH and 0 <= position[1] < state.maze.HEIGHT:
            if state.maze.MAP[position[1]][position[0]] != -1:
                return True
        return False

    def heuristic(self, current, goal):
        return abs(current[0] - goal[0]) + abs(current[1] - goal[1])

    def reconstruct_path(self, node):
        path = []
        while node.parent:
            prev_pos = node.parent.position
            curr_pos = node.position
            if prev_pos[0] - curr_pos[0] == 1:
                path.append(9)  # Left
            elif prev_pos[0] - curr_pos[0] == -1:
                path.append(3)  # Right
            elif prev_pos[1] - curr_pos[1] == 1:
                path.append(0)  # Up
            elif prev_pos[1] - curr_pos[1] == -1:
                path.append(6)  # Down
            node = node.parent
        path.reverse()
        return path
		
# #You code of agent goes here
# # You must create three agents one using A*, second using greedy best first search and third using an uninformed algo of your choice to make a plan

from collections import deque

'''
This agent class will return path using breadth first search algorithm.
'''
class AgentSnakeBFS(Agent):
    def SearchSolution(self, state):
        start_node = (state.snake.HeadPosition.X, state.snake.HeadPosition.Y)
        queue = deque([(start_node, [])])
        visited = set()
        food_position = (state.FoodPosition.X, state.FoodPosition.Y)

        while queue:
            current_position, current_path = queue.popleft()
            if current_position == food_position:
                return current_path

            if current_position in visited:
                continue
            visited.add(current_position)

            for neighbor in self.get_neighbors(current_position, state):
                queue.append((neighbor, current_path + [self.get_direction(current_position, neighbor)]))

        return []

    def get_neighbors(self, position, state):
        directions = [(0, 1), (0, -1), (1, 0), (-1, 0)]
        neighbors = []
        for direction in directions:
            new_position = (position[0] + direction[0], position[1] + direction[1])
            if self.is_valid_position(new_position, state):
                neighbors.append(new_position)
        return neighbors
 
    def is_valid_position(self, position, state):
        if 0 <= position[0] < state.maze.WIDTH - 1 and 0 <= position[1] < state.maze.HEIGHT - 1:
            if state.maze.MAP[position[1]][position[0]] != -1:
               return True
        return False

    def get_direction(self, current_position, neighbor_position):
        x_diff = neighbor_position[0] - current_position[0]
        y_diff = neighbor_position[1] - current_position[1]
        if x_diff == 1:
            return 3  # Right
        elif x_diff == -1:
            return 9  # Left
        elif y_diff == 1:
            return 6  # Down
        elif y_diff == -1:
            return 0  # Up
        return None

 