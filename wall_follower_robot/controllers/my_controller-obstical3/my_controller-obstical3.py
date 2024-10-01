import matplotlib.pyplot as plt
import numpy as np
import math
from controller import Supervisor

# 创建机器人实例
supervisor = Supervisor()

# 获取时间步长
timeStep = int(supervisor.getBasicTimeStep())

# 获取圆柱体节点
circle = supervisor.getFromDef("circle")
circle1 = supervisor.getFromDef("circle1")
circle2 = supervisor.getFromDef("circle2")

def get_circle_info(circle_node):
    if circle_node is None:
        print("Error: Could not find DEF node")
        return None, None, None
    else:
        # 获取圆柱体的几何子节点
        geometry = circle_node.getField('children').getMFNode(0).getField('geometry')
        radius = geometry.getSFNode().getField('radius').getSFFloat()
        height = geometry.getSFNode().getField('height').getSFFloat()
        position = circle_node.getPosition()
        return position, radius, height

position, radius, _ = get_circle_info(circle)
position1, radius1, _ = get_circle_info(circle1)
position2, radius2, _ = get_circle_info(circle2)

class RRT:
    """Implementing the Rapidly-Exploring Random Tree algorithm with circular obstacles"""

    def __init__(self, q_init, q_goal, iterations, obstacles):
        self.q_init = q_init
        self.q_goal = q_goal
        self.iterations = iterations
        self.delta = 0.1
        self.domain = 0.98
        self.q_list = [q_init]
        self.parent = {tuple(q_init): None}
        self.obstacles = obstacles
    
    def generate_random_config(self):
        """Generate a random position in the -1.0 to 1.0 domain"""
        self.q_rand = [self.domain * (2 * np.random.rand() - 1), self.domain * (2 * np.random.rand() - 1)]
        return self.q_rand

    def find_nearest_vertex(self):
        """Find the nearest vertex from a random position in q_list"""
        dist_list = []
        random_vertex = self.generate_random_config()
        for vertex in self.q_list:
            distance = math.dist(vertex, random_vertex)
            dist_list.append(distance)
        nearest_dist = min(dist_list)
        nearest_index = dist_list.index(nearest_dist)
        self.q_near = self.q_list[nearest_index]
        return self.q_near

    def generate_new_config(self):
        """Generate a new vertex"""
        distance = math.dist(self.q_rand, self.q_near)
        x_diff = self.q_rand[0] - self.q_near[0]
        y_diff = self.q_rand[1] - self.q_near[1]
        q_new_x = self.q_near[0] + (self.delta / distance) * x_diff
        q_new_y = self.q_near[1] + (self.delta / distance) * y_diff
        self.q_new = [q_new_x, q_new_y]
        return self.q_new

    def is_collision(self, q_new):
        """Check if the new vertex is within any of the obstacles"""
        for center, radius in self.obstacles:
            distance_to_obstacle = math.dist(q_new, center)
            if distance_to_obstacle <= radius + 0.02:  # Maintain collision distance of 0.02
                return True
        return False

    def plot_result(self):
        """Plot the tree and the obstacles"""
        f, ax = plt.subplots()
        plt.ion()
        ax.set_xlim(-1.0, 1.0)  # Update plot range to 1.0
        ax.set_ylim(-1.0, 1.0)  # Update plot range to 1.0
        ax.set_title("Rapidly-Exploring Random Tree with Circular Obstacles")

        # Plot obstacles
        for center, radius in self.obstacles:
            circle = plt.Circle(center, radius, color='r', fill=True)
            ax.add_patch(circle)

        for iteration in range(self.iterations):
            q_near = self.find_nearest_vertex()
            q_new = self.generate_new_config()
            if not self.is_collision(q_new):
                self.q_list.append(q_new)
                self.parent[tuple(q_new)] = tuple(q_near)
                x = [q_near[0], q_new[0]]
                y = [q_near[1], q_new[1]]
                ax.plot(x, y, color='blue')
                if math.dist(q_new, self.q_goal) <= self.delta:
                    self.q_list.append(self.q_goal)
                    self.parent[tuple(self.q_goal)] = tuple(q_new)
                    break
            plt.pause(0.0001)

        # Plot the start and goal points
        ax.plot(self.q_init[0], self.q_init[1], 'go')  # Start point
        ax.plot(self.q_goal[0], self.q_goal[1], 'ro')  # Goal point
        
        # Retrieve and plot the path
        path = self.get_path()
        for i in range(len(path) - 1):
            x = [path[i][0], path[i + 1][0]]
            y = [path[i][1], path[i + 1][1]]
            ax.plot(x, y, color='black')

        plt.show()
        plt.pause(5)
        
    def get_path(self):
        """Retrieve and print the path from q_init to q_goal"""
        path = []
        current = tuple(self.q_goal)
        while current is not None:
            path.append(current)
            current = self.parent[current]
        path.reverse()
        
        # Print the path
        print("Path from start to goal:")
        for point in path:
            print(point)
        
        return path

def main():
    obstacles = [
        ([position[0], position[1]], radius),
        ([position1[0], position1[1]], radius1),
        ([position2[0], position2[1]], radius2)
    ]
    rrt = RRT(q_init=[0.8, 0.8], q_goal=[-0.8, -0.8], iterations=800, obstacles=obstacles)
    rrt.plot_result()

if __name__ == "__main__":
    main()