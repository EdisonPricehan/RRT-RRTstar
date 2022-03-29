import numpy as np
from Tree import *
from Map import *


class RRT:
    def __init__(self, start=None, goal=None, max_iter=10000, step_size=10, try_goal=False, map_name='world2.png'):
        self.start = start
        self.goal = goal
        self._max_iter = max_iter
        self._step_size = step_size
        self.try_goal = try_goal
        self.map = Map(map_name)
        self.tree = Tree()
        self.path = []

    def update_start_and_goal(self):
        self.map.select_points()
        self.start = self.map.start
        self.goal = self.map.end
        self.tree.set_root(self.start)
        self.tree.set_goal(self.goal)

    @staticmethod
    def distance(node0, node1):
        return np.sqrt((node0.x - node1.x) ** 2 + (node0.y - node1.y) ** 2)

    def steer(self, new_node, nearest_node):
        start = np.array([nearest_node.x, nearest_node.y])
        goal = np.array([new_node.x, new_node.y])
        vector = goal - start
        length = np.linalg.norm(vector)
        vector = (vector / length) * min(self._step_size, int(length))
        target_node = Node(nearest_node.x + vector[0], nearest_node.y + vector[1])
        return target_node

    def nearest(self, new_node):
        nearest_node = None
        min_dist = float('inf')
        for n in list(self.tree.via.keys()):
            dist = self.distance(new_node, n)
            if dist < min_dist:
                nearest_node = n
                min_dist = dist
        return nearest_node

    def path_length(self, path=None):
        if path is None:
            path = self.path

        num = len(path)
        if num <= 1:
            print(f"Cannot get path length with {num} nodes!")
            return 0.

        length = 0.
        for i in range(num - 1):
            length += self.distance(path[i], path[i + 1])
        return length

    def can_reach_goal(self, node):
        return not self.map.detect_collision(node, self.goal)

    def plot_path(self):
        # plot the tree path
        self.map.plot_path(self.path, bgr=(0, 0, 255), hold=False)

        # plot the smooth path
        self.path = self.map.smoothen_path(self.path)
        self.map.plot_path(self.path, bgr=(255, 0, 255), hold=True)

    def solve(self):
        if self.start is None or self.goal is None:
            print("Need to specify start and goal!")
            return

        if not self.map.reachable(self.start) or not self.map.reachable(self.goal):
            print("Either start node or goal node is on obstacle!")
            return

        for it in range(self._max_iter):
            new_node = self.map.generate_random_point()
            # print(f"Random point {new_node.x, new_node.y}")
            if new_node is not None:
                nearest_node = self.nearest(new_node)
                if nearest_node is not None:
                    next_node = self.steer(new_node, nearest_node)

                    if not self.map.detect_collision(nearest_node, next_node):
                        # do not care about edge cost
                        self.tree.add_edge(next_node, nearest_node)

                        # try to connect with goal node, if succeeds, path are solved with early stop
                        if self.try_goal and self.can_reach_goal(next_node):
                            self.tree.add_edge(self.goal, next_node, None)
                            self.path = self.tree.get_path()
                            print(f"Found path to goal by direct reaching after {it + 1} iterations!")
                            break

                        # update new edge on image
                        self.map.plot_segment(next_node, nearest_node)

                        # check if near enough to goal node
                        dist_to_goal = self.distance(next_node, self.goal)
                        if dist_to_goal <= 2 * self._step_size and not self.map.detect_collision(next_node, self.goal):
                            self.tree.add_edge(self.goal, next_node, None)
                            self.path = self.tree.get_path()
                            print(f"Found path to goal after {it + 1} iterations!")
                            break

        # output un-smoothened path length for evaluation and comparison
        print(f"Path has {len(self.path)} nodes, length is {self.path_length()}.")

        self.plot_path()


if __name__ == '__main__':
    rrt = RRT(start=None, goal=None, max_iter=10000, step_size=10, try_goal=True, map_name='world2.png')
    rrt.update_start_and_goal()
    rrt.solve()

