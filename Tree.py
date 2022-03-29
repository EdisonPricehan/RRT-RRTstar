
class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None


class Tree:
    def __init__(self):
        self.root_node = None
        self.goal_node = None
        self.via = {}
        self.costs = {}

    def set_root(self, root):
        assert type(root) == Node, "Root type is not Node!"
        self.root_node = root
        self.via[root] = root
        self.costs[root] = 0.

    def set_goal(self, goal):
        assert type(goal) == Node, "Goal type is not Node!"
        self.goal_node = goal

    def add_edge(self, child, parent, cost=None):
        assert parent in self.via, "Parent node is not in current tree [via]!"
        if cost is not None:
            assert parent in self.costs, "Parent node is not in current tree [costs]!"

        if child in self.via:
            print("Child node already added!")

        self.via[child] = parent

        if cost is not None:
            self.costs[child] = cost + self.costs[parent]

    def get_path(self, goal=None):
        assert self.root_node is not None, "Cannot get path when root node is not set!"
        path = []

        if goal is None:
            goal = self.goal_node

        if goal not in self.via:
            print("Goal node is not in the tree!")
            return path

        path.append(goal)
        if goal == self.root_node:
            print("Goal node is root node!")
            return path

        parent = self.via[goal]
        while parent != self.root_node:
            path.append(parent)
            parent = self.via[parent]
        path.append(self.root_node)
        path.reverse()
        return path


if __name__ == '__main__':
    pass

