from RRT import RRT


class RRTstar(RRT):
    def __init__(self, start=None, goal=None, max_iter=10000, step_size=10, try_goal=False, radius=20, map_name='world2.png'):
        super(RRTstar, self).__init__(start, goal, max_iter, step_size, try_goal, map_name)
        self.radius = radius

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
                        # Choose parent for the new node, the best parent node will still be current nearest node
                        # if all our edge costs are euclidean distance, however we can leverage other aspects to node
                        # cost such as node's distance to surrounding obstacles, node's distance to goal, or robot's
                        # motion constraints for certain inter-node movement. Final objective in this step is to find
                        # the best parent for the newly found (orphan) child
                        cur_dist = self.distance(nearest_node, next_node)
                        cur_cost = self.tree.costs[nearest_node] + cur_dist
                        cur_node = nearest_node
                        for node in list(self.tree.via.keys()):
                            if node == nearest_node:
                                continue
                            print(f"Node {node.x, node.y}")
                            if self.map.detect_collision(node, next_node):
                                continue

                            d = self.distance(node, next_node)
                            cost = self.tree.costs[node] + d
                            if cost < cur_cost:
                                cur_cost = cost
                                cur_dist = d
                                cur_node = node

                        self.tree.add_edge(next_node, cur_node, cur_dist)

                        # try to connect with goal node, if succeeds, path are solved with early stop
                        if self.try_goal and self.can_reach_goal(next_node):
                            self.tree.add_edge(self.goal, next_node, None)
                            self.path = self.tree.get_path()
                            print(f"Found path to goal by direct reaching after {it + 1} iterations!")
                            break

                        # Rewire the edges of neighbouring nodes around the newly added node
                        for node in list(self.tree.via.keys()):
                            if node == next_node:
                                continue

                            dist = self.distance(node, next_node)
                            if dist > self.radius:
                                continue

                            if self.map.detect_collision(node, next_node):
                                continue

                            if self.tree.costs[next_node] + dist < self.tree.costs[node]:
                                self.tree.costs[node] = self.tree.costs[next_node] + dist
                                self.tree.via[node] = next_node

                        # update new edge on image
                        self.map.plot_segment(next_node, nearest_node)

                        # check if near enough to goal node
                        dist_to_goal = self.distance(next_node, self.goal)
                        if dist_to_goal <= 2 * self._step_size and not self.map.detect_collision(next_node, self.goal):
                            self.tree.add_edge(self.goal, next_node, None)
                            self.path = self.tree.get_path(self.goal)
                            print(f"Found path to goal after {it + 1} iterations!")
                            break

        # output un-smoothened path length for evaluation and comparison
        print(f"Path has {len(self.path)} nodes, length is {self.path_length()}.")

        self.plot_path()


if __name__ == '__main__':
    rrt_star = RRTstar(start=None, goal=None, max_iter=10000, step_size=10, try_goal=True, radius=20, map_name='world2.png')
    rrt_star.update_start_and_goal()
    rrt_star.solve()

