import cv2
import os
import random
from Tree import Node


class Map:
    def __init__(self, map_name):
        self.map_path = os.path.join(__file__, '../images', map_name)
        self.img = cv2.imread(self.map_path)
        self.img_gray = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
        h, w = self.img_gray.shape
        print(f"Map height: {h}, width: {w}")
        self.start = None
        self.end = None

    def _draw_circle(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDBLCLK:
            if self.start is None:
                cv2.circle(self.img, (x, y), 5, (255, 0, 0), -1)
                self.start = Node(x, y)
                print(f"Start {x, y}")
            else:
                cv2.circle(self.img, (x, y), 5, (0, 255, 0), -1)
                self.end = Node(x, y)
                print(f"End {x, y}")

            # test bresenham line grids extraction
            # if self.points_ready():
            #     all_grids = self._bresenham_grids(int(self.start.x), int(self.start.y), int(self.end.x), int(self.end.y))
            #     print(f"{len(all_grids)} grids btw nodes!")
            #     for x, y in all_grids:
            #         cv2.circle(vis.img, (x, y), 5, (0, 0, 255), -1)

    def select_points(self):
        cv2.namedWindow('image')
        cv2.setMouseCallback('image', self._draw_circle)
        while True:
            cv2.imshow('image', self.img)
            k = cv2.waitKey(20) & 0xFF
            if k == 27:
                break
            if self.points_ready():
                break
        print("Select points finished!")

    def points_ready(self):
        return self.start is not None and self.end is not None

    def generate_random_point(self):
        h, w, _ = self.img.shape
        count = 0
        while True:
            new_y = random.randint(0, h - 1)
            new_x = random.randint(0, w - 1)
            new_node = Node(new_x, new_y)
            count += 1
            if self.reachable(new_node):
                return new_node
            if count > 1000:
                print(f"Failed to generate valid node!")
                return None

    def within(self, node):
        assert type(node) == Node, "Wrong type of node!"
        h, w, _ = self.img.shape
        return 0 <= node.x < w and 0 <= node.y < h

    def reachable(self, node):
        if not self.within(node):
            return False
        return self.img_gray[int(node.y), int(node.x)] != 0

    def plot_path(self, path, bgr=(0, 0, 255), hold=True):
        length = len(path)
        if length <= 1:
            print(f"Path only contain {length} point!")
            return

        for i in range(length - 1):
            x0, y0 = int(path[i].x), int(path[i].y)
            x1, y1 = int(path[i + 1].x), int(path[i + 1].y)
            # if i < length - 2:
            #     cv2.circle(self.img, (x1, y1), 2, (0, 0, 255), thickness=3, lineType=8)
            cv2.line(self.img, (x0, y0), (x1, y1), bgr, thickness=2, lineType=8)
        cv2.imshow("image", self.img)

        if hold:
            while True:
                k = cv2.waitKey(20) & 0xFF
                if k == 27:
                    break
        else:
            cv2.waitKey(1000)

    def plot_segment(self, node0, node1):
        cv2.circle(self.img, (int(node0.x), int(node0.y)), 2, (0, 0, 255), thickness=3, lineType=8)
        cv2.line(self.img, (int(node0.x), int(node0.y)), (int(node1.x), int(node1.y)), (0, 255, 0), thickness=1, lineType=8)
        cv2.imshow('image', self.img)
        cv2.waitKey(1)

    def smoothen_path(self, path):
        length = len(path)
        if length <= 2:
            print(f"No need to smooth path with {length} nodes!")
            return path

        cur_goal_idx = length - 1
        smooth_path = []
        while cur_goal_idx != 0:
            smooth_path.append(path[cur_goal_idx])
            for idx in range(cur_goal_idx):
                node = path[idx]
                if not self.detect_collision(node, path[cur_goal_idx]):
                    cur_goal_idx = idx
                    break
        smooth_path.append(path[cur_goal_idx])  # append the first node
        smooth_path.reverse()
        return smooth_path


    def detect_collision(self, n0, n1):
        if not self.within(n0):
            print("Node 0 is not inside map!")
            return True
        if not self.within(n1):
            print("Node 1 is not inside map!")
            return True
        if n0 == n1:
            return False

        x0, y0 = int(n0.x), int(n0.y)
        x1, y1 = int(n1.x), int(n1.y)
        all_grids = self._bresenham_grids(x0, y0, x1, y1)

        for x, y in all_grids:
            if self.img_gray[int(y), int(x)] == 0:
                return True
        return False

    def _bresenham_grids(self, x0, y0, x1, y1):
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        error = dx + dy

        all_coords = []
        while True:
            all_coords.append((x0, y0))
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * error
            if e2 >= dy:
                if x0 == x1:
                    break
                error += dy
                x0 += sx
            if e2 <= dx:
                if y0 == y1:
                    break
                error += dx
                y0 += sy
        return all_coords


if __name__ == '__main__':
    vis = Map('world2.png')

    # test select points by mouse left button double clicking
    # vis.select_points()



