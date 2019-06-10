import math

import heapdict


class PrioritySet:
    """Priority Queue w/ O(1) contains check"""

    def __init__(self, elements):
        self.hd = heapdict.heapdict()
        for element in elements:
            self.push(element)

    def push(self, (priority, item)):
        self.hd[item] = priority

    def peek(self):
        if len(self) > 0:
            item, priority = self.hd.peekitem()
            return item

    def pop(self):
        if len(self) > 0:
            item, priority = self.hd.popitem()
            return item

    def __contains__(self, item):
        return item in self.hd

    def __len__(self):
        return len(self.hd)


# Heuristics

def manhattan((x1, y1), (x2, y2)):
    return abs(x1 - x2) + abs(y1 - y2)


def euclidean((x1, y1), (x2, y2)):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


# http://theory.stanford.edu/~amitp/GameProgramming/Heuristics.html#diagonal-distance
def diagonal((x1, y1), (x2, y2)):
    dx = abs(x1 - x2)
    dy = abs(y1 - y2)
    d = 1
    d2 = math.sqrt(2)
    return d * (dx + dy) + (d2 - 2 * d) * min(dx, dy)


# A*

def reconstruct_path(came_from, current):
    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    return path


def find_path(start, reached_goal, neighbors, weight, heuristic=lambda x: 0):
    closed_set = set()
    open_set = PrioritySet([(0, start)])

    came_from = {}
    score = {start: 0}

    best = None

    while len(open_set) > 0:
        current = open_set.peek()

        # if current == goal:
        if reached_goal(current):
            if best is None or score[best] > score[current]:
                best = current

        open_set.pop()
        closed_set.add(current)

        for neighbor in neighbors(current):
            if neighbor in closed_set:
                continue
            elif not weight(neighbor) < float('inf'):
                closed_set.add(neighbor)
                continue

            s = score[current] + heuristic(neighbor) + weight(neighbor)
            if neighbor not in open_set:
                open_set.push((s, neighbor))
            elif s >= score[neighbor]:
                continue

            came_from[neighbor] = current
            score[neighbor] = s

    if best is not None:
        path = reconstruct_path(came_from, best)
        path.reverse()
        return path


def grid_neighbors(grid, jump_size=1):
    diag = jump_size

    def neighbors((x, y)):
        x_min = x >= jump_size
        y_min = y >= jump_size
        x_max = len(grid) > 0 and x < len(grid[0]) - jump_size
        y_max = len(grid) > 0 and y < len(grid) - jump_size
        if x_min: yield (x - jump_size, y)
        if y_min: yield (x, y - jump_size)
        if x_max: yield (x + jump_size, y)
        if y_max: yield (x, y + jump_size)
        if x_min and y_min: yield (x - diag, y - diag)
        if x_max and y_max: yield (x + diag, y + diag)
        if x_min and y_max: yield (x - diag, y + diag)
        if x_max and y_min: yield (x + diag, y - diag)

    return neighbors
