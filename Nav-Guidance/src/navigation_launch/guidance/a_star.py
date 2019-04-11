import heapq

# TODO: Docs
import math


class PrioritySet:
    """Priority Queue w/ O(1) contains check"""

    def __init__(self, elements):
        self.queue = []
        self.set = set()
        for element in elements:
            self.push(element)

    def push(self, item):
        if item[1] not in self.set:
            heapq.heappush(self.queue, item)
            self.set.add(item[1])

    def peek(self):
        if len(self) > 0:
            return self.queue[0][1]

    def pop(self):
        if len(self) > 0:
            item = heapq.heappop(self.queue)
            self.set.remove(item[1])
            return item[1]

    def __contains__(self, item):
        return item in self.set

    def __len__(self):
        return len(self.queue)


def reconstruct_path(came_from, current):
    path = [current]

    while current in came_from:
        current = came_from[current]
        path.append(current)

    return path


def find_path(start, reached_goal, neighbors, weight):
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

            s = score[current] + weight(neighbor)
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


def manhattan((x1, y1), (x2, y2)):
    return abs(x1 - x2) + abs(y1 - y2)


def euclidean((x1, y1), (x2, y2)):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)


def grid_neighbors(grid, jump_size=1):
    def neighbors((x, y)):
        if x >= jump_size:
            yield (x - jump_size, y)
        if y >= jump_size:
            yield (x, y - jump_size)
        if len(grid) > 0 and x < len(grid[0]) - jump_size:
            yield (x + jump_size, y)
        if y < len(grid) - jump_size:
            yield (x, y + jump_size)

    return neighbors


def grid_traversable(grid, traversable):
    return lambda ((x, y)): traversable(grid[y][x])


def find_path_grid(grid, start, goal, traversable, heuristic=manhattan):
    return find_path(start, goal, grid_neighbors(grid),
                     traversable=lambda (x, y): traversable(grid[y][x]),
                     heuristic=heuristic)
