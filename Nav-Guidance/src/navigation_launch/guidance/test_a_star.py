from nose.tools import assert_equal
from unittest import TestCase

from a_star import *


# noinspection PyMethodMayBeStatic
class PrioritySetTest(TestCase):
    def test_pop(self):
        priority_set = PrioritySet([(3, 'c'), (1, 'a'), (2, 'b')])
        assert_equal(priority_set.pop(), 'a')
        assert_equal(priority_set.pop(), 'b')
        assert_equal(priority_set.pop(), 'c')
        assert_equal(priority_set.pop(), None)

    def test_peek(self):
        priority_set = PrioritySet([(1, 'a')])
        assert_equal(priority_set.peek(), 'a')
        assert_equal(priority_set.pop(), 'a')
        assert_equal(priority_set.peek(), None)

    def test_contains(self):
        priority_set = PrioritySet([(1, 'a'), (2, 'b')])
        assert_equal('a' in priority_set, True)
        assert_equal('b' in priority_set, True)
        assert_equal(priority_set.pop(), 'a')
        assert_equal('a' in priority_set, False)
        assert_equal('b' in priority_set, True)


# noinspection PyMethodMayBeStatic
class AStarTest(TestCase):

    def test_valid_path(self):
        grid = [
            [0, 0, 0, 1, 0],
            [0, 1, 1, 0, 0],
            [0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0],
            [1, 0, 0, 0, 0]
        ]

        goal = (4, 4)
        path = find_path(start=(0, 0),
                         reached_goal=lambda p: p == goal,
                         neighbors=grid_neighbors(grid, jump_size=1),
                         weight=lambda (x, y): float('inf') if grid[y][x] > 0 else 1)
        assert_equal(path, [(0, 0), (0, 1), (0, 2), (1, 2), (2, 2), (2, 3), (2, 4), (3, 4), (4, 4)])

    def test_no_path(self):
        grid = [
            [0, 0, 0, 1],
            [1, 1, 1, 0],
        ]

        goal = (4, 1)
        path = find_path(start=(0, 0),
                         reached_goal=lambda p: p == goal,
                         neighbors=grid_neighbors(grid, jump_size=1),
                         weight=lambda (x, y): float('inf') if grid[y][x] > 0 else 1)
        assert_equal(path, None)

    def test_no_heuristic(self):
        grid = [
            [0, 0, 0, 1],
            [1, 1, 1, 0],
        ]

        goal = (4, 1)
        path = find_path(start=(0, 0),
                         reached_goal=lambda p: p == goal,
                         neighbors=grid_neighbors(grid, jump_size=1),
                         weight=lambda (x, y): float('inf') if grid[y][x] > 0 else 1)
        assert_equal(path, None)

    def test_line_goal(self):
        grid = [
            [1, 1, 1, 0],
            [0, 0, 0, 0],
            [0, 0, 0, 0],
        ]

        goal = 0
        path = find_path(start=(0, 2),
                         reached_goal=lambda (x, y): y == goal,
                         neighbors=grid_neighbors(grid, jump_size=1),
                         weight=lambda (x, y): float('inf') if grid[y][x] > 0 else 1)
        assert_equal(path, [(0, 2), (0, 1), (1, 1), (2, 1), (3, 1), (3, 0)])
