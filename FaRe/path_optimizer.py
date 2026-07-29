import random


class WayPointOptimizer:
    """Orders waypoints into a short closed tour.

    Operates on indices into `goals` rather than on coordinates so callers can
    reorder parallel lists (orientations) with the returned order. Coordinates
    cannot serve as identities here: duplicate waypoints occur in practice.
    """

    def __init__(self, goals, alpha, max_iterations):
        self.goals = goals
        self.alpha = alpha or 0.3
        self.max_iterations = max_iterations or 5
        self.best_solution = None
        self.best_cost = float('inf')

    def calculate_distance(self, i, j):
        a, b = self.goals[i], self.goals[j]
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    def greedy_randomized_construction(self):
        current = 0
        solution = [current]
        remaining = list(range(1, len(self.goals)))

        while remaining:
            distances = sorted((self.calculate_distance(current, idx), idx) for idx in remaining)
            top_candidates = distances[:max(1, int(self.alpha * len(distances)))]
            _, next_idx = random.choice(top_candidates)
            solution.append(next_idx)
            remaining.remove(next_idx)
            current = next_idx

        solution.append(0)  # close the tour at the starting waypoint
        return solution

    @staticmethod
    def two_opt_swap(route, i, k):
        return route[:i] + list(reversed(route[i:k + 1])) + route[k + 1:]

    def calculate_total_distance(self, route):
        return sum(self.calculate_distance(route[i], route[i + 1]) for i in range(len(route) - 1))

    def local_search(self, solution):
        best_route = solution
        best_cost = self.calculate_total_distance(solution)

        improved = True
        while improved:
            improved = False
            # k stops short of the last element so the tour keeps starting and ending at index 0
            for i in range(1, len(best_route) - 2):
                for k in range(i + 1, len(best_route) - 1):
                    new_route = self.two_opt_swap(best_route, i, k)
                    new_cost = self.calculate_total_distance(new_route)
                    if new_cost < best_cost:
                        best_route = new_route
                        best_cost = new_cost
                        improved = True

        return best_route

    def run(self):
        """Returns the visiting order as indices into `goals`, starting and ending at 0."""
        for _ in range(self.max_iterations):
            route = self.local_search(self.greedy_randomized_construction())
            cost = self.calculate_total_distance(route)

            if cost < self.best_cost:
                self.best_solution = route
                self.best_cost = cost

        return self.best_solution
