from __future__ import annotations

from typing import Dict, List, Optional, Tuple
import random

from .Function import ProblemData


def is_within_drone_range(data: ProblemData, city: int) -> bool:
    loading_time = 2.0 * data.drone_travel_time[0][city] + data.unloading_time
    return loading_time <= data.drone_limit_time + 1e-9


def phase1_truck_working_time(route: List[int], data: ProblemData) -> float:
    if not route:
        return 0.0

    out_of_range = [c for c in route if not is_within_drone_range(data, c)]
    t = max((data.release_dates[c] for c in out_of_range), default=0.0)

    prev = 0
    for c in route:
        t += data.truck_travel_time[prev][c]
        if data.release_dates[c] > 0 and is_within_drone_range(data, c):
            t = max(t, data.release_dates[c] + data.drone_travel_time[0][c])
        prev = c

    t += data.truck_travel_time[prev][0]
    return t


def phase1_makespan(routes: List[List[int]], data: ProblemData) -> float:
    if not routes:
        return 0.0
    return max(phase1_truck_working_time(route, data) for route in routes)


def copy_routes(routes: List[List[int]]) -> List[List[int]]:
    return [r[:] for r in routes]


def move_1_0(routes: List[List[int]]) -> List[Tuple[List[List[int]], List[int]]]:
    out: List[Tuple[List[List[int]], List[int]]] = []
    for i in range(len(routes)):
        for j in range(len(routes[i])):
            c = routes[i][j]
            for k in range(len(routes)):
                for l in range(len(routes[k]) + 1):
                    if i == k and (l == j or l == j + 1):
                        continue
                    cand = copy_routes(routes)
                    cand[i].pop(j)
                    insert_at = l
                    if i == k and l > j:
                        insert_at -= 1
                    cand[k].insert(insert_at, c)
                    out.append((cand, [c]))
    return out


def move_1_1(routes: List[List[int]]) -> List[Tuple[List[List[int]], List[int]]]:
    out: List[Tuple[List[List[int]], List[int]]] = []
    for i in range(len(routes)):
        for j in range(len(routes[i])):
            for k in range(len(routes)):
                for l in range(len(routes[k])):
                    if i == k and j == l:
                        continue
                    cand = copy_routes(routes)
                    c1 = cand[i][j]
                    c2 = cand[k][l]
                    cand[i][j], cand[k][l] = c2, c1
                    out.append((cand, [c1, c2]))
    return out


def move_2_1(routes: List[List[int]]) -> List[Tuple[List[List[int]], List[int]]]:
    out: List[Tuple[List[List[int]], List[int]]] = []
    for i in range(len(routes)):
        if len(routes[i]) < 2:
            continue
        for j in range(len(routes[i]) - 1):
            seg = [routes[i][j], routes[i][j + 1]]
            for k in range(len(routes)):
                for l in range(len(routes[k])):
                    if i == k and (l == j or l == j + 1):
                        continue
                    cand = copy_routes(routes)
                    c3 = cand[k][l]
                    cand[i].pop(j + 1)
                    cand[i].pop(j)
                    if i == k:
                        if l > j:
                            l -= 2
                        cand[i].insert(j, c3)
                        cand[k][l] = seg[0]
                        cand[k].insert(l + 1, seg[1])
                    else:
                        cand[k][l] = seg[0]
                        cand[k].insert(l + 1, seg[1])
                        cand[i].insert(j, c3)
                    out.append((cand, [seg[0], seg[1], c3]))
    return out


def two_opt(routes: List[List[int]]) -> List[Tuple[List[List[int]], List[int]]]:
    out: List[Tuple[List[List[int]], List[int]]] = []
    for i in range(len(routes)):
        n = len(routes[i])
        for a in range(n):
            for b in range(a + 1, n):
                cand = copy_routes(routes)
                segment = cand[i][a : b + 1]
                cand[i][a : b + 1] = reversed(segment)
                out.append((cand, segment[:2] if len(segment) >= 2 else segment))
    return out


def tabu_search_truck_only(routes: List[List[int]], data: ProblemData) -> List[List[int]]:
    best_routes = copy_routes(routes)
    best_cost = phase1_makespan(best_routes, data)

    current_routes = copy_routes(routes)
    tenure = max(5, int(2 * max(1.0, len([c for r in routes for c in r]) ** 0.5)))
    tabu_until: Dict[int, int] = {}

    neighborhoods = [move_1_0, move_1_1, move_2_1, two_opt]
    max_iters = max(1, len([c for r in routes for c in r]))

    for it in range(max_iters):
        move_builder = random.choice(neighborhoods)
        candidates = move_builder(current_routes)
        if not candidates:
            continue

        chosen_routes: Optional[List[List[int]]] = None
        chosen_cost = float("inf")
        chosen_moved: List[int] = []

        for cand, moved in candidates:
            cost = phase1_makespan(cand, data)
            is_tabu = any(tabu_until.get(c, -1) > it for c in moved)
            aspiration = cost < best_cost
            if is_tabu and not aspiration:
                continue
            if cost < chosen_cost:
                chosen_cost = cost
                chosen_routes = cand
                chosen_moved = moved

        if chosen_routes is None:
            continue

        current_routes = chosen_routes
        for c in chosen_moved:
            tabu_until[c] = it + tenure

        if chosen_cost < best_cost:
            best_cost = chosen_cost
            best_routes = copy_routes(chosen_routes)

    return best_routes
