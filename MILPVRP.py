#!/usr/bin/env python3
"""
vrp_makespan_noservice.py

VRP MILP (K xe) — objective: minimize makespan (thời gian chiếc xe cuối cùng quay về depot).
Phiên bản **không có thời gian phục vụ** (Sigma bị loại bỏ).

Dùng PuLP (với CPLEX_PY / CPLEX_CMD / CBC).

Input: input.dat (text)
Format example:
number_truck 2
truck_speed 0.5
Q 200          # (optional) vehicle capacity
# XCOORD YCOORD DEMAND RELEASE_DATE
0 0 0 0
10 100 1 100
...
"""
import math
import sys
import pulp

# ---------- Parser for input.dat ----------
def read_input_dat(filename):
    params = {}
    nodes = []
    with open(filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split()
            # node line heuristics: at least 4 numeric columns (x y demand release)
            if len(parts) >= 4 and is_float(parts[0]) and is_float(parts[1]) and is_float(parts[2]) and is_float(parts[3]):
                x = float(parts[0]); y = float(parts[1]); demand = float(parts[2]); release = float(parts[3])
                nodes.append({'x': x, 'y': y, 'demand': demand, 'release': release})
            else:
                # param line: key value
                if len(parts) >= 2:
                    key = parts[0]
                    val = parts[1]
                    if is_float(val):
                        if '.' in val or 'e' in val.lower():
                            val_parsed = float(val)
                        else:
                            val_parsed = int(val)
                        params[key] = val_parsed
                    else:
                        params[key] = val
    return params, nodes

def is_float(s):
    try:
        float(s)
        return True
    except:
        return False

# ---------- Helpers ----------
def euclid(a,b):
    return math.hypot(a['x']-b['x'], a['y']-b['y'])

# ---------- Build and solve MILP ----------
def solve_vrp(params, nodes):
    n_nodes = len(nodes)
    if n_nodes == 0:
        print("No nodes found in input.")
        return
    V = list(range(n_nodes))
    K = int(params.get('number_truck', 1))
    truck_speed = float(params.get('truck_speed', 1.0))
    # Sigma removed: do NOT read/use params['Sigma']
    Q = float(params.get('Q', 1e9))
    time_limit = int(params.get('time_limit', 300))  # seconds

    # distances & travel times
    dist = {}
    tau = {}
    for i in V:
        for j in V:
            if i == j:
                continue
            dij = euclid(nodes[i], nodes[j])
            dist[(i,j)] = dij
            tau[(i,j)] = dij / truck_speed if truck_speed > 0 else dij

    # big M: conservative (no sigma term now)
    total_travel = sum(dist.values()) if dist else 0.0
    M_time = total_travel * 10 + 1e3

    # Problem
    prob = pulp.LpProblem("VRP_Makespan_NoService", pulp.LpMinimize)

    # Variables
    x = {}
    for i in V:
        for j in V:
            if i == j:
                continue
            for k in range(K):
                x[(i,j,k)] = pulp.LpVariable(f"x_{i}_{j}_{k}", cat='Binary')

    t = {i: pulp.LpVariable(f"t_{i}", lowBound=0, cat='Continuous') for i in V}
    u = {i: pulp.LpVariable(f"u_{i}", lowBound=0, upBound=n_nodes, cat='Continuous') for i in V}

    # completion times per truck and makespan
    C = {k: pulp.LpVariable(f"C_{k}", lowBound=0, cat='Continuous') for k in range(K)}
    T_max = pulp.LpVariable("T_max", lowBound=0, cat='Continuous')

    # Objective: minimize makespan
    prob += T_max, "MinimizeMakespan"

    # Constraints

    # Each customer visited once (except depot)
    for j in V:
        if j == 0:
            continue
        prob += pulp.lpSum(x[(i,j,k)] for i in V if i != j for k in range(K)) == 1, f"VisitOnce_{j}"

    # Each truck starts and ends at depot exactly once
    for k in range(K):
        prob += pulp.lpSum(x[(0,j,k)] for j in V if j != 0) == 1, f"StartDepot_k{k}"
        prob += pulp.lpSum(x[(i,0,k)] for i in V if i != 0) == 1, f"EndDepot_k{k}"

    # Flow conservation at customers
    for k in range(K):
        for j in V:
            if j == 0:
                continue
            prob += (pulp.lpSum(x[(i,j,k)] for i in V if i != j) - pulp.lpSum(x[(j,i,k)] for i in V if i != j)) == 0, f"FlowCons_k{k}_node{j}"

    # Capacity constraints
    for k in range(K):
        prob += pulp.lpSum(nodes[j]['demand'] * x[(i,j,k)] for i in V for j in V if i != j) <= Q, f"Cap_k{k}"

    # Time & release (no service time)
    for i in V:
        prob += t[i] >= nodes[i]['release'], f"Release_{i}"
    # fix depot departure time to 0
    prob += t[0] == 0, "Depot_time_zero"

    for i in V:
        for j in V:
            if i == j:
                continue
            for k in range(K):
                # without sigma: travel time only
                prob += t[j] >= t[i] + tau.get((i,j), 0) - M_time * (1 - x[(i,j,k)]), f"TimeProp_{i}_{j}_k{k}"

    # MTZ subtour elimination
    prob += u[0] == 0, "MTZ_depot"
    for i in V:
        if i == 0:
            continue
        prob += u[i] >= 1, f"MTZ_low_{i}"
        prob += u[i] <= n_nodes - 1, f"MTZ_up_{i}"
    for i in V:
        if i == 0:
            continue
        for j in V:
            if j == 0 or j == i:
                continue
            prob += u[i] - u[j] + (n_nodes) * pulp.lpSum(x[(i,j,k)] for k in range(K)) <= n_nodes - 1, f"MTZ_{i}_{j}"

    # Link C_k with return arcs to depot: if (i->0) used by k then C_k >= t_i + travel(i,0)
    for k in range(K):
        prob += C[k] >= 0, f"C_nonneg_{k}"
        for i in V:
            if i == 0:
                continue
            if (i,0) in tau:
                prob += C[k] >= t[i] + tau[(i,0)] - M_time * (1 - x[(i,0,k)]), f"CompTimeConstr_{i}_k{k}"

    # T_max >= C_k
    for k in range(K):
        prob += T_max >= C[k], f"Tmax_ge_C_{k}"

    # Solver (keep your CPLEX_PY with timelimit 300s, change if needed)
    solver = pulp.CPLEX_PY(msg=1)
    print("Starting solve (minimize makespan, no service time)...")
    res = prob.solve(solver)
    print("Solver status:", pulp.LpStatus[prob.status])

    if pulp.LpStatus[prob.status] not in ('Optimal', 'Feasible'):
        print("Solver did not find feasible/optimal solution.")
        return

    # Extract routes per vehicle
    routes = {k: [] for k in range(K)}
    for k in range(K):
        succ = {}
        for i in V:
            for j in V:
                if i == j:
                    continue
                var = x.get((i,j,k))
                if var is not None and pulp.value(var) > 0.5:
                    succ[i] = j
        route = [0]
        cur = 0
        visited = set([0])
        while True:
            if cur not in succ:
                break
            nxt = succ[cur]
            if nxt in visited:
                route.append(nxt)
                break
            route.append(nxt)
            visited.add(nxt)
            cur = nxt
            if len(route) > n_nodes + 5:
                break
        routes[k] = route

    # Print results
    print("\nMakespan (T_max) = ", pulp.value(T_max))
    for k in range(K):
        print(f"\nTruck {k+1}:")
        print("  Completion time C_k =", pulp.value(C[k]))
        print("  Route nodes =", routes[k])
        for node in routes[k]:
            print(f"    node {node} (coord {nodes[node]['x']},{nodes[node]['y']}) arrival t = {pulp.value(t[node])}")

    # Selected arcs
    print("\nSelected arcs (i->j by truck):")
    for (i,j,k), var in x.items():
        if pulp.value(var) > 0.5:
            print(f"  truck {k+1}: {i} -> {j}")

    return prob, routes

# ---------- main ----------
if __name__ == "__main__":
    if len(sys.argv) >= 2:
        infile = sys.argv[1]
    else:
        infile = "input.dat"
    params, nodes = read_input_dat(infile)
    if not nodes:
        print("No nodes read. Please check input.dat format.")
        sys.exit(1)
    print("Read params:", params)
    print(f"Number of nodes: {len(nodes)}; number of trucks: {params.get('number_truck',1)}")
    solve_vrp(params, nodes)
