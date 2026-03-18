import random

def modify_mrdr_dataset(
    input_path,
    output_path,
    seed=42
):
    random.seed(seed)

    with open(input_path, "r") as f:
        lines = f.readlines()

    header = []
    data_start = 0

    # Tách header và bảng dữ liệu
    for i, line in enumerate(lines):
        if line.strip().startswith("XCOORD"):
            header = lines[:i+1]
            data_start = i + 1
            break

    data_lines = lines[data_start:]

    # Parse data
    customers = []
    for line in data_lines:
        x, y, d, r = map(float, line.split())
        customers.append([x, y, int(d), int(r)])

    depot = customers[0]
    custs = customers[1:]

    n = len(custs)

    # ---------- STEP 1: Assign DEMAND ----------
    # DEMAND ∈ {1,2}
    for c in custs:
        c[2] = 1 if random.random() < 0.7 else 2

    # ---------- STEP 2: Assign RELEASE DATES (3 waves) ----------
    random.shuffle(custs)

    n1 = max(1, int(0.2 * n))
    n2 = int(0.4 * n)

    wave1 = custs[:n1]
    wave2 = custs[n1:n1+n2]
    wave3 = custs[n1+n2:]

    for c in wave1:
        c[3] = random.randint(0, 5)

    for c in wave2:
        c[3] = random.randint(15, 30)

    for c in wave3:
        c[3] = random.randint(35, 55)

    # Restore original order (optional, but cleaner)
    customers_new = [depot] + custs

    # ---------- WRITE OUTPUT ----------
    with open(output_path, "w") as f:
        for line in header:
            f.write(line)

        for c in customers_new:
            f.write(f"{int(c[0])}\t{int(c[1])}\t{c[2]}\t{c[3]}\n")

    print(f"Modified MR-DR dataset written to: {output_path}")


# ===== RUN =====
modify_mrdr_dataset(
    input_path=r"test_data\data_demand_random\15\C101_0.5.dat",
    output_path="test_data\data_new\C101_0.5_MR15.dat"
)
