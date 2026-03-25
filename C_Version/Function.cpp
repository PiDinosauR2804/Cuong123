#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <cstdlib>
#include <algorithm>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <limits>
#include <random>
#include <chrono>
#include <cctype>
#include <array>
#include <unistd.h>
#include "Solution.hpp"
using namespace std;
static std::string g_data_name;
static constexpr double kSolverTimeLimitSec = 150.0 * 60.0; // 150 minutes
static std::chrono::steady_clock::time_point g_solve_start;
static bool g_has_best_multi_solution = false;
static Solution g_best_multi_solution;

static inline double elapsed_solver_seconds() {
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now() - g_solve_start).count();
}

static inline bool solver_time_limit_reached() {
    return elapsed_solver_seconds() >= kSolverTimeLimitSec;
}


struct Customer {
    double x{}, y{};
    int demand{}, release{};
};

struct Params {
    int n_truck{};
    int n_drone{};
    double truck_speed{};
    double drone_speed{};
    double M_d{};      // drone payload
    double L_d{};      // drone max flight time
    double sigma{};    // unloading time / buffer
    vector<Customer> customers; // includes depot as index 0
    vector<vector<double>> truck_time; // Manhattan / truck_speed
    vector<vector<double>> drone_time; // Euclid   / drone_speed
    vector<int> reachable_by_drone;    // customer indices reachable within endurance (depot round-trip)
    vector<int> unreachable_by_drone;  // customer indices not reachable by drone
    vector<char> reachable_mask;       // size = customers, 1 if reachable
};

// ---------------- Fitness computation ----------------

struct TruckTimeline {
    // arrival time at each stop index in the truck route (same order as TruckRoute.stops)
    std::vector<double> arrival;
    double finish_time{}; // arrival at final depot
};

// forward declarations
static double truck_only_makespan(const Params &p, const Solution &s);
static void print_truck_routes(const Solution &s);
static std::string sanitize_token(const std::string &s);
static std::string format_scalar(double v);
static void print_solution_compact(std::ostream &os, const Solution &sol, const Params &p);
static std::string write_solution_json(const Solution &sol, const Params &p, const std::string &tag);
static bool validate_solution(const Params &p, const Solution &sol, std::string &reason);
static double truck_arrival_at(const Solution &sol, const std::vector<TruckTimeline> &tls, int truck_id, int customer);
static int route_pos(const Solution &s, int truck_id, int customer);
static bool trip_endurance_ok(const Params &p, const Solution &s, const std::vector<TruckTimeline> &tls, const DroneTrip &trip);
static int trip_load(const Params &p, const DroneTrip &trip);
static int find_insert_pos_queue(const Params &p, const Solution &s, const DroneTrip &cand);
static bool trip_endurance_optimistic(const Params &p, const Solution &s, const DroneTrip &trip);
static void normalize_after_transform(const Params &p, Solution &sol);
static std::pair<bool,double> fitness_full(const Params &p, const Solution &sol, double *truck_sum_out = nullptr);
static double fitness(const Params &p, const Solution &sol, bool *valid_out);
static bool relocate_sync_point_first_improve(const Params &p, Solution &s);
static bool relocate_package_first_improve(const Params &p, Solution &s);
static bool reorder_trip_first_improve(const Params &p, Solution &s);
static void improve_truck_routes_2opt(const Params &p, Solution &s);
static void repair_and_refine_after_truck_move(const Params &p, Solution &s, const std::vector<int> &affected_trucks);
static void repair_segment_precedence_same_truck(const Params &p, Solution &sol, int truck_id, int pos_lo, int pos_hi);
static void repair_cross_truck_precedence_and_reassign(const Params &p, Solution &sol, const std::vector<int> &affected_trucks, int city_i, int city_j);
struct EndpointSwapLogInfo {
    int city_i = -1;
    int city_j = -1;
    int changed_events = 0;
};
static EndpointSwapLogInfo swap_endpoint_resupply_same_truck(Solution &sol, int truck_id, int pos_i, int pos_j);
static void apply_drone_local_search_light(const Params &p, Solution &s);

// ------------ Initial truck-only route construction --------------

// Build an initial truck-only solution following the paper heuristic:
// 1) Sort customers by release time.
// 2) Seed each truck with one customer (first |K| customers).
// 3) Insert remaining customers one by one into the truck route that yields minimal completion time
//    under the modified truck-time calculation (no drones yet).
static Solution build_initial_truck_routes(const Params &p) {
    Solution sol = make_empty_solution(p.n_truck);

    // customers sorted by release time (skip depot)
    std::vector<int> custs;
    for (int i = 1; i < (int)p.customers.size(); ++i) custs.push_back(i);
    std::sort(custs.begin(), custs.end(), [&](int a, int b){
        if (p.customers[a].release != p.customers[b].release)
            return p.customers[a].release < p.customers[b].release;
        return a < b;
    });

    // seed each truck with one customer
    int idx = 0;
    for (int k = 0; k < p.n_truck && idx < (int)custs.size(); ++k, ++idx) {
        insert_truck_customer(sol, k, custs[idx]);
    }

    // insert remaining customers greedily
    for (; idx < (int)custs.size(); ++idx) {
        int c = custs[idx];
        double best_inc = 1e100; int best_truck = 0; size_t best_pos = 1;
        for (int k = 0; k < p.n_truck; ++k) {
            auto &stops = sol.trucks[k].stops;
            for (size_t pos = 1; pos <= stops.size()-1; ++pos) { // insert before final depot
                Solution trial = sol;
                trial.trucks[k].stops.insert(trial.trucks[k].stops.begin()+pos, TruckStop{c,{}});
                double t = truck_only_makespan(p, trial);
                if (t < best_inc) { best_inc = t; best_truck = k; best_pos = pos; }
            }
        }
        sol.trucks[best_truck].stops.insert(sol.trucks[best_truck].stops.begin()+best_pos, TruckStop{c,{}});
    }

    return sol;
}

static TruckTimeline compute_truck_timeline(const Params &p, const TruckRoute &route) {
    TruckTimeline tl;
    const auto &stops = route.stops;
    int m = static_cast<int>(stops.size());
    tl.arrival.resize(m, 0.0);

    // Determine which packages are carried from depot: customers on route not listed in any loaded_from_drone.
    std::vector<bool> resupplied(p.customers.size(), false);
    for (const auto &st : stops) {
        for (PackageId pkg : st.loaded_from_drone) {
            if (pkg >= 0 && pkg < (int)resupplied.size()) resupplied[pkg] = true;
        }
    }
    int depot_carry_max_release = 0;
    for (const auto &st : stops) {
        int cust = st.customer;
        if (cust == 0) continue;
        if (!resupplied[cust]) {
            depot_carry_max_release = std::max(depot_carry_max_release, p.customers[cust].release);
        }
    }

    double t = depot_carry_max_release; // truck departure time from depot
    tl.arrival[0] = 0.0; // at depot
    for (int i = 0; i < m - 1; ++i) {
        int from = stops[i].customer;
        int to   = stops[i+1].customer;
        t += p.truck_time[from][to];
        tl.arrival[i+1] = t;
    }
    tl.finish_time = t; // arriving depot at end
    return tl;
}

// Truck-only makespan with waiting rule (no drones):
// - truck start time = max release of unreachable customers on its route, else 0
// - at a reachable customer with release>0, departure >= max(arrival, release + drone_time(depot, customer))
static double truck_only_makespan(const Params &p, const Solution &s) {
    double worst = 0.0;
    for (int k = 0; k < p.n_truck; ++k) {
        const auto &stops = s.trucks[k].stops;
        double t = 0.0;
        bool has_unreachable = false;
        for (size_t i = 1; i+1 < stops.size(); ++i) {
            if (!p.reachable_mask.empty() && !p.reachable_mask[stops[i].customer]) has_unreachable = true;
        }
        if (has_unreachable) {
            int max_rel = 0;
            for (size_t i = 1; i+1 < stops.size(); ++i) {
                if (!p.reachable_mask.empty() && !p.reachable_mask[stops[i].customer])
                    max_rel = std::max(max_rel, p.customers[stops[i].customer].release);
            }
            t = max_rel;
        }
        for (size_t i = 0; i+1 < stops.size(); ++i) {
            int from = stops[i].customer;
            int to   = stops[i+1].customer;
            t += p.truck_time[from][to];
            if (to != 0) {
                if (!p.reachable_mask.empty() && p.reachable_mask[to] && p.customers[to].release > 0) {
                    t = std::max(t, (double)p.customers[to].release + p.drone_time[0][to]);
                }
            }
        }
        worst = std::max(worst, t);
    }
    return worst;
}

// -------- Truck route neighborhoods (best-improvement, evaluated with fitness_full + truck-sum tie-break) --------

static void repair_after_truck_move(Solution &sol);
static void repair_after_truck_move(Solution &sol, const std::vector<int> &affected_trucks);

static inline bool better_pair(double new_fit, double new_sum, double &best_fit, double &best_sum) {
    if (new_fit + 1e-9 < best_fit) return true;
    if (std::fabs(new_fit - best_fit) <= 1e-9 && new_sum + 1e-9 < best_sum) return true;
    return false;
}

struct NeighborhoodApplyResult {
    bool moved = false;
    std::vector<int> touched;
    double fit = std::numeric_limits<double>::infinity();
    double sum = std::numeric_limits<double>::infinity();
};

static bool tabu_allows_move(
    const std::vector<int> &touched,
    const std::vector<int> *tabu_list,
    int loop_idx,
    double tabu_tenure,
    double cand_fit,
    double best_global_fit
) {
    if (!tabu_list) return true;
    if (cand_fit + 1e-9 < best_global_fit) return true; // aspiration
    if (touched.empty()) return true;
    for (int c : touched) {
        if (c <= 0 || c >= (int)tabu_list->size()) continue;
        if ((double)loop_idx - (double)(*tabu_list)[c] > tabu_tenure) return true;
    }
    return false;
}

static int worst_truck_index(const Params &p, const Solution &s) {
    int K = p.n_truck;
    std::vector<TruckTimeline> tls(K);
    for (int k = 0; k < K; ++k) tls[k] = compute_truck_timeline(p, s.trucks[k]);
    int worst = 0; double wt = tls[0].finish_time;
    for (int k = 1; k < K; ++k) if (tls[k].finish_time > wt + 1e-9) { wt = tls[k].finish_time; worst = k; }
    return worst;
}

// 1-0 move: relocate a single customer
static NeighborhoodApplyResult truck_move_1_0_best(
    const Params &p,
    Solution &s,
    const std::vector<int> *tabu_list = nullptr,
    int loop_idx = 0,
    double tabu_tenure = 0.0,
    double best_global_fit = std::numeric_limits<double>::infinity()
) {
    NeighborhoodApplyResult out;
    double base_sum = 0.0; auto base = fitness_full(p, s, &base_sum); if (!base.first) return out;
    double best_fit = std::numeric_limits<double>::infinity();
    double best_sum = std::numeric_limits<double>::infinity();
    Solution best_sol = s;
    std::vector<int> best_touched;
    bool found = false;
    int K = p.n_truck;
    for (int ta = 0; ta < K; ++ta) {
        for (size_t ia = 1; ia + 1 < s.trucks[ta].stops.size(); ++ia) {
            for (int tb = 0; tb < K; ++tb) {
                for (size_t ib = 1; ib < s.trucks[tb].stops.size(); ++ib) {
                    if (ta == tb && (ib == ia || ib == ia+1)) continue; // no change
                    Solution trial = s;
                    auto &ra = trial.trucks[ta].stops;
                    auto &rb = trial.trucks[tb].stops;
                    TruckStop moved = ra[ia];
                    int city_i = moved.customer;
                    int city_j = s.trucks[tb].stops[ib].customer;
                    ra.erase(ra.begin()+ia);
                    size_t adj_ib = ib;
                    if (ta == tb && ib > ia) adj_ib = ib-1; // account for erase shrink
                    if (adj_ib > rb.size()) adj_ib = rb.size();
                    rb.insert(rb.begin()+adj_ib, moved);
                    repair_and_refine_after_truck_move(p, trial, {ta,tb});
                    if (ta == tb) {
                        int lo = std::min((int)ia, (int)adj_ib);
                        int hi = std::max((int)ia, (int)adj_ib);
                        repair_segment_precedence_same_truck(p, trial, ta, lo, hi);
                    } else {
                        repair_cross_truck_precedence_and_reassign(p, trial, {ta, tb}, city_i, city_j);
                    }
                    double ts = 0.0; auto res = fitness_full(p, trial, &ts); if (!res.first) continue;
                    std::vector<int> touched{city_i};
                    if (!tabu_allows_move(touched, tabu_list, loop_idx, tabu_tenure, res.second, best_global_fit)) continue;
                    if (!found || better_pair(res.second, ts, best_fit, best_sum)) {
                        best_fit = res.second; best_sum = ts; best_sol = std::move(trial); best_touched = touched; found = true;
                    }
                }
            }
        }
    }
    if (found) {
        out.moved = true;
        out.fit = best_fit;
        out.sum = best_sum;
        out.touched = std::move(best_touched);
        s = std::move(best_sol);
    }
    return out;
}

// 1-1 move: swap two customers
static NeighborhoodApplyResult truck_move_1_1_best(
    const Params &p,
    Solution &s,
    const std::vector<int> *tabu_list = nullptr,
    int loop_idx = 0,
    double tabu_tenure = 0.0,
    double best_global_fit = std::numeric_limits<double>::infinity()
) {
    NeighborhoodApplyResult out;
    double base_sum = 0.0; auto base = fitness_full(p, s, &base_sum); if (!base.first) return out;
    double best_fit = std::numeric_limits<double>::infinity();
    double best_sum = std::numeric_limits<double>::infinity();
    Solution best_sol = s;
    std::vector<int> best_touched;
    bool found = false;
    int K = p.n_truck;
    for (int t1 = 0; t1 < K; ++t1) {
        for (size_t i = 1; i + 1 < s.trucks[t1].stops.size(); ++i) {
            for (int t2 = t1; t2 < K; ++t2) {
                size_t j_start = (t2 == t1) ? i+1 : 1;
                for (size_t j = j_start; j + 1 < s.trucks[t2].stops.size(); ++j) {
                    int city_i = s.trucks[t1].stops[i].customer;
                    int city_j = s.trucks[t2].stops[j].customer;
                    Solution trial = s;
                    std::swap(trial.trucks[t1].stops[i], trial.trucks[t2].stops[j]);
                    repair_and_refine_after_truck_move(p, trial, {t1,t2});
                    EndpointSwapLogInfo ep_swap;
                    if (t1 == t2) {
                        ep_swap = swap_endpoint_resupply_same_truck(trial, t1, (int)i, (int)j);
                        int lo = std::min((int)i, (int)j);
                        int hi = std::max((int)i, (int)j);
                        repair_segment_precedence_same_truck(p, trial, t1, lo, hi);
                    } else {
                        repair_cross_truck_precedence_and_reassign(p, trial, {t1, t2}, city_i, city_j);
                    }
                    double ts = 0.0; auto res = fitness_full(p, trial, &ts); if (!res.first) continue;
                    std::vector<int> touched{city_i, city_j};
                    if (!tabu_allows_move(touched, tabu_list, loop_idx, tabu_tenure, res.second, best_global_fit)) continue;
                    if (better_pair(res.second, ts, best_fit, best_sum)) {
                        if (t1 == t2 && ep_swap.changed_events > 0) {
                            std::cout << "[REPAIR] feasible endpoint swap truck " << t1
                                      << " city " << ep_swap.city_i << " <-> " << ep_swap.city_j
                                      << " changed_events " << ep_swap.changed_events
                                      << " fit " << res.second << "\n";
                        }
                        best_fit = res.second; best_sum = ts; best_sol = std::move(trial); best_touched = touched; found = true;
                    }
                }
            }
        }
    }
    if (found) {
        out.moved = true;
        out.fit = best_fit;
        out.sum = best_sum;
        out.touched = std::move(best_touched);
        s = std::move(best_sol);
    }
    return out;
}

// 2-1 move: relocate two consecutive customers
static NeighborhoodApplyResult truck_move_2_1_best(
    const Params &p,
    Solution &s,
    const std::vector<int> *tabu_list = nullptr,
    int loop_idx = 0,
    double tabu_tenure = 0.0,
    double best_global_fit = std::numeric_limits<double>::infinity()
) {
    NeighborhoodApplyResult out;
    double base_sum = 0.0; auto base = fitness_full(p, s, &base_sum); if (!base.first) return out;
    double best_fit = std::numeric_limits<double>::infinity();
    double best_sum = std::numeric_limits<double>::infinity();
    Solution best_sol = s;
    std::vector<int> best_touched;
    bool found = false;
    int K = p.n_truck;
    for (int ta = 0; ta < K; ++ta) {
        if (s.trucks[ta].stops.size() <= 4) continue;
        for (size_t ia = 1; ia + 2 < s.trucks[ta].stops.size(); ++ia) {
            for (int tb = 0; tb < K; ++tb) {
                for (size_t ib = 1; ib < s.trucks[tb].stops.size(); ++ib) {
                    if (ta == tb && (ib == ia || ib == ia+1 || ib == ia+2)) continue;
                    Solution trial = s;
                    auto &ra = trial.trucks[ta].stops;
                    auto &rb = trial.trucks[tb].stops;
                    std::vector<TruckStop> seg{ra[ia], ra[ia+1]};
                    int city_i = seg[0].customer;
                    int city_j = seg[1].customer;
                    ra.erase(ra.begin()+ia, ra.begin()+ia+2);
                    size_t adj_ib = ib;
                    if (ta == tb) {
                        if (ib <= ia+1) continue; // would insert inside removed segment
                        adj_ib = ib - 2;
                    }
                    if (adj_ib > rb.size()) adj_ib = rb.size();
                    rb.insert(rb.begin()+adj_ib, seg.begin(), seg.end());
                    repair_and_refine_after_truck_move(p, trial, {ta,tb});
                    if (ta == tb) {
                        int lo = std::min((int)ia, (int)adj_ib);
                        int hi = std::max((int)ia + 1, (int)adj_ib + 1);
                        repair_segment_precedence_same_truck(p, trial, ta, lo, hi);
                    } else {
                        // Reuse cross-truck repair to relocate moved packages to valid owner/resupply points.
                        repair_cross_truck_precedence_and_reassign(p, trial, {ta, tb}, city_i, city_j);
                    }
                    double ts = 0.0; auto res = fitness_full(p, trial, &ts); if (!res.first) continue;
                    int city_k = s.trucks[tb].stops[std::min(ib, s.trucks[tb].stops.size()-1)].customer;
                    std::vector<int> touched{city_i, city_j, city_k};
                    if (!tabu_allows_move(touched, tabu_list, loop_idx, tabu_tenure, res.second, best_global_fit)) continue;
                    if (!found || better_pair(res.second, ts, best_fit, best_sum)) {
                        best_fit = res.second; best_sum = ts; best_sol = std::move(trial); best_touched = touched; found = true;
                    }
                }
            }
        }
    }
    if (found) {
        out.moved = true;
        out.fit = best_fit;
        out.sum = best_sum;
        out.touched = std::move(best_touched);
        s = std::move(best_sol);
    }
    return out;
}

// ---------------- Tabu search on truck routes only ----------------

static void tabu_search_truck(const Params &p, Solution &s) {
    int n = (int)p.customers.size()-1;
    if (n <= 1) return;
    int tabu_tenure = std::max(1, (int)std::round(2*std::log((double)n)));
    std::vector<int> tabu_customer(n+1, -1000000);

    auto best = s;
    double best_sum = 0.0; auto best_res = fitness_full(p, best, &best_sum); if (!best_res.first) return;
    double best_fit = best_res.second;

    const int MAX_IT = 50;
    for (int it = 0; it < MAX_IT; ++it) {
        double cur_best_fit = std::numeric_limits<double>::infinity();
        double cur_best_sum = std::numeric_limits<double>::infinity();
        Solution cur_best = s;
        bool found = false;

        auto consider = [&](Solution &&trial, const std::vector<int> &touched, bool &got_global){
            double ts = 0.0; auto res = fitness_full(p, trial, &ts);
            if (!res.first) return;
            double f = res.second;
            bool tabu = false;
            for (int c : touched) if (c>0 && it < tabu_customer[c]) { tabu=true; break; }
            bool better_global = (f + 1e-9 < best_fit) || (std::fabs(f-best_fit)<=1e-9 && ts + 1e-9 < best_sum);
            if (tabu && !better_global) return; // aspiration
            if ((f + 1e-9 < cur_best_fit) || (std::fabs(f-cur_best_fit)<=1e-9 && ts + 1e-9 < cur_best_sum)) {
                cur_best_fit = f; cur_best_sum = ts; cur_best = std::move(trial); found = true;
                if (better_global) got_global = true;
            }
        };

        bool restrict_worst = false; // first segment iteration explores all trucks
        int worst = worst_truck_index(p, s);

        // Neighborhood 1-0
        for (int ta=0; ta<p.n_truck; ++ta) {
            for (size_t ia=1; ia+1<s.trucks[ta].stops.size(); ++ia) {
                if (restrict_worst && ta != worst) continue;
                for (int tb=0; tb<p.n_truck; ++tb) {
                    for (size_t ib=1; ib<s.trucks[tb].stops.size(); ++ib) {
                        if (ta==tb && (ib==ia || ib==ia+1)) continue;
                        Solution trial=s; auto &ra=trial.trucks[ta].stops; auto &rb=trial.trucks[tb].stops;
                        TruckStop mv=ra[ia]; ra.erase(ra.begin()+ia); rb.insert(rb.begin()+ib,mv);
                        repair_after_truck_move(trial, {ta,tb});
                        bool got=false; consider(std::move(trial), {mv.customer}, got); if (got) restrict_worst = true;
                    }
                }
            }
        }
        // Neighborhood 1-1
        for (int t1=0;t1<p.n_truck;++t1){
            for (size_t i=1;i+1<s.trucks[t1].stops.size();++i){
                for (int t2=t1;t2<p.n_truck;++t2){
                    if (restrict_worst && t1!=worst && t2!=worst) continue;
                    size_t jstart=(t2==t1)?i+1:1;
                    for (size_t j=jstart;j+1<s.trucks[t2].stops.size();++j){
                        Solution trial=s; std::swap(trial.trucks[t1].stops[i], trial.trucks[t2].stops[j]);
                        repair_after_truck_move(trial, {t1,t2});
                        bool got=false; consider(std::move(trial), {s.trucks[t1].stops[i].customer, s.trucks[t2].stops[j].customer}, got); if (got) restrict_worst = true;
                    }
                }
            }
        }
        // Neighborhood 2-1 (two consecutive)
        for (int ta=0; ta<p.n_truck; ++ta){
            if (s.trucks[ta].stops.size()<=4) continue;
            if (restrict_worst && ta != worst) continue;
            for (size_t ia=1; ia+2<s.trucks[ta].stops.size(); ++ia){
                for (int tb=0; tb<p.n_truck; ++tb){
                    for (size_t ib=1; ib<s.trucks[tb].stops.size(); ++ib){
                        if (ta==tb && (ib==ia||ib==ia+1||ib==ia+2)) continue;
                        Solution trial=s; auto &ra=trial.trucks[ta].stops; auto &rb=trial.trucks[tb].stops;
                        std::vector<TruckStop> seg{ra[ia], ra[ia+1]}; int c1=ra[ia].customer, c2=ra[ia+1].customer;
                        ra.erase(ra.begin()+ia, ra.begin()+ia+2); rb.insert(rb.begin()+ib, seg.begin(), seg.end());
                        repair_after_truck_move(trial, {ta,tb});
                        bool got=false; consider(std::move(trial), {c1,c2}, got); if (got) restrict_worst = true;
                    }
                }
            }
        }
        if (!found) break;
        // apply best move
        s = std::move(cur_best);
        // update tabu marks for customers involved in move vs previous s? simplify: mark all customers that changed position.
        // For simplicity mark all customers (excluding depot) on both routes as tabu? too strong. Instead compare to previous best? Skipped: mark none.
        // Update global best
        if ((cur_best_fit + 1e-9 < best_fit) || (std::fabs(cur_best_fit-best_fit)<=1e-9 && cur_best_sum + 1e-9 < best_sum)) {
            best_fit = cur_best_fit; best_sum = cur_best_sum; best = s;
        }
    }
    s = std::move(best);
}

// Simple ATS skeleton on trucks only (no diversification, no drone LS inside)
static bool merge_adjacent_trips(const Params &p, Solution &s);
static bool insert_rendezvous_first_improve(const Params &p, Solution &s);
static bool is_pkg_resupplied(const Solution &s, int pkg);

static void apply_drone_local_search(const Params &p, Solution &s) {
    // Imitate test_similarity: three operators, first-improve, up to 2 passes, stop if a full pass has no improvement.
    for (int pass = 0; pass < 2; ++pass) {
        bool improved = false;
        if (relocate_sync_point_first_improve(p, s)) improved = true;
        if (relocate_package_first_improve(p, s)) improved = true;
        if (reorder_trip_first_improve(p, s)) improved = true;
        if (!improved) break;
    }
}

static NeighborhoodApplyResult apply_truck_neighborhood(
    int nid,
    const Params &p,
    Solution &s,
    const std::vector<int> *tabu_list = nullptr,
    int loop_idx = 0,
    double tabu_tenure = 0.0,
    double best_global_fit = std::numeric_limits<double>::infinity()
) {
    if (nid == 0) return truck_move_1_0_best(p, s, tabu_list, loop_idx, tabu_tenure, best_global_fit);
    if (nid == 1) return truck_move_1_1_best(p, s, tabu_list, loop_idx, tabu_tenure, best_global_fit);
    if (nid == 2) return truck_move_2_1_best(p, s, tabu_list, loop_idx, tabu_tenure, best_global_fit);
    if (nid == 3) {
        // Two-opt inside ATS: try repair/refine; if still infeasible, skip this candidate.
        NeighborhoodApplyResult out;
        double base_sum = 0.0;
        auto base = fitness_full(p, s, &base_sum);
        if (!base.first) return out;
        Solution trial = s;
        improve_truck_routes_2opt(p, trial);
        std::vector<int> all_trucks;
        all_trucks.reserve((size_t)p.n_truck);
        for (int t = 0; t < p.n_truck; ++t) all_trucks.push_back(t);
        repair_and_refine_after_truck_move(p, trial, all_trucks);
        double ts = 0.0;
        auto res = fitness_full(p, trial, &ts);
        if (!res.first) return out;
        std::vector<int> touched;
        if ((int)trial.trucks.size() > 0 && trial.trucks[0].stops.size() > 2) {
            touched.push_back(trial.trucks[0].stops[1].customer);
            touched.push_back(trial.trucks[0].stops[trial.trucks[0].stops.size()-2].customer);
        }
        if (!tabu_allows_move(touched, tabu_list, loop_idx, tabu_tenure, res.second, best_global_fit)) return out;
        if (better_pair(res.second, ts, base.second, base_sum)) {
            s = std::move(trial);
            out.moved = true;
            out.fit = res.second;
            out.sum = ts;
            out.touched = std::move(touched);
        }
        return out;
    }
    return NeighborhoodApplyResult{};
}

// Try to merge consecutive drone trips when feasible (simple heuristic)
static bool merge_adjacent_trips(const Params &p, Solution &s) {
    if (s.drone_queue.size() < 2) return false;
    std::vector<DroneTrip> newq;
    bool merged = false;
    for (size_t i = 0; i < s.drone_queue.size(); ++i) {
        if (merged || i+1 >= s.drone_queue.size()) { newq.push_back(s.drone_queue[i]); continue; }
        auto cand = s.drone_queue[i];
        auto next = s.drone_queue[i+1];
        // simple rule: if first rendezvous of next happens after last of cand on same truck, try concatenation
        int tA = cand.events.front().truck_id;
        int tB = next.events.front().truck_id;
        if (tA != tB) { newq.push_back(s.drone_queue[i]); continue; }
        // build merged trip
        DroneTrip m = cand;
        m.events.insert(m.events.end(), next.events.begin(), next.events.end());
        // quick optimistic endurance check
        if (trip_endurance_optimistic(p, s, m)) {
            newq.push_back(std::move(m));
            merged = true; ++i; // skip next
        } else {
            newq.push_back(s.drone_queue[i]);
        }
    }
    if (merged) s.drone_queue.swap(newq);
    return merged;
}

// Insert a new rendezvous into an existing trip (different truck than already visited) to create multi-visit.
static bool insert_rendezvous_first_improve(const Params &p, Solution &s) {
    int K = p.n_truck;
    int n = (int)p.customers.size()-1;
    if (s.drone_queue.empty()) return false;
    // precompute truck timelines for arrival times
    std::vector<TruckTimeline> tls(K);
    for (int k=0;k<K;++k) tls[k]=compute_truck_timeline(p, s.trucks[k]);

    for (size_t t_idx=0; t_idx<s.drone_queue.size(); ++t_idx) {
        const auto &trip = s.drone_queue[t_idx];
        // trucks already in this trip
        std::unordered_set<int> used_truck;
        for (auto &ev : trip.events) used_truck.insert(ev.truck_id);

        for (int k=0;k<K;++k) {
            if (used_truck.count(k)) continue; // không gặp 1 truck 2 lần
            const auto &stops = s.trucks[k].stops;
            for (size_t pos=1; pos+1<stops.size(); ++pos) {
                int city = stops[pos].customer;
                if (!p.reachable_mask.empty() && !p.reachable_mask[city]) continue;
                if (is_pkg_resupplied(s, city)) continue; // đã resupply
                int demand = p.customers[city].demand;
                if (trip_load(p, trip) + demand > p.M_d + 1e-9) continue;

                // build trial
                Solution trial = s;
                auto &trip_new = trial.drone_queue[t_idx];
                // determine insert position in events by truck arrival order
                double arr_city = tls[k].arrival[pos];
                size_t insert_pos = trip_new.events.size();
                for (size_t e=0; e<trip_new.events.size(); ++e) {
                    int tk = trip_new.events[e].truck_id;
                    int ct = trip_new.events[e].rendezvous_customer;
                    double arr_e = truck_arrival_at(trial, tls, tk, ct);
                    if (arr_city < arr_e) { insert_pos = e; break; }
                }
                ResupplyEvent new_ev{city, k, {city}};
                trip_new.events.insert(trip_new.events.begin()+insert_pos, new_ev);
                // mark loaded_from_drone
                auto &stops_mut = trial.trucks[k].stops;
                for (auto &st : stops_mut) if (st.customer==city) { st.loaded_from_drone.push_back(city); break; }
                // endurance optimistic check
                if (!trip_endurance_optimistic(p, trial, trip_new)) continue;
                double ts=0.0; auto res=fitness_full(p, trial, &ts);
                if (!res.first) continue;
                // accept first improvement
                s = std::move(trial); return true;
            }
        }
    }
    return false;
}

static bool has_multi_visit(const Solution &s);

// Adaptive Tabu Search skeleton (no diversification), with roulette-wheel neighborhood selection.
// Simple diversification: reverse subsequences inside each truck route (excluding depots)
static Solution diversify_solution(const Params &p, const Solution &s, std::mt19937 &rng) {
    Solution best = s;
    double best_sum = 0.0; auto base = fitness_full(p, best, &best_sum);
    if (!base.first) return s; // should not happen, keep original

    for (size_t k = 0; k < s.trucks.size(); ++k) {
        const auto &route = s.trucks[k].stops;
        int cust_cnt = (int)route.size() - 2; // exclude depots
        if (cust_cnt < 4) continue; // need at least 4 to reverse meaningfully
        int max_len = cust_cnt / 2;
        if (max_len < 2) continue;
        std::uniform_int_distribution<int> len_dist(2, max_len);
        int r = len_dist(rng);
        for (int len = r; len <= max_len; ++len) {
            for (int start = 1; start + len <= (int)route.size() - 1; ++start) {
                Solution cand = s;
                auto &st = cand.trucks[k].stops;
                std::reverse(st.begin() + start, st.begin() + start + len);
                // any truck move invalidates drone plan
                repair_after_truck_move(cand);
                double ts = 0.0; auto fr = fitness_full(p, cand, &ts);
                if (fr.first && (fr.second + 1e-9 < base.second) && (fr.second + 1e-9 < best_sum || fr.second + 1e-9 < base.second)) {
                    best = std::move(cand);
                    best_sum = ts;
                    base.second = fr.second;
                }
            }
        }
    }
    return best;
}

static void ats_full(const Params &p, Solution &s, int SEG = 4, double theta = 2.0, int DIV = 3) {
    // Exploration settings tuned per request:
    int NIMP = 20;          // iterations without improvement to end a segment
    SEG = 10;                // segments without improvement before diversification
    DIV = 3;                // diversification rounds without improvement to stop
    const int neigh_count = 4; // truck neighborhoods only: 1-0, 1-1, 2-1, 2-opt
    std::vector<double> weight(neigh_count, 1.0 / neigh_count);
    std::vector<double> score(neigh_count, 0.0);
    std::vector<int> use_cnt(neigh_count, 0);
    const double gamma1 = 5, gamma2 = 2, gamma3 = 1, gamma4 = 0.6;

    struct LogEntry {int seg; int iter; int div; int nid; double fit_cur; double fit_best_global; double fit_best_segment; bool improved; bool multi_visit; double best_multi_fit;};
    std::vector<LogEntry> log_entries;

    double best_sum = 0.0; auto base = fitness_full(p, s, &best_sum); if (!base.first) return;
    double best_fit = base.second; Solution best_sol = s;
    double best_multi_fit = has_multi_visit(s) ? best_fit : std::numeric_limits<double>::infinity();
    Solution best_multi_sol = has_multi_visit(s) ? s : Solution{};

    int seg_no_improve = 0;
    int div_no_improve = 0;
    bool stopped_by_time = false;
    std::mt19937 rng{std::random_device{}()};

    int div_round = 0;
    log_entries.clear();
    while (div_no_improve < DIV) {
        if (solver_time_limit_reached()) { stopped_by_time = true; break; }
        seg_no_improve = 0;
        int seg_idx = 0;
        while (seg_no_improve < SEG) {
            if (solver_time_limit_reached()) { stopped_by_time = true; break; }
            std::fill(score.begin(), score.end(), 0.0);
            std::fill(use_cnt.begin(), use_cnt.end(), 0);
            double tabu_tenure0 = std::uniform_real_distribution<double>(2.0*std::log((double)p.customers.size()), (double)p.customers.size())(rng);
            double tabu_tenure1 = std::uniform_real_distribution<double>(2.0*std::log((double)p.customers.size()), (double)p.customers.size())(rng);
            double tabu_tenure2 = std::uniform_real_distribution<double>(2.0*std::log((double)p.customers.size()), (double)p.customers.size())(rng);
            double tabu_tenure3 = std::uniform_real_distribution<double>(2.0*std::log((double)p.customers.size()), (double)p.customers.size())(rng);
            std::vector<int> tabu0((int)p.customers.size(), -1000000);
            std::vector<int> tabu1((int)p.customers.size(), -1000000);
            std::vector<int> tabu2((int)p.customers.size(), -1000000);
            std::vector<int> tabu3((int)p.customers.size(), -1000000);
            std::array<int,4> loop_idx_by_nei{0,0,0,0};
            int no_imp_iter = 0;
            int iter_idx = 0;
            bool first_iter_segment = true;
            bool segment_improved_global = false;
            while (no_imp_iter < NIMP) {
                if (solver_time_limit_reached()) { stopped_by_time = true; break; }
                // roulette selection
                std::discrete_distribution<int> dist(weight.begin(), weight.end());
                int nid = dist(rng);

                Solution cur = s; // current solution
                std::vector<int> *tabu_ptr = nullptr;
                double tenure = 0.0;
                if (nid == 0) { tabu_ptr = &tabu0; tenure = tabu_tenure0; }
                else if (nid == 1) { tabu_ptr = &tabu1; tenure = tabu_tenure1; }
                else if (nid == 2) { tabu_ptr = &tabu2; tenure = tabu_tenure2; }
                else if (nid == 3) { tabu_ptr = &tabu3; tenure = tabu_tenure3; }
                NeighborhoodApplyResult move_res = apply_truck_neighborhood(
                    nid, p, cur, tabu_ptr, loop_idx_by_nei[nid], tenure, best_fit
                );
                if (!move_res.moved) { no_imp_iter++; continue; }

                // local search on truck result: use drone LS for refinement
                apply_drone_local_search(p, cur);
                if (first_iter_segment) {
                    merge_adjacent_trips(p, cur);
                    first_iter_segment = false;
                }

                double ts = 0.0; auto res = fitness_full(p, cur, &ts);
                if (!res.first) { no_imp_iter++; continue; }
                use_cnt[nid]++;
                for (int c : move_res.touched) {
                    if (tabu_ptr && c > 0 && c < (int)tabu_ptr->size()) {
                        (*tabu_ptr)[c] = loop_idx_by_nei[nid];
                    }
                }
                loop_idx_by_nei[nid]++;
                double current_fit = fitness_full(p, s, nullptr).second;
                bool improved_global = false;
                if (res.second + 1e-9 < best_fit) {
                    score[nid] += gamma1; best_fit = res.second; best_sum = ts; best_sol = cur; s = cur; no_imp_iter = 0; seg_no_improve = 0; improved_global = true; segment_improved_global = true;
                } else if (res.second + 1e-9 < current_fit || (std::fabs(res.second - current_fit) < 1e-9 && ts + 1e-9 < best_sum)) {
                    score[nid] += gamma2; s = cur; no_imp_iter = 0;
                } else { score[nid] += gamma3; no_imp_iter++; }

                if (has_multi_visit(cur) && res.second + 1e-9 < best_multi_fit) {
                    std::string mv_reason;
                    if (validate_solution(p, cur, mv_reason)) {
                        best_multi_fit = res.second;
                        best_multi_sol = cur;
                        std::cout << "[ATS] new best multi-visit fit " << best_multi_fit << "\n";
                    }
                }

                // Debug print per iteration: segment, iteration, diversification count, best global, best current
                double seg_best_fit = fitness_full(p, s, nullptr).second;
                std::cout << "[ATS] seg " << seg_idx << " iter " << iter_idx
                          << " div_round " << div_round
                          << " nid " << nid
                          << " fit_cur " << res.second
                          << " fit_best_global " << best_fit
                          << " fit_best_segment " << seg_best_fit
                          << (improved_global ? " *" : "") << "\n";
                log_entries.push_back({seg_idx, iter_idx, div_round, nid, res.second, best_fit, seg_best_fit, improved_global, has_multi_visit(cur), best_multi_fit});
                iter_idx++;
            }
            if (stopped_by_time) break;

            // segment ends
            // Carry current solution to the next segment (same behavior as test_similarity).
            // Tabu structures are reset per segment, so no forced rollback to best here.
            if (!segment_improved_global) seg_no_improve++;
            seg_idx++;

            // update weights
            for (int i = 0; i < neigh_count; ++i) {
                if (use_cnt[i] > 0) weight[i] = (1 - gamma4) * weight[i] + gamma4 * (score[i] / use_cnt[i]);
            }
        }
        if (stopped_by_time) break;

        // diversification phase
        Solution div_sol = diversify_solution(p, best_sol, rng);
        double div_sum = 0.0; auto fres = fitness_full(p, div_sol, &div_sum);
        if (fres.first && (fres.second + 1e-9 < best_fit)) {
            best_fit = fres.second; best_sum = div_sum; best_sol = div_sol; s = div_sol; div_no_improve = 0;
            if (has_multi_visit(div_sol) && fres.second + 1e-9 < best_multi_fit) {
                std::string mv_reason;
                if (validate_solution(p, div_sol, mv_reason)) {
                    best_multi_fit = fres.second;
                    best_multi_sol = div_sol;
                    std::cout << "[ATS] new best multi-visit fit " << best_multi_fit << " (after diversification)\n";
                }
            }
        } else {
            div_no_improve++;
        }
        std::cout << "[ATS] diversification round " << div_round << " best_fit " << best_fit << "\n";
        log_entries.push_back({-1, -1, div_round, -1, fres.second, best_fit, best_fit, fres.second + 1e-9 < best_fit, has_multi_visit(div_sol), best_multi_fit});
        div_round++;

        // reset weights after diversification
        std::fill(weight.begin(), weight.end(), 1.0 / neigh_count);
    }

    // Final attempt: run merge_adjacent_trips once on best_sol to seek a multi-visit improvement before returning.
    if (!stopped_by_time) {
        Solution final_sol = best_sol;
        if (merge_adjacent_trips(p, final_sol)) {
            double ts = 0.0; auto res = fitness_full(p, final_sol, &ts);
            if (res.first && res.second + 1e-9 < best_fit) {
                best_fit = res.second; best_sum = ts; best_sol = final_sol;
            }
        }
    }
    s = best_sol;
    if (stopped_by_time) {
        std::cout << "[ATS] time limit reached at " << elapsed_solver_seconds()
                  << "s, returning best-so-far\n";
    }

    // Report best multi-visit solution found (if any), even if not global best
    if (best_multi_fit < std::numeric_limits<double>::infinity()) {
        std::cout << "[ATS] best multi-visit makespan: " << best_multi_fit << "\n";
        print_truck_routes(best_multi_sol);
        g_has_best_multi_solution = true;
        g_best_multi_solution = best_multi_sol;
    } else {
        std::cout << "[ATS] no multi-visit solution found during search\n";
        g_has_best_multi_solution = false;
        g_best_multi_solution = Solution{};
    }

    // write log to CSV
    std::string fname = "ats_log_" + g_data_name + ".csv";
    std::ofstream csv(fname);
    if (csv) {
        csv << "seg,iter,div_round,nid,fit_cur,fit_best_global,fit_best_segment,improved,multi_visit,best_multi_fit\n";
        for (const auto &e : log_entries) {
            csv << e.seg << ',' << e.iter << ',' << e.div << ',' << e.nid << ','
                << e.fit_cur << ',' << e.fit_best_global << ',' << e.fit_best_segment << ','
                << (e.improved ? 1 : 0) << ',' << (e.multi_visit ? 1 : 0) << ','
                << e.best_multi_fit << "\n";
        }
        std::cout << "[ATS] log saved to " << fname << "\n";
    }
}

static int route_pos(const Solution &s, int truck_id, int customer) {
    const auto &stops = s.trucks[truck_id].stops;
    for (size_t i = 0; i < stops.size(); ++i) if (stops[i].customer == customer) return (int)i;
    return -1;
}

static int trip_load(const Params &p, const DroneTrip &trip) {
    int load = 0;
    for (const auto &ev : trip.events)
        for (auto pk : ev.packages) load += p.customers[pk].demand;
    return load;
}

static bool is_pkg_resupplied(const Solution &s, int pkg) {
    for (const auto &trip : s.drone_queue)
        for (const auto &ev : trip.events)
            for (auto pk : ev.packages)
                if (pk == pkg) return true;
    return false;
}

// `loaded_from_drone` is derived from `drone_queue` and is used by the timing model.
// Keep it consistent by rebuilding after any move that changes drone rendezvous/packages.
static void rebuild_loaded_from_drone_marks(const Params &p, Solution &s) {
    (void)p;
    for (auto &tr : s.trucks) {
        for (auto &st : tr.stops) st.loaded_from_drone.clear();
    }
    for (const auto &trip : s.drone_queue) {
        for (const auto &ev : trip.events) {
            if (ev.truck_id < 0 || ev.truck_id >= (int)s.trucks.size()) continue;
            auto &stops = s.trucks[ev.truck_id].stops;
            const int rv = ev.rendezvous_customer;
            for (auto &st : stops) {
                if (st.customer == rv) {
                    st.loaded_from_drone.insert(st.loaded_from_drone.end(), ev.packages.begin(), ev.packages.end());
                    break;
                }
            }
        }
    }
}

// forward declarations for helpers used earlier
static bool has_multi_visit(const Solution &s);


static bool has_multi_visit(const Solution &s) {
    for (const auto &t : s.drone_queue) {
        if (t.events.size() <= 1) continue;
        std::unordered_set<int> trucks;
        for (const auto &ev : t.events) trucks.insert(ev.truck_id);
        if (trucks.size() >= 2) return true;
    }
    return false;
}

static std::string sanitize_token(const std::string &s) {
    std::string out;
    out.reserve(s.size());
    for (char ch : s) {
        if (std::isalnum(static_cast<unsigned char>(ch)) || ch == '_' || ch == '-' || ch == '.') out.push_back(ch);
        else out.push_back('_');
    }
    return out;
}

static std::string format_scalar(double v) {
    std::ostringstream oss;
    if (std::fabs(v - std::round(v)) < 1e-9) oss << static_cast<long long>(std::llround(v));
    else {
        oss << std::fixed << std::setprecision(3) << v;
        std::string t = oss.str();
        while (!t.empty() && t.back() == '0') t.pop_back();
        if (!t.empty() && t.back() == '.') t.pop_back();
        return t;
    }
    return oss.str();
}

static void print_solution_compact(std::ostream &ofs, const Solution &sol, const Params &p) {
    // Format:
    // solution = [ [truck routes], [drone trips] ]
    // Truck stop entry: [city, packages_received_at_city]
    // At first depot (0): packages loaded by truck (not resupplied by drone).
    std::vector<char> is_resupplied(p.customers.size(), 0);
    std::unordered_map<long long, std::vector<int>> inferred_resupply;
    auto make_key = [](int truck_id, int city) -> long long {
        return (static_cast<long long>(truck_id) << 32) ^ static_cast<unsigned int>(city);
    };
    for (const auto &trip : sol.drone_queue) {
        for (const auto &ev : trip.events) {
            auto &bucket = inferred_resupply[make_key(ev.truck_id, ev.rendezvous_customer)];
            bucket.insert(bucket.end(), ev.packages.begin(), ev.packages.end());
            for (auto pkg : ev.packages) {
                if (pkg >= 0 && pkg < static_cast<int>(is_resupplied.size())) is_resupplied[pkg] = 1;
            }
        }
    }

    ofs << "solution = [\n";
    ofs << "    [\n";
    for (size_t t = 0; t < sol.trucks.size(); ++t) {
        const auto &tr = sol.trucks[t];
        std::vector<int> depot_loaded;
        std::vector<char> seen(p.customers.size(), 0);
        for (size_t i = 1; i + 1 < tr.stops.size(); ++i) {
            int cust = tr.stops[i].customer;
            if (cust > 0 && cust < static_cast<int>(seen.size()) && !seen[cust] && !is_resupplied[cust]) {
                seen[cust] = 1;
                depot_loaded.push_back(cust);
            }
        }

        ofs << "        [\n";
        for (size_t i = 0; i < tr.stops.size(); ++i) {
            const auto &st = tr.stops[i];
            if (i + 1 == tr.stops.size() && st.customer == 0) continue; // omit final depot
            ofs << "            [" << st.customer << ", [";
            std::vector<int> pkgs;
            if (i == 0) {
                pkgs = depot_loaded;
            } else {
                auto it = inferred_resupply.find(make_key(tr.truck_id, st.customer));
                if (it != inferred_resupply.end()) pkgs = it->second;
                else pkgs.clear();
            }
            for (size_t j = 0; j < pkgs.size(); ++j) {
                if (j) ofs << ", ";
                ofs << pkgs[j];
            }
            ofs << "]]";
            bool has_more = false;
            for (size_t k = i + 1; k < tr.stops.size(); ++k) {
                if (!(k + 1 == tr.stops.size() && tr.stops[k].customer == 0)) { has_more = true; break; }
            }
            if (has_more) ofs << ",";
            ofs << "\n";
        }
        ofs << "        ]";
        if (t + 1 < sol.trucks.size()) ofs << ",";
        ofs << "\n";
    }
    ofs << "    ],\n";
    ofs << "    [\n";
    for (size_t q = 0; q < sol.drone_queue.size(); ++q) {
        const auto &trip = sol.drone_queue[q];
        ofs << "        [\n";
        for (size_t e = 0; e < trip.events.size(); ++e) {
            const auto &ev = trip.events[e];
            ofs << "            [" << ev.rendezvous_customer << ", [";
            for (size_t k = 0; k < ev.packages.size(); ++k) {
                if (k) ofs << ", ";
                ofs << ev.packages[k];
            }
            ofs << "]]";
            if (e + 1 < trip.events.size()) ofs << ",";
            ofs << "\n";
        }
        ofs << "        ]";
        if (q + 1 < sol.drone_queue.size()) ofs << ",";
        ofs << "\n";
    }
    ofs << "    ]\n";
    ofs << "]\n";
}

static std::string write_solution_json(const Solution &sol, const Params &p, const std::string &tag) {
    const char* out_dir_env = std::getenv("SOL_OUT_DIR");
    std::string out_dir = out_dir_env ? out_dir_env : ".";
    std::ostringstream name;
    name << out_dir << "/sol_" << tag
         << "_" << sanitize_token(g_data_name)
         << "_A" << format_scalar(p.M_d)
         << "_L" << format_scalar(p.L_d)
         << "_pid" << static_cast<long long>(::getpid())
         << ".txt";
    std::string path = name.str();

    std::ofstream ofs(path);
    if (!ofs) return std::string();

    print_solution_compact(ofs, sol, p);
    return path;
}

// Lightweight drone LS: single pass, three operators, first-improve only.
static void apply_drone_local_search_light(const Params &p, Solution &s) {
    bool improved = false;
    if (relocate_sync_point_first_improve(p, s)) improved = true;
    if (relocate_package_first_improve(p, s)) improved = true;
    if (reorder_trip_first_improve(p, s)) improved = true;
    (void)improved;
}

// After truck route changes, reset drone assignments to ensure feasibility


// After truck route changes, wipe drone assignments to obtain a guaranteed-feasible base.
static void repair_after_truck_move(Solution &sol, const std::vector<int> &affected_trucks) {
    // Keep as many rendezvous as possible; if city moved to another truck, reassign to that truck.
    std::unordered_set<int> aff(affected_trucks.begin(), affected_trucks.end());
    // Map city -> truck_id containing it (first one if multiple)
    std::unordered_map<int,int> city_owner;
    for (auto &tr : sol.trucks) {
        for (auto &st : tr.stops) {
            if (st.customer==0) continue;
            if (!city_owner.count(st.customer)) city_owner[st.customer] = tr.truck_id;
        }
    }

    // Clear loaded_from_drone on affected trucks; leave others intact
    for (auto &tr : sol.trucks) {
        if (!aff.count(tr.truck_id)) continue;
        for (auto &st : tr.stops) st.loaded_from_drone.clear();
    }

    std::vector<DroneTrip> kept; kept.reserve(sol.drone_queue.size());
    for (auto &trip : sol.drone_queue) {
        DroneTrip filtered;
        for (auto ev : trip.events) {
            auto it = city_owner.find(ev.rendezvous_customer);
            if (it == city_owner.end()) continue; // city no longer exists
            ev.truck_id = it->second; // possibly reassigned to new truck
            filtered.events.push_back(ev);
        }
        if (!filtered.events.empty()) kept.push_back(std::move(filtered));
    }

    // enforce queue ordering: for each truck, trips sorted by first rendezvous order along the truck route
    sol.drone_queue.swap(kept);
    std::stable_sort(sol.drone_queue.begin(), sol.drone_queue.end(), [&](const DroneTrip &a, const DroneTrip &b){
        int ta = a.events.front().truck_id;
        int tb = b.events.front().truck_id;
        if (ta != tb) return ta < tb;
        int ca = a.events.front().rendezvous_customer;
        int cb = b.events.front().rendezvous_customer;
        // find position on truck ta route
        int posa = 1e9, posb = 1e9;
        const auto &stops = sol.trucks[ta].stops;
        for (size_t i=0;i<stops.size();++i){ if (stops[i].customer==ca){ posa=(int)i; break;} }
        for (size_t i=0;i<stops.size();++i){ if (stops[i].customer==cb){ posb=(int)i; break;} }
        return posa < posb;
    });
}

// Aggressive repair: cleanup drone events then run drone local search to rebuild rendezvous (similar tinh thần test_similarity)
static void repair_and_refine_after_truck_move(const Params &p, Solution &sol, const std::vector<int> &affected_trucks) {
    repair_after_truck_move(sol, affected_trucks);
    apply_drone_local_search_light(p, sol);
}

// For same-truck 1-1 swap, explicitly exchange rendezvous endpoints i<->j.
// This captures the case where packages resupplied at i should move to j and vice versa.
static EndpointSwapLogInfo swap_endpoint_resupply_same_truck(Solution &sol, int truck_id, int pos_i, int pos_j) {
    EndpointSwapLogInfo info;
    if (truck_id < 0 || truck_id >= (int)sol.trucks.size()) return info;
    const auto &stops = sol.trucks[truck_id].stops;
    if (pos_i < 0 || pos_j < 0 || pos_i >= (int)stops.size() || pos_j >= (int)stops.size()) return info;
    if (pos_i == pos_j) return info;
    int city_i = stops[pos_i].customer;
    int city_j = stops[pos_j].customer;
    info.city_i = city_i;
    info.city_j = city_j;
    if (city_i <= 0 || city_j <= 0) return info;

    for (auto &trip : sol.drone_queue) {
        for (auto &ev : trip.events) {
            if (ev.truck_id != truck_id) continue;
            int rv = ev.rendezvous_customer;
            if (rv == city_i) { ev.rendezvous_customer = city_j; info.changed_events++; }
            else if (rv == city_j) { ev.rendezvous_customer = city_i; info.changed_events++; }
        }
    }
    return info;
}

// For same-truck 1-0/1-1 moves: if there are rendezvous points inside the moved segment,
// drop packages that would now be resupplied after their customer has been visited.
static void repair_segment_precedence_same_truck(const Params &p, Solution &sol, int truck_id, int pos_lo, int pos_hi) {
    if (truck_id < 0 || truck_id >= (int)sol.trucks.size()) return;
    // Keep a stable snapshot of truck stops; `sol` may be reassigned inside repair attempts.
    const auto route = sol.trucks[truck_id].stops;
    if (route.empty()) return;
    if (pos_lo > pos_hi) std::swap(pos_lo, pos_hi);
    pos_lo = std::max(0, pos_lo);
    pos_hi = std::min((int)route.size() - 1, pos_hi);
    if (pos_lo > pos_hi) return;

    // Only act if the segment contains at least one rendezvous for this truck.
    bool has_sync_in_segment = false;
    for (const auto &trip : sol.drone_queue) {
        for (const auto &ev : trip.events) {
            if (ev.truck_id != truck_id) continue;
            int rv_pos = route_pos(sol, truck_id, ev.rendezvous_customer);
            if (rv_pos >= pos_lo && rv_pos <= pos_hi) {
                has_sync_in_segment = true;
                break;
            }
        }
        if (has_sync_in_segment) break;
    }
    if (!has_sync_in_segment) return;

    const int n = (int)p.customers.size();
    std::vector<int> pos_of(n, -1);
    for (size_t i = 0; i < route.size(); ++i) {
        int c = route[i].customer;
        if (c > 0 && c < n) pos_of[c] = (int)i;
    }
    std::unordered_set<int> displaced;

    // Remove precedence-violating packages from affected sync points.
    for (auto &trip : sol.drone_queue) {
        for (auto &ev : trip.events) {
            if (ev.truck_id != truck_id) continue;
            int rv_pos = route_pos(sol, truck_id, ev.rendezvous_customer);
            if (rv_pos < pos_lo || rv_pos > pos_hi) continue;

            std::vector<PackageId> kept;
            kept.reserve(ev.packages.size());
            for (PackageId pkg : ev.packages) {
                if (pkg <= 0 || pkg >= n) continue;
                int pkg_pos = pos_of[pkg];
                if (pkg_pos >= rv_pos) kept.push_back(pkg);
                else displaced.insert(pkg);
            }
            ev.packages.swap(kept);
        }
    }

    // Cleanup empty events/trips to keep the solution compact.
    for (auto &trip : sol.drone_queue) {
        std::vector<ResupplyEvent> kept_events;
        kept_events.reserve(trip.events.size());
        for (auto &ev : trip.events) if (!ev.packages.empty()) kept_events.push_back(ev);
        trip.events.swap(kept_events);
    }
    std::vector<DroneTrip> kept_trips;
    kept_trips.reserve(sol.drone_queue.size());
    for (auto &trip : sol.drone_queue) if (!trip.events.empty()) kept_trips.push_back(trip);
    sol.drone_queue.swap(kept_trips);

    auto city_has_rendezvous = [&](int city) {
        for (const auto &trip : sol.drone_queue)
            for (const auto &ev : trip.events)
                if (ev.rendezvous_customer == city) return true;
        return false;
    };

    auto try_assign_pkg_same_truck = [&](int pkg) -> bool {
        if (pkg <= 0 || pkg >= n) return false;
        if (pos_of[pkg] <= 0) return false;
        if (p.customers[pkg].demand > p.M_d + 1e-9) return false;
        if (is_pkg_resupplied(sol, pkg)) return true;

        for (int pos = pos_of[pkg]; pos >= 1; --pos) {
            int recv = route[pos].customer;
            if (recv <= 0) continue;
            if (!p.reachable_mask.empty() && !p.reachable_mask[recv]) continue;

            // Reuse existing event first.
            for (auto &trip : sol.drone_queue) {
                for (auto &ev : trip.events) {
                    if (ev.truck_id != truck_id || ev.rendezvous_customer != recv) continue;
                    if (trip_load(p, trip) + p.customers[pkg].demand > p.M_d + 1e-9) continue;
                    ev.packages.push_back(pkg);
                    if (trip_endurance_optimistic(p, sol, trip)) return true;
                    ev.packages.pop_back();
                }
            }

            // Create new singleton trip only if no trip already uses this rendezvous city.
            if (city_has_rendezvous(recv)) continue;
            DroneTrip new_trip;
            new_trip.events.push_back(ResupplyEvent{recv, truck_id, {pkg}});
            int ins_pos = find_insert_pos_queue(p, sol, new_trip);
            if (ins_pos < 0) continue;
            Solution trial = sol;
            trial.drone_queue.insert(trial.drone_queue.begin() + ins_pos, new_trip);
            if (!trip_endurance_optimistic(p, trial, trial.drone_queue[ins_pos])) continue;
            std::string reason;
            if (!validate_solution(p, trial, reason)) continue;
            sol = std::move(trial);
            return true;
        }
        return false;
    };

    std::vector<int> displaced_list(displaced.begin(), displaced.end());
    std::sort(displaced_list.begin(), displaced_list.end());
    for (int pkg : displaced_list) (void)try_assign_pkg_same_truck(pkg);
}

// For cross-truck 1-0/1-1 moves:
// - remove invalid drone package assignments on affected trucks
// - try to reassign removed packages onto valid sync points of their owner truck
// - fallback to depot-loading if no valid drone reassignment is found
static void repair_cross_truck_precedence_and_reassign(const Params &p, Solution &sol, const std::vector<int> &affected_trucks, int city_i, int city_j) {
    if (affected_trucks.empty()) return;
    std::unordered_set<int> aff(affected_trucks.begin(), affected_trucks.end());
    const int n = (int)p.customers.size();

    std::vector<int> owner_truck(n, -1), owner_pos(n, -1);
    for (size_t t = 0; t < sol.trucks.size(); ++t) {
        for (size_t pos = 0; pos < sol.trucks[t].stops.size(); ++pos) {
            int c = sol.trucks[t].stops[pos].customer;
            if (c > 0 && c < n) {
                owner_truck[c] = (int)t;
                owner_pos[c] = (int)pos;
            }
        }
    }

    auto strip_pkg_everywhere = [&](int pkg) {
        if (pkg <= 0 || pkg >= n) return;
        for (auto &trip : sol.drone_queue) {
            for (auto &ev : trip.events) {
                ev.packages.erase(
                    std::remove(ev.packages.begin(), ev.packages.end(), pkg),
                    ev.packages.end()
                );
            }
        }
    };

    auto cleanup_empty = [&]() {
        for (auto &trip : sol.drone_queue) {
            std::vector<ResupplyEvent> kept_events;
            kept_events.reserve(trip.events.size());
            for (auto &ev : trip.events) if (!ev.packages.empty()) kept_events.push_back(ev);
            trip.events.swap(kept_events);
        }
        std::vector<DroneTrip> kept_trips;
        kept_trips.reserve(sol.drone_queue.size());
        for (auto &trip : sol.drone_queue) if (!trip.events.empty()) kept_trips.push_back(trip);
        sol.drone_queue.swap(kept_trips);
    };

    auto swap_endpoints_packages = [&](int a, int b) {
        if (a <= 0 || b <= 0 || a >= n || b >= n || a == b) return;
        std::vector<std::pair<size_t,size_t>> ev_a, ev_b;
        for (size_t ti = 0; ti < sol.drone_queue.size(); ++ti) {
            for (size_t ei = 0; ei < sol.drone_queue[ti].events.size(); ++ei) {
                auto &ev = sol.drone_queue[ti].events[ei];
                if (ev.rendezvous_customer == a) ev_a.push_back({ti, ei});
                else if (ev.rendezvous_customer == b) ev_b.push_back({ti, ei});
            }
        }
        if (ev_a.empty() && ev_b.empty()) return;
        for (auto &x : ev_a) sol.drone_queue[x.first].events[x.second].rendezvous_customer = b;
        for (auto &x : ev_b) sol.drone_queue[x.first].events[x.second].rendezvous_customer = a;
    };

    // First attempt requested: exchange resupply endpoints between i and j.
    swap_endpoints_packages(city_i, city_j);
    // Remove i/j from old resupply assignments (including implicit depot by ensuring not in drone queue).
    strip_pkg_everywhere(city_i);
    strip_pkg_everywhere(city_j);
    cleanup_empty();

    std::unordered_set<int> displaced;

    // Remove invalid package assignments around affected trucks.
    for (auto &trip : sol.drone_queue) {
        for (auto &ev : trip.events) {
            if (!aff.count(ev.truck_id)) continue;
            int rv_pos = route_pos(sol, ev.truck_id, ev.rendezvous_customer);
            std::vector<PackageId> kept;
            kept.reserve(ev.packages.size());
            for (PackageId pkg : ev.packages) {
                if (pkg <= 0 || pkg >= n) continue;
                bool valid_owner = (owner_truck[pkg] == ev.truck_id);
                bool valid_order = (rv_pos >= 0 && owner_pos[pkg] >= rv_pos);
                if (valid_owner && valid_order) kept.push_back(pkg);
                else displaced.insert(pkg);
            }
            ev.packages.swap(kept);
        }
    }

    // Cleanup empty events/trips.
    cleanup_empty();

    auto max_depot_release = [&](int truck_id) {
        int mx = 0;
        for (const auto &st : sol.trucks[truck_id].stops) {
            int c = st.customer;
            if (c <= 0 || c >= n) continue;
            if (!is_pkg_resupplied(sol, c)) mx = std::max(mx, p.customers[c].release);
        }
        return mx;
    };

    auto try_assign_pkg = [&](int pkg) -> bool {
        if (pkg <= 0 || pkg >= n) return false;
        int t = owner_truck[pkg];
        int pos_pkg = owner_pos[pkg];
        if (t < 0 || t >= (int)sol.trucks.size() || pos_pkg <= 0) return false;
        if (p.customers[pkg].demand > p.M_d + 1e-9) return false;

        const auto &stops = sol.trucks[t].stops;
        for (int pos = pos_pkg; pos >= 1; --pos) {
            int recv = stops[pos].customer;
            if (recv <= 0) continue;
            if (!p.reachable_mask.empty() && !p.reachable_mask[recv]) continue;

            // Prefer reusing an existing event at the same (truck, rendezvous city).
            for (size_t trip_idx = 0; trip_idx < sol.drone_queue.size(); ++trip_idx) {
                auto &trip = sol.drone_queue[trip_idx];
                for (auto &ev : trip.events) {
                    if (ev.truck_id != t || ev.rendezvous_customer != recv) continue;
                    if (std::find(ev.packages.begin(), ev.packages.end(), pkg) != ev.packages.end()) return true;
                    if (trip_load(p, trip) + p.customers[pkg].demand > p.M_d + 1e-9) continue;
                    ev.packages.push_back(pkg);
                    if (trip_endurance_optimistic(p, sol, trip)) return true;
                    ev.packages.pop_back();
                }
            }

            // Otherwise create a new singleton trip.
            DroneTrip new_trip;
            new_trip.events.push_back(ResupplyEvent{recv, t, {pkg}});
            int ins = find_insert_pos_queue(p, sol, new_trip);
            if (ins < 0) continue;
            Solution trial = sol;
            trial.drone_queue.insert(trial.drone_queue.begin() + ins, new_trip);
            if (!trip_endurance_optimistic(p, trial, trial.drone_queue[ins])) continue;
            std::string reason;
            if (!validate_solution(p, trial, reason)) continue;
            sol = std::move(trial);
            return true;
        }
        return false;
    };

    auto demote_to_depot_with_release_cleanup = [&](int pkg) {
        if (pkg <= 0 || pkg >= n) return;
        int t = owner_truck[pkg];
        if (t < 0 || t >= (int)sol.trucks.size()) return;
        int rel_pkg = p.customers[pkg].release;
        // If pkg goes to depot, remove earlier-release same-truck packages from drone trips.
        for (auto &trip : sol.drone_queue) {
            for (auto &ev : trip.events) {
                std::vector<PackageId> kept;
                kept.reserve(ev.packages.size());
                for (PackageId q : ev.packages) {
                    if (q <= 0 || q >= n) continue;
                    if (owner_truck[q] == t && p.customers[q].release < rel_pkg) continue;
                    kept.push_back(q);
                }
                ev.packages.swap(kept);
            }
        }
        cleanup_empty();
    };

    // Ensure i/j are handled by the requested priority logic even if not in displaced set.
    if (city_i > 0 && city_i < n) displaced.insert(city_i);
    if (city_j > 0 && city_j < n) displaced.insert(city_j);

    // Try to reinsert displaced packages; otherwise they remain depot-loaded.
    std::vector<int> displaced_list(displaced.begin(), displaced.end());
    std::sort(displaced_list.begin(), displaced_list.end());
    for (int pkg : displaced_list) {
        if (pkg <= 0 || pkg >= n) continue;
        int t = owner_truck[pkg];
        if (t < 0 || t >= (int)sol.trucks.size()) continue;
        int rel_pkg = p.customers[pkg].release;
        int depot_max_rel = max_depot_release(t);

        // If release <= max release of depot-loads, depot is already feasible;
        // still try drone reassignment first, then fallback to depot.
        bool ok = try_assign_pkg(pkg);
        if (!ok && rel_pkg <= depot_max_rel) {
            demote_to_depot_with_release_cleanup(pkg);
        } else if (!ok) {
            demote_to_depot_with_release_cleanup(pkg);
        }
    }
}

// legacy helper: if no list provided, assume all trucks affected
static void repair_after_truck_move(Solution &sol) {
    std::vector<int> all; all.reserve(sol.trucks.size());
    for (auto &tr : sol.trucks) all.push_back(tr.truck_id);
    repair_after_truck_move(sol, all);
}

static bool trip_endurance_ok(const Params &p, const Solution &s, const std::vector<TruckTimeline> &tls, const DroneTrip &trip) {
    if (trip.events.empty()) return true;
    int max_rel = 0;
    for (const auto &ev : trip.events)
        for (auto pk : ev.packages) max_rel = std::max(max_rel, p.customers[pk].release);
    const auto &first = trip.events.front();
    double truck_arrival_first = truck_arrival_at(s, tls, first.truck_id, first.rendezvous_customer);
    double launch_earliest = truck_arrival_first - p.drone_time[0][first.rendezvous_customer];
    double depart = std::max({0.0, (double)max_rel, launch_earliest});
    double t = depart;
    int last = 0;
    for (const auto &ev : trip.events) {
        t += p.drone_time[last][ev.rendezvous_customer];
        double ta = truck_arrival_at(s, tls, ev.truck_id, ev.rendezvous_customer);
        if (t < ta) { t = ta; }
        t += p.sigma; // unload time excluded from endurance rule
        last = ev.rendezvous_customer;
    }
    t += p.drone_time[last][0];
    double trip_time = t - depart;
    double endurance_measure = trip_time - p.sigma * (double)trip.events.size();
    return endurance_measure <= p.L_d + 1e-9;
}

// Optimistic (lower-bound) endurance check using truck free-flow arrival (no waits).
static bool trip_endurance_optimistic(const Params &p, const Solution &s, const DroneTrip &trip) {
    if (trip.events.empty()) return true;
    // precompute prefix times for each truck (no waiting)
    static thread_local std::vector<std::vector<double>> prefix;
    prefix.resize(s.trucks.size());
    for (size_t k = 0; k < s.trucks.size(); ++k) {
        const auto &stops = s.trucks[k].stops;
        prefix[k].assign(stops.size(), 0.0);
        for (size_t i = 1; i < stops.size(); ++i) {
            int from = stops[i-1].customer;
            int to   = stops[i].customer;
            prefix[k][i] = prefix[k][i-1] + p.truck_time[from][to];
        }
    }
    auto earliest_truck = [&](int k, int cust){
        int pos = route_pos(s, k, cust);
        if (pos < 0) return 1e18; // invalid
        return prefix[k][pos];
    };

    int max_rel = 0;
    for (const auto &ev : trip.events)
        for (auto pk : ev.packages) if (pk >=0 && pk < (int)p.customers.size()) max_rel = std::max(max_rel, p.customers[pk].release);
    const auto &first = trip.events.front();
    double truck_arr_first = earliest_truck(first.truck_id, first.rendezvous_customer);
    double launch_earliest = truck_arr_first - p.drone_time[0][first.rendezvous_customer];
    double t = std::max({0.0, (double)max_rel, launch_earliest});
    double travel_only = 0.0;
    int last = 0;
    for (const auto &ev : trip.events) {
        double leg = p.drone_time[last][ev.rendezvous_customer];
        travel_only += leg; t += leg;
        double ta = earliest_truck(ev.truck_id, ev.rendezvous_customer);
        if (t < ta) { t = ta; }
        t += p.sigma; // unload time excluded from endurance rule
        last = ev.rendezvous_customer;
    }
    travel_only += p.drone_time[last][0];
    return travel_only <= p.L_d + 1e-9;
}

// Find earliest queue position where inserting cand keeps queue-order constraint.
// Returns index in [0, s.drone_queue.size()] (insert before that index). Returns -1 if impossible.
static int find_insert_pos_queue(const Params &p, const Solution &s, const DroneTrip &cand) {
    // precompute first positions of cand per truck
    std::vector<int> cand_pos(p.n_truck, -1);
    for (const auto &ev : cand.events) {
        int pos = route_pos(s, ev.truck_id, ev.rendezvous_customer);
        if (pos < 0) return -1;
        if (cand_pos[ev.truck_id] == -1 || pos < cand_pos[ev.truck_id]) cand_pos[ev.truck_id] = pos;
    }

    for (size_t insert_idx = 0; insert_idx <= s.drone_queue.size(); ++insert_idx) {
        std::vector<int> last_pos(p.n_truck, -1);
        bool ok = true;
        for (size_t i = 0; i <= s.drone_queue.size(); ++i) {
            const DroneTrip *trip_ptr;
            DroneTrip temp;
            if (i == insert_idx) trip_ptr = &cand;
            else {
                size_t idx = (i < insert_idx) ? i : i-1;
                trip_ptr = &s.drone_queue[idx];
            }
            std::unordered_map<int,int> first_pos;
            for (const auto &ev : trip_ptr->events) {
                int t = ev.truck_id;
                int pos = route_pos(s, t, ev.rendezvous_customer);
                if (pos < 0) { ok = false; break; }
                auto it = first_pos.find(t);
                if (it == first_pos.end() || pos < it->second) first_pos[t] = pos;
            }
            if (!ok) break;
            for (auto &kv : first_pos) {
                int t = kv.first; int pos = kv.second;
                if (last_pos[t] > pos) { ok = false; break; }
                last_pos[t] = pos;
            }
            if (!ok) break;
        }
        if (ok) return (int)insert_idx;
    }
    return -1;
}

static void print_truck_routes(const Solution &s) {
    for (const auto &tr : s.trucks) {
        std::cout << "Truck " << tr.truck_id << ": ";
        for (size_t i = 0; i < tr.stops.size(); ++i) {
            std::cout << tr.stops[i].customer;
            if (!tr.stops[i].loaded_from_drone.empty()) {
                std::cout << "[";
                for (size_t j = 0; j < tr.stops[i].loaded_from_drone.size(); ++j) {
                    std::cout << tr.stops[i].loaded_from_drone[j];
                    if (j+1 < tr.stops[i].loaded_from_drone.size()) std::cout << ",";
                }
                std::cout << "]";
            }
            if (i+1 < tr.stops.size()) std::cout << " -> ";
        }
        std::cout << "\n";
    }
}

static double truck_arrival_at(const Solution &sol, const std::vector<TruckTimeline> &tls, int truck_id, int customer) {
    const auto &stops = sol.trucks[truck_id].stops;
    auto it = std::find_if(stops.begin(), stops.end(), [&](const TruckStop &st){ return st.customer == customer; });
    if (it == stops.end()) throw std::runtime_error("customer not on truck route");
    size_t idx = std::distance(stops.begin(), it);
    return tls[truck_id].arrival[idx];
}

// Normalize transformed solutions before evaluation:
// - remove invalid/empty package assignments
// - remove empty rendezvous events
// - remove empty drone trips
// - rebuild loaded_from_drone marks from normalized queue
static void normalize_after_transform(const Params &p, Solution &sol) {
    const int n = (int)p.customers.size();
    for (auto &trip : sol.drone_queue) {
        std::vector<ResupplyEvent> kept_events;
        kept_events.reserve(trip.events.size());
        for (auto &ev : trip.events) {
            std::vector<int> pk;
            pk.reserve(ev.packages.size());
            for (int x : ev.packages) {
                if (x > 0 && x < n) pk.push_back(x);
            }
            if (pk.empty()) continue;
            ev.packages.swap(pk);
            kept_events.push_back(std::move(ev));
        }
        trip.events.swap(kept_events);
    }
    std::vector<DroneTrip> kept_trips;
    kept_trips.reserve(sol.drone_queue.size());
    for (auto &trip : sol.drone_queue) {
        if (!trip.events.empty()) kept_trips.push_back(std::move(trip));
    }
    sol.drone_queue.swap(kept_trips);
    rebuild_loaded_from_drone_marks(p, sol);
}

// Synchronized fitness: simulate trucks and drones together (queue order for drones).
static std::pair<bool,double> fitness_full(const Params &p, const Solution &sol, double *truck_sum_out) {
    Solution normalized = sol;
    normalize_after_transform(p, normalized);
    const Solution &s_eval = normalized;
    // Always gate objective evaluation by full feasibility.
    std::string reason;
    if (!validate_solution(p, s_eval, reason)) {
        return {false, std::numeric_limits<double>::infinity()};
    }

    int K = p.n_truck;
    const int n = (int)p.customers.size();
    // mark packages resupplied by drones
    std::vector<char> is_resupplied(n, 0);
    for (const auto &trip : s_eval.drone_queue)
        for (const auto &ev : trip.events)
            for (auto pk : ev.packages) if (pk >= 0 && pk < n) is_resupplied[pk] = 1;

    // state per truck
    std::vector<size_t> idx(K, 0); // position in stops
    std::vector<double> t_truck(K, 0.0);
    std::vector<std::vector<int>> stops_cust(K);
    std::vector<std::vector<int>> route_pos(K, std::vector<int>(n, -1));
    for (int k = 0; k < K; ++k) {
        int pos = 0;
        for (const auto &st : s_eval.trucks[k].stops) {
            stops_cust[k].push_back(st.customer);
            if (st.customer >=0 && st.customer < n) route_pos[k][st.customer] = pos;
            ++pos;
        }
        // start time = max release of any package carried by truck at depot (i.e., not resupplied)
        double start = 0.0;
        for (size_t i = 1; i + 1 < stops_cust[k].size(); ++i) {
            int c = stops_cust[k][i];
            if (!is_resupplied[c])
                start = std::max(start, (double)p.customers[c].release);
        }
        t_truck[k] = start;
    }

    auto move_truck_to = [&](int k, int target)->double {
        // Advance truck k until it reaches 'target'. If target is not on the
        // route, return +inf to mark the solution infeasible instead of
        // overflowing the vector.
        while (idx[k] < stops_cust[k].size() && stops_cust[k][idx[k]] != target) {
            if (idx[k] + 1 >= stops_cust[k].size())
                return std::numeric_limits<double>::infinity();
            int from = stops_cust[k][idx[k]];
            int to = stops_cust[k][idx[k] + 1];
            t_truck[k] += p.truck_time[from][to];
            idx[k]++;
        }
        if (idx[k] >= stops_cust[k].size())
            return std::numeric_limits<double>::infinity();
        return t_truck[k];
    };

    // drone availability PQ
    using Node = std::pair<double,int>;
    auto cmp = [](const Node &a, const Node &b){ return a.first > b.first; };
    std::priority_queue<Node, std::vector<Node>, decltype(cmp)> dq(cmp);
    for (int d = 0; d < p.n_drone; ++d) dq.push({0.0, d});
    double max_drone_finish = 0.0;

    for (const auto &trip : s_eval.drone_queue) {
        if (trip.events.empty()) continue;
        auto [avail, drone_id] = dq.top(); dq.pop();

        int max_release = 0;
        for (const auto &ev : trip.events)
            for (auto pkg : ev.packages)
                if (pkg >=0 && pkg < (int)p.customers.size()) max_release = std::max(max_release, p.customers[pkg].release);

        const auto &first_ev = trip.events.front();
        if (first_ev.rendezvous_customer<0 || first_ev.rendezvous_customer>=n || route_pos[first_ev.truck_id][first_ev.rendezvous_customer] < 0)
            return {false, std::numeric_limits<double>::infinity()};
        if (!p.reachable_mask.empty() && !p.reachable_mask[first_ev.rendezvous_customer]) return {false, std::numeric_limits<double>::infinity()};
        double truck_arrival_first = move_truck_to(first_ev.truck_id, first_ev.rendezvous_customer);
        if (!std::isfinite(truck_arrival_first)) return {false, std::numeric_limits<double>::infinity()};
        double launch_earliest = truck_arrival_first - p.drone_time[0][first_ev.rendezvous_customer];
        double depart = std::max({avail, (double)max_release, launch_earliest});
        double t = depart;
        int last = 0;

        for (const auto &ev : trip.events) {
            if (ev.rendezvous_customer<0 || ev.rendezvous_customer>=n) return {false, std::numeric_limits<double>::infinity()};
            if (route_pos[ev.truck_id][ev.rendezvous_customer] < 0) return {false, std::numeric_limits<double>::infinity()};
            if (!p.reachable_mask.empty() && !p.reachable_mask[ev.rendezvous_customer]) return {false, std::numeric_limits<double>::infinity()};
            t += p.drone_time[last][ev.rendezvous_customer];
            double truck_arrival = move_truck_to(ev.truck_id, ev.rendezvous_customer);
            if (!std::isfinite(truck_arrival)) return {false, std::numeric_limits<double>::infinity()};
            double drone_wait = 0, truck_wait = 0;
            if (t < truck_arrival) { drone_wait = truck_arrival - t; t = truck_arrival; }
            else { truck_wait = t - truck_arrival; t_truck[ev.truck_id] += truck_wait; }
            t += p.sigma; // service
            t_truck[ev.truck_id] += p.sigma;
            last = ev.rendezvous_customer;
        }
        t += p.drone_time[last][0];
        double trip_time = t - depart;
        double endurance_measure = trip_time - p.sigma * (double)trip.events.size();
        if (endurance_measure > p.L_d + 1e-9) return {false, std::numeric_limits<double>::infinity()};
        max_drone_finish = std::max(max_drone_finish, t);
        dq.push({t, drone_id});
    }

    // finish remaining truck legs
    for (int k = 0; k < K; ++k) {
        while (idx[k] + 1 < stops_cust[k].size()) {
            int from = stops_cust[k][idx[k]];
            int to = stops_cust[k][idx[k]+1];
            t_truck[k] += p.truck_time[from][to];
            idx[k]++;
        }
    }
    double max_truck_finish = 0.0; double sum_truck = 0.0;
    for (double tt : t_truck) { max_truck_finish = std::max(max_truck_finish, tt); sum_truck += tt; }
    if (truck_sum_out) *truck_sum_out = sum_truck;
    return {true, std::max(max_truck_finish, max_drone_finish)};
}

struct TruckRowLog {
    int truck;
    int route_index;
    int city;
    double arrival;
    double departure;
    double waiting;
};

struct DroneLegLog {
    int leg;
    int launch_city;
    int owner_truck;
    double drone_arrival;
    double truck_arrival;
    double drone_wait;
    double truck_wait;
    double sync_time;
    double service_end;
    std::vector<int> customers;
};

struct DroneTripLog {
    int trip_idx;
    int drone_id;
    double drone_ready_at_depot;
    double release_bound;
    int first_launch_city;
    double first_truck_arrival;
    double depart_depot;
    double return_depot;
    std::vector<DroneLegLog> legs;
};

struct SimLog {
    std::vector<TruckRowLog> truck_rows;
    std::vector<DroneTripLog> drone_rows;
    double makespan;
    std::vector<double> truck_return;
    double max_drone_finish;
};

static SimLog simulate_with_log(const Params &p, const Solution &sol) {
    SimLog log;
    int K = p.n_truck; int n = (int)p.customers.size();
    std::vector<size_t> idx(K,0);
    std::vector<double> t_tr(K,0.0);
    std::vector<std::vector<int>> stops(K);
    std::vector<std::vector<int>> route_pos(K, std::vector<int>(n, -1));
    std::vector<int> last_row(K,-1);
    std::vector<char> is_resupplied(n,0);
    for (const auto &trip : sol.drone_queue)
        for (const auto &ev : trip.events)
            for (auto pk : ev.packages) if (pk>=0 && pk<n) is_resupplied[pk]=1;
    for(int k=0;k<K;++k){
        int pos=0;
        for(auto &st: sol.trucks[k].stops){ 
            stops[k].push_back(st.customer);
            if(st.customer>=0 && st.customer<n) route_pos[k][st.customer]=pos;
            ++pos;
        }
        double start = 0.0;
        for (size_t i = 1; i + 1 < stops[k].size(); ++i) {
            int c = stops[k][i];
            if (!is_resupplied[c])
                start = std::max(start, (double)p.customers[c].release);
        }
        t_tr[k] = start;
        // log depot
        log.truck_rows.push_back({k,0,stops[k][0],start,start,0.0});
        last_row[k]= (int)log.truck_rows.size()-1;
    }

    auto log_move = [&](int k,int to){
        int from = stops[k][idx[k]];
        t_tr[k] += p.truck_time[from][to];
        idx[k]++;
        log.truck_rows.push_back({k,(int)idx[k],to,t_tr[k],t_tr[k],0.0});
        last_row[k]=(int)log.truck_rows.size()-1;
    };

    auto move_truck_to = [&](int k,int target){
        while(idx[k] < stops[k].size() && stops[k][idx[k]] != target){
            if(idx[k] + 1 >= stops[k].size())
                return std::numeric_limits<double>::infinity();
            int next = stops[k][idx[k]+1];
            log_move(k,next);
        }
        if(idx[k] >= stops[k].size())
            return std::numeric_limits<double>::infinity();
        return t_tr[k];
    };

    using Node=std::pair<double,int>;
    auto cmp=[](const Node&a,const Node&b){return a.first>b.first;};
    std::priority_queue<Node,std::vector<Node>,decltype(cmp)> dq(cmp);
    for(int d=0;d<p.n_drone;++d) dq.push({0.0,d});
    double max_drone=0.0;

    for(size_t trip_idx=0; trip_idx<sol.drone_queue.size(); ++trip_idx){
        const auto &trip=sol.drone_queue[trip_idx];
        if(trip.events.empty()) continue;
        auto [avail, drone_id]=dq.top(); dq.pop();
        int max_rel=0;
        for(auto &ev: trip.events) for(auto pk: ev.packages) if(pk>=0 && pk<(int)p.customers.size()) max_rel=std::max(max_rel,p.customers[pk].release);
        const auto &first=trip.events.front();
        if(first.rendezvous_customer<0 || first.rendezvous_customer>=n || route_pos[first.truck_id][first.rendezvous_customer]<0){
            dq.push({avail,drone_id});
            continue;
        }
        double truck_arr_first=move_truck_to(first.truck_id, first.rendezvous_customer);
        if(!std::isfinite(truck_arr_first)) { dq.push({avail,drone_id}); continue; }
        double depart = std::max({avail,(double)max_rel, truck_arr_first - p.drone_time[0][first.rendezvous_customer]});
        double t=depart; int last=0; double flight_wait=0.0;
        DroneTripLog tlog; tlog.trip_idx=trip_idx; tlog.drone_id=drone_id; tlog.drone_ready_at_depot=avail; tlog.release_bound=max_rel; tlog.first_launch_city=first.rendezvous_customer; tlog.first_truck_arrival=truck_arr_first; tlog.depart_depot=depart;

        for(size_t leg_idx=0; leg_idx<trip.events.size(); ++leg_idx){
            const auto &ev=trip.events[leg_idx];
            if(ev.rendezvous_customer<0 || ev.rendezvous_customer>=n || route_pos[ev.truck_id][ev.rendezvous_customer]<0){ t=std::numeric_limits<double>::infinity(); break; }
            t += p.drone_time[last][ev.rendezvous_customer];
            flight_wait += p.drone_time[last][ev.rendezvous_customer];
            double truck_arr = move_truck_to(ev.truck_id, ev.rendezvous_customer);
            if(!std::isfinite(truck_arr)) { t = std::numeric_limits<double>::infinity(); break; }
            double drone_wait=0, truck_wait=0;
            if(t < truck_arr){ drone_wait = truck_arr - t; t = truck_arr; flight_wait += drone_wait; }
            else { truck_wait = t - truck_arr; t_tr[ev.truck_id] += truck_wait; }
            t += p.sigma; t_tr[ev.truck_id] += p.sigma;
            auto &row = log.truck_rows[last_row[ev.truck_id]]; row.departure = t_tr[ev.truck_id]; row.waiting = truck_wait;
            DroneLegLog lleg{(int)leg_idx, ev.rendezvous_customer, ev.truck_id, t - p.sigma - truck_wait, truck_arr, drone_wait, truck_wait, t - p.sigma, t, ev.packages};
            tlog.legs.push_back(lleg);
            last = ev.rendezvous_customer;
        }
        if(!std::isfinite(t)) { dq.push({avail,drone_id}); continue; }
        t += p.drone_time[last][0]; flight_wait += p.drone_time[last][0];
        tlog.return_depot = t;
        dq.push({t, drone_id});
        max_drone = std::max(max_drone, t);
        log.drone_rows.push_back(std::move(tlog));
    }

    for(int k=0;k<K;++k){
        while(idx[k]+1 < stops[k].size()){
            int next = stops[k][idx[k]+1];
            log_move(k,next);
        }
    }
    log.truck_return = t_tr;
    log.max_drone_finish = max_drone;
    double max_truck = *max_element(t_tr.begin(), t_tr.end());
    log.makespan = std::max(max_truck, max_drone);
    return log;
}

// Compatibility wrapper: returns +inf if invalid; optionally returns validity flag.
static double fitness(const Params &p, const Solution &sol, bool *valid_out = nullptr) {
    auto res = fitness_full(p, sol, nullptr);
    if (valid_out) *valid_out = res.first;
    return res.first ? res.second : std::numeric_limits<double>::infinity();
}

// 2-opt improvement on each truck route with truck-only objective
static void improve_truck_routes_2opt(const Params &p, Solution &s) {
    bool improved = true;
    while (improved) {
        improved = false;
        double best_val = truck_only_makespan(p, s);
        int best_truck = -1; size_t best_i = 0, best_k = 0;
        for (int t = 0; t < p.n_truck; ++t) {
            auto &stops = s.trucks[t].stops;
            if (stops.size() <= 3) continue; // depot-depot only
            size_t n = stops.size()-2; // excluding last depot
            for (size_t i = 1; i < n; ++i) {
                for (size_t k = i+1; k < n+1; ++k) { // up to last customer
                    Solution trial = s;
                    std::reverse(trial.trucks[t].stops.begin()+i, trial.trucks[t].stops.begin()+k+1);
                    double val = truck_only_makespan(p, trial);
                    if (val + 1e-9 < best_val) {
                        best_val = val; best_truck = t; best_i = i; best_k = k; improved = true;
                    }
                }
            }
        }
        if (improved && best_truck != -1) {
            std::reverse(s.trucks[best_truck].stops.begin()+best_i, s.trucks[best_truck].stops.begin()+best_k+1);
        }
    }
}

// -------- Drone local search: Synchronization Point Relocation (first-improvement) --------
// Move one rendezvous event from its current trip into another trip (creating multi-visit) while keeping constraints:
// - A drone trip cannot rendezvous with the same truck more than once
// - Trip load <= capacity
// - Flight+wait (excluding unload) <= L_d
// - Rendezvous must be within drone range (already checked by validate)
// If a move improves global fitness, apply first found.
static bool relocate_sync_point_first_improve(const Params &p, Solution &s) {
    if (s.drone_queue.size() < 2) return false; // need at least 2 trips to merge

    // precompute truck timelines for validation
    std::vector<TruckTimeline> timelines(s.trucks.size());
    for (size_t k = 0; k < s.trucks.size(); ++k) timelines[k] = compute_truck_timeline(p, s.trucks[k]);

    double best_truck_sum = 0.0;
    double current_fit = fitness_full(p, s, &best_truck_sum).second;
    double best_fit = current_fit;
    Solution best_sol = s;
    bool improved_any = false;
    auto better = [&](double f, double sum){
        return (f + 1e-9 < best_fit) || (std::fabs(f - best_fit) <= 1e-9 && sum + 1e-9 < best_truck_sum);
    };

    for (size_t from_trip = 0; from_trip < s.drone_queue.size(); ++from_trip) {
        auto &trip_from = s.drone_queue[from_trip];
        if (trip_from.events.size() <= 1) continue; // moving single event would empty trip; keep at least one? we allow empty removal later by erase if becomes empty
        for (size_t ev_idx = 0; ev_idx < trip_from.events.size(); ++ev_idx) {
            ResupplyEvent ev = trip_from.events[ev_idx];

            for (size_t to_trip = 0; to_trip < s.drone_queue.size(); ++to_trip) {
                if (to_trip == from_trip) continue;
                auto &trip_to = s.drone_queue[to_trip];

                // forbid same truck twice in target trip
                bool truck_dup = false;
                for (const auto &e2 : trip_to.events) if (e2.truck_id == ev.truck_id) { truck_dup = true; break; }
                if (truck_dup) continue;

                // capacity quick check
                int load = trip_load(p, trip_to);
                int add = 0; for (auto pkg : ev.packages) add += p.customers[pkg].demand;
                if (load + add > p.M_d + 1e-9) continue;

                // try insertion positions: here append to end (simple, enough for multi-visit)
                Solution trial = s;
                trial.drone_queue[from_trip].events.erase(trial.drone_queue[from_trip].events.begin() + ev_idx);
                trial.drone_queue[to_trip].events.push_back(ev);
                // remove trip if empty
                bool from_removed = false;
                if (trial.drone_queue[from_trip].events.empty()) {
                    trial.drone_queue.erase(trial.drone_queue.begin() + from_trip);
                    from_removed = true;
                }

                rebuild_loaded_from_drone_marks(p, trial);
                // quick endurance checks for affected trips (optimistic)
                if (!trip_endurance_optimistic(p, trial, trial.drone_queue[to_trip])) continue;
                if (!from_removed && from_trip < trial.drone_queue.size() && !trip_endurance_optimistic(p, trial, trial.drone_queue[from_trip])) continue;

                double ts = 0.0; auto res = fitness_full(p, trial, &ts);
                if (!res.first) continue;
                double new_fit = res.second;
                if (better(new_fit, ts)) { best_fit = new_fit; best_truck_sum = ts; best_sol = std::move(trial); improved_any = true; }
            }
        }
    }
    if (improved_any) { s = std::move(best_sol); return true; }
    return false;
}

// -------- Drone local search: Package Relocation (first-improvement) --------
// Move a package between rendezvous points on the SAME truck (or back to depot load) respecting:
// - Package stays with its serving truck.
// - New rendezvous must be at the same or earlier position on that truck route than the package's delivery point.
// - Capacity/endurance/range constraints enforced via validate_solution.
static bool relocate_package_first_improve(const Params &p, Solution &s) {
    // precompute timelines once
    std::vector<TruckTimeline> timelines(s.trucks.size());
    for (size_t k = 0; k < s.trucks.size(); ++k) timelines[k] = compute_truck_timeline(p, s.trucks[k]);

    double best_truck_sum = 0.0;
    double best_fit = fitness_full(p, s, &best_truck_sum).second;
    Solution best_sol = s;
    bool improved_any = false;
    std::string reason;
    auto better = [&](double f, double sum){
        return (f + 1e-9 < best_fit) || (std::fabs(f - best_fit) <= 1e-9 && sum + 1e-9 < best_truck_sum);
    };

    // quick map of resupplied packages
    std::vector<char> is_resupplied(p.customers.size(), 0);
    for (const auto &trip : s.drone_queue)
        for (const auto &ev : trip.events)
            for (auto pk : ev.packages) if (pk>=0 && pk<(int)is_resupplied.size()) is_resupplied[pk]=1;

    // Pull a truck-carried package (not resupplied) and try to resupply it at its customer or earlier stops on the same truck.
    struct Cand {int truck; size_t pos; int pkg; int rel; double truck_finish;};
    std::vector<Cand> cands;
    for (int k = 0; k < p.n_truck; ++k) {
        double finish_k = timelines[k].arrival.empty() ? 0.0 : timelines[k].arrival.back();
        for (size_t pos = 1; pos + 1 < s.trucks[k].stops.size(); ++pos) {
            int pkg = s.trucks[k].stops[pos].customer;
            if (is_resupplied[pkg]) continue;
            if (!p.reachable_mask.empty() && !p.reachable_mask[pkg]) continue;
            if (p.customers[pkg].demand > p.M_d + 1e-9) continue;
            cands.push_back({k,pos,pkg,p.customers[pkg].release, finish_k});
        }
    }
    std::sort(cands.begin(), cands.end(), [](const Cand&a,const Cand&b){
        if (a.truck_finish != b.truck_finish) return a.truck_finish > b.truck_finish; // truck with larger completion first
        if (a.rel != b.rel) return a.rel > b.rel; // then higher release
        return a.pos < b.pos; // earlier position
    });

    auto release_span_ok = [&](const std::vector<int>& pkgs)->bool{
        if (pkgs.empty()) return true;
        int mn = p.customers[pkgs[0]].release, mx = mn, sum = 0;
        for (int pk : pkgs) { int r = p.customers[pk].release; mn = std::min(mn, r); mx = std::max(mx, r); sum += r; }
        double avg = (double)sum / pkgs.size();
        return (mx - mn) <= avg/2.0 + 1e-9;
    };

    for (const auto &c : cands) {
        int k = c.truck; int pkg = c.pkg; size_t pos_del = c.pos; int rel = c.rel;

        // build a list of other non-resupplied, reachable packages on same truck with the SAME release time
        std::vector<int> same_rel;
        for (size_t pos2 = pos_del+1; pos2 + 1 < s.trucks[k].stops.size(); ++pos2) {
            int pk2 = s.trucks[k].stops[pos2].customer;
            if (is_resupplied[pk2]) continue;
            if (p.customers[pk2].release != rel) continue;
            if (!p.reachable_mask.empty() && !p.reachable_mask[pk2]) continue;
            if (p.customers[pk2].demand > p.M_d + 1e-9) continue;
            same_rel.push_back(pk2);
        }

        // try bundles: start with {pkg}, then progressively add same-release packages in route order until capacity full
        std::vector<int> bundle;
        bundle.push_back(pkg);
        std::vector<int> bundle_positions; bundle_positions.push_back((int)pos_del);

        auto try_bundle = [&](const std::vector<int>& pkgs, const std::vector<int>& pos_list)->bool{
            int total_demand = 0; size_t min_pos = pos_list[0];
            for (size_t i=0;i<pkgs.size();++i){ total_demand += p.customers[pkgs[i]].demand; if ((size_t)pos_list[i] < min_pos) min_pos = pos_list[i]; }
            if (total_demand > p.M_d + 1e-9) return false;
            // rendezvous can be any stop up to min_pos (inclusive)
            for (size_t l = 1; l <= min_pos; ++l) {
                int recv = s.trucks[k].stops[l].customer;
                if (!p.reachable_mask.empty() && !p.reachable_mask[recv]) continue;

                // try insert into existing trip event
                bool inserted = false;
                Solution trial = s;
                for (size_t tgi = 0; tgi < trial.drone_queue.size() && !inserted; ++tgi) {
                    for (size_t egi = 0; egi < trial.drone_queue[tgi].events.size() && !inserted; ++egi) {
                        const auto &ev0 = trial.drone_queue[tgi].events[egi];
                        if (ev0.truck_id != k || ev0.rendezvous_customer != recv) continue;
                        if (trip_load(p, trial.drone_queue[tgi]) + total_demand > p.M_d + 1e-9) continue;

                        bool dup = false;
                        for (int pk : pkgs) {
                            if (std::find(ev0.packages.begin(), ev0.packages.end(), pk) != ev0.packages.end()) { dup = true; break; }
                        }
                        if (dup) continue;

                        Solution cand = trial;
                        auto &evm = cand.drone_queue[tgi].events[egi];
                        std::vector<int> merged = evm.packages;
                        merged.insert(merged.end(), pkgs.begin(), pkgs.end());
                        if (!release_span_ok(merged)) continue;
                        evm.packages.swap(merged);

                        rebuild_loaded_from_drone_marks(p, cand);
                        if (!trip_endurance_optimistic(p, cand, cand.drone_queue[tgi])) continue;
                        double ts_loc = 0.0; auto res = fitness_full(p, cand, &ts_loc);
                        if (!res.first) continue;
                        double new_fit = res.second;
                        if (better(new_fit, ts_loc)) { best_fit = new_fit; best_truck_sum = ts_loc; best_sol = std::move(cand); improved_any = true; }
                        inserted = true;
                    }
                }
                if (inserted) continue;

                // create new trip
                if (!release_span_ok(pkgs)) continue;
                DroneTrip new_trip; new_trip.events.push_back(ResupplyEvent{recv, k, pkgs});
                int ins_pos = find_insert_pos_queue(p, s, new_trip);
                if (ins_pos == -1) continue;
                Solution trial2 = s;
                trial2.drone_queue.insert(trial2.drone_queue.begin()+ins_pos, new_trip);
                rebuild_loaded_from_drone_marks(p, trial2);
                if (!trip_endurance_optimistic(p, trial2, trial2.drone_queue[ins_pos])) continue;
                double ts2 = 0.0; auto res2 = fitness_full(p, trial2, &ts2);
                if (!res2.first) continue;
                double new_fit2 = res2.second;
                if (better(new_fit2, ts2)) { best_fit = new_fit2; best_truck_sum = ts2; best_sol = std::move(trial2); improved_any = true; }
            }
            return false;
        };

        // try single and growing bundle
        try_bundle(bundle, bundle_positions);
        for (int pk2 : same_rel) {
            bundle.push_back(pk2);
            // find position of pk2 on truck k
            int pos2 = route_pos(s, k, pk2);
            bundle_positions.push_back(pos2);
            try_bundle(bundle, bundle_positions);
        }
    }

    for (size_t trip_idx = 0; trip_idx < s.drone_queue.size(); ++trip_idx) {
        for (size_t ev_idx = 0; ev_idx < s.drone_queue[trip_idx].events.size(); ++ev_idx) {
            auto &ev = s.drone_queue[trip_idx].events[ev_idx];
            int truck_id = ev.truck_id;
            for (size_t pkg_idx = 0; pkg_idx < ev.packages.size(); ++pkg_idx) {
                PackageId pkg = ev.packages[pkg_idx];
                // position of package delivery on this truck
                int pos_del = route_pos(s, truck_id, pkg);
                if (pos_del < 0) continue; // safety

                // Try moving package to another rendezvous on same truck (before or at delivery)
                for (size_t trip2 = 0; trip2 < s.drone_queue.size(); ++trip2) {
                    for (size_t ev2_idx = 0; ev2_idx < s.drone_queue[trip2].events.size(); ++ev2_idx) {
                        if (trip2 == trip_idx && ev2_idx == ev_idx) continue;
                        const auto &ev2 = s.drone_queue[trip2].events[ev2_idx];
                        if (ev2.truck_id != truck_id) continue;
                        int pos_rv = route_pos(s, truck_id, ev2.rendezvous_customer);
                        if (pos_rv < 0 || pos_rv > pos_del) continue; // must be <= delivery position
                        if (ev2.rendezvous_customer == pkg) continue; // avoid resupply at same customer

                        Solution trial = s;
                        auto &from_ev = trial.drone_queue[trip_idx].events[ev_idx];
                        auto &to_ev   = trial.drone_queue[trip2].events[ev2_idx];
                        from_ev.packages.erase(from_ev.packages.begin() + pkg_idx);
                        std::vector<int> merged = to_ev.packages; merged.push_back(pkg);
                        if (!release_span_ok(merged)) continue;
                        to_ev.packages.swap(merged);
                        if (from_ev.packages.empty()) {
                            trial.drone_queue[trip_idx].events.erase(trial.drone_queue[trip_idx].events.begin()+ev_idx);
                            if (trial.drone_queue[trip_idx].events.empty()) trial.drone_queue.erase(trial.drone_queue.begin()+trip_idx);
                        }
                        rebuild_loaded_from_drone_marks(p, trial);
                        // Avoid relying on stale vector indices after potential erase of source event/trip.
                        // Validate the whole candidate before scoring.
                        std::string reason_local;
                        if (!validate_solution(p, trial, reason_local)) continue;
                        double ts = 0.0; auto res = fitness_full(p, trial, &ts);
                        if (!res.first) continue;
                        double new_fit = res.second;
                        if (better(new_fit, ts)) { best_fit = new_fit; best_truck_sum = ts; best_sol = std::move(trial); improved_any = true; }
                    }
                }

                // Try creating a new drone trip delivering this package at its own customer
                {
                    DroneTrip new_trip;
                    ResupplyEvent new_ev{pkg, truck_id, {pkg}}; // rendezvous at customer itself
                    if (!p.reachable_mask.empty() && !p.reachable_mask[pkg]) goto skip_new_trip; // cannot reach
                    if (p.customers[pkg].demand > p.M_d + 1e-9) goto skip_new_trip; // package too heavy
                    new_trip.events.push_back(new_ev);
                    // remove pkg from current event
                    Solution trial = s;
                    auto &from_ev_new = trial.drone_queue[trip_idx].events[ev_idx];
                    from_ev_new.packages.erase(from_ev_new.packages.begin() + pkg_idx);
                    if (from_ev_new.packages.empty()) {
                        trial.drone_queue[trip_idx].events.erase(trial.drone_queue[trip_idx].events.begin()+ev_idx);
                        if (trial.drone_queue[trip_idx].events.empty()) trial.drone_queue.erase(trial.drone_queue.begin()+trip_idx);
                    }
                    rebuild_loaded_from_drone_marks(p, trial);
                    int ins_pos = find_insert_pos_queue(p, trial, new_trip);
                    if (ins_pos != -1) {
                        trial.drone_queue.insert(trial.drone_queue.begin()+ins_pos, new_trip);
                        rebuild_loaded_from_drone_marks(p, trial);
                        if (!trip_endurance_optimistic(p, trial, trial.drone_queue[ins_pos])) goto skip_new_trip; // quick check
                        // Source trip index may shift after erase; use full validation instead of index-based check.
                        {
                            std::string reason_local;
                            if (!validate_solution(p, trial, reason_local)) goto skip_new_trip;
                        }
                        double ts = 0.0; auto res = fitness_full(p, trial, &ts);
                        if (!res.first) goto skip_new_trip;
                        double new_fit = res.second;
                        if (better(new_fit, ts)) { best_fit = new_fit; best_truck_sum = ts; best_sol = std::move(trial); improved_any = true; }
                    }
                }
                skip_new_trip: ;

                // Option: move package back to depot load (remove from drone) on same truck.
                // If we do this for pkg, then ALL packages of the same truck with release <= release(pkg)
                // are also truck-loaded from depot (removed from any drone trips).
                Solution trial = s;
                auto &from_ev2 = trial.drone_queue[trip_idx].events[ev_idx];
                from_ev2.packages.erase(from_ev2.packages.begin() + pkg_idx);
                if (from_ev2.packages.empty()) {
                    trial.drone_queue[trip_idx].events.erase(trial.drone_queue[trip_idx].events.begin()+ev_idx);
                    if (trial.drone_queue[trip_idx].events.empty()) trial.drone_queue.erase(trial.drone_queue.begin()+trip_idx);
                }

                int rel_pkg = p.customers[pkg].release;
                // remove other packages (same truck) with release <= rel_pkg from all drone trips
                for (size_t trp = 0; trp < trial.drone_queue.size(); ) {
                    auto &dq = trial.drone_queue[trp].events;
                    for (size_t evi = 0; evi < dq.size(); ) {
                        if (dq[evi].truck_id != truck_id) { ++evi; continue; }
                        auto &vec = dq[evi].packages;
                        vec.erase(std::remove_if(vec.begin(), vec.end(), [&](int pk){ return p.customers[pk].release <= rel_pkg; }), vec.end());
                        if (vec.empty()) { dq.erase(dq.begin()+evi); }
                        else { ++evi; }
                    }
                    if (dq.empty()) trial.drone_queue.erase(trial.drone_queue.begin()+trp);
                    else ++trp;
                }

                rebuild_loaded_from_drone_marks(p, trial);
                double ts = 0.0; auto res = fitness_full(p, trial, &ts);
                if (!res.first) continue;
                double new_fit = res.second;
                if (better(new_fit, ts)) { best_fit = new_fit; best_truck_sum = ts; best_sol = std::move(trial); improved_any = true; }
            }
        }
    }
    if (improved_any) { s = std::move(best_sol); return true; }
    return false;
}

// -------- Drone local search: Trip Reordering (first-improvement) --------
// Move a drone trip to another position in the queue (same trip contents) while preserving truck-order constraint.
// If move improves global fitness, apply first found.
static bool reorder_trip_first_improve(const Params &p, Solution &s) {
    if (s.drone_queue.size() < 2) return false;

    std::vector<TruckTimeline> timelines(s.trucks.size());
    for (size_t k = 0; k < s.trucks.size(); ++k) timelines[k] = compute_truck_timeline(p, s.trucks[k]);
    double best_truck_sum = 0.0;
    double best_fit = fitness_full(p, s, &best_truck_sum).second;
    Solution best_sol = s;
    std::string reason;
    bool improved_any = false;
    auto better = [&](double f, double sum){
        return (f + 1e-9 < best_fit) || (std::fabs(f - best_fit) <= 1e-9 && sum + 1e-9 < best_truck_sum);
    };

    for (size_t i = 0; i < s.drone_queue.size(); ++i) {
        for (size_t j = 0; j < s.drone_queue.size(); ++j) {
            if (i == j) continue;
            Solution trial = s;
            DroneTrip trip = trial.drone_queue[i];
            trial.drone_queue.erase(trial.drone_queue.begin()+i);
            size_t insert_pos = (i < j) ? j-1 : j;
            trial.drone_queue.insert(trial.drone_queue.begin()+insert_pos, trip);

            // quick endurance check for this trip only (optimistic)
            if (!trip_endurance_optimistic(p, trial, trip)) continue;
            double ts = 0.0; auto res = fitness_full(p, trial, &ts);
            if (!res.first) continue;
            double new_fit = res.second;
            if (better(new_fit, ts)) { best_fit = new_fit; best_truck_sum = ts; best_sol = std::move(trial); improved_any = true; }
        }
    }
    if (improved_any) { s = std::move(best_sol); return true; }
    return false;
}
// ---------------- Feasibility checks ----------------

static bool validate_solution(const Params &p, const Solution &sol, std::string &reason) {
    const int n = static_cast<int>(p.customers.size());

    // 1) Each customer appears once on trucks
    std::vector<int> cust_count(n, 0);
    for (const auto &tr : sol.trucks) {
        for (const auto &st : tr.stops) {
            if (st.customer == 0) continue;
            if (st.customer >= n || st.customer < 0) { reason = "Truck stop uses invalid customer id"; return false; }
            cust_count[st.customer] += 1;
        }
    }
    for (int i = 1; i < n; ++i) {
        if (cust_count[i] != 1) { reason = "Customer " + std::to_string(i) + " not visited exactly once"; return false; }
    }

    // 2) Each package resupplied at most once
    std::vector<int> pkg_resupplied(n, 0);
    for (const auto &trip : sol.drone_queue) {
        for (const auto &ev : trip.events) {
            for (PackageId pkg : ev.packages) {
                if (pkg <= 0 || pkg >= n) { reason = "Invalid package id in drone trip"; return false; }
                pkg_resupplied[pkg] += 1;
                if (pkg_resupplied[pkg] > 1) { reason = "Package resupplied more than once"; return false; }
            }
        }
    }

    // 3) At most one drone rendezvous location per customer
    std::unordered_set<int> rendezvous_seen;
    for (const auto &trip : sol.drone_queue) {
        for (const auto &ev : trip.events) {
            if (ev.rendezvous_customer <= 0 || ev.rendezvous_customer >= n) { reason = "Invalid rendezvous customer id"; return false; }
            if (!p.reachable_mask.empty() && !p.reachable_mask[ev.rendezvous_customer]) {
                reason = "Rendezvous customer outside drone range"; return false;
            }
            if (!rendezvous_seen.insert(ev.rendezvous_customer).second) {
                reason = "Multiple drone trips visit customer " + std::to_string(ev.rendezvous_customer);
                return false;
            }
        }
    }

    // 3a) A drone trip cannot rendezvous with the same truck more than once.
    for (const auto &trip : sol.drone_queue) {
        std::unordered_set<int> seen_trucks;
        for (const auto &ev : trip.events) {
            if (!seen_trucks.insert(ev.truck_id).second) {
                reason = "A drone trip meets truck " + std::to_string(ev.truck_id) + " more than once";
                return false;
            }
        }
    }

    // 3b) Queue order consistency: for each truck, trips must follow the truck's visit order
    // If two trips rendezvous with the same truck at a1 then a2, and a1 precedes a2 on the truck route,
    // then the trip containing a1 must appear no later than the trip containing a2 in the queue.
    std::vector<int> last_pos(sol.trucks.size(), -1); // per truck, last rendezvous position seen in queue order
    for (const auto &trip : sol.drone_queue) {
        // compute earliest rendezvous position on each truck within this trip
        std::unordered_map<int,int> first_pos;
        for (const auto &ev : trip.events) {
            int t = ev.truck_id;
            int pos = route_pos(sol, t, ev.rendezvous_customer);
            if (pos < 0) { reason = "Rendezvous customer not on its truck route"; return false; }
            auto it = first_pos.find(t);
            if (it == first_pos.end() || pos < it->second) first_pos[t] = pos;
        }
        // enforce non-decreasing vs last_pos
        for (auto &kv : first_pos) {
            int t = kv.first; int pos = kv.second;
            if (last_pos[t] > pos) {
                reason = "Drone queue violates truck order for truck " + std::to_string(t);
                return false;
            }
            last_pos[t] = pos;
        }
    }

    // 3c) Package-truck consistency and precedence:
    // - A package can only be resupplied to the truck that visits its customer.
    // - The truck must receive the package no later than the stop where that customer is served.
    std::vector<int> customer_truck(n, -1), customer_pos(n, -1);
    for (size_t t = 0; t < sol.trucks.size(); ++t) {
        for (size_t pos = 0; pos < sol.trucks[t].stops.size(); ++pos) {
            int c = sol.trucks[t].stops[pos].customer;
            if (c <= 0 || c >= n) continue;
            customer_truck[c] = static_cast<int>(t);
            customer_pos[c] = static_cast<int>(pos);
        }
    }
    for (const auto &trip : sol.drone_queue) {
        for (const auto &ev : trip.events) {
            int rv_pos = route_pos(sol, ev.truck_id, ev.rendezvous_customer);
            if (rv_pos < 0) { reason = "Rendezvous customer not on its truck route"; return false; }
            for (PackageId pkg : ev.packages) {
                if (pkg <= 0 || pkg >= n) { reason = "Invalid package id in drone trip"; return false; }
                if (customer_truck[pkg] != ev.truck_id) {
                    reason = "Package " + std::to_string(pkg) + " resupplied to wrong truck";
                    return false;
                }
                if (customer_pos[pkg] < rv_pos) {
                    reason = "Package " + std::to_string(pkg) + " resupplied after its customer was already served";
                    return false;
                }
            }
        }
    }

    // Precompute truck timelines for time checks
    std::vector<TruckTimeline> timelines(sol.trucks.size());
    for (size_t k = 0; k < sol.trucks.size(); ++k) timelines[k] = compute_truck_timeline(p, sol.trucks[k]);

    // 4) Drone capacity per trip
    for (const auto &trip : sol.drone_queue) {
        int load = 0;
        for (const auto &ev : trip.events) {
            for (PackageId pkg : ev.packages) load += p.customers[pkg].demand;
        }
        if (load > p.M_d + 1e-9) { reason = "Drone trip exceeds capacity"; return false; }
    }

    // 5) Flight endurance hard rule:
    // (return_time - depart_time) - (#rendezvous * sigma) <= L_d
    for (const auto &trip : sol.drone_queue) {
        if (trip.events.empty()) continue;

        // release constraint
        int max_release = 0;
        for (const auto &ev : trip.events) {
            for (PackageId pkg : ev.packages) max_release = std::max(max_release, p.customers[pkg].release);
        }

        // timing simulation (availability assumed 0)
        double t = 0.0;
        int last_loc = 0;
        double unload_total = p.sigma * trip.events.size();

        // align start so that truck is reachable and packages ready
        const auto &first = trip.events.front();
        double truck_arrival_first = truck_arrival_at(sol, timelines, first.truck_id, first.rendezvous_customer);
        double launch_earliest = truck_arrival_first - p.drone_time[0][first.rendezvous_customer];
        double depart = std::max({0.0, (double)max_release, launch_earliest});
        t = depart;
        for (const auto &ev : trip.events) {
            double leg = p.drone_time[last_loc][ev.rendezvous_customer];
            t += leg;
            double truck_arrival = truck_arrival_at(sol, timelines, ev.truck_id, ev.rendezvous_customer);
            if (t < truck_arrival) t = truck_arrival;
            t += p.sigma; // unload time excluded from endurance rule
            last_loc = ev.rendezvous_customer;
        }
        // return to depot
        t += p.drone_time[last_loc][0];
        double trip_time = t - depart;
        double endurance_measure = trip_time - unload_total;

        if (endurance_measure - 1e-9 > p.L_d) {
            reason = "Drone trip exceeds flight endurance";
            return false;
        }
    }

    return true;
}

static vector<string> tokenize(const string &line){
    vector<string> out; string tmp; std::istringstream iss(line);
    while (iss >> tmp) out.push_back(tmp);
    return out;
}

static void read_instance(const string &path, Params &p){
    ifstream in(path);
    if(!in) throw runtime_error("Cannot open file: " + path);
    // remember basename for logging
    auto pos = path.find_last_of("/\\");
    g_data_name = (pos == string::npos) ? path : path.substr(pos+1);
    string line;
    // Detect format: either parameterized (number_truck ... Sigma ...) or bare table (XCOORD ...).
    getline(in, line);
    auto first_tokens = tokenize(line);
    auto looks_numeric = [](const string &s){
        if (s.empty()) return false;
        char* end=nullptr;
        std::strtod(s.c_str(), &end);
        return end && *end == '\0';
    };

    bool parameterized = !first_tokens.empty() && first_tokens[0] == "number_truck";
    bool header_only   = !parameterized && !first_tokens.empty() && !looks_numeric(first_tokens[0]); // e.g., "XCOORD"

    // Defaults matching Python prototype for Solomon-style files
    auto set_defaults = [&](){
        p.n_truck     = 2;
        p.n_drone     = 2;
        p.truck_speed = 0.5;
        p.drone_speed = 1.0;
        p.M_d         = 4.0;
        p.L_d         = 90.0;
        p.sigma       = 5.0;
    };

    string first_data_line;
    if (parameterized) {
        p.n_truck    = stoi(first_tokens.back());
        getline(in,line); p.n_drone    = stoi(tokenize(line).back());
        getline(in,line); p.truck_speed= stod(tokenize(line).back());
        getline(in,line); p.drone_speed= stod(tokenize(line).back());
        getline(in,line); p.M_d        = stod(tokenize(line).back());
        getline(in,line); p.L_d        = stod(tokenize(line).back());
        getline(in,line); p.sigma      = stod(tokenize(line).back());
        getline(in,line); /* skip header row: XCOORD YCOORD DEMAND RELEASE_DATE */
    } else if (header_only) {
        set_defaults();
        // first line was just the header; no data to store
    } else {
        // No header at all: treat first line as data row with default parameters
        set_defaults();
        first_data_line = line;
    }

    // remainder: customers (including depot) until EOF
    p.customers.clear();
    auto parse_customer = [&](const string &ln){
        if(ln.empty()) return;
        auto tok = tokenize(ln);
        if(tok.size() < 4) return;
        Customer c; c.x = stod(tok[0]); c.y = stod(tok[1]);
        c.demand = stoi(tok[2]); c.release = stoi(tok[3]);
        p.customers.push_back(c);
    };

    if (!first_data_line.empty()) parse_customer(first_data_line);
    while(getline(in,line)){
        if(line.empty()) continue;
        parse_customer(line);
    }
    const int n = static_cast<int>(p.customers.size());
    p.truck_time.assign(n, vector<double>(n, 0.0));
    p.drone_time.assign(n, vector<double>(n, 0.0));
    for(int i=0;i<n;++i){
        for(int j=0;j<n;++j){
            const auto &a = p.customers[i];
            const auto &b = p.customers[j];
            double manhattan = fabs(a.x - b.x) + fabs(a.y - b.y);
            double euclid = hypot(a.x - b.x, a.y - b.y);
            p.truck_time[i][j] = manhattan / p.truck_speed;
            p.drone_time[i][j] = euclid    / p.drone_speed;
        }
    }

    // classify customers by drone reachability (round trip from depot within L_d)
    p.reachable_by_drone.clear();
    p.unreachable_by_drone.clear();
    p.reachable_mask.assign(n, 0);
    for (int i = 1; i < n; ++i) { // skip depot
        double round_trip = 2.0 * p.drone_time[0][i];
        if (round_trip <= p.L_d + 1e-9) {
            p.reachable_by_drone.push_back(i);
            p.reachable_mask[i] = 1;
        } else {
            p.unreachable_by_drone.push_back(i);
        }
    }
}

int main(int argc, char** argv){
    try {
        g_solve_start = std::chrono::steady_clock::now();
        // Khi chạy từ thư mục C_Version, file dữ liệu nằm ở ../test_data/...
        if (argc < 2) {
            std::cerr << "Usage: " << argv[0] << " <data_file> [M_d] [L_d]\n";
            return 1;
        }
        string path = argv[1];
        Params p; read_instance(path, p);
        if (argc >= 3) p.M_d = stod(argv[2]);
        if (argc >= 4) p.L_d = stod(argv[3]);
    cout << "Loaded instance: " << path << "\n";
    cout << "Customers (incl. depot): " << p.customers.size() << "\n";
    cout << "Trucks: " << p.n_truck << ", Drones: " << p.n_drone << "\n";
    cout << fixed << setprecision(2);
    cout << "Truck time matrix [0..4][0..4]:\n";
    for(int i=0;i<min<size_t>(5,p.truck_time.size());++i){
        for(int j=0;j<min<size_t>(5,p.truck_time[i].size());++j){
            cout << p.truck_time[i][j] << (j+1==min<size_t>(5,p.truck_time[i].size())?"\n":"\t");
        }
    }
    cout << "Drone time matrix [0..4][0..4]:\n";
    for(int i=0;i<min<size_t>(5,p.drone_time.size());++i){
        for(int j=0;j<min<size_t>(5,p.drone_time[i].size());++j){
            cout << p.drone_time[i][j] << (j+1==min<size_t>(5,p.drone_time[i].size())?"\n":"\t");
        }
    }

    Solution sol = build_initial_truck_routes(p);
    std::cout << "--- After initial truck build ---\n";
    print_truck_routes(sol);
    auto sim0 = simulate_with_log(p, sol);
    std::cout << "Fitness_full (makespan): " << sim0.makespan << "\n";
    std::cout << "Objective (full) makespan: " << sim0.makespan << "\n";
    std::cout << "Truck-only makespan: " << truck_only_makespan(p, sol) << "\n";

    improve_truck_routes_2opt(p, sol);
    std::cout << "--- After truck 2-opt ---\n";
    print_truck_routes(sol);
    auto sim1 = simulate_with_log(p, sol);
    std::cout << "Fitness_full (makespan): " << sim1.makespan << "\n";
    std::cout << "Objective (full) makespan: " << sim1.makespan << "\n";
    std::cout << "Truck-only makespan: " << truck_only_makespan(p, sol) << "\n";

    // ATS full (truck neighborhoods + drone local search per iteration)
    int n_cust = (int)p.customers.size() - 1;
    (void)n_cust;
    ats_full(p, sol, /*SEG*/4, /*theta*/2.0, /*DIV*/3);
    std::cout << "--- After ATS ---\n";
    auto sim_tab = simulate_with_log(p, sol);
    std::cout << "Fitness_full (makespan): " << sim_tab.makespan << "\n";
    print_truck_routes(sol);

    // Drone local search: iterate LS operators until a full cycle gives no improvement
    double base_fit = fitness_full(p, sol).second;
    std::cout << "[LS] start fitness: " << base_fit << "\n";
    bool improved_cycle = true;
    while (improved_cycle) {
        if (solver_time_limit_reached()) {
            std::cout << "[LS] time limit reached at " << elapsed_solver_seconds()
                      << "s, returning current best\n";
            break;
        }
        improved_cycle = false;
        if (relocate_sync_point_first_improve(p, sol)) {
            double f = fitness_full(p, sol).second;
            std::cout << "[LS] sync relocation applied, fitness: " << f << "\n";
            improved_cycle = true;
        }
        if (relocate_package_first_improve(p, sol)) {
            double f = fitness_full(p, sol).second;
            std::cout << "[LS] package relocation applied, fitness: " << f << "\n";
            improved_cycle = true;
        }
        if (reorder_trip_first_improve(p, sol)) {
            double f = fitness_full(p, sol).second;
            std::cout << "[LS] trip reorder applied, fitness: " << f << "\n";
            improved_cycle = true;
        }
    }

    std::cout << "--- After drone local search init ---\n";
    print_truck_routes(sol);
    auto sim = simulate_with_log(p, sol);
    for (const auto &row : sim.truck_rows) {
        std::cout << "TRUCK " << row.truck << " idx " << row.route_index << " city " << row.city
                  << " arr " << row.arrival << " dep " << row.departure << " wait " << row.waiting << "\n";
    }
    for (const auto &trip : sim.drone_rows) {
        std::cout << "DRONE trip " << trip.trip_idx << " id " << trip.drone_id << " ready " << trip.drone_ready_at_depot
                  << " dep " << trip.depart_depot << " ret " << trip.return_depot << "\n";
        for (const auto &leg : trip.legs) {
            std::cout << "  leg " << leg.leg << " city " << leg.launch_city << " truck " << leg.owner_truck
                      << " d_arr " << leg.drone_arrival << " t_arr " << leg.truck_arrival
                      << " d_wait " << leg.drone_wait << " t_wait " << leg.truck_wait
                      << " service_end " << leg.service_end << " pkgs ";
            for (auto pk : leg.customers) std::cout << pk << " ";
            std::cout << "\n";
        }
    }
    std::cout << "Makespan: " << sim.makespan << " (truck max " << *max_element(sim.truck_return.begin(), sim.truck_return.end())
              << ", drone max " << sim.max_drone_finish << ")\n";
    std::cout << "[SOL] best solution detail:\n";
    print_solution_compact(std::cout, sol, p);

    std::string best_path = write_solution_json(sol, p, "best");
    if (!best_path.empty()) {
        std::cout << "[SOL] best solution file: " << best_path << "\n";
    } else {
        std::cout << "[SOL] best solution file: (write failed)\n";
    }

    if (g_has_best_multi_solution) {
        std::cout << "[SOL] best multi-visit solution detail:\n";
        print_solution_compact(std::cout, g_best_multi_solution, p);
        std::string best_multi_path = write_solution_json(g_best_multi_solution, p, "best_multi");
        if (!best_multi_path.empty()) {
            std::cout << "[SOL] best multi-visit solution file: " << best_multi_path << "\n";
        } else {
            std::cout << "[SOL] best multi-visit solution file: (write failed)\n";
        }
    } else {
        std::cout << "[SOL] best multi-visit solution file: (not found)\n";
    }
    } catch (const std::exception &ex) {
        std::cerr << "Error: " << ex.what() << "\n";
        return 1;
    }
    return 0;
}
