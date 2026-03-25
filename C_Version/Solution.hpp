#pragma once
#include <vector>
#include <string>
#include <utility>
#include <cstdint>

// A package is identified by customer id whose demand must be delivered.
using PackageId = int; // 0 is depot, >0 customers

struct ResupplyEvent {
    int rendezvous_customer;   // node i where drone meets truck
    int truck_id;              // truck k that receives the packages
    std::vector<PackageId> packages; // list of customer ids j delivered at this rendezvous
};

// One drone trip can have multiple rendezvous with possibly different trucks.
struct DroneTrip {
    std::vector<ResupplyEvent> events; // ordered along the drone sortie
};

// Stop on a truck route: customer visited and packages picked up from drone earlier on the route.
struct TruckStop {
    int customer;                 // node id, 0 = depot
    std::vector<PackageId> loaded_from_drone; // packages the truck receives at this stop (may be empty)
};

struct TruckRoute {
    int truck_id{};
    std::vector<TruckStop> stops; // includes depot at start and end
};

// Overall solution representation mirroring the paper and the Python prototype.
struct Solution {
    std::vector<TruckRoute> trucks;     // size = number of trucks
    std::vector<DroneTrip> drone_queue; // FIFO assignment to the first available drone
};

// Helpers to build an empty solution skeleton.
inline Solution make_empty_solution(int n_truck) {
    Solution s;
    s.trucks.resize(n_truck);
    for (int k = 0; k < n_truck; ++k) {
        s.trucks[k].truck_id = k;
        s.trucks[k].stops = {TruckStop{0, {}}, TruckStop{0, {}}}; // depot->depot
    }
    return s;
}

// Append a customer to a given truck route (before the final depot).
inline void insert_truck_customer(Solution &s, int truck_id, int customer) {
    auto &stops = s.trucks[truck_id].stops;
    stops.insert(stops.end() - 1, TruckStop{customer, {}});
}

// Add a drone trip with one rendezvous.
inline void add_single_rendezvous_trip(Solution &s, int rendezvous_customer, int truck_id, const std::vector<PackageId> &pkgs) {
    DroneTrip trip;
    trip.events.push_back(ResupplyEvent{rendezvous_customer, truck_id, pkgs});
    s.drone_queue.push_back(std::move(trip));
}

// Convenience: record that at stop `customer` on truck `truck_id`, the truck receives given packages.
inline void mark_truck_receives(Solution &s, int truck_id, int customer, const std::vector<PackageId> &pkgs) {
    auto &stops = s.trucks[truck_id].stops;
    for (auto &st : stops) {
        if (st.customer == customer) {
            st.loaded_from_drone.insert(st.loaded_from_drone.end(), pkgs.begin(), pkgs.end());
            return;
        }
    }
}
