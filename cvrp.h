#pragma once
#include "gurobi_c++.h"

struct Target {
	std::pair<int, int> location;
	int weight;
};

struct Vehicle {
	std::pair<int, int> depot_location;
	int capacity;
};

const int MAX_COORDINATE = 1000;
const int MIN_CAPACITY = 20;
const int MAX_CAPACITY = 100;
const int MIN_WEIGHT = 10;
const int MAX_WEIGHT = 80;

std::vector<Vehicle>	targets;
std::vector<Vehicle>    vehicles;
std::vector<GRBVar>		bid_variables;
GRBEnv*					environment;
GRBModel				model;

/*
 * Generate problem instance with randomized locations and capacities.
 * @param num_vehicles, num_targets - The desired numbers of vehicles/depots and targets
 */
void generate_random_instance(int num_targets, int num_vehicles) {
	// Generate targets
	for (int i = 0; i < num_targets; i++) {
		Target new_target = {};
		int x_coor = rand() % (MAX_COORDINATE + 1);
		int y_coor = rand() % (MAX_COORDINATE + 1);
		new_target.location = make_pair(x_coor, y_coor);
		new_target.weight = rand() % (MAX_WEIGHT - MIN_WEIGHT + 1) + MIN_WEIGHT;
		targets.push_back(new_target);
	}

	// Generate vehicles
	for (int i = 0; i < num_vehicles; i++) {
		Vehicle new_vehicle = {};
		int x_coor = rand() % (MAX_COORDINATE + 1);
		int y_coor = rand() % (MAX_COORDINATE + 1);
		new_vehicle.location = make_pair(x_coor, y_coor);
		new_vehicle.capacity = rand() % (MAX_CAPACITY - MIN_CAPACITY + 1) + MIN_CAPACITY;
		vehicles.push_back(new_vehicle);
	}
}

void euclidean_distance(std::pair<int, int> location1, std::pair<int, int> location2) {
	return sqrt()
}

/*
 * Create bids for targets and target pairs based on the following heuristic:
 * For any given target or target pair, only the closest vehicle bids on it
 */
void create_bids() {
	// Create bids for each "singleton"
	for (Target target : targets) {
		float min_distance;
		Vehicle closest_vehicle;

		for (Vehicle vehicle : vehicles) {
			// Complete
		}

		GRBVar bid_var = model.addVar(0.0, 1.0, min_distance, GRB_BINARY, "");
		bid_variables.push_back(bid_var);
	}
}

/*
 * Solve the combinatorial auction such that
 * total distance is minimized and all targets are serviced
 */
void solve() {
	// Initialize environment and suppress output
	environment = new GRBEnv();
	environment -> set(GRB_IntParam_OutputFlag, 0);

	// Initialize model
	model = GRBModel(*environment);

	// Create objective function
    GRBLinExpr* objFunction = new GRBLinExpr();
    for (int i = 0; i < bid_variables.size(); i++) {
        GRBLinExpr* term = new GRBLinExpr(bid_variables[i], bid_variables[i].obj);
        *objFunction += *term;
    }

    // Goal is to minimize objectvie function
    model.setObjective(*objFunction, GRB_MINIMIZE);

}
