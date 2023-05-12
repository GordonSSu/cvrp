#pragma once
#include <cmath>
#include <unordered_map>
#include <vector>
#include "gurobi_c++.h"

struct Target {
	std::pair<int, int> location;
	int weight;
	int id;
};

struct Vehicle {
	std::pair<int, int> depot_location;
	int capacity;
	int id;
};

const int MAX_COORDINATE = 1000;
const int MIN_CAPACITY = 100;
const int MAX_CAPACITY = 500;
const int MIN_WEIGHT = 10;
const int MAX_WEIGHT = 100;

std::unordered_map<int, Target>			targets;
std::unordered_map<int, Vehicle>		vehicles;
std::vector<int>						target_ids;

double euclidean_distance(std::pair<int, int> location1, std::pair<int, int> location2);
void generate_random_instance(int num_targets, int num_vehicles);
void create_singleton_bids(bool consider_only_best_bid, GRBModel& model, GRBLinExpr* objFunction, std::unordered_map<int, std::vector<GRBVar>>& bids_for_target);
void create_pair_bids(bool consider_only_best_bid, GRBModel& model, GRBLinExpr* objFunction, std::unordered_map<int, std::vector<GRBVar>>& bids_for_target);
void create_constraints(GRBModel& model, std::unordered_map<int, std::vector<GRBVar>> bids_for_target);
void output_results(GRBModel& model);
void cvrp(bool consider_only_best_bid);

/*
 * Return Euclidean distance between two points
 * @param location1, location2 - The coordinate pairs to measure the distance between
 */
double euclidean_distance(std::pair<int, int> location1, std::pair<int, int> location2) {
	int x_dist = location1.first;
	int y_dist = location2.second;
	return sqrt(x_dist * x_dist + y_dist * y_dist);
}

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
		new_target.location = std::make_pair(x_coor, y_coor);
		new_target.weight = rand() % (MAX_WEIGHT - MIN_WEIGHT + 1) + MIN_WEIGHT;
		new_target.id = i;
		targets.insert(std::make_pair(new_target.id, new_target));
		target_ids.push_back(new_target.id);
	}

	// Generate vehicles
	for (int i = 0; i < num_vehicles; i++) {
		Vehicle new_vehicle = {};
		int x_coor = rand() % (MAX_COORDINATE + 1);
		int y_coor = rand() % (MAX_COORDINATE + 1);
		new_vehicle.depot_location = std::make_pair(x_coor, y_coor);
		new_vehicle.capacity = rand() % (MAX_CAPACITY - MIN_CAPACITY + 1) + MIN_CAPACITY;
		new_vehicle.id = i;
		vehicles.insert(std::make_pair(new_vehicle.id, new_vehicle));
	}
}

/*
 * Creates singleton bids
 * @param consider_only_best_bid - Flag indicating whether to consider only the best bid for each itemset or all of them
 * @param model - GRB model
 * @param objFunction - Objective function to minimize
 * @param bids_for_target - Map storing bid variables for each target (both singleton and pair bids)
 */
void create_singleton_bids(bool consider_only_best_bid, GRBModel& model, GRBLinExpr* objFunction, std::unordered_map<int, std::vector<GRBVar>>& bids_for_target) {
	for (auto& target_entry : targets) {
		Target target = target_entry.second;

		// All singleton bids for the current target
		std::vector<GRBVar> target_singletons;

		// Store best bid (in case it should be the only bid considered)
		std::string best_bid_var_name;
		double min_tour_length = -1;

		// Create bid for each vehicle based on tour length
		for (auto& vehicle_entry : vehicles) {
			Vehicle vehicle = vehicle_entry.second;
			std::string var_name = std::to_string(vehicle.id) + "," + std::to_string(target.id);
			double tour_length = 2.0 * euclidean_distance(target.location, vehicle.depot_location);	

			// Update best bid, if applicable
			if (consider_only_best_bid) {
				if (tour_length < min_tour_length || min_tour_length == -1) {
					best_bid_var_name = var_name;
					min_tour_length = tour_length;
				}
			}

			// Add bid to objective function
			else {
				GRBVar bid_var = model.addVar(0.0, 1.0, tour_length, GRB_BINARY, var_name);
				target_singletons.push_back(bid_var);
				GRBLinExpr* term = new GRBLinExpr(bid_var, tour_length);
    			*objFunction += *term;
			}
		}

		// If the best bid is the only one to be considered, add it to the objective function
		if (consider_only_best_bid) {
			GRBVar best_bid_var = model.addVar(0.0, 1.0, min_tour_length, GRB_BINARY, best_bid_var_name);
			target_singletons.push_back(best_bid_var);
			GRBLinExpr* term = new GRBLinExpr(best_bid_var, min_tour_length);
    		*objFunction += *term;
		}

		// Store singleton bids for current target
		bids_for_target.insert(std::make_pair(target.id, target_singletons));
	}
}

/*
 * Creates pair bids
 * @param consider_only_best_bid - Flag indicating whether to consider only the best bid for each itemset or all of them
 * @param model - GRB model
 * @param objFunction - Objective function to minimize
 * @param bids_for_target - Map storing bid variables for each target (both singleton and pair bids)
 */
void create_pair_bids(bool consider_only_best_bid, GRBModel& model, GRBLinExpr* objFunction, std::unordered_map<int, std::vector<GRBVar>>& bids_for_target) {
	for (int target_id_index_1 = 0; target_id_index_1 < targets.size() - 1; target_id_index_1++) {
		for (int target_id_index_2 = target_id_index_1 + 1; target_id_index_2 < targets.size(); target_id_index_2++) {
			int target_id_1 = target_id_index_1;
			int target_id_2 = target_id_index_2;
			Target target_1 = targets[target_id_1];
			Target target_2 = targets[target_id_2];

			// All pair bids for the current target
			std::vector<GRBVar> target_pair_bids;

			// Store best bid (in case it should be the only bid considered)
			std::string best_bid_var_name;
			double min_tour_length = -1;

			// Create bid for each vehicle based on tour length
			for (auto& vehicle_entry : vehicles) {
				Vehicle vehicle = vehicle_entry.second;
				std::string var_name = std::to_string(vehicle.id) + "," + std::to_string(target_1.id) + "," + std::to_string(target_2.id);
				double tour_length = euclidean_distance(target_1.location, target_2.location)
									+ euclidean_distance(target_1.location, vehicle.depot_location)
									+ euclidean_distance(target_2.location, vehicle.depot_location);

				// Update best bid, if applicable
				if (consider_only_best_bid) {
					if (tour_length < min_tour_length || min_tour_length == -1) {
						best_bid_var_name = var_name;
						min_tour_length = tour_length;
					}
				}

				// Add bid to objective function
				else {
					GRBVar bid_var = model.addVar(0.0, 1.0, tour_length, GRB_BINARY, var_name);
					target_pair_bids.push_back(bid_var);
					GRBLinExpr* term = new GRBLinExpr(bid_var, tour_length);
	    			*objFunction += *term;
				}
			}

			// If the best bid is the only one to be considered, add it to the objective function
			if (consider_only_best_bid) {
				GRBVar best_bid_var = model.addVar(0.0, 1.0, min_tour_length, GRB_BINARY, best_bid_var_name);
				target_pair_bids.push_back(best_bid_var);
				GRBLinExpr* term = new GRBLinExpr(best_bid_var, min_tour_length);
	    		*objFunction += *term;
			}

			// Store pair bids for current targets
			std::vector<GRBVar>& target_1_bids = bids_for_target[target_1.id];
			target_1_bids.insert(target_1_bids.end(), target_pair_bids.begin(), target_pair_bids.end());
			std::vector<GRBVar>& target_2_bids = bids_for_target[target_2.id];
			target_2_bids.insert(target_2_bids.end(), target_pair_bids.begin(), target_pair_bids.end());
		}
	}
}

/*
 * Creates constraints such that each target is serviced exactly once
 * @param model - GRB model
 * @param bids_for_target - Map storing bid variables for each target (both singleton and pair bids)
 */
void create_constraints(GRBModel& model, std::unordered_map<int, std::vector<GRBVar>> bids_for_target) {
	for (auto& target_bids: bids_for_target) {
		GRBLinExpr* target_constraint = new GRBLinExpr();

		for (GRBVar& bid: target_bids.second) {
			*target_constraint += *(new GRBLinExpr(bid));
		}

		// Constraint: target must be serviced exactly once
		model.addConstr(*target_constraint == 1.0f, "");
	}
}

/*
 * Outputs vehicle-target assignments
 * @param model - Optimized Gurobi model
 */
void output_results(GRBModel& model, std::unordered_map<int, std::vector<GRBVar>> bids_for_target) {
	// Output minimum sum of tour lengths
	std::cout << "Minimum Sum of Tour Lengths: " << static_cast<long long>(model.get(GRB_DoubleAttr_ObjVal)) << std::endl;

	// TO FINISH!!!
}

/*
 * Sets up and solves CVRP problem with desired problem formulation
 * @param consider_only_best_bid - Flag indicating whether to consider only the best bid for each itemset or all of them
 */
void cvrp(bool consider_only_best_bid) {
	try {
		// Initialize environment and suppress output
		GRBEnv* environment = new GRBEnv();
		environment -> set(GRB_IntParam_OutputFlag, 0);

		// Initialize model and objective function
		GRBModel model = GRBModel(*environment);
		GRBLinExpr* objFunction = new GRBLinExpr();

		// Store bids for each target (for XOR constraints)
		std::unordered_map<int, std::vector<GRBVar>> bids_for_target;

		// Create bids
		create_singleton_bids(consider_only_best_bid, model, objFunction, bids_for_target);
		create_pair_bids(consider_only_best_bid, model, objFunction, bids_for_target);

	    // Goal is to minimize objective function
	    model.setObjective(*objFunction, GRB_MINIMIZE);

	    // Create constraints
	    create_constraints(model, bids_for_target);

	    // Optimize objective
		model.optimize();

		// Output results
		output_results(model, bids_for_target);
	} catch (GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    } catch (...) {
        std::cout << "Exception during optimization" << std::endl;
    }
}
