#pragma once
#include <cmath>
#include <fstream>
#include <iostream>
#include <regex>
#include <stdio.h>
#include <string>
#include <string.h>
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

// Parameters for random capacity/weight generation
const int MIN_CAPACITY = 100;
const int MAX_CAPACITY = 500;
const int MIN_WEIGHT = 10;
const int MAX_WEIGHT = 100;

// General use fields
std::unordered_map<int, Target>					targets;
std::unordered_map<int, Vehicle>				vehicles;
std::vector<int>								target_ids;
std::vector<int>								vehicle_ids;
std::unordered_map<int, std::vector<GRBVar>>	bids_for_target;	// Stores bid variables for each target (both singleton and pair bids)
std::unordered_map<int, std::vector<GRBVar>>	vehicle_bids;		// Stores bid variables for each vehicle (both singleton and pair bids)

// General use functions
double euclidean_distance(std::pair<int, int> location1, std::pair<int, int> location2);
void generate_random_instance(int grid_size_x, int grid_size_y, int num_targets, int num_vehicles);
void create_singleton_bids(bool consider_only_best_bid, GRBModel& model, GRBLinExpr* objFunction);
void create_pair_bids(bool consider_only_best_bid, GRBModel& model, GRBLinExpr* objFunction);
void create_constraints(GRBModel& model);
void append_results_to_file(std::string output_file_name, GRBModel& model, std::string instance_name);
void print_results(GRBModel& model, std::string instance_name);
void cvrp(bool consider_only_best_bid, std::string output_file_name, std::string instance_name = "CVRP Instance");

// Dataset testing functions
void reset_state();
void generate_instance(std::string vehicle_locations_line, std::string target_locations_line, std::string weights_line);
void dataset_cvrp(bool consider_only_best_bid, std::string input_file_name, std::string output_file_name);

/*
 * Return Euclidean distance between two points
 * @param location1, location2 - The coordinate pairs to measure the distance between
 */
double euclidean_distance(std::pair<int, int> location1, std::pair<int, int> location2) {
	int x_dist = location1.first - location2.first;
	int y_dist = location1.second - location2.second;
	return sqrt(x_dist * x_dist + y_dist * y_dist);
}

/*
 * Generate problem instance with randomized locations and capacities
 * @param grid_size_x, grid_size_y - The desired grid dimensions
 * @param num_vehicles, num_targets - The desired numbers of vehicles/depots and targets
 */
void generate_random_instance(int grid_size_x, int grid_size_y, int num_targets, int num_vehicles) {
	// Generate targets
	for (int i = 0; i < num_targets; i++) {
		Target new_target = {};
		int x_coor = rand() % (grid_size_x + 1);
		int y_coor = rand() % (grid_size_y + 1);
		new_target.location = std::make_pair(x_coor, y_coor);
		new_target.weight = rand() % (MAX_WEIGHT - MIN_WEIGHT + 1) + MIN_WEIGHT;
		new_target.id = i;
		targets.insert(std::make_pair(new_target.id, new_target));
		target_ids.push_back(new_target.id);
	}

	// Generate vehicles
	for (int i = 0; i < num_vehicles; i++) {
		Vehicle new_vehicle = {};
		int x_coor = rand() % (grid_size_x + 1);
		int y_coor = rand() % (grid_size_y + 1);
		new_vehicle.depot_location = std::make_pair(x_coor, y_coor);
		new_vehicle.capacity = rand() % (MAX_CAPACITY - MIN_CAPACITY + 1) + MIN_CAPACITY;
		new_vehicle.id = i;
		vehicles.insert(std::make_pair(new_vehicle.id, new_vehicle));
		vehicle_ids.push_back(new_vehicle.id);
	}
}

/*
 * Create singleton bids
 * @param consider_only_best_bid - Flag indicating whether to consider only the best bid for each itemset or all of them
 * @param model - GRB model
 * @param objFunction - Objective function to minimize
 */
void create_singleton_bids(bool consider_only_best_bid, GRBModel& model, GRBLinExpr* objFunction) {
	for (const auto& target_entry : targets) {
		Target target = target_entry.second;

		// All singleton bids for the current target
		std::vector<GRBVar> target_singletons;

		// Store best bids (in case they should be the only bids considered)
		std::vector<std::string> best_bids_var_names;
		std::vector<int> best_bids_vehicle_ids;
		double min_tour_length = -1;

		// Create bid for each vehicle based on tour length
		for (const auto& vehicle_entry : vehicles) {
			Vehicle vehicle = vehicle_entry.second;
			std::string var_name = std::to_string(vehicle.id) + "," + std::to_string(target.id);
			double tour_length = 2.0 * euclidean_distance(target.location, vehicle.depot_location);

			// If the target's weight exceeds the vehicle's capacity, set tour length to large value
			if (target.weight > vehicle.capacity) {
				tour_length = std::numeric_limits<float>::max();
			}

			// Update best bid if applicable
			if (consider_only_best_bid) {
				// New best bid
				if (tour_length < min_tour_length || min_tour_length == -1) {
					best_bids_var_names.clear();
					best_bids_vehicle_ids.clear();
					best_bids_var_names.push_back(var_name);
					best_bids_vehicle_ids.push_back(vehicle.id);
					min_tour_length = tour_length;
				}

				// Tie for best bid
				else if (tour_length == min_tour_length) {
					best_bids_var_names.push_back(var_name);
					best_bids_vehicle_ids.push_back(vehicle.id);
				}
			}

			// Add bid to objective function
			else {
				GRBVar bid_var = model.addVar(0.0, 1.0, tour_length, GRB_BINARY, var_name);
				target_singletons.push_back(bid_var);
				GRBLinExpr* term = new GRBLinExpr(bid_var, tour_length);
    			*objFunction += *term;

    			// Account for vehicle's bid
				if (vehicle_bids.find(vehicle.id) == vehicle_bids.end()) {
					std::vector<GRBVar> bids;
					bids.push_back(bid_var);
					vehicle_bids.insert(std::make_pair(vehicle.id, bids));
				} else {
					vehicle_bids.find(vehicle.id) -> second.push_back(bid_var);
				}
			}
		}

		// If the best bids are the only ones to be considered, add them to the objective function
		if (consider_only_best_bid) {
	    	for (int i = 0; i < best_bids_var_names.size(); i++) {
	    		std::string best_bid_var_name = best_bids_var_names[i];
	    		int best_bids_vehicle_id = best_bids_vehicle_ids[i];

	    		GRBVar best_bid_var = model.addVar(0.0, 1.0, min_tour_length, GRB_BINARY, best_bid_var_name);
				target_singletons.push_back(best_bid_var);
				GRBLinExpr* term = new GRBLinExpr(best_bid_var, min_tour_length);
	    		*objFunction += *term;

	    		// Account for vehicle's bid
				if (vehicle_bids.find(best_bids_vehicle_id) == vehicle_bids.end()) {
					std::vector<GRBVar> bids;
					bids.push_back(best_bid_var);
					vehicle_bids.insert(std::make_pair(best_bids_vehicle_id, bids));
				} else {
					vehicle_bids.find(best_bids_vehicle_id) -> second.push_back(best_bid_var);
				}
	    	}
		}

		// Store singleton bids for current target
		bids_for_target.insert(std::make_pair(target.id, target_singletons));
	}
}

/*
 * Create pair bids
 * @param consider_only_best_bid - Flag indicating whether to consider only the best bid for each itemset or all of them
 * @param model - GRB model
 * @param objFunction - Objective function to minimize
 */
void create_pair_bids(bool consider_only_best_bid, GRBModel& model, GRBLinExpr* objFunction) {
	for (int target_id_index_1 = 0; target_id_index_1 < targets.size() - 1; target_id_index_1++) {
		for (int target_id_index_2 = target_id_index_1 + 1; target_id_index_2 < targets.size(); target_id_index_2++) {
			int target_id_1 = target_id_index_1;
			int target_id_2 = target_id_index_2;
			Target target_1 = targets[target_id_1];
			Target target_2 = targets[target_id_2];

			// All bids for the current pair of targets
			std::vector<GRBVar> target_pair_bids;

			// Store best bids (in case they should be the only bids considered)
			std::vector<std::string> best_bids_var_names;
			std::vector<int> best_bids_vehicle_ids;
			double min_tour_length = -1;

			// Create bid for each vehicle based on tour length
			for (const auto& vehicle_entry : vehicles) {
				Vehicle vehicle = vehicle_entry.second;
				std::string var_name = std::to_string(vehicle.id) + "," + std::to_string(target_1.id) + "," + std::to_string(target_2.id);
				double tour_length = euclidean_distance(target_1.location, target_2.location)
									+ euclidean_distance(target_1.location, vehicle.depot_location)
									+ euclidean_distance(target_2.location, vehicle.depot_location);

				// If the target's weight exceeds the vehicle's capacity, set tour length to large value
				if (target_1.weight + target_2.weight > vehicle.capacity) {
					tour_length = std::numeric_limits<float>::max();
				}

				// Update best bid if applicable
				if (consider_only_best_bid) {
					// New best bid
					if (tour_length < min_tour_length || min_tour_length == -1) {
						best_bids_var_names.clear();
						best_bids_vehicle_ids.clear();
						best_bids_var_names.push_back(var_name);
						best_bids_vehicle_ids.push_back(vehicle.id);
						min_tour_length = tour_length;
					}

					// Tie for best bid
					else if (tour_length == min_tour_length) {
						best_bids_var_names.push_back(var_name);
						best_bids_vehicle_ids.push_back(vehicle.id);
					}
				}

				// Add bid to objective function
				else {
					GRBVar bid_var = model.addVar(0.0, 1.0, tour_length, GRB_BINARY, var_name);
					target_pair_bids.push_back(bid_var);
					GRBLinExpr* term = new GRBLinExpr(bid_var, tour_length);
	    			*objFunction += *term;

	    			// Account for vehicle's bid
					if (vehicle_bids.find(vehicle.id) == vehicle_bids.end()) {
						std::vector<GRBVar> bids;
						bids.push_back(bid_var);
						vehicle_bids.insert(std::make_pair(vehicle.id, bids));
					} else {
						vehicle_bids.find(vehicle.id) -> second.push_back(bid_var);
					}
				}
			}

			// If the best bids are the only ones to be considered, add them to the objective function
			if (consider_only_best_bid) {
				for (int i = 0; i < best_bids_var_names.size(); i++) {
		    		std::string best_bid_var_name = best_bids_var_names[i];
		    		int best_bids_vehicle_id = best_bids_vehicle_ids[i];
		    		
		    		GRBVar best_bid_var = model.addVar(0.0, 1.0, min_tour_length, GRB_BINARY, best_bid_var_name);
					target_pair_bids.push_back(best_bid_var);
					GRBLinExpr* term = new GRBLinExpr(best_bid_var, min_tour_length);
		    		*objFunction += *term;

		    		// Account for vehicle's bid
					if (vehicle_bids.find(best_bids_vehicle_id) == vehicle_bids.end()) {
						std::vector<GRBVar> bids;
						bids.push_back(best_bid_var);
						vehicle_bids.insert(std::make_pair(best_bids_vehicle_id, bids));
					} else {
						vehicle_bids.find(best_bids_vehicle_id) -> second.push_back(best_bid_var);
					}
		    	}
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
 * Create constraints such that each target is serviced exactly once
 * @param model - GRB model
 */
void create_constraints(GRBModel& model) {
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
 * Output vehicle-target assignments to file
 * @param output_file_name - Name of file to output results to
 * @param model - Optimized Gurobi model
 * @param instance_name - Name of current problem instance
 */
void append_results_to_file(std::string output_file_name, GRBModel& model, std::string instance_name) {
	// Create output file stream
    std::ofstream outfile;
    outfile.open(output_file_name, std::ios_base::app); // Append to file rather than overwrite

    if (outfile.is_open()) {
    	// Write instance name
    	outfile << instance_name << "\n";

		// Write minimum sum of tour lengths
		outfile << "Minimum Sum of Tour Lengths: " << model.get(GRB_DoubleAttr_ObjVal) << "\n";
		
		// Track assignments of targets to vehicles
		for (const auto& vehicle_entry : vehicles) {
			Vehicle vehicle = vehicle_entry.second;

			// Write vehicle information
			outfile << "Vehicle: " << vehicle.depot_location.first << "," << vehicle.depot_location.second << "\n";

			if (vehicle_bids.find(vehicle.id) != vehicle_bids.end()) {
				// Track the number of tours for the current vehicle
				int tour_count = 1;

				for (auto& bid : vehicle_bids[vehicle.id]) {
					// Bid has been "accepted by auctioneer"
					if (bid.get(GRB_DoubleAttr_X) == 1) {
						std::string bid_var_name = bid.get(GRB_StringAttr_VarName);
						char* split_element = strtok(&bid_var_name[0], ",");
						split_element = strtok(NULL, ",");
						int target_id_1 = std::stoi(split_element);
						Target target_1 = targets[target_id_1];

						// Output tour count and first target location
						outfile << "Tour " << tour_count++ << ": " << target_1.location.first << "," << target_1.location.second;

						// If the tour encompasses two targets, output second target location
						split_element = strtok(NULL, ",");
						if (split_element != NULL) {
							int target_id_2 = std::stoi(split_element);
							Target target_2 = targets[target_id_2];
							outfile << ";" << target_2.location.first << "," << target_2.location.second << "\n";
						} else {
							outfile << "\n";
						}
					}
				}
			}

			outfile << "\n";
		}

    	outfile.close();
    }
}

/*
 * Print vehicle-target assignments
 * @param model - Optimized Gurobi model
 * @param instance_name - Name of current problem instance
 */
void print_results(GRBModel& model, std::string instance_name) {
	// Print instance name
	printf("%s\n", &instance_name[0]);

	// Print minimum sum of tour lengths
	printf("Minimum Sum of Tour Lengths: %f\n", model.get(GRB_DoubleAttr_ObjVal));
	
	// Track assignments of targets to vehicles
	for (const auto& vehicle_entry : vehicles) {
		Vehicle vehicle = vehicle_entry.second;

		// Print vehicle information
		printf("Vehicle %d (%d,%d)\n", vehicle.id, vehicle.depot_location.first, vehicle.depot_location.second);

		if (vehicle_bids.find(vehicle.id) != vehicle_bids.end()) {
			// Track the number of tours for the current vehicle
			int tour_count = 1;

			for (auto& bid : vehicle_bids[vehicle.id]) {
				// Bid has been "accepted by auctioneer"
				if (bid.get(GRB_DoubleAttr_X) == 1) {
					std::string bid_var_name = bid.get(GRB_StringAttr_VarName);
					char* split_element = strtok(&bid_var_name[0], ",");
					split_element = strtok(NULL, ",");
					int target_id_1 = std::stoi(split_element);
					Target target_1 = targets[target_id_1];

					// Print tour count and first target location
					printf("\tTour %d: Target %d (%d,%d)", tour_count++, target_id_1, target_1.location.first, target_1.location.second);

					// If the tour encompasses two targets, print second target location
					split_element = strtok(NULL, ",");
					if (split_element != NULL) {
						int target_id_2 = std::stoi(split_element);
						Target target_2 = targets[target_id_2];
						printf(", Target %d (%d,%d)\n", target_id_2, target_2.location.first, target_2.location.second);
					} else {
						printf("\n");
					}
				}
			}
		}

		printf("\n");
	}
}

/*
 * Set up and solve CVRP problem with desired problem formulation
 * @param consider_only_best_bid - Flag indicating whether to consider only the best bid for each itemset or all of them
 * @param output_file_name - Name of file to output results to
 * @param instance_name - Name of current problem instance
 */
void cvrp(bool consider_only_best_bid, std::string output_file_name, std::string instance_name) {
	try {
		// Initialize environment and suppress output
		GRBEnv* environment = new GRBEnv();
		environment -> set(GRB_IntParam_OutputFlag, 0);

		// Initialize model and objective function
		GRBModel model = GRBModel(*environment);
		GRBLinExpr* objFunction = new GRBLinExpr();

		// Create bids
		create_singleton_bids(consider_only_best_bid, model, objFunction);
		create_pair_bids(consider_only_best_bid, model, objFunction);

	    // Goal is to minimize objective function
	    model.setObjective(*objFunction, GRB_MINIMIZE);

	    // Create constraints
	    create_constraints(model);

	    // Optimize objective
		model.optimize();

		// Print results
		// print_results(model, instance_name);

		// Output results to file
		append_results_to_file(output_file_name, model, instance_name);
	} catch (GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    } catch (...) {
        std::cout << "Exception during optimization" << std::endl;
    }
}

/*
 * Reset problem state
 */
void reset_state() {
	targets.clear();
	vehicles.clear();
	target_ids.clear();
	vehicle_ids.clear();
	bids_for_target.clear();
	vehicle_bids.clear();
}

/*
 * Generate problem instance from input strings
 * @param vehicle_locations_line - String specifying vehicle depot coordinate locations
 * @param target_locations_line - String specifying target coordinate locations
 * @param weights_line - String specifying target weights
 */
void generate_instance(std::string vehicle_locations_line, std::string target_locations_line, std::string weights_line) {
	// Initialize vectors to store instance data
	std::vector<std::pair<int, int>> vehicle_locations;
	std::vector<std::pair<int, int>> target_locations;
	std::vector<int> weights;

	// Parse vehicle location data
	char *vehicle_locations_save_pointer;
	char* vehicle_location = strtok_r(&vehicle_locations_line[0], ";", &vehicle_locations_save_pointer);
	while (vehicle_location != NULL) {
		char *coordinates_save_pointer;
		char* x_coor = strtok_r(vehicle_location, ",", &coordinates_save_pointer);
		char* y_coor = strtok_r(NULL, ",", &coordinates_save_pointer);
		vehicle_locations.push_back(std::make_pair(std::stoi(x_coor), std::stoi(y_coor)));
		vehicle_location = strtok_r(NULL, ";", &vehicle_locations_save_pointer);
	}

	// Parse target location data
	char *target_locations_save_pointer;
	char* target_location = strtok_r(&target_locations_line[0], ";", &target_locations_save_pointer);
	while (target_location != NULL) {
		char *coordinates_save_pointer;
		char* x_coor = strtok_r(target_location, ",", &coordinates_save_pointer);
		char* y_coor = strtok_r(NULL, ",", &coordinates_save_pointer);
		target_locations.push_back(std::make_pair(std::stoi(x_coor), std::stoi(y_coor)));
		target_location = strtok_r(NULL, ";", &target_locations_save_pointer);
	}

	// Parse weight data
	char* weight = strtok(&weights_line[0], ",");
	while (weight != NULL) {
		weights.push_back(std::stoi(weight));
		weight = strtok(NULL, ",");
	}

	// Generate targets
	for (int i = 0; i < target_locations.size(); i++) {
		Target new_target = {};
		int x_coor = target_locations[i].first;
		int y_coor = target_locations[i].second;
		new_target.location = std::make_pair(x_coor, y_coor);
		new_target.weight = weights[i];
		new_target.id = i;
		targets.insert(std::make_pair(new_target.id, new_target));
		target_ids.push_back(new_target.id);
	}

	// Generate vehicles
	for (int i = 0; i < vehicle_locations.size(); i++) {
		Vehicle new_vehicle = {};
		int x_coor = vehicle_locations[i].first;
		int y_coor = vehicle_locations[i].second;
		new_vehicle.depot_location = std::make_pair(x_coor, y_coor);






		// INVESTIGATE VEHICLE CAPACITITES!!!
		new_vehicle.capacity = 400;






		
		new_vehicle.id = i;
		vehicles.insert(std::make_pair(new_vehicle.id, new_vehicle));
		vehicle_ids.push_back(new_vehicle.id);
	}
}

/*
 * Read CVRP problem instances from file and solve each with desired problem formulation
 * @param consider_only_best_bid - Flag indicating whether to consider only the best bid for each itemset or all of them
 * @param input_file_name - Name of input data file
 * @param output_file_name - Name of file to output results to
 */
void dataset_cvrp(bool consider_only_best_bid, std::string input_file_name, std::string output_file_name) {
	// Create input stream for input dataset file
    std::ifstream infile(input_file_name);

    if (infile.is_open()) {
    	// Initialize variables for each problem instance
    	std::string dataset_name_line;
    	std::string vehicle_locations_line;
    	std::string target_locations_line;
    	std::string weights_line;

        // Read datasets iteratively
        while (std::getline(infile, dataset_name_line)) {
        	std::getline(infile, vehicle_locations_line);
        	std::getline(infile, target_locations_line);
        	std::getline(infile, weights_line);

        	// Remove labels
        	vehicle_locations_line = vehicle_locations_line.substr(vehicle_locations_line.find_first_of(":") + 1);
        	target_locations_line = target_locations_line.substr(target_locations_line.find_first_of(":") + 1);
        	weights_line = weights_line.substr(weights_line.find_first_of("=") + 2);

			// Clear problem state
			reset_state();

        	// Generate specified problem instance
        	generate_instance(vehicle_locations_line, target_locations_line, weights_line);

        	// Solve problem instance and append results to file
        	cvrp(consider_only_best_bid, output_file_name, dataset_name_line);
        }

        infile.close();
    }
}
