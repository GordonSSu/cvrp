#pragma once
#include <cmath>
#include <fstream>
#include <iostream>
#include <stdio.h>
#include <string>
#include <string.h>
#include <vector>

struct Target {
	std::pair<int, int> location;
	int id;
};

struct Vehicle {
	std::pair<int, int> depot_location;
	int id;
};

int 						delivery_reward;
std::vector<Target>			targets;
std::vector<Vehicle>		vehicles;
std::vector<int>			target_ids;
std::vector<int>			vehicle_ids;

/*
 * Reset instance state
 */
void reset_uvrp_state() {
	targets.clear();
	vehicles.clear();
	target_ids.clear();
	vehicle_ids.clear();
}

/*
 * Return Euclidean distance between two points
 */
double euclidean_distance(std::pair<int, int> location1, std::pair<int, int> location2) {
	int x_dist = location1.first - location2.first;
	int y_dist = location1.second - location2.second;
	return sqrt(x_dist * x_dist + y_dist * y_dist);
}

/*
 * Store vehicle and target data and write to files
 */
void handle_targets_vehicles(std::string dataset_name_line, std::string target_locations_line, std::string vehicle_locations_line, std::string target_data_file_name, std::string vehicle_data_file_name) {
	// Create output file streams
    std::ofstream target_outfile;
    std::ofstream vehicle_outfile;
    target_outfile.open(target_data_file_name, std::ios_base::app);
    vehicle_outfile.open(vehicle_data_file_name, std::ios_base::app);

    // Track coordinate extrema for "delivery reward" calculation
    int min_x_coor = INT_MAX;
    int min_y_coor = INT_MAX;
    int max_x_coor = INT_MIN;
    int max_y_coor = INT_MIN;

    // Parse target location data
    if (target_outfile.is_open()) {
		target_outfile << dataset_name_line << std::endl;
		int target_id = 0;
		char *target_locations_save_pointer;
		char* target_location = strtok_r(&target_locations_line[0], ";", &target_locations_save_pointer);

		while (target_location != NULL) {
			char* coordinates_save_pointer;
			int x_coor = std::stoi(strtok_r(target_location, ",", &coordinates_save_pointer));
			int y_coor = std::stoi(strtok_r(NULL, ",", &coordinates_save_pointer));
			target_location = strtok_r(NULL, ";", &target_locations_save_pointer);

			// Instantiate target
			Target new_target = {};
			new_target.location = std::make_pair(x_coor, y_coor);
			new_target.id = target_id++;
			targets.push_back(new_target);
			target_ids.push_back(new_target.id);

			// Update coordinate extrema
			min_x_coor = std::min(min_x_coor, x_coor);
			min_y_coor = std::min(min_y_coor, y_coor);
    		max_x_coor = std::max(max_x_coor, x_coor);
    		max_y_coor = std::max(max_y_coor, y_coor);

			// Write target data to file
			target_outfile << new_target.id << " " << x_coor << " " << y_coor << std::endl;
		}

		target_outfile << std::endl;
		target_outfile.close();
	}

	// Parse vehicle location data
	if (vehicle_outfile.is_open()) {
		vehicle_outfile << dataset_name_line << std::endl;
		int vehicle_id = 0;
		char* vehicle_locations_save_pointer;
		char* vehicle_location = strtok_r(&vehicle_locations_line[0], ";", &vehicle_locations_save_pointer);

		while (vehicle_location != NULL) {
			char* coordinates_save_pointer;
			int x_coor = std::stoi(strtok_r(vehicle_location, ",", &coordinates_save_pointer));
			int y_coor = std::stoi(strtok_r(NULL, ",", &coordinates_save_pointer));
			vehicle_location = strtok_r(NULL, ";", &vehicle_locations_save_pointer);

			// Instantiate vehicle
			Vehicle new_vehicle = {};
			new_vehicle.depot_location = std::make_pair(x_coor, y_coor);
			new_vehicle.id = vehicle_id++;
			vehicles.push_back(new_vehicle);
			vehicle_ids.push_back(new_vehicle.id);

			// Update coordinate extrema
			min_x_coor = std::min(min_x_coor, x_coor);
			min_y_coor = std::min(min_y_coor, y_coor);
    		max_x_coor = std::max(max_x_coor, x_coor);
    		max_y_coor = std::max(max_y_coor, y_coor);

			// Write vehicle data to file
			vehicle_outfile << new_vehicle.id << " " << x_coor << " " << y_coor << std::endl;
		}

		vehicle_outfile << std::endl;
		vehicle_outfile.close();
	}

	// Calculate vehicle reward
	delivery_reward = 2 * euclidean_distance(std::make_pair(min_x_coor, min_y_coor), std::make_pair(max_x_coor, max_y_coor));
}

/*
 * For each target singleton and pair, considering the best bid over all vehicles
 */
void create_bids(std::string dataset_name_line, std::string auctions_file_name) {
	// Create output file stream
    std::ofstream auctions_outfile;
    auctions_outfile.open(auctions_file_name, std::ios_base::app);

    // Create bids
    if (auctions_outfile.is_open()) {
    	// Calculate number of goods and bids and write to file
    	int num_goods = targets.size();
    	int num_bids = num_goods + 0.5 * num_goods * (num_goods - 1);
    	auctions_outfile << dataset_name_line << std::endl;
    	auctions_outfile << num_goods << " " << num_bids << std::endl;

		// Iterate over all targets (for singleton bids)
		for (const auto& target : targets) {
			int closest_vehicle_id = -1;
			double shortest_tour = -1;

			// Iterate over all vehicles
			for (const auto& vehicle : vehicles) {
				double tour_length = 2.0 * euclidean_distance(target.location, vehicle.depot_location);

				// Update closest vehicle
				if (closest_vehicle_id == -1 || tour_length < shortest_tour) {
					closest_vehicle_id = vehicle.id;
					shortest_tour = tour_length;
				}
			}

			// Write best singleton bid to file
			auctions_outfile << closest_vehicle_id << " " << delivery_reward - shortest_tour << " " << target.id << std::endl;
		}

		// Iterate over all target pairs (for pair bids)
		for (const auto& target_1 : targets) {
			for (const auto& target_2 : targets) {
				if (target_1.id > target_2.id) {
					int closest_vehicle_id = -1;
					double shortest_tour = -1;

					// Iterate over all vehicles
					for (const auto& vehicle : vehicles) {
						double tour_length = euclidean_distance(target_1.location, target_2.location)
									+ euclidean_distance(target_1.location, vehicle.depot_location)
									+ euclidean_distance(target_2.location, vehicle.depot_location);

						// Update closest vehicle
						if (closest_vehicle_id == -1 || tour_length < shortest_tour) {
							closest_vehicle_id = vehicle.id;
							shortest_tour = tour_length;
						}
					}

					// Write best pair bid to file
					auctions_outfile << closest_vehicle_id << " " << 2 * delivery_reward - shortest_tour << " " << target_1.id << " " << target_2.id << std::endl;
				}
			}
		}

		auctions_outfile << std::endl;
		auctions_outfile.close();
	}
}

/*
 * Convert UVRP instances to CA instances
 */
void uvrp_to_ca(std::string uvrp_file_name, std::string target_data_file_name, std::string vehicle_data_file_name, std::string auctions_file_name) {
	// Create input stream for input dataset file
    std::ifstream infile(uvrp_file_name);

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
			reset_uvrp_state();

			// Read, store, and write target and vehicle data to files
			handle_targets_vehicles(dataset_name_line, target_locations_line, vehicle_locations_line, target_data_file_name, vehicle_data_file_name);

			// Create CA and output to file
			create_bids(dataset_name_line, auctions_file_name);
        }

        infile.close();
    }
}
