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

// Genetic algorithm hyperparameters
const int POPULATION_SIZE = 50;
const int MAX_GENERATIONS = 1000;
const double MUTATION_RATE = 0.02;

std::vector<std::vector<double>>										distance_matrix;
std::unordered_map<std::string, std::vector<std::pair<int, int>>>		dataset_targets;
std::unordered_map<std::string, std::vector<std::pair<int, int>>>		dataset_vehicles;

/*
 * Read in target and vehicle data from files
 */
void read_target_vehicle_data(std::string target_data_file_name, std::string vehicle_data_file_name) {
	// Create input stream for target and vehicle dataset files
    std::ifstream target_infile(target_data_file_name);
    std::ifstream vehicle_infile(vehicle_data_file_name);

    // Read all target data
    if (target_infile.is_open()) {
    	std::string dataset_name_line;
    	
    	// Read datasets iteratively
        while (std::getline(target_infile, dataset_name_line)) {
        	std::string line;
        	std::getline(target_infile, line);
        	std::vector<std::pair<int, int>> target_coors;

        	// Individual targets
        	while (line.length() > 0) {
        		int target_id;
        		int x_coor;
        		int y_coor;
        		std::istringstream line_stream(line);
	            line_stream >> target_id;
	            line_stream >> x_coor;
	            line_stream >> y_coor;

	            target_coors.push_back(std::make_pair(x_coor, y_coor));
	            std::getline(target_infile, line);
        	}

        	dataset_targets.insert(make_pair(dataset_name_line, target_coors));
        }

        target_infile.close();
    }

    // Read all vehicle data
    if (vehicle_infile.is_open()) {
    	std::string dataset_name_line;
    	
    	// Read datasets iteratively
        while (std::getline(vehicle_infile, dataset_name_line)) {
        	std::string line;
        	std::getline(vehicle_infile, line);
        	std::vector<std::pair<int, int>> vehicle_coors;

        	// Individual vehicles
        	while (line.length() > 0) {
        		int vehicle_id;
        		int x_coor;
        		int y_coor;
        		std::istringstream line_stream(line);
	            line_stream >> vehicle_id;
	            line_stream >> x_coor;
	            line_stream >> y_coor;

	            vehicle_coors.push_back(std::make_pair(x_coor, y_coor));
	            std::getline(vehicle_infile, line);
        	}

        	dataset_vehicles.insert(make_pair(dataset_name_line, vehicle_coors));
        }

        vehicle_infile.close();
    }
}

/*
 * Generates random permutation of vehicle and targets
 */
std::vector<int> generate_random_permutation(int size) {
    std::vector<int> permutation(size);

    for (int i = 0; i < size; ++i) {
        permutation[i] = i;
    }

    std::random_shuffle(permutation.begin(), permutation.end());
    return permutation;
}

/*
 * Calculated the total distance of a route
 */
double calculate_route_distance(const std::vector<int>& route) {
    double distance = 0.0;

    for (int i = 0; i < route.size() - 1; ++i) {
        distance += distance_matrix[route[i]][route[i + 1]];
    }

    // Account for the return leg to the starting location
    distance += distance_matrix[route.back()][route.front()];
    return distance;
}

/*
 * Perform crossover (OX method)
 */
std::vector<int> crossover(const std::vector<int>& parent_1, const std::vector<int>& parent_2) {
    int size = parent_1.size();
    int start = rand() % size;
    int end = rand() % size;

    // Ensure that start <= end
    if (start > end) {
        std::swap(start, end);
    }

    // From start to end, child duplicates parent 1
    std::vector<int> child(size, -1);
    for (int i = start; i <= end; ++i) {
        child[i] = parent_1[i];
    }

    // The remaining indices in child are filled sequentially from parent 2
    int parent_2_index = 0;
    for (int i = 0; i < size; ++i) {
        if (child[i] == -1) {
            while (std::find(child.begin(), child.end(), parent_2[parent_2_index]) != child.end()) {
                ++parent_2_index;
            }

            child[i] = parent_2[parent_2_index++];
        }
    }

    return child;
}

/*
 * Perform mutation (swap method)
 */
void mutate(std::vector<int>& route) {
    int size = route.size();
    int pos1 = rand() % size;
    int pos2 = rand() % size;
    std::swap(route[pos1], route[pos2]);
}

/*
 * Genetic algorithm for vehicle routing
 */
double genetic_algorithm() {
    // Initialization
    std::vector<std::vector<int>> population;
    for (int i = 0; i < POPULATION_SIZE; ++i) {
        population.push_back(generate_random_permutation(distance_matrix.size()));
    }

    // Iterate over generations
    for (int generation = 0; generation < MAX_GENERATIONS; ++generation) {
        // Evaluate fitness
        std::vector<std::pair<int, double>> fitness;
        for (int i = 0; i < POPULATION_SIZE; ++i) {
            fitness.push_back({i, calculate_route_distance(population[i])});
        }

        std::sort(fitness.begin(), fitness.end(), [](auto& a, auto& b) {
            return a.second < b.second;
        });

        // Select parents for crossover
        int elite_size = 0.1 * POPULATION_SIZE;
        std::vector<std::vector<int>> new_population;
        for (int i = 0; i < elite_size; ++i) {
            new_population.push_back(population[fitness[i].first]);
        }

        for (int i = elite_size; i < POPULATION_SIZE; ++i) {
            int parent_1_index = rand() % (POPULATION_SIZE / 2);
            int parent_2_index = rand() % (POPULATION_SIZE / 2);
            std::vector<int> child = crossover(population[parent_1_index], population[parent_2_index]);
            
            if ((rand() % 100) < MUTATION_RATE * 100) {
                mutate(child);
            }

            new_population.push_back(child);
        }

        // Replace old population with new population
        population = new_population;
    }

    // Find the best route from the final population
    double min_distance = calculate_route_distance(population[0]);
    return min_distance;
}

/*
 * Generate distance matrix and invoke genetic algorithm
 */
double tsp(std::string dataset_name_line, int curr_vehicle_id, std::vector<int> target_assignments) {
	// Fetch coordinates
	std::vector<std::pair<int, int>> coordinates;
	coordinates.push_back(dataset_vehicles[dataset_name_line][curr_vehicle_id]);
	for (const auto& target_id : target_assignments) {
		coordinates.push_back(dataset_targets[dataset_name_line][target_id]);
	}

	// Generate distance matrix
	distance_matrix.clear();
	for (const auto& coor_1 : coordinates) {
		std::vector<double> distance_row;

        for (const auto& coor_2 : coordinates) {
            double distance = euclidean_distance(coor_1, coor_2);
            distance_row.push_back(distance);
        }

        distance_matrix.push_back(distance_row);
	}

	// Invoke genetic algorithm
	return genetic_algorithm();
}

/*
 * Calculate the total distance required for a given dataset and output to file
 */
void calculate_and_store_total_distance(std::string results_file_name, std::string dataset_name_line, std::unordered_map<int, std::vector<int>> vehicle_target_assignments) {
	// Create results output file stream
	std::ofstream outfile;
	outfile.open(results_file_name, std::ios_base::app);

	if (outfile.is_open()) {
		outfile << dataset_name_line << std::endl;
		double total_distance = 0;

		// Calculate distance travelled per vehicle
		for (int curr_vehicle_id = 0; curr_vehicle_id < dataset_vehicles[dataset_name_line].size(); curr_vehicle_id++) {
			double curr_vehicle_distance = 0;

			// Calculate distance via travelling salesman problem
			if (vehicle_target_assignments.find(curr_vehicle_id) != vehicle_target_assignments.end()) {
				curr_vehicle_distance = tsp(dataset_name_line, curr_vehicle_id, vehicle_target_assignments[curr_vehicle_id]);
			}

			total_distance += curr_vehicle_distance;
			outfile << curr_vehicle_id << ", " << curr_vehicle_distance << std::endl;
		}

		// Output total distance
		outfile << total_distance << std::endl << std::endl;
		outfile.close();
	}
}

/*
 * Given the assignments of target singletons and pairs to vehicles,
 * determine the (approximately) optimal routes for vehicles and
 * calculate the total distance required for each dataset
 */
void vehicle_tsp(std::string winners_file_name, std::string results_file_name, std::string target_data_file_name, std::string vehicle_data_file_name) {
	// Read target and vehicle data
	read_target_vehicle_data(target_data_file_name, vehicle_data_file_name);

	// Create input and output file streams
    std::ifstream infile(winners_file_name);

    if (infile.is_open()) {
    	std::string dataset_name_line;
    	
    	// Read auctions iteratively
        while (std::getline(infile, dataset_name_line)) {
        	std::string line;
        	std::getline(infile, line);
        	std::unordered_map<int, std::vector<int>> vehicle_target_assignments;

        	// Handle bid
        	while (line.length() > 0) {
        		// Read in vehicle ID
        		int vehicle_id;
        		std::istringstream line_stream(line);
	            line_stream >> vehicle_id;

	            if (vehicle_target_assignments.find(vehicle_id) == vehicle_target_assignments.end()) {
	            	std::vector<int> targets;
	            	vehicle_target_assignments.insert(make_pair(vehicle_id, targets));
	            }

	            // Read bid's goods
	            int good;
	            std::vector<int> bid_goods;
	            while (line_stream >> good) {
	            	vehicle_target_assignments[vehicle_id].push_back(good);
	            }

        		std::getline(infile, line);
        	}

        	// Determine total distance for dataset and store results
	        calculate_and_store_total_distance(results_file_name, dataset_name_line, vehicle_target_assignments);
        }

        infile.close();
    }
}
