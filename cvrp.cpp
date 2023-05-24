#include "cvrp.h"

/*
 * Run Commands:
 * g++ -std=c++11 -m64 -g cvrp.cpp -o cvrp -Iinclude/ -Llib -lgurobi_c++ -lgurobi95 -lm
 * ./cvrp grid_size_x grid_size_y num_targets num_vehicles consider_only_best_bid
 */

int main(int argc, char *argv[]) {
    /*
    // Check command line arguments
    if (argc != 6) {
        std::cerr << "Incorrect argument(s)." << std::endl;
        std::cout << "Usage: ./cvrp [grid_size_x] [grid_size_y] [num_targets] [num_vehicles] [consider_only_best_bid_flag]" << std::endl;
        return 1;
    }

    // Read in desired numbers of vehicles and targets
    int grid_size_x = atoi(argv[1]);
    int grid_size_y = atoi(argv[2]);
    int num_targets = atoi(argv[3]);
    int num_vehicles = atoi(argv[4]);
    bool consider_only_best_bid = atoi(argv[5]);

    // Generate a problem instance with random locations and capacities
    generate_random_instance(grid_size_x, grid_size_y, num_targets, num_vehicles);

    // Create CVRP and find solution
    cvrp(consider_only_best_bid, "results.txt");
    */

    // Read dataset, solve each cvrp, and output results to output file
    dataset_cvrp(true, "CVRP_commondatasets.txt", "best_bid_results.txt");   // Considering only best bids
    dataset_cvrp(false, "CVRP_commondatasets.txt", "all_bids_results.txt");   // Considering all bids
}