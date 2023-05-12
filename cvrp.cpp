#include "cvrp.h"

/*
 * Run Commands:
 * g++ -std=c++11 -m64 -g cvrp.cpp -o cvrp -Iinclude/ -Llib -lgurobi_c++ -lgurobi95 -lm
 * ./cvrp num_targets num_vehicles consider_only_best_bid
 */

int main(int argc, char *argv[]) {
    // Check command line arguments
    if (argc != 4) {
        std::cerr << "Incorrect argument(s)." << std::endl;
        std::cout << "Usage: ./cvrp [num_targets] [num_vehicles] [consider_only_best_bid_flag]" << std::endl;
        return 1;
    }

    // Read in desired numbers of vehicles and targets
    int num_targets = atoi(argv[1]);
    int num_vehicles = atoi(argv[2]);
    bool consider_only_best_bid = atoi(argv[3]);

    // Generate a problem instance with random locations and capacities
    generate_random_instance(num_targets, num_vehicles);

    // Create CVRP and find solution
    cvrp(consider_only_best_bid);
}