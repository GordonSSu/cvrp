#include "cvrp.h"

int main(int argc, char *argv[]) {
    // Check command line arguments
    if (argc != 3) {
        std::cerr << "Incorrect argument(s)." << std::endl;
        std::cout << "Usage: ./cvrp [num_targets] [num_vehicles]" << std::endl;
        return 1;
    }

    // Read in desired numbers of vehicles and targets
    int num_targets = atoi(argv[1]);
    int num_vehicles = atoi(argv[2]);

    std::cout << num_targets << std::endl;

    // Generate a problem instance with random locations and capacities
    generate_random_instance(num_targets, num_vehicles);

    // Assign targets to vehicles
    solve();
}