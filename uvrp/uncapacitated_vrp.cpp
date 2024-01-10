#include <cstdio>
#include "uvrp_to_ca.h"
#include "winner_determination.h"
#include "vehicle_tsp.h"

/*
 * Run Commands:
 * g++ -std=c++11 -m64 -g uncapacitated_vrp.cpp -o uncapacitated_vrp -Iinclude/ -Llib -lgurobi_c++ -lgurobi95 -lm
 * ./uncapacitated_vrp
 */

int main(int argc, char *argv[]) {
    remove("target_data.txt");
    remove("vehicle_data.txt");
    remove("auction_instances.txt");
    remove("winners.txt");
    remove("results.txt");

    // Convert UVRP instances to CA instances and output results to files
    // std::string uvrp_file_name = "dummy_instances.txt";
    std::string uvrp_file_name = "CVRP_10vehicles_100targets.txt";
    std::string target_data_file_name = "target_data.txt";
    std::string vehicle_data_file_name = "vehicle_data.txt";
    std::string auctions_file_name = "auction_instances.txt";
    uvrp_to_ca(uvrp_file_name, target_data_file_name, vehicle_data_file_name, auctions_file_name);

    // Solve the WDP on CA instances
    std::string winners_file_name = "winners.txt";
    winner_determination(auctions_file_name, winners_file_name);

    // Determine total distance travelled for each instance
    std::string results_file_name = "results.txt";
    vehicle_tsp(winners_file_name, results_file_name, target_data_file_name, vehicle_data_file_name);
}