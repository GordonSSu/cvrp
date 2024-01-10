#pragma once
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <string>
#include <string.h>
#include <unordered_map>
#include <vector>
#include "gurobi_c++.h"

struct Bid {
    std::vector<int> bid_goods;
    int vehicle_id;
    int bidId;
    double value;
};

struct Edge {
    int v1;
    int v2;
};

int                     	num_goods;
int                     	num_original_bids;
int 						num_remaining_bids;
int                     	num_pruned0;
int                     	num_pruned1;
long long               	total_value;
long long               	excluded_bids_values = 0;
long long               	included_bids_values = 0;
std::vector<Bid>        	bids;
std::vector<Bid>        	bids_included_in_mwvc_by_kernalization;
std::vector<Bid>        	bids_excluded_from_mwvc_by_kernalization;
std::vector<Bid>        	bids_remaining_after_kernalization;
std::vector<Edge>       	edges;

/*
 * Resets auction state
 */
void reset_auction_state() {
    num_goods = 0;
    num_original_bids = 0;
    num_remaining_bids = 0;
    num_pruned0 = 0;
    num_pruned1 = 0;
    total_value = 0;
    excluded_bids_values = 0;
    included_bids_values = 0;

    bids.clear();
    bids_included_in_mwvc_by_kernalization.clear();
    bids_excluded_from_mwvc_by_kernalization.clear();
    bids_remaining_after_kernalization.clear();
    edges.clear();
}

/*
 * Returns whether two bids share a good,
 * assuming that the bid_goods vectors are sorted
 */
bool intersects(Bid& bid1, Bid& bid2) {
	auto b1_it = bid1.bid_goods.begin();
    auto b2_it = bid2.bid_goods.begin();

    // Loop: if shared good is found, return true (bid_goods is sorted)
    while (b1_it != bid1.bid_goods.end() && 
        b2_it != bid2.bid_goods.end()) {
        if (*b1_it < *b2_it) {
            ++b1_it;
        } else if (*b1_it > *b2_it) {
            ++b2_it;
        } else {
            return true;
        }
    }

    return false;    
}

/*
 * Build the auction's conflict graph,
 * given that the bids vector is populated
 */
void build_conflict_graph() {
    // Iterate over all pairs of bids
    for (int bidIndex1 = 0; bidIndex1 < (bids.size() - 1); bidIndex1++) {
        for (int bidIndex2 = bidIndex1 + 1; 
                bidIndex2 < bids.size(); bidIndex2++) {
            Bid& bid1 = bids[bidIndex1];
            Bid& bid2 = bids[bidIndex2];

            // Add edge if shared goods are found
            if (intersects(bid1, bid2)) {
                Edge newEdge = {bid1.bidId, bid2.bidId};
                edges.push_back(newEdge);
            }
        }
    }
}

/*
 * Refactor the conflict graph,
 * removing edges containing bids "pruned" by the kernalization
 */
void refactor_conflict_graph() {
    edges.clear();

    // Refactor edges
    for (int bidIndex1 = 0; bidIndex1 < (bids_remaining_after_kernalization.size() - 1); bidIndex1++) {
        for (int bidIndex2 = bidIndex1 + 1; 
                bidIndex2 < bids_remaining_after_kernalization.size(); bidIndex2++) {
            Bid& bid1 = bids_remaining_after_kernalization[bidIndex1];
            Bid& bid2 = bids_remaining_after_kernalization[bidIndex2];

            // Add edge if shared goods are found
            if (intersects(bid1, bid2)) {
                Edge newEdge = {bid1.bidId, bid2.bidId};
                edges.push_back(newEdge);
            }
        }
    }
}

/*
 * Kernalize conflict graph via a MWVC
 * (with LP relaxation to leverage the half-integrality property)
 * Return the number of bids remaining to search through
 * (to determine whether fastwvc is necessary)
 */
void kernalize() {
    try {
        // Create new environment and suppress output
        GRBEnv* env = new GRBEnv();
        env -> set(GRB_IntParam_OutputFlag, 0);

        // Create new model
        GRBModel model = GRBModel(*env);
        model.set(GRB_StringAttr_ModelName, "kernelization");

        std::vector<GRBVar> bidVars;

        // Create a decision variable for each bid
        for (int i = 0; i < num_original_bids; i++) {
        	GRBVar newVar = model.addVar(0.0, 1.0, bids[i].value, GRB_CONTINUOUS, "");
            bidVars.push_back(newVar);
        }

		// Create objective function
        GRBLinExpr* objFunction = new GRBLinExpr();
        for (int i = 0; i < num_original_bids; i++) {
            GRBLinExpr* term = new GRBLinExpr(bidVars[i], bids[i].value);
            *objFunction += *term;
        }

        // Minimize objective function
        model.setObjective(*objFunction, GRB_MINIMIZE);

        // Add edge constraints
        for (Edge& edge : edges) {
            model.addConstr(bidVars[edge.v1] + bidVars[edge.v2] >= 1.0f, "");
        }

        // Solve
        model.optimize();

        // Reconfigure bid vectors based on kernalization results
        for (int i = 0; i < num_original_bids; i++) {
            // By the half-integrality property, assignedValue must be in {0, 0.5, 1}
            double assignedValue = bidVars[i].get(GRB_DoubleAttr_X);

			// Bids confirmed to be excluded from the MWVC
            if (assignedValue == 0.0) {
                bids_excluded_from_mwvc_by_kernalization.push_back(bids[i]);
                excluded_bids_values += bids[i].value;
                num_pruned0++;
            }

            // Bids confirmed to be included in the MWVC
            else if (assignedValue == 1.0) {
            	bids_included_in_mwvc_by_kernalization.push_back(bids[i]);
                included_bids_values += bids[i].value;
                num_pruned1++;
            }

            // Ambiguous bids that still require search
            else {
                bids_remaining_after_kernalization.push_back(bids[i]);
            }
        }

        num_remaining_bids = bids_remaining_after_kernalization.size();

        // Bids pruned
        if (num_original_bids > num_remaining_bids) {
            // Reconfigure bids and edges in conflict graph
            total_value = total_value - excluded_bids_values - included_bids_values;
            refactor_conflict_graph();
        }
        
        // Output number of pruned bids
        std::cout << "Num pruned = 0: " << num_pruned0 << std::endl;
        std::cout << "Num pruned = 1: " << num_pruned1 << std::endl;
    } catch (GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    } catch (...) {
        std::cout << "Exception during optimization" << std::endl;
    }
}

/*
 * Formulate combinatorial auction as min weighted vertex cover problem
 * Solve by invoking Gurobi
 */
void gurobi_mwvc_solve(std::string dataset_name_line, std::string winners_file_name) {
    try {
        // Create new environment and suppress output
        GRBEnv* env = new GRBEnv();
        env -> set(GRB_IntParam_OutputFlag, 0);

        // Create new model
        GRBModel model = GRBModel(*env);
        model.set(GRB_StringAttr_ModelName, "gurobi_mwvc");

        std::vector<GRBVar> bidVars;

        // Map each bid ID to its index in the bidVars vector
        std::unordered_map<int, int> bid_id_to_index;
        std::unordered_map<int, Bid> bid_index_to_obj;

        // Create a decision variable for each bid
        for (int i = 0; i < num_remaining_bids; i++) {
            GRBVar newVar = model.addVar(0.0, 1.0, bids_remaining_after_kernalization[i].value, GRB_BINARY, "");
            bid_id_to_index.insert(std::make_pair(bids_remaining_after_kernalization[i].bidId, i));
            bid_index_to_obj.insert(std::make_pair(i, bids_remaining_after_kernalization[i]));
            bidVars.push_back(newVar);
        }

		// Create objective function
        GRBLinExpr* objFunction = new GRBLinExpr();
        for (int i = 0; i < num_remaining_bids; i++) {
            GRBLinExpr* term = new GRBLinExpr(bidVars[i], bids[i].value);
            *objFunction += *term;
        }

        // Set objective function to minimize
        model.setObjective(*objFunction, GRB_MINIMIZE);

		// Add edge constraints
        for (Edge& edge : edges) {
			model.addConstr(bidVars[bid_id_to_index[edge.v1]] + bidVars[bid_id_to_index[edge.v2]] >= 1.0f, "");
        }

        // Solve
        model.optimize();

		// std::cout << std::endl;
		// std::cout << bidVar.size() << std::endl;
		// std::cout << std::endl;

        // for (GRBVar& bidVar : bidVars) {
        // 	std::cout << bidVar.get(GRB_DoubleAttr_X) << std::endl;
        // }

        // for (int i = 0; i < bidVars.size(); i++) {
        // 	// GRBVar& bidVar = bidVars[i];
        // 	// std::cout << bidVar.get(GRB_DoubleAttr_X) << std::endl;
        // 	// Bid& bid_obj = bid_index_to_obj[i];
        // 	// for (int bid_good : bid_obj.bid_goods) {
        // 	// 	std::cout << bid_good << ", ";
        // 	// }
        // 	// std::cout << std::endl;

        // 	GRBVar& bidVar = bidVars[i];
        // 	if (bidVar.get(GRB_DoubleAttr_X) == 0.0) {
	       //  	Bid& bid_obj = bid_index_to_obj[i];
	       //  	for (int bid_good : bid_obj.bid_goods) {
	       //  		std::cout << bid_good << ", ";
	       //  	}
	       //  	std::cout << std::endl;
	       //  }
        // }

        // std::cout << std::endl;
        // std::cout << total_value + excluded_bids_values - static_cast<long long>(model.get(GRB_DoubleAttr_ObjVal)) << std::endl;

        std::ofstream winners_outfile;
        winners_outfile.open(winners_file_name, std::ios_base::app);

        // Write "winning" assignments to file
    	if (winners_outfile.is_open()) {
    		winners_outfile << dataset_name_line << std::endl;

    		// Iterate over bids
    		for (int i = 0; i < bidVars.size(); i++) {
	        	// Bid is excluded from MWVC and is therefore in MWIS
	        	if (bidVars[i].get(GRB_DoubleAttr_X) == 0.0) {
		        	Bid& bid_obj = bid_index_to_obj[i];

		        	winners_outfile << bid_obj.vehicle_id;

		        	for (int bid_good : bid_obj.bid_goods) {
		        		winners_outfile << " " << bid_good;
		        	}

		        	winners_outfile << std::endl;
		        }
	        }

	        // Write bids excluded from MWVC by kernelization
	        for (Bid bid_excluded_from_mwvc_by_kernalization: bids_excluded_from_mwvc_by_kernalization) {
	        	winners_outfile << bid_excluded_from_mwvc_by_kernalization.vehicle_id;

	        	for (int bid_good : bid_excluded_from_mwvc_by_kernalization.bid_goods) {
	        		winners_outfile << " " << bid_good;
	        	}

	        	winners_outfile << std::endl;
	        }

	        winners_outfile << std::endl;
    	}
    } catch (GRBException e) {
        std::cout << "Error code = " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
    } catch (...) {
        std::cout << "Exception during optimization" << std::endl;
    }
}

/*
 * Perform winner determination on all combinatorial auctions
 */
void winner_determination(std::string auctions_file_name, std::string winners_file_name) {
	// Create input stream for input auction file
    std::ifstream infile(auctions_file_name);

    if (infile.is_open()) {
    	std::string dataset_name_line;
    	
    	// Read auctions iteratively
        while (std::getline(infile, dataset_name_line)) {
        	// Clear problem state
	        reset_auction_state();

	        // Read number of goods and bids
	        infile >> num_goods >> num_original_bids;

        	// Consume newline character
	        std::string line;
	        std::getline(infile, line);

	        // Read all bids and populate bids vector
	        for (int bidNum = 0; bidNum < num_original_bids; bidNum++) {
	            std::getline(infile, line);
	            std::istringstream line_stream(line);
	            
	            Bid newBid = {};
	            newBid.bidId = bidNum;

	            // Read bid vehicle and value
	            int vehicle_id;
	            double bid_value;
	            line_stream >> vehicle_id;
	            line_stream >> bid_value;
	            newBid.vehicle_id = vehicle_id;
	            newBid.value = bid_value;
	            total_value += static_cast<long long>(bid_value);

	            // Read bid's goods
	            int good;
	            std::vector<int> bid_goods;
	            while (line_stream >> good) {
	            	bid_goods.push_back(good);
	            }
	            
	            std::sort(bid_goods.begin(), bid_goods.end());
	            newBid.bid_goods = bid_goods;
	            bids.push_back(newBid);
	        }

	        // Build the auction conflict graph
    		build_conflict_graph();

	        // Kernalize with the NT reduction
		    kernalize();

		    // Solve auction and output results
	        gurobi_mwvc_solve(dataset_name_line, winners_file_name);

	        // Consume newline character
	        std::getline(infile, line);
        }

        infile.close();
    }
}
