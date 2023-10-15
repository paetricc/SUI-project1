#include <algorithm>
#include <queue>
#include <set>

#include "search-strategies.h"

// TODO kontrola pameti
std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
    // TODO podle discordu (dedObed puvodni stav nemuze byt konecny)
    // if (init_state.isFinal()) {
    //    return {};
    // }

    std::queue<SearchState> queue_open;
    std::vector<SearchAction> solution = {};
    std::set<SearchState> set_closed = {init_state};
    std::map<SearchState, std::pair<SearchState, SearchAction>> predecessors;

    // Push the first state into the queue_open.
    queue_open.push(init_state);

    while (!queue_open.empty()) {
        // Take the first node from the queue.
        SearchState working_state = queue_open.front();
        queue_open.pop();

        // Expand the node.
        std::vector<SearchAction> actions = working_state.actions();

        for (SearchAction action : actions) {
            SearchState action_state = action.execute(working_state);

            //std::cout << "Searching actions \n";

            // If the current state is a goal.
            if (action_state.isFinal()) {
                //std::cout << "Found final \n";

                solution.push_back(action);

                auto parent = predecessors.find(working_state);

                while (parent != predecessors.end()) {
                    //std::cout << "searching root\n";

                    // Add the previous action which ends in the current known state from the path.
                    solution.push_back(parent->second.second);
                    // Find the previous state from which the action begins.
                    parent = predecessors.find(parent->second.first);
                }

                // Reverse the pointers of the vector to get the solution in the right order.
                std::reverse(solution.begin(), solution.end());

                return solution;
            }

            // If the state is not in set.
            if (set_closed.find(action_state) == set_closed.end()) {
                // Add the node to the queue and to the set.
                set_closed.insert(action_state);
                queue_open.push(action_state);
                predecessors.insert(std::make_pair(action_state,std::make_pair(working_state, action)));
            }
        }
    }

    return solution;
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	return {};
}
