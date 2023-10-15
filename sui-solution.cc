#include <algorithm>
#include <queue>
#include <set>

#include "search-strategies.h"

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
    // TODO podle discordu (dedObed puvodni stav nemuze byt konecny)
    // if (init_state.isFinal()) {
    //    return {};
    // }

    std::queue<SearchState> queue_open;
    std::vector<SearchAction> solution = {};


    std::set<SearchState> set_closed = {init_state};
    std::map<SearchState, std::pair<SearchState, SearchAction>> states_relations;

    // Push the first state into the queue_open.
    queue_open.push(init_state);

    while (!queue_open.empty()) {
        // Take the first node from the queue.
        SearchState working_state = queue_open.front();
        queue_open.pop();

        // Expand the node.
        std::vector<SearchAction> actions = working_state.actions();

        for (SearchAction action : actions) {
            // TODO asi se da action state deklarovat jenom jednou a pak jen prirazovat
            SearchState action_state = action.execute(working_state);


            // If the current state is a goal.
            if (action_state.isFinal()) {
                std::cout << "Found final \n";

                solution.push_back(action);

                // TODO optimalizovat
                auto parent = states_relations.find(working_state);

                while (parent != states_relations.end()) {
                    std::cout << "searching root\n";

                    solution.push_back(parent->second.second);
                    parent = states_relations.find(parent->second.first);
                }

                std::reverse(solution.begin(), solution.end());

                return solution;
            }

            // If the state is not in set.
            if (set_closed.find(action_state) == set_closed.end()) {
                // Add the node to the queue and to the set.
                set_closed.insert(action_state);
                queue_open.push(action_state);
                // TODO asi nejak s pointerama a neukladat stavy dvakrat (spojit asi nejak s mnozinou closed)
                states_relations.insert(std::make_pair(action_state, std::make_pair(working_state, action)));
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
