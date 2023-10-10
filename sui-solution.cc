#include <queue>
#include <set>

#include "search-strategies.h"

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
    if (init_state.isFinal()) {
        return {};
    }

    std::queue<SearchState> queue_open;
    std::set<SearchState> set_closed = {init_state};
    std::vector<SearchAction> solution = {};

    // Push the first state into the queue_open.
    queue_open.push(init_state);

    while (!queue_open.empty()) {
        // Take the first node from the queue.
        SearchState working_state(queue_open.front());
        queue_open.pop();

        // Expand the node.
        std::vector<SearchAction> actions = working_state.actions();

        for (SearchAction action : actions) {
            // TODO: Fix invalid action move.
            SearchState action_state(action.execute(working_state));

            // If the current state is a goal.
            if (action_state.isFinal()) {
                return solution;
            }

            // If the state is not in set.
            if (set_closed.find(action_state) == set_closed.end()) {
                // Add the node to the queue and to the set.
                set_closed.insert(action_state);
                queue_open.push(action_state);

                // Add the node to the solution.
                solution.push_back(action);
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
