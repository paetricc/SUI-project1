#include <queue>
#include <set>

#include "search-strategies.h"

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
    if (init_state.isFinal()) {
        return {};
    }

    // TODO jestli misto queue nema byt seznam, abych mohl prochazet
    // zda neni ten stav uz ve fronte
    std::queue<SearchState> queue_open;
    std::set<SearchState> set_closed = {};
    std::vector<SearchAction> solution = {};

    // Push the first state into the queue_open.
    queue_open.push(init_state);

    while (!queue_open.empty()) {
        // Take the first node from the queue.
        SearchState working_state(queue_open.front());
        queue_open.pop();

        set_closed.insert(working_state);

        // Expand the node.
        std::vector<SearchAction> actions = working_state.actions();

        for (SearchAction action : actions) {
            SearchState action_state(action.execute(working_state));

            // If the state is not in set.
            // TODO jestli testovat taky s open (frontier)
            // TODO pridavat nekde akce do solution
            if (set_closed.find(action_state) == set_closed.end()) {
                if (action_state.isFinal()) {
                    return solution;
                }

                queue_open.push(action_state);
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
