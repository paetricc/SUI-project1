#include <queue>
#include <set>
#include <algorithm>

#include "search-strategies.h"

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}
/*
double compute_heuristic(const SearchState &state, const AStarHeuristicItf &heuristic) {
    return heuristic.distanceLowerBound(state.state_);
}*/

typedef std::pair<SearchState, double> QUEUE_PAIR;

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
    std::set<QUEUE_PAIR> queue_open;
    std::vector<SearchAction> solution = {};
    std::map<SearchState, int> map_costs_g = {std::make_pair(init_state, 0)}; // Used as a "closed" lookup table.
    std::map<SearchState, std::pair<SearchState, SearchAction>> predecessors;

    // TODO asi jeste pouzit seznam closed: https://iopscience.iop.org/article/10.1088/1742-6596/1898/1/012047/pdf

    double cost_f;

    // TODO jestli osetrovat pointery
    if (this->heuristic_) {
        // f(n) = g(n) + h(n) = 0 + h(n) = h(n)
        cost_f = compute_heuristic(init_state, *(this->heuristic_));
        queue_open.insert(std::make_pair(init_state, cost_f));
    } else {
        // TODO snad by i nemelo nastat
    }

    while(!queue_open.empty()) {
        // Take the first node from the priority queue with the lowest value.
        // https://stackoverflow.com/a/53619673
        std::pair<SearchState, double> queue_top = *std::min_element(queue_open.begin(), queue_open.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.second < rhs.second;
        });

        SearchState working_state = queue_top.first;

        if (working_state.isFinal()) {
            // TODO Poznamka: skoro stejnej postup jako u BFS. treba by se dalo hodit do funkce
            auto parent = predecessors.find(working_state);

            while (parent != predecessors.end()) {
                // Add the previous action which ends in the current known state from the path.
                solution.push_back(parent->second.second);
                // Find the previous state from which the action begins.
                parent = predecessors.find(parent->second.first);
            }

            // Reverse the pointers of the vector to get the solution in the right order.
            std::reverse(solution.begin(), solution.end());

            return solution;
        }

        queue_open.erase(queue_open.begin());

        // Expand the node.
        std::vector<SearchAction> actions = working_state.actions();

        for (SearchAction action : actions) {
            SearchState action_state = action.execute(working_state);

            // Pro parenta ziskam hodnotu g(n) a vypoctu novou hodnotu pro potomky - g(n) + 1
            auto state_cost_g = map_costs_g.find(working_state);

            int cost_g = 0;

            if (state_cost_g != map_costs_g.end()) {
                cost_g = state_cost_g->second;
            } else {
                // TODO - nemelo by pravdepodobne nastat
            }

            double tentative_cost_g = cost_g + 1;

            // Tedka mam hodnotu g(n) pro potomky aktualniho stavu - tentative cost g.

            // Zjistim jestli je child v mape. - Pokud ne, vlozim ho tam s aktualni vypocitanou hodnotou g. Pokud ano,
            // vlozim pouze tehdy, pokud aktualni hodnote je mensi nez ulozena - nahradim novou hodnotou

            // Pro childa
            state_cost_g = map_costs_g.find(action_state);

            if (state_cost_g != map_costs_g.end()) {
                // Child se nachazi jiz v mape. Ziskam jeho hodnotu g z mapy.
                double child_cost_g = state_cost_g->second;

                if (tentative_cost_g < child_cost_g) {
                    // Nahrazuju hodnotu novou hodnotou.
                    map_costs_g[action_state] = tentative_cost_g;

                    // Nahradim i hodnotu v open.

                    // 1. ziskam hodnotu heuristiky
                    // f(n) = g(n) + h(n)
                    cost_f = tentative_cost_g + compute_heuristic(action_state, *(this->heuristic_));
                    // 2. vlozeni do fronty open.
                    auto state = std::find_if(queue_open.begin(), queue_open.end(),
                                              [&action_state](const QUEUE_PAIR & element){ return !(element.first < action_state) && !(action_state < element.first);} );

                    if (state != queue_open.end()) {
                        queue_open.erase(std::make_pair(state->first, state->second));
                        queue_open.insert(std::make_pair(action_state, cost_f));
                    }

                    std::map<SearchState, std::pair<SearchState, SearchAction>>::iterator it = predecessors.find(action_state);
                    if (it != predecessors.end()) {
                        it->second = std::make_pair(working_state, action);
                    } else {
                        // TODO nemelo by nastat
                    }
                }
            } else {
                // Child neni v pame, vidime ho poprve - vkladam.
                map_costs_g.emplace(action_state, tentative_cost_g);

                // Neni ani v open, vlozim ho tam.

                // 1. ziskam hodnotu heuristiky
                // f(n) = g(n) + h(n)
                cost_f = tentative_cost_g + compute_heuristic(action_state, *(this->heuristic_));
                // 2. vlozeni do fronty open.
                queue_open.insert(std::make_pair(action_state, cost_f));
                predecessors.insert(std::make_pair(action_state,std::make_pair(working_state, action)));
            }
        }
    }


	return {};
}
