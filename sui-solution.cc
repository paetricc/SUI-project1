#include <stack>
#include <vector>
#include <algorithm>

#include "search-strategies.h"
#include "memusage.h"

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
    std::vector<SearchAction> result;
    std::vector<SearchAction> vector_actions;
    std::stack<std::pair<SearchState, int>> state_stack;
    int depth = 0;
    state_stack.emplace(init_state, depth);

    while(!state_stack.empty()) {
        if(getCurrentRSS() + 50000000 > this->mem_limit_)
            return {};

        auto node = state_stack.top();
        state_stack.pop();

        if(node.first.isFinal()) {
            return result;
        }

        if(node.second == this->depth_limit_) {
            if(state_stack.empty())
                continue;
            size_t result_size = result.size();

            for(size_t i = state_stack.top().second; i <= result_size; i++){
                if(!result.empty() && !vector_actions.empty()) {
                    result.pop_back();
                    vector_actions.pop_back();
                }
            }
            if(!vector_actions.empty())
                result.emplace_back(vector_actions.back());
            continue;
        }

        auto actions = node.first.actions();
        for(auto action : actions) {
            state_stack.emplace(action.execute(node.first), node.second+1);
            vector_actions.emplace_back(action);
        }

        result.emplace_back(vector_actions.back());
    }
	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	return {};
}
