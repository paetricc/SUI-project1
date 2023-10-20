#include <stack>
#include <vector>
#include <algorithm>

#include "search-strategies.h"
#include "memusage.h"

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
    const u_char DEPTH = 0;
    const u_int MEM_RESERVE = 50000000;

    std::vector<SearchAction> vector_result;
    std::vector<SearchAction> vector_actions;
    std::stack<std::pair<SearchState, int>> state_stack;

    state_stack.emplace(init_state, DEPTH);

    while(!state_stack.empty()) {
        // Memory check
        if(getCurrentRSS() + MEM_RESERVE > this->mem_limit_)
            return {};
        // Get new state to processing
        auto node = state_stack.top();
        state_stack.pop();
        // Check if actual node result of game
        if(node.first.isFinal())
            return vector_result;
        // Depth-limit occurred
        if(node.second == this->depth_limit_) {
            if(state_stack.empty())
                continue;
            // Depth changes so pop from vector_result and vector_action until
            // vector_result size not equal to new state depth from top of state_stack
            size_t result_size = vector_result.size();
            for(size_t i = state_stack.top().second; i <= result_size; i++){
                if(!vector_result.empty() && !vector_actions.empty()) {
                    vector_result.pop_back();
                    vector_actions.pop_back();
                }
            }
            // New possible result action to vector_result
            if(!vector_actions.empty())
                vector_result.emplace_back(vector_actions.back());

            continue;
        }
        // Expand node and save actions and states to vector_actions and state_stack
        for(auto action : node.first.actions()) {
            state_stack.emplace(action.execute(node.first), node.second + 1);
            vector_actions.emplace_back(action);
        }
        // New possible result action to vector_result
        vector_result.emplace_back(vector_actions.back());
    }
	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	return {};
}
