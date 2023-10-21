#include <stack>
#include "search-strategies.h"
#include "memusage.h"

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
    return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
    constexpr u_char DEPTH = 0;
    constexpr u_int MEM_RESERVE = 50000000;

    std::vector<std::shared_ptr<SearchAction>> result;
    std::vector<std::shared_ptr<SearchAction>> vector_actions;
    std::stack<std::pair<std::shared_ptr<SearchState>, int>> state_stack;

    state_stack.emplace(std::make_shared<SearchState>(init_state), DEPTH);

    while (!state_stack.empty()) {
        if (getCurrentRSS() + MEM_RESERVE > this->mem_limit_)
            return {};

        auto node = state_stack.top();
        state_stack.pop();

        if (node.first->isFinal()) {
            std::vector<SearchAction> res;
            for (const auto &action_ptr: result) {
                res.emplace_back(*action_ptr);
            }
            return res;
        }

        if (node.second == this->depth_limit_) {
            if (state_stack.empty())
                continue;

            for (int i = 0; i <= node.second - state_stack.top().second; i++) {
                result.pop_back();
                vector_actions.pop_back();
            }
            if (!vector_actions.empty())
                result.push_back(vector_actions.back());
            continue;
        }

        auto actions = node.first->actions();
        for (auto action: actions) {
            state_stack.emplace(std::make_shared<SearchState>(action.execute(*node.first)), node.second + 1);
            vector_actions.emplace_back(std::make_shared<SearchAction>(action));
        }

        result.push_back(vector_actions.back());
    }
    return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
    return 0;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
    return {};
}
