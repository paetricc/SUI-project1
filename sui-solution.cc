#include <algorithm>
#include <queue>
#include <set>
#include <utility>

#include "memusage.h"
#include "search-strategies.h"

constexpr u_int MEM_RESERVE = 50000000;

template<typename T>
class TreeNode
{
public:
    TreeNode()
    = default;

    TreeNode(std::shared_ptr<T>  _value, std::shared_ptr<TreeNode<T>> _parent)
        : Value(std::move(_value)), Parent(std::move(_parent))
    {
    }

    ~TreeNode()
    = default;

    std::shared_ptr<T> Value;
    std::shared_ptr<TreeNode<T>> Parent;
};

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
    std::queue<std::pair<SearchState, std::shared_ptr<TreeNode<SearchAction>>>> queue_open;
    std::vector<SearchAction> solution = {};
    std::set<SearchState> set_closed = {init_state};

    std::shared_ptr<TreeNode<SearchAction>> root_node =
            std::make_shared<TreeNode<SearchAction>>(nullptr, nullptr);

    // Push the first state into the queue_open.
    queue_open.emplace(init_state, root_node);

    while (!queue_open.empty()) {
        // Memory check.
        if (getCurrentRSS() + MEM_RESERVE > this->mem_limit_) {
            return {};
        }

        // Take the first node from the queue.
        SearchState working_state = queue_open.front().first;
        std::shared_ptr<TreeNode<SearchAction>> working_action = queue_open.front().second;
        queue_open.pop();

        // Expand the node.
        std::vector<SearchAction> actions = working_state.actions();

        for (SearchAction action : actions) {
            SearchState action_state = action.execute(working_state);

            // If the current state is a goal.
            if (action_state.isFinal()) {
                solution.push_back(action);
                std::shared_ptr<TreeNode<SearchAction>> action_in_tree = working_action;

                while (action_in_tree->Parent != nullptr) {
                    solution.push_back(*(action_in_tree->Value));
                    action_in_tree = action_in_tree->Parent;
                }

                // Reverse the pointers of the vector to get the solution in the right order.
                std::reverse(solution.begin(), solution.end());

                return solution;
            }

            // If the state is not in set.
            if (set_closed.find(action_state) == set_closed.end()) {
                // Add the node to the queue and to the set.
                set_closed.insert(action_state);

                std::shared_ptr<SearchAction> tree_action = std::make_shared<SearchAction>(action);
                std::shared_ptr<TreeNode<SearchAction>> tree_node =
                        std::make_shared<TreeNode<SearchAction>>(tree_action, working_action);

                queue_open.emplace(action_state, tree_node);
            }
        }
    }

    // Return an empty list if the solution was not found.
    return {};
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
