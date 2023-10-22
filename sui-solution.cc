#include <algorithm>
#include <queue>
#include <set>
#include <utility>

#include "memusage.h"
#include "search-strategies.h"

// Memory reserve.
constexpr u_int MEM_RESERVE = 50000000;

/*
 * The implementation of an n-ary tree.
 *
 * The tree is implemented for saving trees of actions that are later used for a traceback
 * of the resulting path of actions.
 */
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

    // Push the first state into the queue_open.
    queue_open.emplace(init_state, nullptr);

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

                // Traceback the path of action using a tree of actions (the root node is
                // the first action from the rood SearchState that is a predecessor of
                // the current action).
                while (action_in_tree != nullptr) {
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

                // Create a tree node which stores the action that was used to get to the
                // action state.
                std::shared_ptr<SearchAction> tree_action = std::make_shared<SearchAction>(action);
                std::shared_ptr<TreeNode<SearchAction>> tree_node =
                        std::make_shared<TreeNode<SearchAction>>(tree_action, working_action);

                // Put the search state with the action which leads to this state into
                // the open queue.
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
    int cards_out_of_home = 364; // 91 * 4, 91 = 13 + 12 + ... + 1

    for (const auto &stack: state.stacks) {
        for (const auto &home: state.homes) {
            if (stack.topCard().has_value()) {
                if (home.canAccept(stack.topCard().value())) {
                    cards_out_of_home -= stack.topCard()->value;
                }
            }
            if (home.topCard().has_value()) {
                cards_out_of_home -= home.topCard()->value;
            }
        }
    }

    return cards_out_of_home;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
    std::set<std::pair<std::shared_ptr<SearchState>, double>> queue_open;
    std::vector<SearchAction> solution = {};
    std::map<SearchState, double> map_costs_g = {std::make_pair(init_state, 0.0)};
    std::map<SearchState, std::pair<std::shared_ptr<SearchState>, SearchAction>> predecessors;

    queue_open.insert(std::make_pair(std::make_shared<SearchState>(init_state), 0.0));

    while(!queue_open.empty()) {
        // Memory check.
        if (getCurrentRSS() + MEM_RESERVE > this->mem_limit_) {
            return {};
        }

        // Take the first node from the priority queue with the lowest value.
        std::pair<std::shared_ptr<SearchState>, double> queue_top =
                *std::min_element(queue_open.begin(), queue_open.end(),
                                  [](const auto& lhs, const auto& rhs) {
            return lhs.second < rhs.second;
        });

        SearchState working_state = *(queue_top.first);

        queue_open.erase(queue_top);

        // Expand the node.
        std::vector<SearchAction> actions = working_state.actions();

        for (SearchAction action : actions) {
            SearchState action_state = action.execute(working_state);

            if (action_state.isFinal()) {
                solution.push_back(action);
                auto parent = predecessors.find(working_state);

                while (parent != predecessors.end()) {
                    // Add the previous action which ends in the current known state from the path.
                    solution.push_back(parent->second.second);
                    // Find the previous state from which the action begins.
                    parent = predecessors.find(*(parent->second.first));
                }

                // Reverse the pointers of the vector to get the solution in the right order.
                std::reverse(solution.begin(), solution.end());

                return solution;
            }

            // Get the parent g(n) value and calculate a new value for children (g(n) + 1).
            auto state_cost_g = map_costs_g.find(working_state);

            double tentative_cost_g = 0.0;

            if (state_cost_g != map_costs_g.end()) {
                tentative_cost_g = state_cost_g->second + 1.0;
            }

            // Get the child g(n) value.
            state_cost_g = map_costs_g.find(action_state);

            if (state_cost_g == map_costs_g.end()) {
                // The child is not in the map.
                map_costs_g.emplace(action_state, tentative_cost_g);

                // Calculate the new f(n) value.
                double cost_f = tentative_cost_g + compute_heuristic(action_state, *(this->heuristic_));

                queue_open.insert(std::make_pair(std::make_shared<SearchState>(action_state), cost_f));

                predecessors.insert(std::make_pair(action_state,
                                                   std::make_pair(std::make_shared<SearchState>(working_state), action)));

            } else if (tentative_cost_g < state_cost_g->second) {
                // The child is in the map and a new value is smaller than the actual.
                state_cost_g->second = tentative_cost_g;

                // Calculate the new f(n) value.

                double cost_f = tentative_cost_g + compute_heuristic(action_state, *(this->heuristic_));

                auto state = std::find_if(queue_open.begin(), queue_open.end(),
                                          [&action_state](const std::pair<std::shared_ptr<SearchState>, double> & element){
                                              return !(*(element.first) < action_state) && !(action_state < *(element.first));
                                          } );

                if (state != queue_open.end()) {
                    queue_open.erase(state);
                    queue_open.insert(std::make_pair(std::make_shared<SearchState>(action_state), cost_f));
                }

                auto it = predecessors.find(action_state);

                if (it != predecessors.end()) {
                    it->second = std::make_pair(std::make_shared<SearchState>(working_state), action);
                }
            }
        }
    }

	return {};
}
