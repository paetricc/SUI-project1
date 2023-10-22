#include <algorithm>
#include <queue>
#include <set>
#include <stack>
#include <utility>
#include <vector>

#include "search-strategies.h"
#include "memusage.h"

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
    constexpr u_char DEPTH = 0;

    std::vector<SearchAction> vector_result;
    std::vector<SearchAction> vector_actions;
    std::stack<std::pair<SearchState, int>> state_stack;
    state_stack.emplace(init_state, DEPTH);
    bool swap = false;
    while (!state_stack.empty()) {
        // Memory check
        if (swap) {
            if (getCurrentRSS() + MEM_RESERVE > this->mem_limit_)
                return {};
        }
        swap = !swap;
        // Get new state to processing
        auto node = state_stack.top();
        state_stack.pop();
        // Check if actual node result of game
        if (node.first.isFinal())
            return vector_result;
        // Depth-limit occurred
        if (node.second == this->depth_limit_) {
            if (state_stack.empty())
                continue;
            // Depth changes so pop from vector_result and vector_action until
            // vector_result size not equal to new state depth from top of state_stack
            for (int i = 0; i <= node.second - state_stack.top().second; i++) {
                vector_result.pop_back();
                vector_actions.pop_back();
            }
            // New possible result action to vector_result
            if (!vector_actions.empty())
                vector_result.emplace_back(vector_actions.back());

            continue;
        }
        // Expand node and save actions and states to vector_actions and state_stack
        for (auto action: node.first.actions()) {
            state_stack.emplace(action.execute(node.first), node.second + 1);
            vector_actions.emplace_back(action);
        }
        // New possible result action to vector_result
        vector_result.emplace_back(vector_actions.back());
    }
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

/*
 * Struct for comparison of the f(n) values for the A-star method.
 */
struct compare {
    constexpr bool operator()(
            std::pair<double, std::shared_ptr<SearchState>> const& a,
            std::pair<double, std::shared_ptr<SearchState>> const& b)
    const noexcept
    {
        return a.first > b.first;
    }
};

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
    std::priority_queue<std::pair<double, std::shared_ptr<SearchState>>,
            std::vector<std::pair<double, std::shared_ptr<SearchState>>>, compare> queue_open;

    std::map<SearchState, double> map_costs_g = {std::make_pair(init_state, 0.0)};
    std::map<SearchState, std::pair<std::shared_ptr<SearchState>, SearchAction>> predecessors;
    std::set<SearchState> set_closed;
    std::vector<SearchAction> solution = {};

    queue_open.emplace(0.0, std::make_shared<SearchState>(init_state));

    while(!queue_open.empty()) {
        // Memory check.
        if (getCurrentRSS() + MEM_RESERVE > this->mem_limit_) {
            return {};
        }

        // Take the first node from the priority queue with the lowest value.
        SearchState working_state = *(queue_open.top().second);
        queue_open.pop();

        if (set_closed.find(working_state) == set_closed.end()) {
            // The state is not in the closed set.
            set_closed.insert(working_state);

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

                    queue_open.emplace(cost_f, std::make_shared<SearchState>(action_state));

                    predecessors.insert(std::make_pair(action_state,
                                                       std::make_pair(std::make_shared<SearchState>(working_state), action)));

                } else if (tentative_cost_g < state_cost_g->second) {
                    // The child is in the map and a new value is smaller than the actual.
                    state_cost_g->second = tentative_cost_g;

                    // Calculate the new f(n) value.
                    double cost_f = tentative_cost_g + compute_heuristic(action_state, *(this->heuristic_));
                    queue_open.emplace(cost_f, std::make_shared<SearchState>(action_state));

                    auto it = predecessors.find(action_state);

                    if (it != predecessors.end()) {
                        it->second = std::make_pair(std::make_shared<SearchState>(working_state), action);
                    }
                }
            }
        }
    }

    return {};
}
