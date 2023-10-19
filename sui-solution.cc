#include <algorithm>
#include <queue>
#include <set>
#include <utility>

#include "search-strategies.h"

// https://stackoverflow.com/questions/3686588/implementing-a-tree-in-c
template<typename T>
class TreeNode
{
public:
    TreeNode()
    = default;

    TreeNode(T*  _value, TreeNode<T>* _parent)
        : Value(_value), Parent(_parent)
    {
    }

    ~TreeNode()
    {
        for (TreeNode<T>* child : Children) {
            delete child;
            child = nullptr;
        }

        delete Value;
        Value = nullptr;
    }

    T* Value;
    TreeNode<T>* Parent;
    std::vector<TreeNode<T>* > Children;
};

// TODO kontrola pameti
// TODO jestli pouzit nejaky shared ptr
// TODO jestli se daji klasicky C-style ukazatele nahradit nejakou konstrukci z cpp
std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
    std::queue<std::pair<SearchState, TreeNode<SearchAction>*>> queue_open;
    std::vector<SearchAction> solution = {};
    std::set<SearchState> set_closed = {init_state};

    auto root_node = new TreeNode<SearchAction>(nullptr, nullptr);

    // Push the first state into the queue_open.
    queue_open.emplace(init_state, root_node);

    while (!queue_open.empty()) {
        // Take the first node from the queue.
        SearchState working_state = queue_open.front().first;
        TreeNode<SearchAction>* working_action = queue_open.front().second;
        queue_open.pop();

        // Expand the node.
        std::vector<SearchAction> actions = working_state.actions();

        for (SearchAction action : actions) {
            SearchState action_state = action.execute(working_state);

            // If the current state is a goal.
            if (action_state.isFinal()) {
                solution.push_back(action);
                TreeNode<SearchAction>* action_in_tree = working_action;

                while (action_in_tree->Parent != nullptr) {
                    //solution.push_back(action_in_tree->Value);
                    solution.push_back(*(action_in_tree->Value));
                    action_in_tree = action_in_tree->Parent;
                }

                // Reverse the pointers of the vector to get the solution in the right order.
                std::reverse(solution.begin(), solution.end());

                delete root_node;

                return solution;
            }

            // If the state is not in set.
            if (set_closed.find(action_state) == set_closed.end()) {
                // Add the node to the queue and to the set.
                set_closed.insert(action_state);

                auto tree_action = new SearchAction(action);
                auto tree_node = new TreeNode<SearchAction>(tree_action, working_action);
                working_action->Children.push_back(tree_node);

                queue_open.emplace(action_state, tree_node);
            }
        }
    }

    delete root_node;

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
