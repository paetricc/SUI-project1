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

    /*
    explicit TreeNode(T  value, TreeNode<T> parent)
            : Value(std::move(value)), Parent(parent)
    {
    }*/

    TreeNode(T  _value, TreeNode<T>* _parent)
        : Value(std::move(_value)), Parent(_parent)
    {
    }

    T Value;
    TreeNode<T>* Parent;
    //std::list<TreeNode<T> > Children;
};

// TODO kontrola pameti
// TODO pro objekty vytvorene pomoci new take odstranit je pomoci delete
std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
    std::queue<std::pair<SearchState, TreeNode<SearchAction>*>> queue_open;
    std::vector<SearchAction> solution = {};

    std::set<SearchState> set_closed = {init_state}; // TODO jestli mozna nespojit set_closed a predecessors std::set<std::tuple<SearchState, SearchAction, SearchState>> - porovnat performance

/*
    for (int i = 0; i < 2; i++) {
        TreeNode<int>* tree_node1 = new TreeNode<int>(1, nullptr);
        std::cout << "tree node 1: " << tree_node1 << "\n";
        //TreeNode<int> tree_node1(1, nullptr);
        //std::cout << "tree node 1: " << &tree_node1 << "\n";

    }*/

    // TreeNode<SearchAction> parent_tree_node(nullptr, nullptr);

    // TODO projit
    // std::map<SearchState, std::pair<SearchState, SearchAction>> predecessors;

    // Push the first state into the queue_open.
    queue_open.emplace(init_state, nullptr);

    while (!queue_open.empty()) {
        // Take the first node from the queue.
        SearchState working_state = queue_open.front().first;
        TreeNode<SearchAction>* working_action = queue_open.front().second;
        queue_open.pop();

        // Expand the node.
        std::vector<SearchAction> actions = working_state.actions();

        for (SearchAction action : actions) {
            SearchState action_state = action.execute(working_state);

            //std::cout << "Searching actions \n";

            // If the current state is a goal.
            if (action_state.isFinal()) {

                //std::cout << "here\n";

                solution.push_back(action);

                TreeNode<SearchAction>* action_in_tree = working_action;

                //std::cout << "action in tree: " << action_in_tree << "\n";
                //std::cout << "bool value: " << (action_in_tree == nullptr) << "\n";
                //std::cout << "action in tree parent: " << action_in_tree->Parent << "\n";


                while (action_in_tree != nullptr) {
                    //std::cout << "loop: " << action_in_tree << "\n";
                    solution.push_back(action_in_tree->Value);
                    action_in_tree = action_in_tree->Parent;
                }
                /*
                auto parent = predecessors.find(working_state);

                while (parent != predecessors.end()) {
                    //std::cout << "searching root\n";

                    // Add the previous action which ends in the current known state from the path.
                    solution.push_back(parent->second.second);
                    // Find the previous state from which the action begins.
                    parent = predecessors.find(parent->second.first);
                }
                */
                // Reverse the pointers of the vector to get the solution in the right order.




                std::reverse(solution.begin(), solution.end());

                //std::cout << "returning ...\n";

                return solution;
            }

            // If the state is not in set.
            if (set_closed.find(action_state) == set_closed.end()) {
                // Add the node to the queue and to the set.
                set_closed.insert(action_state);

                //auto tree_node = new TreeNode<SearchAction>(action, working_action);
                auto tree_node = new TreeNode<SearchAction>(action, working_action);
                //std::cout << "Tree Node: " << tree_node << "\n";
/*
                std::cout << "Action: " << action << "\n";
                std::cout << "Action in tree: " << tree_node->Value << "\n";
                std::cout << "Parent: " << working_action << "\n";
                std::cout << "Parent in tree: " << tree_node->Parent << "\n";
                std::cout << "\n";

*/
                //queue_open.emplace(action_state, &tree_node);
                queue_open.emplace(action_state, tree_node);

                //queue_open.emplace(action_state, nullptr);
                // std::cout << &tree_node << "\n";

                //predecessors.insert(std::make_pair(action, working_action));
                //predecessors.insert(std::make_pair(action_state,std::make_pair(working_state, action)));
            }
        }

        //std::cout << "----------------------------------------------------------------\n";
        //std::cout << "----------------------------------------------------------------\n";
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
