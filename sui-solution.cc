#include "search-strategies.h"

std::vector<SearchAction> BreadthFirstSearch::solve(const SearchState &init_state) {
	return {};
}

std::vector<SearchAction> DepthFirstSearch::solve(const SearchState &init_state) {
	return {};
}

double StudentHeuristic::distanceLowerBound(const GameState &state) const {
/*
    // 1. základní heuristika, která se kouká na vrchol stacku pouze
    int cards_out_of_home = king_value * colors_list.size(); // Počet karet: počet různých čísel * počet barev
    for (const auto &home : state.homes) {
        auto home_top = home.topCard(); // Karta na vrchu home.

        if (home_top.has_value()) {
            for (const auto &stack : state.stacks) {
                auto stack_top = stack.topCard();

                if (stack_top.has_value()) {
                    if (stack_top->value == home_top->value + 1) {
                        cards_out_of_home = cards_out_of_home - 1;
                    }
                }
            }

            cards_out_of_home -= home_top->value; // Odečtení počtu karet, co jsou v konkrétním home od celkového počtu karet.
        }
    }

    return cards_out_of_home;
*/

    // 2. totozne ale namisto poctu karet se berou jejich hodnoty
    int cards_out_of_home = 364; // 91 * 4, 91 = 13 + 12 + ... + 1
    for (const auto &home : state.homes) {
        auto home_top = home.topCard(); // Karta na vrchu home.

        if (home_top.has_value()) {
            for (const auto &stack : state.stacks) {
                auto stack_top = stack.topCard();

                if (stack_top.has_value()) {
                    if (stack_top->value == home_top->value + 1) {
                        cards_out_of_home = cards_out_of_home - stack_top->value;
                    }
                }
            }

            int value = home_top->value;

            while (value >= 1) {
                cards_out_of_home -= value; // Odečtení počtu karet, co jsou v konkrétním home od celkového počtu karet.
                value = value - 1;
            }
        }
    }

    return cards_out_of_home;
}

std::vector<SearchAction> AStarSearch::solve(const SearchState &init_state) {
	return {};
}
