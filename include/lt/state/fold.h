#pragma once

#include "lt/state/mealy.h"

namespace lt::state {

// Use a Mealy machine that folds some subset of its Input, and propagates any
// other Input elements unaltered. Typically this will be used in sequence with
// some other Mealy machine to fold its output.
template <typename State, typename Input, typename Output>
class Fold : public Mealy<State, Input, Output>
{
   public:
    using Mealy_ = Mealy<State, Input, Output>;

    using transition_function_type = typename Mealy_::transition_function_type;

    template <typename InnerMealy>
    Fold(const InnerMealy& inner_mealy,
        std::enable_if<(
            // The State is just the inner state type
            std::is_same_v<State, get_state_type<InnerMealy>> &&

            // The inner input is the value type of the Input
            std::is_same_v<
                get_input_type<InnerMealy>,
                lt::core::value_type<Input>
            > &&

            // The value type of the output is the Output value type
            std::is_same_v<
                lt::core::value_type<get_output_type<InnerMealy>>,
                lt::core::value_type<Output>
            >
        ), int>* = 0
        )
        : Mealy<State, Input, Output>(make_fold<InnerMealy>(inner_mealy))
    {}

   private:
    template <typename InnerMealy>
    static Mealy_ make_fold(const InnerMealy& inner_mealy)
    {
        State init = inner_mealy.initial_state();
        auto transition_function = fold_transition_function<InnerMealy>(inner_mealy);

        return Mealy<State, Input, Output>(init, transition_function);
    }

    // Transition function for Fold, runs InnerMealy::transition_function on
    // each element of the Input
    template <typename InnerMealy>
    static transition_function_type fold_transition_function(
        const InnerMealy& inner_mealy)
    {
        return [=](State state, Input input) -> std::tuple<State, Output> {
            static auto const f = inner_mealy.transition_function_;
            auto output = Output();

            for (auto && i : input) {
                auto && [s, o] = f(state, i);
                state = s;
                if constexpr (std::is_same_v<get_output_type<InnerMealy>, Output>) {
                    output.insert(output.end(),
                        std::make_move_iterator(o.begin()),
                        std::make_move_iterator(o.end()));
                } else {
                    output.emplace_back(o);
                }
            }

            return { state, output };
        };
    }
};

} // namespace lt::state

