#pragma once

#include "lt/state/mealy.h"

namespace lt::state {

// Use a Mealy machine that filters some subset of its Input, and propagates any
// other Input elements unaltered. Typically this will be used in sequence with
// some other Mealy machine to filter its output.
template <typename State, typename Input, typename Output>
class Filter : public Mealy<State, Input, Output>
{
   public:
    using Mealy_ = Mealy<State, Input, Output>;

    using transition_function_type = typename Mealy_::transition_function_type;

    template <typename FilterMealy>
    Filter(const FilterMealy& filter_mealy,
        std::enable_if<(
            // The State is just the filter's state type
            std::is_same_v<State, get_state_type<FilterMealy>> &&

            // The filter input is a subvariant of the Input
            lt::core::is_subvariant_v<get_input_type<FilterMealy>, Input> &&

            // The value type of the filter output is a subvariant of the
            // Output value type
            lt::core::is_subvariant_v<
                lt::core::value_type<get_output_type<FilterMealy>>,
                lt::core::value_type<Output>
            >
        ), int>* = 0
        )
        : Mealy<State, Input, Output>(make_filter<FilterMealy>(filter_mealy))
    {}

   private:
    template <typename FilterMealy>
    static Mealy_ make_filter(const FilterMealy& filter_mealy)
    {
        State init = filter_mealy.initial_state();
        auto transition_function = filter_transition_function<FilterMealy>(filter_mealy);

        return Mealy<State, Input, Output>(init, transition_function);
    }

    // Transition function for Filter, runs FilterMealy::transition_function if
    // the given Input is accepted by FilterMealy, otherwise propagates the Input
    // unaltered
    template <typename FilterMealy>
    static transition_function_type filter_transition_function(
        const FilterMealy& filter_mealy)
    {
        return [=](State state0, Input input) -> std::tuple<State, Output> {
            return std::visit(lt::core::match{
                [filter_mealy, &state0](auto && inner_input) {
                    static auto const f = filter_mealy.transition_function_;
                    if constexpr (
                        lt::core::is_variant_member_v<
                            std::decay_t<decltype(inner_input)>,
                            get_input_type<FilterMealy>
                    >) {
                        auto && [s, inner_o] = f(state0, inner_input);
                        if constexpr (lt::core::is_container_v<
                            get_output_type<FilterMealy>
                        >) {
                            auto o = Output();
                            for (auto && effect : inner_o) {
                                o.emplace_back(lt::core::variant_cast(effect));
                            }
                            return std::tuple<State, Output>(s, o);
                        } else {
                            Output o = lt::core::variant_cast(inner_o);
                            return std::tuple<State, Output>(s, o);
                        }
                    } else if constexpr (lt::core::is_container_v<Output>) {
                        return std::tuple<State, Output>(state0, { inner_input });
                    } else {
                        return std::tuple<State, Output>(state0, inner_input);
                    }
                }
            }, input);
        };
    }
};

} // namespace lt::state
