#pragma once

#include "lt/state/mealy.h"

namespace lt::state {

template <typename Output, typename SubOutput>
static void push_back_convert(Output& outputs, const SubOutput& sub,
    std::enable_if<(
        lt::core::is_subvariant_v<SubOutput, lt::core::value_type<Output>>
    ), int>* = 0)
{
    std::visit(lt::core::match{
        [&](auto && inner) {
            if constexpr (
                !std::is_same_v<std::decay_t<decltype(inner)>, std::monostate>
            ) {
                outputs.emplace_back(inner);
            }
        }
    }, sub);
}

// Multiple Mealy machines in parallel, with possibly common inputs and outputs:
//   Mn = (Sn, In) -> (Sn, On)
// where
//   State = (Sn...)
//   Input = union (In...)
//   Output = container < union (On...) >
//
template <typename State, typename Input, typename Output>
class Fanout : public Mealy<State, Input, Output>
{
   public:
    using Mealy_ = Mealy<State, Input, Output>;

    using value_type = typename Output::value_type;
    using transition_function_type = typename Mealy_::transition_function_type;

    template <typename... Mealys>
    Fanout(const std::tuple<Mealys...>& mealys,
        std::enable_if<(
            // The State is a tuple of all the Mealy machine states
            std::is_same_v<State, group_state_type<Mealys...>> &&

            // All Mealy machine inputs are subvariants of the Input
            std::conjunction_v<
                lt::core::is_subvariant<get_input_type<Mealys>, Input>...> &&

            // The Mealy machines all have std::variant<> outputs with std::monostate
            std::conjunction_v<
                lt::core::variant_has_monostate<get_output_type<Mealys>>...> &&

            // All Mealy machine outputs are subvariants of the Output
            std::conjunction_v<
                //lt::core::is_subvariant<get_output_type<Mealys>, typename Output::value_type>...>
                lt::core::is_subvariant<get_output_type<Mealys>, lt::core::value_type<Output>>...>
        ), int>* = 0)
        : Mealy<State, Input, Output>(make_fanout<Mealys...>(mealys))
    {}

   private:
    template <typename... Mealys, std::size_t... I>
    static Mealy_ make_fanout(const std::tuple<Mealys...>& mealys)
    {
        auto indices = std::make_index_sequence<sizeof...(Mealys)>{};
        State init = initial_states<Mealys...>(mealys, indices);
        auto transition_function = fanout_transition_function<Mealys...>(mealys, indices);

        return Mealy<State, Input, Output>(init, transition_function);
    }

    // Transition function for Fanout, runs all the transition_functions and collects outputs
    template <typename... Mealys, std::size_t... I>
    static transition_function_type fanout_transition_function(
        const std::tuple<Mealys...>& mealys, std::index_sequence<I...>)
    {
        static auto const functions = std::tuple { Mealy_::wrap_transition_function(std::get<I>(mealys).transition_function_)... };

        return [=](State states0, Input input) -> std::tuple<State, Output> {
            auto states_outputs = std::make_tuple( (std::invoke(std::get<I>(functions), std::get<I>(states0), input))... );
            auto states = std::tuple{ std::get<0>(std::get<I>(states_outputs))... };
            auto outputs = std::tuple{ std::get<1>(std::get<I>(states_outputs))... };

            auto output = Output();
            std::apply([&output](auto&&... es) {
                (push_back_convert<Output, decltype(es)>(output, std::forward<decltype(es)>(es)), ...);
            }, outputs);

            return std::make_tuple(states, output);
        };
    }
};

// Multiple Mealy machines in parallel, with possibly common inputs and outputs:
//   Mn = (Sn, In) -> (Sn, On)
// where
//   is_container(On)
//   State = (Sn...)
//   Input = union (In...)
//   Output = container < union (On::value_type...) >
//
template <typename State, typename Input, typename Output>
class FanoutConcat : public Mealy<State, Input, Output>
{
   public:
    using Mealy_ = Mealy<State, Input, Output>;

    using value_type = typename Output::value_type;
    using transition_function_type = typename Mealy_::transition_function_type;

    template <typename... Mealys>
    FanoutConcat(const std::tuple<Mealys...>& mealys,
        std::enable_if<(
            // The State is a tuple of all the Mealy machine states
            std::is_same_v<
                State, group_state_type<Mealys...>
            > &&

            // All Mealy machine inputs are subvariants of the Input
            std::conjunction_v<
                lt::core::is_subvariant<
                    get_input_type<Mealys>, Input
                >...
            > &&

            // The Mealy machines all have container outputs
            std::conjunction_v<
                lt::core::is_container<
                    get_output_type<Mealys>
                >...
            > &&

            // The value_type of the outputs of all The Mealy machines are
            // std::variant<> with std::monostate
            std::conjunction_v<
                lt::core::variant_has_monostate<
                    lt::core::value_type<get_output_type<Mealys>>
                >...
            > &&

            // The value_type of the outputs of all The Mealy machines are
            // subvariants of the Output
            std::conjunction_v<
                lt::core::is_subvariant<
                    lt::core::value_type<get_output_type<Mealys>>,
                    lt::core::value_type<Output>
                >...
            >
        ), int>* = 0)
        : Mealy<State, Input, Output>(make_fanout_concat<Mealys...>(mealys))
    {}

   private:
    template <typename... Mealys, std::size_t... I>
    static Mealy_ make_fanout_concat(const std::tuple<Mealys...>& mealys)
    {
        auto indices = std::make_index_sequence<sizeof...(Mealys)>{};
        State init = initial_states<Mealys...>(mealys, indices);
        auto transition_function = fanout_concat_transition_function<Mealys...>(mealys, indices);

        return Mealy<State, Input, Output>(init, transition_function);
    }

    template <typename SubOutput>
    static void concat_convert(Output& outputs, const SubOutput& sub,
        std::enable_if<(
            lt::core::is_container_v<Output> &&
            lt::core::is_container_v<SubOutput>
        ), int>* = 0,
        std::enable_if<(
            lt::core::is_subvariant_v<
                lt::core::value_type<SubOutput>, lt::core::value_type<Output>
            >
        ), int>* = 0)
    {
        for (auto && effect : sub) {
            push_back_convert<Output, decltype(effect)>(outputs, effect);
        }
    }

    // Transition function for Fanout, runs all the transition_functions, collects and concats outputs
    template <typename... Mealys, std::size_t... I>
    static transition_function_type fanout_concat_transition_function(
        const std::tuple<Mealys...>& mealys, std::index_sequence<I...>)
    {
        static auto const functions = std::tuple { Mealy_::wrap_transition_function(std::get<I>(mealys).transition_function_)... };

        return [=](State states0, Input input) -> std::tuple<State, Output> {
            auto states_outputs = std::make_tuple( (std::invoke(std::get<I>(functions), std::get<I>(states0), input))... );
            auto states = std::tuple{ std::get<0>(std::get<I>(states_outputs))... };
            auto outputs = std::tuple{ std::get<1>(std::get<I>(states_outputs))... };

            auto output = Output();
            std::apply([&output](auto&&... es) {
                (concat_convert(output, std::forward<decltype(es)>(es)), ...);
            }, outputs);

            return std::make_tuple(states, output);
        };
    }
};

} // namespace lt::state
