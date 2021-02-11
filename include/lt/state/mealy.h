#pragma once

#include <functional>
#include <type_traits>
#include <utility>

#include "lt/core/container.h"
#include "lt/core/tuple.h"
#include "lt/core/variant.h"

namespace lt::state
{

template <typename M>
using get_state_type = typename M::state_type;

template <typename M>
using get_input_type = typename M::input_type;

template <typename M>
using get_output_type = typename M::output_type;

template <typename M>
using get_state_output_type = typename M::state_output_type;

template <typename... Ms>
using group_input_type = std::tuple<get_input_type<Ms>...>;

template <typename... Ms>
using group_output_type = std::tuple<get_output_type<Ms>...>;

template <typename... Ms>
using group_state_type = std::tuple<get_state_type<Ms>...>;

// Initial state for parallel(), spray(), Fanout: collects all initial states of input machines
template <typename... Mealys, std::size_t... I>
static auto initial_states(const std::tuple<Mealys...>& mealys, std::index_sequence<I...>)
{
    return std::tuple{ std::get<I>(mealys).initial_state()... };
}

template <typename State, typename Input, typename Output>
class Mealy
{
   public:
    template <typename, typename, typename>
    friend class Mealy;

    template <typename, typename, typename>
    friend class ValueMealy;

    template <typename, typename, typename>
    friend class Filter;

    template <typename, typename, typename>
    friend class Fold;

    template <typename, typename, typename>
    friend class Fanout;

    template <typename, typename, typename>
    friend class FanoutConcat;

    using state_type = State;
    using input_type = Input;
    using output_type = Output;
    using value_type = Output;
    using state_output_type = std::tuple<State, Output>;

    using transition_function_type = std::function<const std::tuple<State, Output>(const State&, const Input&)>;

    // A Mealy machine defined in terms of a single combined transition function
    //   T : (S, I) -> (S, O)
    Mealy(const State& initial_state,
        std::function<const std::tuple<State, Output>(const State&, const Input&)> transition_function)
        : initial_state_(initial_state), state_(initial_state), transition_function_(transition_function)
    {
    }

    // A Mealy machine deifned in terms of independent transition and output functions
    //   T : (S, I) -> S
    //   G : (S, I) -> O
    static Mealy make_independent(const State& initial_state,
        std::function<const State(const State&, const Input&)> transition_function,
        std::function<const Output(const State&, const Input&)> output_function)
    {
        auto independent_transition =
            [=](const State& s0, const Input& i) -> std::tuple<State, Output> {
                auto s = transition_function(s0, i);
                auto o = output_function(s0, i);
                return {s, o};
            };

        return Mealy<State, Input, Output>(initial_state, independent_transition);
    }

    // A Mealy machine defined in terms of transition and differential output
    // functions, where the output is determined only by the input and output
    // states
    //   T : (S, I) -> S
    //   G : (S, S) -> O
    static Mealy make_differential(const State& initial_state,
        std::function<const State(const State&, const Input&)> transition_function,
        std::function<const Output(const State& prev, const State& next)> output_function)
    {
        auto differential_transition =
            [=](const State& s0, const Input& i) -> std::tuple<State, Output> {
                auto s = transition_function(s0, i);
                auto o = output_function(s0, s);
                return {s, o};
            };

        return Mealy<State, Input, Output>(initial_state, differential_transition);
    }

    // A sequence of two Mealy machines:
    //   M1 = (S1, Input) -> (S1, O1)
    //   M2 = (S2, O1) -> (S2, Output)
    // where
    //   State = (S1, S2)
    template <typename S1, typename S2, typename O1>
    static Mealy sequence(Mealy<S1, Input, O1> m1, Mealy<S2, O1, Output> m2,
        std::enable_if<(
            std::is_same_v<State, std::tuple<S1, S2>>
        ), int>* = 0)
    {
        auto sequence_transition =
            [=](const State& s0, const Input& i) -> std::tuple<State, Output> {
                const auto [s0_1, s0_2] = s0;
                const auto && [s1, o1] = m1.transition_function_(s0_1, i);
                const auto && [s2, o] = m2.transition_function_(s0_2, o1);
                auto s = std::make_tuple(s1, s2);
                return {s, o};
            };

        auto combined_initial_state = std::make_tuple(m1.initial_state_, m2.initial_state_);

        return Mealy<State, Input, Output>(combined_initial_state, sequence_transition);
    }

    // Two Mealy machines in parallel:
    //   M1 = (S1, I1) -> (S1, O1)
    //   M2 = (S2, I2) -> (S2, O2)
    // where
    //   State = (S1, S2)
    //   Input = (I1, I2)
    //   Output = (O1, O2)
    template <typename S1, typename I1, typename O1,
              typename S2, typename I2, typename O2>
    static Mealy pair(Mealy<S1, I1, O1> m1, Mealy<S2, I2, O2> m2,
        std::enable_if<(
            std::is_same_v<State, std::tuple<S1, S2>> &&
            std::is_same_v<Input, std::tuple<I1, I2>> &&
            std::is_same_v<Output, std::tuple<O1, O2>>
        ), int>* = 0)
    {
        auto pair_transition =
            [=](const State& s0, const Input& i0) -> std::tuple<State, Output> {
                const auto [s1_0, s2_0] = s0;
                const auto [i1_0, i2_0] = i0;
                const auto && [s1, o1] = m1.transition_function_(s1_0, i1_0);
                const auto && [s2, o2] = m2.transition_function_(s2_0, i2_0);
                auto s = std::make_tuple(s1, s2);
                auto o = std::make_tuple(o1, o2);
                return {s, o};
            };

        auto combined_initial_state = std::make_tuple(m1.initial_state_, m2.initial_state_);

        return Mealy<State, Input, Output>(combined_initial_state, pair_transition);
    }

    // Multiple Mealy machines in parallel:
    //   Mn = (Sn, In) -> (Sn, On)
    // where
    //   State = (Sn...)
    //   Input = (In...)
    //   Output = (On...)
    //
    template <typename... Mealys, std::size_t... I>
    static Mealy parallel(const std::tuple<Mealys...>& mealys,
        std::enable_if<(
            std::is_same_v<State, group_state_type<Mealys...>> &&
            std::is_same_v<Input, group_input_type<Mealys...>> &&
            std::is_same_v<Output, group_output_type<Mealys...>>
        ), int>* = 0)
    {
        auto indices = std::make_index_sequence<sizeof...(Mealys)>{};
        State init = initial_states<Mealys...>(mealys, indices);
        auto transition_function = parallel_transition_function<Mealys...>(mealys, indices);

        return Mealy<State, Input, Output>(init, transition_function);
    }

    // Multiple Mealy machines in parallel, with possibly common inputs:
    //   Mn = (Sn, In) -> (Sn, On)
    // where
    //   State = (Sn...)
    //   Input = union (In...)
    //   Output = (On...)
    //
    template <typename... Mealys, std::size_t... I>
    static Mealy spray(const std::tuple<Mealys...>& mealys,
        std::enable_if<(
            // The State is a tuple of all the Mealy machine states
            std::is_same_v<State, group_state_type<Mealys...>> &&

            // All Mealy machine inputs are subvariants of the Input
            std::conjunction_v<
                lt::core::is_subvariant<get_input_type<Mealys>, Input>...> &&

            // The Output is a tuple of all the Mealy machine outputs
            std::is_same_v<Output, group_output_type<Mealys...>> &&

            // The Mealy machines all have std::variant<> outputs with std::monostate
            std::conjunction_v<
                lt::core::variant_has_monostate<get_output_type<Mealys>>...>
        ), int>* = 0)
    {
        auto indices = std::make_index_sequence<sizeof...(Mealys)>{};
        State init = initial_states<Mealys...>(mealys, indices);
        auto transition_function = spray_transition_function<Mealys...>(mealys, indices);

        return Mealy<State, Input, Output>(init, transition_function);
    }



    // Returns the current State
    const State& state() const { return state_; }

    const State& initial_state() const { return initial_state_; }

    // Run the machine for one input. The internal state is updated.
    // Returns the output from this step.
    const Output step(const Input& input) {
        const auto && [new_state, output] = transition_function_(state_, input);
        state_ = new_state;
        return output;
    }

    // Run the machine for one input. The internal state is updated.
    // Returns both the new state and the output from this step.
    //
    // auto machine = Mealy(...);
    // auto input = ...; // input type
    //
    // Then:
    //     auto && [new_state, output] = machine.step_state(input);
    //
    // is equivalent to:
    //     auto output = machine.step(input);
    //     auto new_state = machine.state();
    const std::tuple<State, Output> step_state(const Input& input) {
         auto output = step(input);
         return { state_, output };
    }

    // Run the machine over a sequence of inputs. The internal state is
    // updated. Returns the resulting sequence of outputs.
    const std::vector<Output> run(const std::vector<Input>& inputs)
    {
        auto outputs = std::vector<Output>();

        for (auto && input : inputs) {
            outputs.emplace_back(step(input));
        }

        return outputs;
    }

    // Run the machine over a sequence of inputs. The internal state is
    // updated. Enabled only if the Output type is a container.
    // Returns a vector that concatenates the resulting sequence of outputs
    template <typename Output_ = Output,
        typename std::enable_if<(
            lt::core::is_container_v<Output_>
        ), int>::type = 0>
    const std::vector<typename Output_::value_type> run_concat(const std::vector<Input>& inputs)
    {
        auto outputs = std::vector<typename Output_::value_type>();

        for (auto && input : inputs) {
            auto output = step(input);

            outputs.insert(outputs.end(),
                std::make_move_iterator(output.begin()),
                std::make_move_iterator(output.end()));
        }

        return outputs;
    }

    // Run the machine over a sequence of inputs. The internal state is
    // updated. Returns a sequence of both the intermediate states and outputs.
    const std::vector<std::tuple<State, Output>> run_state(const std::vector<Input>& inputs) {
        auto outputs = std::vector<std::tuple<State, Output>>();

        for (auto && input : inputs) {
            outputs.emplace_back(run_state(input));
        }

        return outputs;
    }

   protected:
    State initial_state_;
    State state_;
    transition_function_type transition_function_;

    template <typename S, typename I, typename O>
    static std::function<const std::pair<S, O>(const S&, const Input&)> wrap_transition_function(std::function<const std::tuple<S, O>(const S&, const I&)> f,
        std::enable_if<(
            lt::core::is_variant_member_v<S, State> &&
            ((lt::core::is_variant_member_v<O, Output> && lt::core::variant_has_monostate_v<Output>) ||
             lt::core::is_variant_member_v<O, lt::core::value_type<Output>>)
        ), int>* = 0)
    {
        return [=](const S& state0, const Input& input) -> const std::pair<S, O> {
            return std::visit(lt::core::match{
                [f, &state0](auto && inner_input) {
                    if constexpr (lt::core::is_variant_member_v<std::decay_t<decltype(inner_input)>, I>) {
                        auto && [s, o] = f(state0, inner_input);
                        return std::pair<S, O>(s, o);
                    } else if constexpr (lt::core::variant_has_monostate_v<O>) {
                        return std::pair<S, O>(state0, std::monostate{});
                    } else if constexpr (lt::core::is_container_v<O>) {
                        return std::pair<S, O>(state0, O());
                    }
                }
            }, input);
        };
    }

   private:

    // Transition function for parallel(), runs all the transition_functions and collects outputs
    template <typename... Mealys, std::size_t... I>
    static transition_function_type parallel_transition_function(
        const std::tuple<Mealys...>& mealys, std::index_sequence<I...>)
    {
        return [=](const State& states0, const Input& inputs) -> const std::tuple<State, Output> {
            auto states_outputs = std::tuple{ std::get<I>(mealys).transition_function_(std::get<I>(states0), std::get<I>(inputs))... };
            auto states = std::tuple{ std::get<0>(std::get<I>(states_outputs))... };
            auto outputs = std::tuple{ std::get<1>(std::get<I>(states_outputs))... };

            return std::make_tuple(states, outputs);
        };
    }

    // Transition function for spray(), runs all the transition_functions
    template <typename... Mealys, std::size_t... I>
    static transition_function_type spray_transition_function(
        const std::tuple<Mealys...>& mealys, std::index_sequence<I...>)
    {
        static auto const functions = std::tuple { wrap_transition_function(std::get<I>(mealys).transition_function_)... };

        return [=](State states0, Input input) -> std::tuple<State, Output> {
            auto states_outputs = std::make_tuple( (std::invoke(std::get<I>(functions), std::get<I>(states0), input))... );
            auto states = std::tuple{ std::get<0>(std::get<I>(states_outputs))... };
            auto outputs = std::tuple{ std::get<1>(std::get<I>(states_outputs))... };
            return std::make_tuple(states, outputs);
        };
    }
};

/*
 * Wrap a Mealy machine which has a Container output type,
 * declaring the Container::value_type as the Mealy::value_type.
 *
 * This is useful for using a single Mealy machine in a
 * Stream, without making it part of a FanoutConcat.
 */
template <typename State, typename Input, typename Output>
class ValueMealy : public Mealy<State, Input, Output>
{
   public:
    using value_type = typename Output::value_type;

    ValueMealy(Mealy<State, Input, Output>&& mealy)
        : Mealy<State, Input, Output>(mealy)
    {}
};

} // namespace lt::state
