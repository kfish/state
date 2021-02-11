# lt::state

## Overview

`lt::state` provides state machine handling.

 * **Mealy**: A [Mealy machine](https://en.wikipedia.org/wiki/Mealy_machine)
   parameterized over State, Input and Output types.

## Mealy machines

In general, the Input and Output types would be `std::variant<...>`. However, the
below examples illustrate construction of a Mealy machine with simple types.

<img src="https://upload.wikimedia.org/wikipedia/commons/b/b4/Mealy.png" align="right" />

> A simple Mealy machine has one input and one output. Each transition edge is
> labeled with the value of the input (shown in red) and the value of the output
> (shown in blue). The machine starts in state Si. (In this example, the output is
> the exclusive-or of the two most-recent input values; thus, the machine implements
> an edge detector, outputting a one every time the input flips and a zero
> otherwise.)

### Construction

All constructors include an initial state and some function(s) to specify the
behaviour of the machine.

#### `lt::state::Mealy<State, Input, Output>`

The most general constructor takes a single `transition_function` as argument,
which has:
  * Inputs: the current state and the input
  * Outputs: the new state and the output

```cpp
    auto transition_function = [](int s0, bool i) -> std::tuple<int, bool> {
        int s = i ? 1 : 0;
        bool o = (s0 == 0 && s == 1) || (s0 == 1 && s == 0);
        return { s, o };
    };

    auto mealy = Mealy<int, bool, bool>(-1, transition_function);
```

#### `lt::state::Mealy<State, Input, Output>::make_independent`

If the model can be separated into independent functions for evaluating the new
state and the output, these can be specified independently:

```cpp
    auto transition_function = [](int s0, bool i) -> int {
        int s = i ? 1 : 0;
        return s;
    };

    auto output_function = [](int s0, bool i) -> bool {
        int s = i ? 1 : 0;
        bool o = (s0 == 0 && s == 1) || (s0 == 1 && s == 0);
        return o;
    };

    auto mealy = Mealy<int, bool, bool>::make_independent(-1, transition_function, output_function);
```

#### `lt::state::Mealy<State, Input, Output>::make_differential`

If the output can be determined only by considering the old and new states,
irregardless of the specific input that caused the transition, then the machine
can be constructed with state transition and differential output functions:

```cpp
    auto transition_function = [](int s0, bool i) -> int {
        int s = i ? 1 : 0;
        return s;
    };

    auto output_function = [](int s0, int s) -> bool {
        return (s0 == 0 && s == 1) || (s0 == 1 && s == 0);
    };

    auto mealy = Mealy<int, bool, bool>::make_differential(-1, transition_function, output_function);
```
### Composition

Composition operators (combinators) allow you to build up a complex Mealy machine from smaller, simpler machines.

#### `lt::state::Mealy<State, Input, Output>::sequence`

Use the output of one machine as the input to another.

```cpp
    auto mealy1 = Mealy<State1, Input1, Output1>( ... );
    auto mealy2 = Mealy<State2, Output1, Output2>( ... );

    // Use the output of mealy1 as the input of mealy2
    auto chained_machine = Mealy<std::tuple<S1, S2>, Input1, Output2>::sequence(mealy1, mealy2);
```

For example the example Mealy machine above detects edges. If it outputs `true` twice in succession
then the original input has spiked, ie. a sequence of either `false,true,false` or `true,false,true`.

We can build a spike detector by sequencing an edge detector with a machine that detects `true,true`:

```cpp
    auto edge_detect_fn = [](int s0, bool i) -> std::tuple<int, bool> {
        int s = i ? 1 : 0;
        bool o = (s0 == 0 && s == 1) || (s0 == 1 && s == 0);
        return { s, o };
    };
    auto edge_detect = Mealy<int, bool, bool>(-1, edge_detect_fn);

    auto truetrue_detect_fn = [](bool s0, bool i) -> std::tuple<bool, bool> {
        bool o = s0 && i;
        return { i, o };
    };
    auto truetrue_detect = Mealy<bool, bool, bool>(false, truetrue_detect_fn);

    auto spike_detect = Mealy<std::tuple<int,bool>, bool, bool>::sequence(edge_detect, truetrue_detect);
```

#### `lt::state::Mealy<State, Input, Output>::pair`

Use two Mealy machines in parallel, with independent inputs, states and outputs:

```cpp
    auto mealy1 = Mealy<State1, Input1, Output1>( ... );
    auto mealy2 = Mealy<State2, Input2, Output2>( ... );

    // Use both machines independently but in lockstep
    auto paired_machine = Mealy<
        std::tuple<State1, State2>,
        std::tuple<Input1, Input2>,
        std::tuple<Output1, Output2>>::pair(mealy1, mealy2);
```

For an example that generates FizzBuzz by running a pair of machines `<Fizz, Buzz>`, see
https://github.com/lhftio/cframework/blob/master/test/mealy-parallel-test.cpp

#### `lt::state::Mealy<State, Input, Output>::parallel`

Use multiple Mealy machines in parallel, with independent inputs, states and outputs:

```cpp
    auto mealy1 = Mealy<State1, Input1, Output1>( ... );
    auto mealy2 = Mealy<State2, Input2, Output2>( ... );
    ...
    auto mealyN = Mealy<StateN, InputN, OutputN>( ... );

    // Use all machines independently but in lockstep
    auto parallel_machine = Mealy<
        std::tuple<State1, State2, ..., StateN>,
        std::tuple<Input1, Input2, ..., InputN>,
        std::tuple<Output1, Output2, ..., OutputN>>::parallel(mealy1, mealy2, ..., mealyN);
```

For an example that generates a generalized FizzBuzz by running a colleciton of machines
`<Fizz, Buzz, Baxx, ...>`, see
https://github.com/lhftio/cframework/blob/master/test/mealy-parallel-test.cpp

#### `lt::state::Mealy<State, Input, Output>::spray`

Use multiple Mealy machines in parallel with independent states and outputs, where:

  * the inputs are subvariants of a common Input variant
  * all outputs are variants containing `std::monostate` as an element

```cpp
    using InputAll = std::variant<InputA, InputB, ... InputZ>;

    using Input1 = std::variant<InputA, InputB>;
    auto mealy1 = Mealy<State1, Input1, Output1>( ... );

    using Input2 = std::variant<InputA, InputD, InputQ>;
    auto mealy2 = Mealy<State2, Input2, Output2>( ... );
    ...
    using InputN = std::variant<InputC, InputD, InputL, InputX>;
    auto mealyN = Mealy<StateN, InputN, OutputN>( ... );

    // Use all machines independently but in lockstep

    auto spray_machine = Mealy<
        std::tuple<State1, State2, ..., StateN>,
        InputAll,
        std::tuple<Output1, Output2, ..., OutputN>>::spray(mealy1, mealy2, ..., mealyN);
```

At each `step(input)`, only the machines for which `input` is valid (ie. an element of its `Input` variant) will be stepped; the others will retain their state and output `std::monostate{}`.

For an example that models the activity of various people at a cafe, all reacting to
various inputs of availble ingredients `<Egg, Flour, Milk>`, see
https://github.com/lhftio/cframework/blob/master/test/mealy-fanout-test.cpp

#### `lt::state::Mealy<State, Input, Output>::fanout`

Use multiple Mealy machines in parallel with independent states, where:

  * the inputs are subvariants of a common `Input` variant
  * the `Output` is a container (eg. std::vector) of some variant type
  * the outputs are subvariants of `Output::value_type`
  * all outputs are variants containing `std::monostate` as an element

```cpp
    using InputAll = std::variant<InputA, InputB, ... InputZ>;
    using OutputAll = std::variant<OutputA, OutputB, ... OutputX>;

    using Input1 = std::variant<InputA, InputB>;
    using Output1 = std::variant<OutputC, OutputG>;
    auto mealy1 = Mealy<State1, Input1, Output1>( ... );

    using Input2 = std::variant<InputA, InputD, InputQ>;
    using Output1 = std::variant<OutputB, OutputD, OutputK>;
    auto mealy2 = Mealy<State2, Input2, Output2>( ... );
    ...
    using InputN = std::variant<InputC, InputD, InputL, InputX>;
    using OutputN = std::variant<OutputQ, OutputU, OutputX>;
    auto mealyN = Mealy<StateN, InputN, OutputN>( ... );

    // Use all machines independently but in lockstep

    auto fanout_machine = Mealy<
        std::tuple<State1, State2, ..., StateN>,
        InputAll,
        std::vector<OutputAl>l
        >::fanout(mealy1, mealy2, ..., mealyN);
```

At each `step(input)`:
    * only the machines for which `input` is valid (ie. an element of its `Input` variant) will be stepped; the others will retain their state and output `std::monostate{}`
    * the tuple of output values will be flattened into the Output container type, with any `std::monostate` elements removed.

For example, if while stepping the above machine (with N=3):
    * `mealy1` outputs `OutputG{}`
    * `mealy2` outputs '{}'
    * `mealy3` outputs `OutputX{}`
then the result of `fanout_machine.step(input)` will be the vector:
```
    { OutputG{}, OutputX{} }
```

For an example that models the activity of various people at a cafe, all reacting to
various inputs of availble ingredients `<Egg, Flour, Milk>`, see
https://github.com/lhftio/cframework/blob/master/test/mealy-fanout-test.cpp

### Usage

#### `state()`

> Returns the current state.

```cpp
    auto state = mealy.state();
```

#### `step(input)`

> Run the machine for one input. The internal state is updated.
> Returns the output from this step.

```cpp
    auto output = mealy.step(input);
```

#### `step_state()`

> Run the machine for one input. The internal state is updated.
> Returns both the new state and the output from this step.

```cpp
    auto && [new_state, output] = mealy.step_state(input);
```

### Testing

For simple testing of known input and expected output values, `run()` and
`run_state()` take a vector of input values which represent the sequence of
inputs to the machine, returning a vector of the resulting intermediate outputs.

#### `run(inputs)`

> Run the machine over a sequence of inputs. The internal state is
> updated. Returns the resulting sequence of outputs.

```cpp
    auto outputs = mealy.run(inputs);

    for (auto && output : outputs) {
        // Do something with an intermediate output value
    }
```

#### `run_state(inputs)`

> Run the machine over a sequence of inputs. The internal state is
> updated. Returns a sequence of both the intermediate states and outputs.

```cpp
    auto states_outputs = mealy.run_state(inputs);

    for (auto && [state, output] : states_outputs) {
        // Do something with intermediate state and output value
    }
```

### Testing with `Boost.Test`

```cpp
bool edge_detect(Mealy<int, bool, bool>& mealy)
{
    std::vector<bool> input =
        { true, true, false, true, false, false, false, true };
    std::vector<bool> expected =
        { false, false, true, true, true, false, false, true };

    auto output = mealy.run(input);

    BOOST_CHECK(output == expected);
}
```

### Testing with `Rapidcheck`

To test with Rapidcheck:
  1. define a property function for your machine. For example, here we define
  a property that checks the output matches that derived from a pairwise loop
  over the inputs
  2. create an `RC_BOOST_PROP` test with a random input vector

```cpp
std::vector<bool> make_expected(std::vector<bool> xs)
{
    if (xs.empty()) {
        return std::vector<bool>();
    }

    auto ys = xs;
    xs.erase(xs.end());
    ys.erase(ys.begin());

    auto expected = std::vector<bool>();
    expected.push_back(false);

    for (auto xy : boost::combine(xs, ys)) {
        bool x = xy.get<0>();
        bool y = xy.get<1>();
        bool result = x != y;
        expected.emplace_back(result);
    }

    return expected;
}

void prop_edge_detection(Mealy<int, bool, bool> mealy, const std::vector<bool>& inputs)
{
    auto outputs = mealy.run(inputs);

    // Ensure we have as many outputs as inputs
    RC_ASSERT(outputs.size() == inputs.size());

    // Iteratively check edges, compare the result with the machine output
    auto expected = make_expected(inputs);
    RC_ASSERT(outputs == expected);
}

RC_BOOST_PROP(edge_detect_prop_combined, (std::vector<bool> inputs))
{
    auto mealy = ...;
    prop_edge_detection(mealy, inputs);
}
```
### Using the `state-check` library

You can reduce boilerplate in test cases with the `state-check` library, which
provides functions for boost and rapidcheck testing.

```cpp
#include "lt/state/check.h"
```

First design your state machine as a class that inherits from `Mealy<...>` and can
be constructed with no arguments:

```cpp
class EdgeDetectorCombined : public Mealy<int, bool, bool>
{
   public:
    EdgeDetectorCombined() : Mealy(-1,
        [](int s0, bool i) -> std::tuple<int, bool> {
            int s = i ? 1 : 0;
            bool o = (s0 == 0 && s == 1) || (s0 == 1 && s == 0);
            return { s, o };
        })
    {}
};
```

#### `check<M>(input, expected>()`

The `check` function, templated on the machine type, will:
  * instantiate a new instance of the state machine
  * run it on the given `input`
  * `BOOST_CHECK()` that the `output` matches `expected`

```cpp
BOOST_AUTO_TEST_CASE(case4)
{
    std::vector<bool> input = { false, false, true, false };
    std::vector<bool> expected = { false, false, true, true };

    lt::state::check<EdgeDetectorCombined>(input, expected);
}
```

#### `Expected<M>(make_expected)`

The `Expected` class provides boost and rapidcheck functions for simple use
cases where it is possible to write a function that generates the expected
output for any input.

```cpp
BOOST_AUTO_TEST_CASE(case7)
{
    std::vector<bool> input = { false, false, true, false, true, false, true };
    Expected<EdgeDetectorCombined>(make_expected).boost_check(input);
}

RC_BOOST_PROP(edge_detect_prop_combined, (std::vector<bool> input))
{
    Expected<EdgeDetectorCombined>(make_expected).rc_assert(input);
}
```
#### `Property<M>(property)`

The `Property` class provides more general checking that some property
`bool property(input, output)`holds. For example, we might check the property
that an edge must exist if and only if the input contains both true and false
values.

```cpp
bool any_false(const std::vector<bool>& v)
{
    return std::any_of(v.cbegin(), v.cend(), [](bool b) { return !b; });
}

bool any_true(const std::vector<bool>& v)
{
    return std::any_of(v.cbegin(), v.cend(), [](bool b) { return b; });
}

bool edge_exists(const std::vector<bool>&input, const std::vector<bool>& output)
{
    bool input_flips = any_false(input) && any_true(input);
    return input_flips == any_true(output);
}

RC_BOOST_PROP(edge_detect_prop_combined, (std::vector<bool> inputs))
{
    Property<EdgeDetectorCombined>(edge_exists).rc_assert(inputs);
}
```
