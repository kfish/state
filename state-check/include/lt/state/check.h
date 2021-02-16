#pragma once

#include "lt/state/mealy.h"
#include "lt/state/stream.h"

#include <boost/test/unit_test.hpp>
#include <rapidcheck/boost_test.h>

#include "lt/core/container.h"
#include "lt/core/variant.h"

namespace lt::state {

template <typename M>
void check(const std::vector<typename M::input_type>& input,
           const std::vector<typename M::output_type>& expected)
{
    auto output = M().run(input);

    if (output != expected) {
        std::cout << "Input: " << input << std::endl;
        std::cout << "Expected: " << expected << std::endl;
        std::cout << "Output: " << output << std::endl;
    }

    BOOST_CHECK(output == expected);
}

template <typename M>
void check_fanout(const std::vector<typename M::input_type>& input,
                  const typename M::output_type& expected,
    std::enable_if<(
        lt::core::is_container_v<typename M::output_type>
    ), int>* = 0)
{
    auto output = lt::core::concat(M().run(input));
    BOOST_CHECK(output == expected);
}

template <typename M>
void check_fanout_concat(const std::vector<typename M::input_type>& input,
                  const typename M::output_type& expected,
    std::enable_if<(
        lt::core::is_container_v<typename M::output_type>
    ), int>* = 0)
{
    auto output = M().run_concat(input);
    BOOST_CHECK(output == expected);
}

template <typename S>
void check_stream(const std::vector<typename S::input_type>& input,
                  const std::vector<typename S::value_type>& expected)
{
    auto output = S().run(input);
    BOOST_CHECK(output == expected);
}

template <typename M>
class Property
{
   public:
    using state_type = typename M::state_type;
    using input_type = typename M::input_type;
    using output_type = typename M::output_type;

    Property(std::function<bool(const std::vector<input_type>&,
                                const std::vector<output_type>&)> property)
        : property_(property)
    {
    }

    void boost_check(const std::vector<input_type>& input)
    {
        auto output = M().run(input);

        // Check property
        BOOST_CHECK(property_(input, output));
    }

    void rc_assert(const std::vector<input_type>& input)
    {
        auto output = M().run(input);

        // Check property
        RC_ASSERT(property_(input, output));
    }

   private:
    std::function<bool(const std::vector<input_type>&, const std::vector<output_type>&)> property_;
};

template <typename M>
class Expected : public Property<M>
{
   public:
    using state_type = typename M::state_type;
    using input_type = typename M::input_type;
    using output_type = typename M::output_type;

    Expected(std::function<
        std::vector<output_type>(const std::vector<input_type>&)> make_expected)
        : Property<M>(
            [=](const std::vector<input_type>& input,
                const std::vector<output_type>& output) -> bool {
                    return (make_expected(input) == output);
                }
            )
    {
    }
};

} // namespace lt::state
