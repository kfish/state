#pragma once

#include "lt/state/fanout.h"

#include <deque>
#include <sstream>

#include "lt/core/set.h"

namespace lt::state {

template <typename M>
class Stream
{
   public:
    using state_type = typename M::state_type;
    using input_type = typename M::input_type;
    using value_type = typename M::value_type;
    using output_type = value_type;

    using normalize_function_type =
        std::function<void(std::deque<value_type>&)>;

    Stream(const M& mealy,
           normalize_function_type normalize = nullptr)
        : mealy_(mealy), normalize_(normalize)
    {}

    template <template <typename> class Container>
    Stream(const Mealy<state_type, input_type, Container<value_type>>& mealy,
           normalize_function_type normalize = nullptr,
        std::enable_if<(
            std::is_same_v<M, Mealy<state_type, input_type, Container<value_type>>>
        ), int>* =0)
        : mealy_(mealy), normalize_(normalize)
    {}

    void push_back(const input_type& input)
    {
        auto output = mealy_.step(input);

        if constexpr (lt::core::is_container_v<decltype(output)>) {
            if constexpr(std::is_same_v<typename decltype(output)::value_type, value_type>) {
                // move elements of output onto memory_
                memory_.insert(std::end(memory_),
                        std::make_move_iterator(std::begin(output)),
                        std::make_move_iterator(std::end(output)));
            }
        } else {
            // If output is not a container, just append it
            memory_.emplace_back(output);
        }

        dirty_ = true;
    }

    // Replace failed effects ...
    void push_front(const value_type& value)
    {
        memory_.emplace_front(value);
        dirty_ = true;
    }


    std::optional<value_type> pop_front()
    {
        if (dirty_) {
            normalize();
        }

        if (memory_.empty()) {
            return std::nullopt;
        }

        auto output = memory_.front();
        memory_.pop_front();
        return output;
    }

    void inject(const std::vector<input_type>& inputs)
    {
        for (auto && input : inputs) {
            push_back(input);
        }
    }

    std::vector<value_type> view()
    {
        if (dirty_) {
            normalize();
        }

        auto output = std::vector<value_type>();
        output.assign(std::begin(memory_), std::end(memory_));
        return output;
    }

    std::vector<value_type> run(const std::vector<input_type>& inputs)
    {
        inject(inputs);
        return view();
    }

   private:
    M mealy_;
    normalize_function_type normalize_{nullptr};
    std::deque<value_type> memory_{};
    bool dirty_{false};

    void normalize() {
        if (normalize_) {
            normalize_(memory_);
        }
        dirty_ = false;
    }
};

// A Stream that will reject unknown input messages at runtime
template <typename M>
class RuntimeRejectStream : public Stream<M>
{
   public:
    using state_type = typename M::state_type;
    using input_type = typename M::input_type;
    using value_type = typename M::value_type;
    using output_type = value_type;

    using normalize_function_type = typename Stream<M>::normalize_function_type;

    template <typename... Args>
    RuntimeRejectStream(Args&&... args)
        : Stream<M>(std::forward<Args>(args)...) {}

    template <typename T>
    void push_back(const T& input,
        std::enable_if<(
            lt::core::is_variant_v<input_type>
        ), int>* =0)
    {
        if constexpr (lt::core::is_variant_member_v<T, input_type>) {
            Stream<M>::push_back(input);
        } else {
            std::ostringstream os;
            os << "Stream.push_back("
            << typeid(T).name() << "): not a member of input variant "
            << typeid(input_type).name();
            throw std::runtime_error(os.str());
        }
    }
};

template <typename Output, typename WriteCmd, typename ResetCmd>
inline std::function<void(std::deque<Output>&)> make_normalize_unique()
{
    return [=](std::deque<Output>& output) {
        bool seen = false;
        bool have_reset = false;

        auto it=std::end(output);
        while (it > std::begin(output)) {
            --it;
            std::visit(lt::core::match {
                [&](const WriteCmd& write_cmd) {
                    if (have_reset || seen) {
                        it = output.erase(it);
                    } else {
                        seen = true;
                    }
                },
                [&](const ResetCmd&) {
                    if (have_reset) {
                        it = output.erase(it);
                    }
                    have_reset = true;
                },
                [&](const std::monostate&) {
                    it = output.erase(it);
                },
                [&](auto) {
                }
            }, *it);
        }
    };
}

template <typename Output, typename Key, typename WriteCmd, typename ResetCmd>
inline std::function<void(std::deque<Output>&)> make_normalize_unique_key(
    std::function<Key(const WriteCmd&)> extract)
{
    return [=](std::deque<Output>& output) {
        auto seen = std::set<Key>();
        bool have_reset = false;

        auto it=std::end(output);
        while (it > std::begin(output)) {
            --it;
            std::visit(lt::core::match {
                [&](const WriteCmd& write_cmd) {
                    auto && key = extract(write_cmd);
                    if (have_reset || lt::core::contains(seen, key)) {
                        it = output.erase(it);
                    } else {
                        seen.insert(key);
                    }
                },
                [&](const ResetCmd&) {
                    if (have_reset) {
                        it = output.erase(it);
                    }
                    have_reset = true;
                },
                [&](const std::monostate&) {
                    it = output.erase(it);
                },
                [&](auto) {
                }
            }, *it);
        }
    };
}

template <typename Output>
inline std::function<void(std::deque<Output>&)> chain_normalize(
    const std::vector<std::function<void(std::deque<Output>&)>> fs)
{
    return [=](std::deque<Output>& output) -> void {
        for (auto && f : fs) {
            f(output);
        }
    };
}


} // namespace lt::state
