#pragma once

#include <functional>

#include "lt/core/set.h"
#include "lt/core/variant.h"

namespace lt::state {

// Property: Only one attempt should be made to write
template <typename Input, typename Output, typename WriteCmd>
std::function<bool(std::vector<Input> input, std::vector<Output> output)>
    prop_unique_write()
{
    return [=](std::vector<Input> input, std::vector<Output> output) -> bool {
        bool written = false;

        for (auto && o : output) {
            bool ok = std::visit(lt::core::match {
                [&](const WriteCmd& write_cmd) {
                    if (written) {
                        return false;
                    } else {
                        written = true;
                        return true;
                    }
                },

                [&](auto) {
                    return true;
                },
            }, o);
            if (!ok) {
                return false;
            }
        }

        return true;
    };
}

// Property: Only one attempt should be made to write each key
template <typename Input, typename Output, typename Key, typename WriteCmd>
std::function<bool(std::vector<Input> input, std::vector<Output> output)>
    prop_unique_write_key(std::function<Key(const WriteCmd&)> extract)
{
    return [=](std::vector<Input> input, std::vector<Output> output) -> bool {
        auto written = std::set<Key>();

        for (auto && o : output) {
            bool ok = std::visit(lt::core::match {
                [&](const WriteCmd& write_cmd) {
                    auto && key = extract(write_cmd);
                    if (lt::core::contains(written, key)) {
                        return false;
                    } else {
                        written.insert(key);
                        return true;
                    }
                },

                [&](auto) {
                    return true;
                },
            }, o);
            if (!ok) {
                return false;
            }
        }

        return true;
    };
}

// Property: The write is the last appearing in the input
template <typename Input, typename Output, typename WriteCmd>
std::function<bool(std::vector<Input> input, std::vector<Output> output)>
    prop_last_write()
{
    return [=](std::vector<Input> input, std::vector<Output> output) -> bool {
        for (auto && o : output) {
            bool ok = std::visit(lt::core::match {
                [&](const WriteCmd& write_cmd) {
                    auto found = std::find_if(input.rbegin(), input.rend(),
                        [&](const auto& i) {
                            return std::visit(lt::core::match {
                                [&](const WriteCmd& w) {
                                    return true;
                                },
                                [](auto) {
                                    return false;
                                }
                            }, i);
                        });
                    if (found == input.rend()) {
                        return false;
                    }
                    auto && w = std::get<WriteCmd>(*found);
                    return w == write_cmd;
                },

                [&](auto) {
                    return true;
                },
            }, o);
            if (!ok) {
                return false;
            }
        }

        return true;
    };
}

// Property: The write is the last appearing in the input for that key
template <typename Input, typename Output, typename Key, typename WriteCmd>
std::function<bool(std::vector<Input> input, std::vector<Output> output)>
    prop_last_write_key(std::function<Key(const WriteCmd&)> extract)
{
    return [=](std::vector<Input> input, std::vector<Output> output) -> bool {
        for (auto && o : output) {
            bool ok = std::visit(lt::core::match {
                [&](const WriteCmd& write_cmd) {
                    auto && key = extract(write_cmd);
                    auto found = std::find_if(input.rbegin(), input.rend(),
                        [&](const auto& i) {
                            return std::visit(lt::core::match {
                                [&](const WriteCmd& w) {
                                    return extract(w) == key;
                                },
                                [](auto) {
                                    return false;
                                }
                            }, i);
                        });
                    if (found == input.rend()) {
                        return false;
                    }
                    auto && w = std::get<WriteCmd>(*found);
                    return w == write_cmd;
                },

                [&](auto) {
                    return true;
                },
            }, o);
            if (!ok) {
                return false;
            }
        }

        return true;
    };
}

// Property: Only one Reset may appear, and no Writes may appear before it
template <typename Input, typename Output, typename WriteCmd, typename ResetCmd>
std::function<bool(std::vector<Input> input, std::vector<Output> output)>
    prop_unique_reset()
{
    return [=](std::vector<Input> input, std::vector<Output> output) -> bool {
        bool seen_reset = false;
        bool seen_write = false;

        for (auto && o : output) {
            bool ok = std::visit(lt::core::match {
                [&](const ResetCmd&) {
                    bool ret = !seen_reset && !seen_write;
                    seen_reset = true;
                    return ret;
                },

                [&](const WriteCmd&) {
                    seen_write = true;
                    return true;
                },

                [&](auto) {
                    return true;
                },
            }, o);
            if (!ok) {
                return false;
            }
        }

        return true;
    };
}

// Property: Normalized stream should contain no {}
template <typename Input, typename Output>
std::function<bool(std::vector<Input> input, std::vector<Output> output)>
    prop_no_monostate()
{
    return [=](std::vector<Input> input, std::vector<Output> output) -> bool {
        for (auto && o : output) {
            bool ok = std::visit(lt::core::match {
                [&](const std::monostate&) {
                    return false;
                },

                [&](auto) {
                    return true;
                },
            }, o);
            if (!ok) {
                return false;
            }
        }

        return true;
    };
}

} // namespace lt::state
