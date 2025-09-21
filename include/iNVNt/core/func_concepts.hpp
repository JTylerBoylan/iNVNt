#pragma once

#include <type_traits>
#include <concepts>
#include <utility>
#include <functional>
#include <string>
#include <string_view>

namespace nvn
{
    template <class F, class... A>
    using result_t = std::invoke_result_t<F &, A...>;

    template <class F, class... A>
    concept Callable =
        std::invocable<F &, A...>;

    template <class T, class R>
    concept ReturnMatches =
        (std::same_as<R, void> && std::same_as<T, void>) ||
        (!std::same_as<R, void> && std::convertible_to<T, R>);

    template <class F, class R, class... A>
    concept Signature =
        Callable<F, A...> && ReturnMatches<result_t<F, A...>, R>;

    template <class F, class R, class... A>
    concept Reader =
        Signature<F, R, A...>;

    template <class F, class T>
    concept Writer =
        (Callable<F, T> || Callable<F, const T &>);

    template <class F>
    concept StringWriter =
        Writer<F, std::string> || Writer<F, std::string_view>;

    template <class F, class... A>
    concept Predicate =
        Signature<F, bool, A...>;

    template <class F, class G, class... A>
    concept Chainable =
        Callable<F, A...> &&
        requires(F &f, G &g, A &&...a) {
            std::invoke(g, std::invoke(f, std::forward<A>(a)...));
        };

    template <class Then, class Else, class... A>
    concept CompatibleBranchResults =
        (std::same_as<result_t<Then, A...>, void> &&
         std::same_as<result_t<Else, A...>, void>) ||
        requires {
            typename std::common_type_t<
                result_t<Then, A...>,
                result_t<Else, A...>>;
        };

    template <class Cond, class Then, class Else, class... A>
    concept Branchable =
        Predicate<Cond, A...> &&
        Callable<Then, A...> &&
        Callable<Else, A...> &&
        CompatibleBranchResults<Then, Else, A...>;
}