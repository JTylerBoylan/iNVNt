#pragma once

#include "iNVNt/core/func_concepts.hpp"

namespace nvn
{
    template <typename F1, typename F2>
    struct Chain
    {
        using T1 = std::unwrap_ref_decay_t<F1>;
        using T2 = std::unwrap_ref_decay_t<F2>;

        T1 func1;
        T2 func2;

        template <typename G1, typename G2>
        Chain(G1 &&f1, G2 &&f2)
            : func1(std::forward<G1>(f1)),
              func2(std::forward<G2>(f2)) {}

        template <typename... A>
            requires Chainable<T1, T2, A...>
        decltype(auto) operator()(A &&...args)
        {
            return std::invoke(func2, std::invoke(func1, std::forward<A>(args)...));
        }
    };

    template <typename G1, typename G2>
    Chain(G1 &&, G2 &&) -> Chain<G1, G2>; // CTAD: let `Chain(f1,f2)` deduce to decayed types

    template <typename Cond, typename Then, typename Else>
    struct Branch
    {
        using C = std::unwrap_ref_decay_t<Cond>;
        using T = std::unwrap_ref_decay_t<Then>;
        using E = std::unwrap_ref_decay_t<Else>;

        C cond;
        T then_;
        E else_;

        template <typename C0, typename T0, typename E0>
        Branch(C0 &&c, T0 &&t, E0 &&e)
            : cond(std::forward<C0>(c)), then_(std::forward<T0>(t)), else_(std::forward<E0>(e)) {}

        template <typename... A>
            requires Branchable<C, T, E, A...>
        decltype(auto) operator()(A &&...a)
        {
            if constexpr (std::same_as<result_t<T, A...>, void> &&
                          std::same_as<result_t<E, A...>, void>)
            {
                if (std::invoke(cond, a...))
                    (void)std::invoke(then_, std::forward<A>(a)...);
                else
                    (void)std::invoke(else_, std::forward<A>(a)...);
                return;
            }
            else
            {
                using R = std::common_type_t<result_t<T, A...>, result_t<E, A...>>;
                if (std::invoke(cond, a...))
                    return static_cast<R>(std::invoke(then_, std::forward<A>(a)...));
                else
                    return static_cast<R>(std::invoke(else_, std::forward<A>(a)...));
            }
        }
    };

    // CTAD
    template <class C, class T, class E>
    Branch(C &&, T &&, E &&) -> Branch<C, T, E>;

    template <typename T>
    struct ReadWrite
    {
        T value;

        ReadWrite() = default;
        explicit ReadWrite(const T &v) : value(v) {}
        explicit ReadWrite(T &&v) : value(std::move(v)) {}

        inline T &operator()() noexcept { return value; }
        inline const T &operator()() const noexcept { return value; }

        inline void operator()(const T &v) { value = v; }
        inline void operator()(T &&v) { value = std::move(v); }
    };
}