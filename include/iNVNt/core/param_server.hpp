#pragma once

#include <string>
#include <memory>
#include <unordered_map>
#include <typeinfo>
#include <type_traits>
#include <stdexcept>

namespace nvn
{

    struct IParam
    {
        virtual ~IParam() = default;
        virtual const std::type_info &type() const noexcept = 0;
    };

    template <typename T>
    struct TypedParam final : IParam
    {
        T value;

        TypedParam() = default;
        explicit TypedParam(const T &v) : value(v) {}
        explicit TypedParam(T &&v) : value(std::move(v)) {}

        // Getters/setters
        inline T &operator()() noexcept { return value; }
        inline const T &operator()() const noexcept { return value; }

        inline void operator()(const T &v) { value = v; }
        inline void operator()(T &&v) { value = std::move(v); }

        const std::type_info &type() const noexcept override { return typeid(T); }
    };

    struct ParameterServer
    {
        std::unordered_map<std::string, std::unique_ptr<IParam>> params;

        ParameterServer() = default;

        // Untyped access
        IParam *getParam(const std::string &key) noexcept
        {
            auto it = params.find(key);
            return (it != params.end()) ? it->second.get() : nullptr;
        }
        const IParam *getParam(const std::string &key) const noexcept
        {
            auto it = params.find(key);
            return (it != params.end()) ? it->second.get() : nullptr;
        }

        // Typed access
        template <typename T>
        TypedParam<T> *getParam(const std::string &key) noexcept
        {
            if (auto *p = getParam(key))
            {
                return dynamic_cast<TypedParam<T> *>(p);
            }
            return nullptr;
        }
        template <typename T>
        const TypedParam<T> *getParam(const std::string &key) const noexcept
        {
            if (auto *p = getParam(key))
            {
                return dynamic_cast<const TypedParam<T> *>(p);
            }
            return nullptr;
        }

        // Get value into an out param
        template <typename T>
        bool getParamValue(const std::string &key, T &out) const
        {
            if (auto *p = getParam<T>(key))
            {
                out = p->value;
                return true;
            }
            return false;
        }

        // Set/insert a fully-typed param object
        template <typename Param>
            requires std::derived_from<Param, IParam>
        Param *setParam(const std::string &key, std::unique_ptr<Param> p)
        {
            auto *raw = p.get();
            params[key] = std::move(p); // insert or replace
            return raw;
        }

        // Set/replace just the value (type-safe)
        template <typename T>
        TypedParam<T> *setParamValue(const std::string &key, const T &value)
        {
            if (auto it = params.find(key); it != params.end())
            {
                IParam *base = it->second.get();
                if (base->type() != typeid(T))
                {
                    throw std::runtime_error("ParameterServer: type mismatch for key '" + key + "'");
                }
                auto *tp = dynamic_cast<TypedParam<T> *>(base);
                if (!tp)
                {
                    throw std::runtime_error("ParameterServer: bad cast for key '" + key + "'");
                }
                tp->operator()(value);
                return tp;
            }
            return setParam(key, std::make_unique<TypedParam<T>>(value));
        }

        template <typename T>
        TypedParam<T> *setParamValue(const std::string &key, T &&value)
        {
            if (auto it = params.find(key); it != params.end())
            {
                IParam *base = it->second.get();
                if (base->type() != typeid(T))
                {
                    throw std::runtime_error("ParameterServer: type mismatch for key '" + key + "'");
                }
                auto *tp = dynamic_cast<TypedParam<T> *>(base);
                if (!tp)
                {
                    throw std::runtime_error("ParameterServer: bad cast for key '" + key + "'");
                }
                tp->operator()(std::move(value));
                return tp;
            }
            return setParam(key, std::make_unique<TypedParam<T>>(std::move(value)));
        }

        template <typename T>
        T &at(const std::string &key)
        {
            auto *tp = getParam<T>(key);
            if (!tp)
                throw std::runtime_error("ParameterServer: missing or wrong type for key '" + key + "'");
            return tp->value;
        }
        template <typename T>
        const T &at(const std::string &key) const
        {
            auto *tp = getParam<T>(key);
            if (!tp)
                throw std::runtime_error("ParameterServer: missing or wrong type for key '" + key + "'");
            return tp->value;
        }
    };

} // namespace nvn