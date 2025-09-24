#pragma once

#include <unordered_map>

#include "iNVNt/core/math_types.hpp"
#include "iNVNt/core/math_concepts.hpp"
#include "iNVNt/core/dag_graph.hpp"

namespace nvn
{
    struct ITransform3D
    {
        virtual ~ITransform3D() = default;
        virtual translation3d_t translation() const = 0;
        virtual quaternion_t quaternion() const = 0;
        inline transform3d_t transform() const
        {
            transform3d_t T = transform3d_t::Identity();
            T.linear() = this->quaternion().toRotationMatrix();
            T.translation() = this->translation();
            return T;
        }
    };

    template <Translation3DReader ReadP, QuaternionReader ReadR>
    struct Transform3D final : ITransform3D
    {
        using P = std::unwrap_ref_decay_t<ReadP>;
        using R = std::unwrap_ref_decay_t<ReadR>;
        P read_p;
        R read_R;

        template <typename RP, typename RR>
        Transform3D(RP &&p, RR &&R)
            : read_p(std::forward<RP>(p)), read_R(std::forward<RR>(R)) {}

        translation3d_t translation() const override { return read_p(); }
        quaternion_t quaternion() const override { return read_R(); }
    };

    struct IdentityTransform3D final : ITransform3D
    {
        inline static const translation3d_t read_p{0.0, 0.0, 0.0};
        inline static const quaternion_t read_R{1.0, 0.0, 0.0, 0.0}; // w, x, y, z

        translation3d_t translation() const override { return read_p; }
        quaternion_t quaternion() const override { return read_R; }
        inline transform3d_t transform() const { return transform3d_t::Identity(); }
    };

    using TransformTreeGraph = DirectedAcyclicGraph<ITransform3D>;
    using TransformPathSearch = DAGPathSearch<ITransform3D>;
    struct TransformPath final : DAGPathSearchResult<ITransform3D>
    {
        TransformPath() = default;

        TransformPath(DAGPathSearchResult<ITransform3D> &&dag)
            : DAGPathSearchResult(std::forward<DAGPathSearchResult<ITransform3D>>(dag)) {}

        inline transform3d_t operator()() const
        {
            if (up.empty() || down.empty())
                return transform3d_t::Identity(); // change this to something identifiable

            transform3d_t result = transform3d_t::Identity();
            for (auto it = up.cbegin(); it != up.cend() - 1; ++it)
            {
                result = result * (*it)->transform().inverse();
            }
            for (auto it = down.cbegin(); it != down.cend(); ++it)
            {
                result = result * (*it)->transform();
            }
            return result;
        }
    };

    using TransformPathCache = std::unordered_map<std::pair<index_t, index_t>, TransformPath, IndexPairHash>;
    struct TransformTree
    {
        TransformPathSearch search;

        TransformTree(const TransformTreeGraph *tf) : search(tf), cache() {}

        inline TransformPath lookup(index_t A, index_t B)
        {
            TransformPath path;
            if (this->get(A, B, path))
                return path;
            path = search(A, B);
            this->set(A, B, path);
            return path;
        }

    private:
        TransformPathCache cache;

        bool get(index_t A, index_t B, TransformPath &out) const
        {
            auto it = cache.find({A, B});
            if (it != cache.end())
            {
                out = it->second;
                return true;
            }
            return false;
        }

        void set(index_t A, index_t B, const TransformPath &path)
        {
            cache[{A, B}] = path;
        }
    };
}