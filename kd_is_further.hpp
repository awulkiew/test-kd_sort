// Copyright (c) 2014 Adam Wulkiewicz, Lodz, Poland.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_INDEX_DETAIL_KD_IS_FURTHER_HPP
#define BOOST_GEOMETRY_INDEX_DETAIL_KD_IS_FURTHER_HPP

// TODO: change to kd_distance

#include <boost/geometry.hpp>

namespace boost { namespace geometry { namespace index { namespace detail {

template <std::size_t I,
          typename G1, typename G2,
          typename Tag1 = typename geometry::tag<G1>::type,
          typename Tag2 = typename geometry::tag<G2>::type>
struct kd_is_further_impl
{
    BOOST_MPL_ASSERT_MSG(false, NOT_IMPLEMENTED, (G1, G2));
};

template <std::size_t I, typename G1, typename G2>
struct kd_is_further_impl<I, G1, G2, point_tag, point_tag>
{
    template <typename CDist>
    static inline bool apply(G1 const& smaller, G2 const& greater, CDist const& smallest_cdist)
    {
        CDist axis_cdist = geometry::get<I>(greater) - geometry::get<I>(smaller);
        axis_cdist *= axis_cdist;

        // TODO use math::equals()?
        // If it were used in the less() then not here
        return smallest_cdist < axis_cdist;
    }
};

template <std::size_t I, typename G1, typename G2>
struct kd_is_further_impl<I, G1, G2, point_tag, box_tag>
{
    template <typename CDist>
    static inline bool apply(G1 const& smaller, G2 const& greater, CDist const& smallest_cdist)
    {
        CDist axis_cdist = geometry::get<min_corner, I>(greater) - geometry::get<I>(smaller);
        axis_cdist *= axis_cdist;

        // TODO use math::equals()?
        // If it were used in the less() then not here
        return smallest_cdist < axis_cdist;
    }
};

template <std::size_t I, typename G1, typename G2>
struct kd_is_further_impl<I, G1, G2, box_tag, point_tag>
{
    template <typename CDist>
    static inline bool apply(G1 const& smaller, G2 const& greater, CDist const& smallest_cdist)
    {
        CDist axis_cdist = geometry::get<I>(greater) - geometry::get<max_corner, I>(smaller);
        axis_cdist *= axis_cdist;

        // TODO use math::equals()?
        // If it were used in the less() then not here
        return smallest_cdist < axis_cdist;
    }
};

template <std::size_t I, typename G1, typename G2>
struct kd_is_further_impl<I, G1, G2, box_tag, box_tag>
{
    template <typename CDist>
    static inline bool apply(G1 const& smaller, G2 const& greater, CDist const& smallest_cdist)
    {
        CDist axis_cdist
            = geometry::get<max_corner, I>(smaller) < geometry::get<min_corner, I>(greater) ?
                geometry::get<min_corner, I>(greater) - geometry::get<max_corner, I>(smaller) :
                0;
        axis_cdist *= axis_cdist;

        // TODO use math::equals()?
        // If it were used in the less() then not here
        return smallest_cdist < axis_cdist;
    }
};

template <std::size_t I, typename G1, typename G2, typename CDist>
inline bool kd_is_further(G1 const& l, G2 const& r, CDist const& smallest_cdist)
{
    return kd_is_further_impl<I, G1, G2>::apply(l, r, smallest_cdist);
}

}}}} // namespace boost::geometry::index::detail

#endif // BOOST_GEOMETRY_INDEX_DETAIL_KD_IS_FURTHER_HPP
