// Copyright (c) 2014 Adam Wulkiewicz, Lodz, Poland.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_INDEX_DETAIL_KD_LESS_HPP
#define BOOST_GEOMETRY_INDEX_DETAIL_KD_LESS_HPP

#include <boost/geometry.hpp>

namespace boost { namespace geometry { namespace index { namespace detail {

template <std::size_t I,
          typename G1, typename G2,
          typename Tag1 = typename geometry::tag<G1>::type,
          typename Tag2 = typename geometry::tag<G2>::type>
struct kd_less_impl
{
    BOOST_MPL_ASSERT_MSG(false, NOT_IMPLEMENTED, (G1, G2));
};

template <std::size_t I, typename G1, typename G2>
struct kd_less_impl<I, G1, G2, point_tag, point_tag>
{
    static inline bool apply(G1 const& l, G2 const& r)
    {
        // TODO use math::equals()?
        // If yes, then this function may return pair<lesser, greater>
        return geometry::get<I>(l) < geometry::get<I>(r);
    }
};

template <std::size_t I, typename G1, typename G2>
struct kd_less_impl<I, G1, G2, point_tag, box_tag>
{
    static inline bool apply(G1 const& l, G2 const& r)
    {
        return geometry::get<I>(l) < geometry::get<min_corner, I>(r);
    }
};

template <std::size_t I, typename G1, typename G2>
struct kd_less_impl<I, G1, G2, box_tag, point_tag>
{
    static inline bool apply(G1 const& l, G2 const& r)
    {
        return geometry::get<max_corner, I>(l) < geometry::get<I>(r);
    }
};

template <std::size_t I, typename G1, typename G2>
struct kd_less_impl<I, G1, G2, box_tag, box_tag>
{
    static inline bool apply(G1 const& l, G2 const& r)
    {
        return geometry::get<max_corner, I>(l) < geometry::get<min_corner, I>(r)
            || ( geometry::get<min_corner, I>(l) < geometry::get<min_corner, I>(r)
              && geometry::get<max_corner, I>(l) < geometry::get<max_corner, I>(r) )
            || ( geometry::get<min_corner, I>(l) + geometry::get<max_corner, I>(l) )/* * 0.5f*/
                    < ( geometry::get<min_corner, I>(r) + geometry::get<max_corner, I>(r) )/* * 0.5f*/;

        // take the width into account?
    }
};

template <std::size_t I, typename G1, typename G2>
inline bool kd_less(G1 const& l, G2 const& r)
{
    return kd_less_impl<I, G1, G2>::apply(l, r);
}

}}}} // namespace boost::geometry::index::detail

#endif // BOOST_GEOMETRY_INDEX_DETAIL_KD_LESS_HPP
