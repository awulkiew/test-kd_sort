// Copyright (c) 2014 Adam Wulkiewicz, Lodz, Poland.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_INDEX_DETAIL_KD_LESS_HPP
#define BOOST_GEOMETRY_INDEX_DETAIL_KD_LESS_HPP

#include <boost/geometry.hpp>

namespace boost { namespace geometry { namespace index { namespace detail {

template <std::size_t I, typename Geometry, typename Tag = typename geometry::tag<Geometry>::type>
struct kd_less_impl
{
    BOOST_MPL_ASSERT_MSG(false, NOT_IMPLEMENTED, (Geometry));
};

template <std::size_t I, typename Geometry>
struct kd_less_impl<I, Geometry, point_tag>
{
    static inline bool apply(Geometry const& l, Geometry const& r)
    {
        // TODO use math::equals()?
        // If yes, then this function may return pair<lesser, greater>
        return geometry::get<I>(l) < geometry::get<I>(r);
    }
};

template <std::size_t I, typename Geometry>
struct kd_less_impl<I, Geometry, box_tag>
{
    static inline bool apply(Geometry const& l, Geometry const& r)
    {
        return geometry::get<1, I>(l) < geometry::get<0, I>(r)                  // lmax < rmin
            || ( geometry::get<0, I>(l) < geometry::get<0, I>(r)
              && geometry::get<1, I>(l) < geometry::get<1, I>(r) )              // lmin < rmin && lmax < rmax
            || ( geometry::get<0, I>(l) + geometry::get<1, I>(l) )/* * 0.5f*/
                    < ( geometry::get<0, I>(r) + geometry::get<1, I>(r) )/* * 0.5f*/;  // lcenter < rcenter

        // take the width into account?
    }
};

template <std::size_t I, typename Geometry>
inline bool kd_less(Geometry const& l, Geometry const& r)
{
    return kd_less_impl<I, Geometry>::apply(l, r);
}

}}}} // namespace boost::geometry::index::detail

#endif // BOOST_GEOMETRY_INDEX_DETAIL_KD_LESS_HPP
