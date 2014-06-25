// Copyright (c) 2014 Adam Wulkiewicz, Lodz, Poland.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_INDEX_DETAIL_KD_SORT_HPP
#define BOOST_GEOMETRY_INDEX_DETAIL_KD_SORT_HPP

#include <algorithm>

#include "kd_less.hpp"
#include "kd_is_further.hpp"

namespace boost { namespace geometry { namespace index { namespace detail {

// ---------------------------------------------------------------------- //

#define BOOST_GEOMETRY_INDEX_DETAIL_KD_SORT_VALUES_MIN 8
#if BOOST_GEOMETRY_INDEX_DETAIL_KD_SORT_VALUES_MIN < 1
#error "invalid value"
#endif

// ---------------------------------------------------------------------- //

template <typename Point, std::size_t I = 0>
struct kd_sort_impl
{
    static const std::size_t next_dimension = (I+1) % dimension<Point>::value;

    template <typename It>
    static inline void apply(It first, It last)
    {
        std::size_t size = static_cast<std::size_t>(std::distance(first, last));
        std::size_t lsize = size / 2;
        std::size_t rsize = size - lsize - 1;
        
        It nth = first + lsize;
        std::nth_element(first, nth, last, kd_less<I, Point, Point>);

        if ( lsize > BOOST_GEOMETRY_INDEX_DETAIL_KD_SORT_VALUES_MIN )
        {
            kd_sort_impl<Point, next_dimension>::apply(first, nth);
        }
        if ( rsize > BOOST_GEOMETRY_INDEX_DETAIL_KD_SORT_VALUES_MIN )
        {
            kd_sort_impl<Point, next_dimension>::apply(nth+1, last);
        }
    }
};

template <typename RandomIt>
inline void kd_sort(RandomIt first, RandomIt last)
{
    typedef typename boost::iterator_value<RandomIt>::type point_type;
    if ( std::distance(first, last) > 1 )
    {
        kd_sort_impl<point_type>::apply(first, last);
    }
}

// ---------------------------------------------------------------------- //

template <typename Point, std::size_t I = 0>
struct kd_binary_search_impl
{
    static const std::size_t next_dimension = (I+1) % dimension<Point>::value;

    template <typename It, typename Value>
    static inline bool per_branch(It first, It last, Value const& value)
    {
        std::size_t size = static_cast<std::size_t>(std::distance(first, last));

        if ( size > BOOST_GEOMETRY_INDEX_DETAIL_KD_SORT_VALUES_MIN )
        {
            return kd_binary_search_impl<Point, next_dimension>::apply(first, last, value);
        }
        else
        {
            for ( ; first != last ; ++first )
            {
                if ( geometry::equals(*first, value) )
                    return true;
            }
        }

        return false;
    }

    template <typename It, typename Value>
    static inline bool apply(It first, It last, Value const& value)
    {
        std::size_t size = static_cast<std::size_t>(std::distance(first, last));
        std::size_t lsize = size / 2;

        It nth = first + lsize;

        if ( geometry::equals(*nth, value) )
            return true;

        if ( size == 1 ) // probably not needed
            return false;

        if ( kd_less<I>(value, *nth) )
        {
            return per_branch(first, nth, value);
        }
        else if ( kd_less<I>(*nth, value) )
        {
            return per_branch(nth+1, last, value);
        }
        else
        {
            return per_branch(first, nth, value)
                || per_branch(nth+1, last, value);
        }
    }
};

template <typename RandomIt, typename Value>
inline bool kd_binary_search(RandomIt first, RandomIt last, Value const& value)
{
    if ( std::distance(first, last) < 1 )
        return false;

    typedef typename boost::iterator_value<RandomIt>::type point_type;
    return kd_binary_search_impl<point_type>::apply(first, last, value);
}

// ---------------------------------------------------------------------- //

template <typename Point, std::size_t I = 0>
struct kd_nearest_impl
{
    static const std::size_t next_dimension = (I+1) % dimension<Point>::value;

    template <typename It, typename Value, typename CDist>
    static inline bool update_one(It it, Value const& point, It & out_it, CDist & smallest_cdist)
    {
        CDist cdist = geometry::comparable_distance(point, *it);
        if ( cdist < smallest_cdist )
        {
            smallest_cdist = cdist;
            out_it = it;
        }

        return math::equals(smallest_cdist, CDist(0));
    }

    template <typename It, typename Value, typename CDist>
    static inline bool per_branch(It first, It last, Value const& point, It & out_it, CDist & smallest_cdist)
    {
        std::size_t size = static_cast<std::size_t>(std::distance(first, last));

        if ( size > BOOST_GEOMETRY_INDEX_DETAIL_KD_SORT_VALUES_MIN )
        {
            if ( kd_nearest_impl<Point, next_dimension>::apply(first, last, point, out_it, smallest_cdist) )
                return true;
        }
        else
        {
            for ( ; first != last ; ++first )
            {
                if ( update_one(first, point, out_it, smallest_cdist) )
                    return true;
            }
        }

        return false;
    }

    template <typename It, typename Value, typename CDist>
    static inline bool apply(It first, It last, Value const& point, It & out_it, CDist & smallest_cdist)
    {
        std::size_t size = static_cast<std::size_t>(std::distance(first, last));
        std::size_t lsize = size / 2;
        It nth = first + lsize;

        if ( update_one(nth, point, out_it, smallest_cdist) )
            return true;

        if ( kd_less<I>(point, *nth) )
        {
            if ( per_branch(first, nth, point, out_it, smallest_cdist) )
                return true;

            if ( kd_is_further<I>(point, *nth, smallest_cdist) )
                return false;

            return per_branch(nth+1, last, point, out_it, smallest_cdist);
        }
        else if ( kd_less<I>(*nth, point) )
        {
            if ( per_branch(nth+1, last, point, out_it, smallest_cdist) )
                return true;

            if ( kd_is_further<I>(*nth, point, smallest_cdist) )
                return false;

            return per_branch(first, nth, point, out_it, smallest_cdist);
        }
        else
        {
            if ( per_branch(first, nth, point, out_it, smallest_cdist) )
                return true;

            return per_branch(nth+1, last, point, out_it, smallest_cdist);
        }

        return false;
    }
};

template <typename RandomIt, typename Point, typename Value>
inline bool kd_nearest(RandomIt first, RandomIt last, Point const& point, Value & result)
{
    if ( std::distance(first, last) < 1 )
        return false;

    typedef typename boost::iterator_value<RandomIt>::type point_type;

    typename geometry::default_comparable_distance_result<point_type>::type
        cdist = geometry::comparable_distance(point, *first);
    RandomIt out_it = first;
    
    kd_nearest_impl<point_type>::apply(first, last, point, out_it, cdist);

    result = *out_it;

    return true;
}

// ---------------------------------------------------------------------- //

}}}} // namespace boost::geometry::index::detail

#endif // BOOST_GEOMETRY_INDEX_DETAIL_KD_SORT_HPP
