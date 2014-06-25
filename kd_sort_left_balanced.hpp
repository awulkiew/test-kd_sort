// Copyright (c) 2014 Adam Wulkiewicz, Lodz, Poland.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#ifndef BOOST_GEOMETRY_INDEX_DETAIL_KD_SORT_LEFT_BALANCED_HPP
#define BOOST_GEOMETRY_INDEX_DETAIL_KD_SORT_LEFT_BALANCED_HPP

#include <algorithm>
#include "kd_less.hpp"
#include "kd_is_further.hpp"

namespace boost { namespace geometry { namespace index { namespace detail {

// ---------------------------------------------------------------------- //

template <typename Point, std::size_t I = 0>
struct kd_sort_left_balanced_impl
{
    static const std::size_t next_dimension = (I+1) % dimension<Point>::value;

    // the range of elements is [start, stop] - inclusive!
    // for the C++ range defined as [0, count) the values must be [1, count]
    static inline std::size_t calc_median(std::size_t start, std::size_t stop)
    {
        std::size_t const size = stop - start + 1;

        std::size_t median = 1;
        while ( 4 * median <= size )
        {
            median += median;
        }
        if ( 3 * median <= size )
        {
            median += median;
            median += start - 1;
        }
        else
        {
            median = stop - median + 1;
        }

        return median;
    }

    template <typename It, typename OutIt>
    static inline void apply(It first, It last,
                             std::size_t start, std::size_t stop,
                             std::size_t index, OutIt out_first)
    {
        std::size_t median = calc_median(start, stop);
        It median_it = first + (median - start);

        std::nth_element(first, median_it, last, kd_less<I, Point, Point>);

        *(out_first + (index - 1)) = *median_it;

        if ( start < median )
        {
            std::size_t const new_stop = median - 1;
            if ( start < new_stop )
            {
                kd_sort_left_balanced_impl<Point, next_dimension>
                    ::apply(first, median_it, start, new_stop, 2 * index, out_first);
            }
            else
            {
                *(out_first + (2 * index - 1)) = *first;
            }
        }

        if ( median < stop )
        {
            std::size_t const new_start = median + 1;
            if ( new_start < stop  )
            {
                kd_sort_left_balanced_impl<Point, next_dimension>
                    ::apply(median_it + 1, last, new_start, stop, 2 * index + 1, out_first);
            }
            else
            {
                *(out_first + (2 * index /*+ 1 - 1*/)) = *(last - 1);
            }
        }
    }
};

template <typename RandomIt>
inline void kd_sort_left_balanced(RandomIt first, RandomIt last)
{
    typedef typename boost::iterator_value<RandomIt>::type point_type;
    typename boost::iterator_difference<RandomIt>::type
        count = std::distance(first, last);
    if ( count > 1 )
    {
        std::vector<point_type> temp(first, last);
        kd_sort_left_balanced_impl<point_type>
            ::apply(temp.begin(), temp.end(), 1, count, 1, first);
    }
}

// ---------------------------------------------------------------------- //

template <typename Point, std::size_t I = 0>
struct kd_nearest_left_balanced_impl
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
    static inline bool per_branch(It first,
                                  std::size_t index, std::size_t max_index,
                                  Value const& point,
                                  It & out_it, CDist & smallest_cdist)
    {
        return kd_nearest_left_balanced_impl<Point, next_dimension>
                    ::apply(first, index, max_index, point, out_it, smallest_cdist);
    }

    template <typename It, typename Value, typename CDist>
    static inline bool apply(It first,
                             std::size_t index, std::size_t const max_index,
                             Value const& point,
                             It & out_it, CDist & smallest_cdist)
    {
        It nth = first + index - 1;

        if ( update_one(nth, point, out_it, smallest_cdist) )
            return true;

        if ( kd_less<I>(point, *nth) )
        {
            std::size_t next_index = 2 * index;
            if ( next_index > max_index )
                return false;

            if ( per_branch(first, next_index, max_index, point, out_it, smallest_cdist) )
                return true;

            ++next_index;
            if ( next_index > max_index )
                return false;

            if ( kd_is_further<I>(point, *nth, smallest_cdist) )
                return false;

            return per_branch(first, 2 * index + 1, max_index, point, out_it, smallest_cdist);
        }
        else if ( kd_less<I>(*nth, point) )
        {
            std::size_t next_index = 2 * index + 1;
            if ( next_index <= max_index )
                if ( per_branch(first, 2 * index + 1, max_index, point, out_it, smallest_cdist) )
                    return true;

            --next_index;
            if ( next_index > max_index )
                return false;

            if ( kd_is_further<I>(*nth, point, smallest_cdist) )
                return false;

            return per_branch(first, 2 * index, max_index, point, out_it, smallest_cdist);
        }
        else
        {
            std::size_t next_index = 2 * index;
            if ( next_index > max_index )
                return false;

            if ( per_branch(first, next_index, max_index, point, out_it, smallest_cdist) )
                return true;

            ++next_index;
            if ( next_index > max_index )
                return false;

            return per_branch(first, next_index, max_index, point, out_it, smallest_cdist);
        }

        return false;
    }
};

template <typename RandomIt, typename Point, typename Value>
inline bool kd_nearest_left_balanced(RandomIt first, RandomIt last, Point const& point, Value & result)
{
    typename boost::iterator_difference<RandomIt>::type
        d = std::distance(first, last);

    if ( d < 1 )
        return false;

    std::size_t size = static_cast<std::size_t>(d);

    typedef typename boost::iterator_value<RandomIt>::type point_type;

    typename geometry::default_comparable_distance_result<point_type>::type
        cdist = geometry::comparable_distance(point, *first);
    RandomIt out_it = first;

    kd_nearest_left_balanced_impl<point_type>
        ::apply(first, 1, size, point, out_it, cdist);

    result = *out_it;

    return true;
}

// ---------------------------------------------------------------------- //

}}}} // namespace boost::geometry::index::detail

#endif // BOOST_GEOMETRY_INDEX_DETAIL_KD_SORT_LEFT_BALANCED_HPP
