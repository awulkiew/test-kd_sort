// Copyright (c) 2014 Adam Wulkiewicz, Lodz, Poland.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#include <iostream>
#include <algorithm>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <boost/random.hpp>

namespace boost { namespace geometry { namespace index { namespace detail {

#define BOOST_GEOMETRY_INDEX_DETAIL_KD_SORT_VALUES_MIN 8
#if BOOST_GEOMETRY_INDEX_DETAIL_KD_SORT_VALUES_MIN < 1
#error "invalid value"
#endif

template <typename Point, std::size_t I = 0>
struct kd_sort_impl
{
    static inline bool less(Point const& l, Point const& r)
    {
        return geometry::get<I>(l) < geometry::get<I>(r);
    }

    template <typename It>
    static inline void apply(It first, It last)
    {
        std::size_t size = static_cast<std::size_t>(std::distance(first, last));
        std::size_t lsize = size / 2;
        std::size_t rsize = size - lsize - 1;
        
        It nth = first + lsize;
        std::nth_element(first, nth, last, less);
        
        static const std::size_t next_dimension = (I+1) % dimension<Point>::value;
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

template <typename Point, std::size_t I = 0>
struct kd_binary_search_impl
{
    static const std::size_t next_dimension = (I+1) % dimension<Point>::value;

    static inline bool less(Point const& l, Point const& r)
    {
        return geometry::get<I>(l) < geometry::get<I>(r);
    }

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

        if ( less(value, *nth) )
        {
            return per_branch(first, nth, value);
        }
        else if ( less(*nth, value) )
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

template <typename Point, std::size_t I = 0>
struct kd_nearest_impl
{
    static const std::size_t next_dimension = (I+1) % dimension<Point>::value;

    static inline bool less(Point const& l, Point const& r)
    {
        return geometry::get<I>(l) < geometry::get<I>(r);
    }

    template <typename Result>
    static inline Result comparable_distance(Point const& l, Point const& r)
    {
        Result temp = geometry::get<I>(l) > geometry::get<I>(r) ?
                      geometry::get<I>(l) - geometry::get<I>(r) :
                      geometry::get<I>(r) - geometry::get<I>(l);
        return temp * temp;
    }

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

    template <typename Value, typename CDist>
    static inline bool is_axis_further(Value const& smaller, Value const& greater, CDist & smallest_cdist)
    {
        CDist axis_cdist = geometry::get<I>(greater) - geometry::get<I>(smaller);
        axis_cdist *= axis_cdist;

        return smallest_cdist < axis_cdist; // use math::equals?
    }

    template <typename It, typename Value, typename CDist>
    static inline bool apply(It first, It last, Value const& point, It & out_it, CDist & smallest_cdist)
    {
        std::size_t size = static_cast<std::size_t>(std::distance(first, last));
        std::size_t lsize = size / 2;
        It nth = first + lsize;

        if ( update_one(nth, point, out_it, smallest_cdist) )
            return true;

        if ( less(point, *nth) )
        {
            if ( per_branch(first, nth, point, out_it, smallest_cdist) )
                return true;

            if ( is_axis_further(point, *nth, smallest_cdist) )
                return false;

            return per_branch(nth+1, last, point, out_it, smallest_cdist);
        }
        else if ( less(*nth, point) )
        {
            if ( per_branch(nth+1, last, point, out_it, smallest_cdist) )
                return true;

            if ( is_axis_further(*nth, point, smallest_cdist) )
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

template <typename RandomIt, typename Value>
inline bool kd_nearest(RandomIt first, RandomIt last, Value const& point, Value & result)
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

}}}} // namespace boost::geometry::index::detail

int main()
{
    namespace bg = boost::geometry;
    namespace bgi = bg::index;
    typedef boost::chrono::thread_clock clock_t;
    typedef boost::chrono::duration<float> dur_t;

    size_t values_count = 1000000;
    //size_t values_count = 100;

    std::vector< std::pair<float, float> > coords;

    //randomize values
    {
        boost::mt19937 rng;
        //rng.seed(static_cast<unsigned int>(std::time(0)));
        boost::uniform_real<float> range(-1000, 1000);
        boost::variate_generator<boost::mt19937&, boost::uniform_real<float> > rnd(rng, range);

        coords.reserve(values_count);

        std::cout << "randomizing data\n";
        for ( size_t i = 0 ; i < values_count ; ++i )
        {
            coords.push_back(std::make_pair(rnd(), rnd()));
        }
        std::cout << "randomized\n";
    }

    typedef bg::model::point<double, 2, bg::cs::cartesian> P;

    for (;;)
    {
        typedef std::pair<float, float> pt_data;

        std::vector<P> v1, v2;
        
        {
            v1.reserve(values_count);
            v2.reserve(values_count);
            BOOST_FOREACH(pt_data const& c, coords)
            {
                v1.push_back(P(c.first, c.second));
                v2.push_back(P(c.first, c.second));
            }
        }

        std::cout << "------------------------------------------------" << std::endl;

        clock_t::time_point start = clock_t::now();
        bgi::rtree<P, bgi::linear<8> > rt(v1.begin(), v1.end());
        dur_t time = clock_t::now() - start;
        std::cout << time << " - rtree()" << std::endl;

        {
            clock_t::time_point start = clock_t::now();
            std::sort(v1.begin(), v1.end(), bg::less<P>());
            dur_t time = clock_t::now() - start;
            std::cout << time << " - std::sort()" << std::endl;
        }

        {
            clock_t::time_point start = clock_t::now();
            bgi::detail::kd_sort(v2.begin(), v2.end());
            dur_t time = clock_t::now() - start;
            std::cout << time << " - kd_sort()" << std::endl;
        }

        std::cout << "------------------------------------------------" << std::endl;

        {
            std::size_t dummy = 0;
            clock_t::time_point start = clock_t::now();
            BOOST_FOREACH(pt_data const& c, coords)
            {
                bool is = rt.count(P(c.first, c.second)) > 0;

                dummy += int(is);
            }
            dur_t time = clock_t::now() - start;
            std::cout << time << " - rtree::count()" << std::endl;
            std::cout << "dummy: " << dummy << std::endl;
        }

        {
            std::size_t dummy = 0;
            clock_t::time_point start = clock_t::now();
            BOOST_FOREACH(pt_data const& c, coords)
            {
                bool is = std::binary_search(v1.begin(), v1.end(), P(c.first, c.second), bg::less<P>());

                dummy += int(is);
            }
            dur_t time = clock_t::now() - start;
            std::cout << time << " - std::binary_search()" << std::endl;
            std::cout << "dummy: " << dummy << std::endl;
        }

        {
            std::size_t dummy = 0;
            clock_t::time_point start = clock_t::now();
            BOOST_FOREACH(pt_data const& c, coords)
            {
                bool is = bgi::detail::kd_binary_search(v2.begin(), v2.end(), P(c.first, c.second));

                dummy += int(is);
            }
            dur_t time = clock_t::now() - start;
            std::cout << time << " - kd_binary_search()" << std::endl;
            std::cout << "dummy: " << dummy << std::endl;
        }

        std::cout << "------------------------------------------------" << std::endl;

        {
            std::size_t dummy = 0;
            clock_t::time_point start = clock_t::now();
            BOOST_FOREACH(pt_data const& c, coords)
            {
                P p(c.first, 0);
                P r;
                dummy += rt.query(bgi::nearest(p, 1), &r);
            }
            dur_t time = clock_t::now() - start;
            std::cout << time << " - rtree::nearest()" << std::endl;
            std::cout << "dummy: " << dummy << ' ' << std::endl;
        }

        {
            std::size_t dummy = 0;
            clock_t::time_point start = clock_t::now();
            BOOST_FOREACH(pt_data const& c, coords)
            {
                P p(c.first, 0);
                P r;
                bool is = bgi::detail::kd_nearest(v2.begin(), v2.end(), p, r);
                dummy += int(is);
            }
            dur_t time = clock_t::now() - start;
            std::cout << time << " - kd_nearest()" << std::endl;
            std::cout << "dummy: " << dummy << ' ' << std::endl;
        }

        std::cout << "------------------------------------------------" << std::endl;

        {
            BOOST_FOREACH(pt_data const& c, coords)
            {
                P p(c.first, 0);
                P p1, p2;
                bool r1 = rt.query(bgi::nearest(p, 1), &p1) > 0;
                bool r2 = bgi::detail::kd_nearest(v2.begin(), v2.end(), p, p2);
                
                if ( r1 != r2
                  || bg::comparable_distance(p, p1) != bg::comparable_distance(p, p2) )
                {
                    std::cout << "nearest() and kd_nearest results not compatible!";
                    break;
                }
            }
        }
    }

    return 0;
}
