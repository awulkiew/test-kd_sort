// Copyright (c) 2014 Adam Wulkiewicz, Lodz, Poland.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <boost/random.hpp>
#include <boost/tuple/tuple.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "kd_sort.hpp"
#include "kd_sort_left_balanced.hpp"

typedef boost::tuple<float, float, float, float> pt_data;

namespace bg = boost::geometry;
namespace bgi = bg::index;

typedef bg::model::point<double, 2, bg::cs::cartesian> P;
typedef bg::model::box<P> B;

void print(P const& p)
{
    std::cout << bg::get<0>(p) << ", " <<  bg::get<1>(p);
}

void print(B const& b)
{
    std::cout << bg::get<bg::min_corner, 0>(b) << ", " <<  bg::get<bg::min_corner, 1>(b) << " x "
              << bg::get<bg::max_corner, 0>(b) << ", " <<  bg::get<bg::max_corner, 1>(b);
}

#ifndef TEST_BOXES
typedef P V;
P to_v(pt_data const& c)
{
    return P(boost::get<0>(c), boost::get<1>(c));
}
#else
typedef B V;
B to_v(pt_data const& c)
{
    return B(P(boost::get<0>(c), boost::get<1>(c)), P(boost::get<0>(c) + boost::get<2>(c), boost::get<1>(c) + boost::get<3>(c)));
}
#endif

int main()
{
    typedef boost::chrono::thread_clock clock_t;
    typedef boost::chrono::duration<float> dur_t;

#if !defined(_DEBUG) || defined(NDEBUG)
    size_t values_count = 1000000;
#else
    size_t values_count = 100;
#endif

    std::vector<pt_data> coords;

    //randomize values
    {
        boost::mt19937 rng;
        //rng.seed(static_cast<unsigned int>(std::time(0)));
        boost::uniform_real<float> range_pos(-1000, 1000);
        boost::variate_generator<boost::mt19937&, boost::uniform_real<float> > rnd_pos(rng, range_pos);
        boost::uniform_real<float> range_size(10, 50);
        boost::variate_generator<boost::mt19937&, boost::uniform_real<float> > rnd_size(rng, range_size);

        coords.reserve(values_count);

        std::cout << "randomizing data\n";
        for ( size_t i = 0 ; i < values_count ; ++i )
        {
            coords.push_back(boost::make_tuple(rnd_pos(), rnd_pos(), rnd_size(), rnd_size()));
        }
        std::cout << "randomized\n";
    }

    for (;;)
    {
        std::vector<V> v1, v2, v3;
        
        {
            v1.reserve(values_count);
            v2.reserve(values_count);
            v3.reserve(values_count);
            BOOST_FOREACH(pt_data const& c, coords)
            {
                v1.push_back(to_v(c));
                v2.push_back(to_v(c));
                v3.push_back(to_v(c));
            }
        }

        std::cout << "------------------------------------------------" << std::endl;

        clock_t::time_point start = clock_t::now();
        bgi::rtree<V, bgi::linear<8> > rt(v1.begin(), v1.end());
        dur_t time = clock_t::now() - start;
        std::cout << time << " - rtree()" << std::endl;

#ifndef TEST_BOXES
        {
            clock_t::time_point start = clock_t::now();
            std::sort(v1.begin(), v1.end(), bg::less<P>());
            dur_t time = clock_t::now() - start;
            std::cout << time << " - std::sort()" << std::endl;
        }
#endif

        {
            clock_t::time_point start = clock_t::now();
            bgi::detail::kd_sort(v2.begin(), v2.end());
            dur_t time = clock_t::now() - start;
            std::cout << time << " - kd_sort()" << std::endl;
        }

        {
            clock_t::time_point start = clock_t::now();
            bgi::detail::kd_sort_left_balanced(v3.begin(), v3.end());
            dur_t time = clock_t::now() - start;
            std::cout << time << " - kd_sort_left_balanced()" << std::endl;
        }

        std::cout << "------------------------------------------------" << std::endl;

        {
            std::size_t dummy = 0;
            clock_t::time_point start = clock_t::now();
            BOOST_FOREACH(pt_data const& c, coords)
            {
                bool is = rt.count(to_v(c)) > 0;

                dummy += int(is);
            }
            dur_t time = clock_t::now() - start;
            std::cout << time << " - rtree::count()" << std::endl;
            std::cout << "dummy: " << dummy << std::endl;
        }

#ifndef TEST_BOXES
        {
            std::size_t dummy = 0;
            clock_t::time_point start = clock_t::now();
            BOOST_FOREACH(pt_data const& c, coords)
            {
                bool is = std::binary_search(v1.begin(), v1.end(), to_v(c), bg::less<P>());

                dummy += int(is);
            }
            dur_t time = clock_t::now() - start;
            std::cout << time << " - std::binary_search()" << std::endl;
            std::cout << "dummy: " << dummy << std::endl;
        }
#endif

        {
            std::size_t dummy = 0;
            clock_t::time_point start = clock_t::now();
            BOOST_FOREACH(pt_data const& c, coords)
            {
                bool is = bgi::detail::kd_binary_search(v2.begin(), v2.end(), to_v(c));

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
                P p(boost::get<0>(c), 0);
                V r;
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
                P p(boost::get<0>(c), 0);
                V r;
                bool is = bgi::detail::kd_nearest(v2.begin(), v2.end(), p, r);
                dummy += int(is);
            }
            dur_t time = clock_t::now() - start;
            std::cout << time << " - kd_nearest()" << std::endl;
            std::cout << "dummy: " << dummy << ' ' << std::endl;
        }

        {
            std::size_t dummy = 0;
            clock_t::time_point start = clock_t::now();
            BOOST_FOREACH(pt_data const& c, coords)
            {
                P p(boost::get<0>(c), 0);
                V r;
                bool is = bgi::detail::kd_nearest_left_balanced(v3.begin(), v3.end(), p, r);
                dummy += int(is);
            }
            dur_t time = clock_t::now() - start;
            std::cout << time << " - kd_nearest_left_balanced()" << std::endl;
            std::cout << "dummy: " << dummy << ' ' << std::endl;
        }

        std::cout << "------------------------------------------------" << std::endl;

        {
            int errors = 0;
            BOOST_FOREACH(pt_data const& c, coords)
            {
                P p(boost::get<0>(c), 0);
                V p1, p2, p3;
                bool r1 = rt.query(bgi::nearest(p, 1), &p1) > 0;
                bool r2 = bgi::detail::kd_nearest(v2.begin(), v2.end(), p, p2);
                bool r3 = bgi::detail::kd_nearest_left_balanced(v3.begin(), v3.end(), p, p3);
                
                if ( r1 != r2
                  || bg::comparable_distance(p, p1) != bg::comparable_distance(p, p2) )
                {
                    std::cout << "nearest() and kd_nearest results not compatible!" << std::endl;
                    std::cout << r1 << ' ' << r2 << std::endl;
                    std::cout << bg::comparable_distance(p, p1) << ' ' << bg::comparable_distance(p, p2) << std::endl;
                    print(p); std::cout << std::endl;
                    print(p1); std::cout << std::endl;
                    print(p2); std::cout << std::endl;
                    ++errors;
                }

                if ( r1 != r3
                  || bg::comparable_distance(p, p1) != bg::comparable_distance(p, p3) )
                {
                    std::cout << "nearest() and kd_nearest_left_balanced results not compatible!";
                    std::cout << r1 << ' ' << r3 << std::endl;
                    std::cout << bg::comparable_distance(p, p1) << ' ' << bg::comparable_distance(p, p3) << std::endl;
                    print(p); std::cout << std::endl;
                    print(p1); std::cout << std::endl;
                    print(p3); std::cout << std::endl;
                    ++errors;
                }

                if ( errors > 10 )
                    break;
            }
        }

    }

    return 0;
}
