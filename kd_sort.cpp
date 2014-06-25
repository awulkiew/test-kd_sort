// Copyright (c) 2014 Adam Wulkiewicz, Lodz, Poland.

// Use, modification and distribution is subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

#include <boost/chrono.hpp>
#include <boost/foreach.hpp>
#include <boost/random.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/index/rtree.hpp>

#include "kd_sort.hpp"
#include "kd_sort_left_balanced.hpp"

int main()
{
    namespace bg = boost::geometry;
    namespace bgi = bg::index;
    typedef boost::chrono::thread_clock clock_t;
    typedef boost::chrono::duration<float> dur_t;

#if !defined(_DEBUG) || defined(NDEBUG)
    size_t values_count = 1000000;
#else
    size_t values_count = 100;
#endif

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

        std::vector<P> v1, v2, v3;
        
        {
            v1.reserve(values_count);
            v2.reserve(values_count);
            v3.reserve(values_count);
            BOOST_FOREACH(pt_data const& c, coords)
            {
                v1.push_back(P(c.first, c.second));
                v2.push_back(P(c.first, c.second));
                v3.push_back(P(c.first, c.second));
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

        {
            std::size_t dummy = 0;
            clock_t::time_point start = clock_t::now();
            BOOST_FOREACH(pt_data const& c, coords)
            {
                P p(c.first, 0);
                P r;
                bool is = bgi::detail::kd_nearest_left_balanced(v3.begin(), v3.end(), p, r);
                dummy += int(is);
            }
            dur_t time = clock_t::now() - start;
            std::cout << time << " - kd_nearest_left_balanced()" << std::endl;
            std::cout << "dummy: " << dummy << ' ' << std::endl;
        }

        std::cout << "------------------------------------------------" << std::endl;

        {
            BOOST_FOREACH(pt_data const& c, coords)
            {
                P p(c.first, 0);
                P p1, p2, p3;
                bool r1 = rt.query(bgi::nearest(p, 1), &p1) > 0;
                bool r2 = bgi::detail::kd_nearest(v2.begin(), v2.end(), p, p2);
                bool r3 = bgi::detail::kd_nearest_left_balanced(v3.begin(), v3.end(), p, p3);
                
                if ( r1 != r2
                  || bg::comparable_distance(p, p1) != bg::comparable_distance(p, p2) )
                {
                    std::cout << "nearest() and kd_nearest results not compatible!";
                    break;
                }

                if ( r1 != r3
                  || bg::comparable_distance(p, p1) != bg::comparable_distance(p, p3) )
                {
                    std::cout << "nearest() and kd_nearest_left_balanced results not compatible!";
                    break;
                }
            }
        }
    }

    return 0;
}
