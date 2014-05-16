test-kd_sort
============

A small, std::sort() or std::make_heap() - like utility for spatial indexing

The example results for 1M Points gathered on VS2010 /O2:

  ------------------------------------------------
  0.421203 seconds - rtree()
  0.296402 seconds - std::sort()
  0.468003 seconds - kd_sort()
  ------------------------------------------------
  0.592804 seconds - rtree::count()
  0.436803 seconds - std::binary_search()
  0.624004 seconds - kd_binary_search()
  ------------------------------------------------
  1.96561 seconds - rtree::nearest()
  0.920406 seconds - kd_nearest()
  ------------------------------------------------
