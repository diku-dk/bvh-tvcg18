#ifndef CONVEX_OUTSIDE_VERTEX_EDGE_VORONOI_PLANE_H
#define CONVEX_OUTSIDE_VERTEX_EDGE_VORONOI_PLANE_H

#include <tiny_vector_functions.h>

#include <cassert>

namespace convex
{
  
  /**
   * Test if point is outside a vertex edge voronoi plane.
   * The vertex edge voronoi plane is defined such that the plane normal is given
   * as the unit vector of the vector \fA-B\f and the point \fA\f is defined to
   * lie in the plane, and the point \fB\f behind the plane.
   *
   * @param p    The point that should be tested.
   * @param A    The first vertex of the edge.
   * @param B    The second vertex of the edge.
   *
   * @return     If point is outside or on plane then return value is true otherwise it is false.
   */
  template< typename V >
  inline bool outside_vertex_edge_voronoi_plane(
                                         V const & p
                                         , V const & A
                                         , V const & B
                                         )
  {
    typedef typename V::value_traits    VT;
    typedef typename V::real_type       T;
    
    V const n = (A-B);
    assert( tiny::inner_prod( n, n ) > VT::zero() || !"outside_vertex_edge_voronoi_plane(): Degenerate edge encountered");
    
    T const sign_p = tiny::inner_prod( n, (p-A) );
    assert( is_number( sign_p ) || !"outside_vertex_edge_voronoi_plane(): Not a Number encountered");
    
    return sign_p >= VT::zero();
  }

} // namespace convex

// CONVEX_OUTSIDE_VERTEX_EDGE_VORONOI_PLANE_H
#endif