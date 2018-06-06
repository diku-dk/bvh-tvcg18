#ifndef CONVEX_SIGNED_DISTANCE_TO_VERTEX_EDGE_VORONOI_PLANE_H
#define CONVEX_SIGNED_DISTANCE_TO_VERTEX_EDGE_VORONOI_PLANE_H

#include <tiny_vector_functions.h>

#include <cassert>

namespace convex
{
  /**
   * Signed Distance between a point and a vertex edge voronoi plane.
   * The vertex edge voronoi plane is defined such that the plane normal is given
   * as the unit vector of the vector \fA-B\f and the point \fA\f is defined to
   * lie in the plane, and the point \fB\f behind the plane.
   *
   * @param p    The point that should be tested.
   * @param A    The first vertex of the edge.
   * @param B    The second vertex of the edge.
   *
   * @return     The signed distance of the point p.
   */
  template< typename V >
  inline typename V::real_type signed_distance_to_vertex_edge_voronoi_plane(
                                                                     V const & p
                                                                     , V const & A
                                                                     , V const & B
                                                                     )
  {
    typedef typename V::value_traits   VT;
    typedef typename V::real_type      T;
    
    V const m = (A-B);
    
    assert( tiny::inner_prod( m, m ) > VT::zero() || !"signed_distance_to_vertex_edge_voronoi_plane(): Degenerate edge encountered");
    
    V const n = tiny::unit( m );
    
    T sign_p = tiny::inner_prod( n, (p-A) );
    
    assert( is_number( sign_p ) || !"signed_distance_to_vertex_edge_voronoi_plane(): Not a Number encountered");
    
    return sign_p;
  }

} // namespace convex

// CONVEX_SIGNED_DISTANCE_TO_VERTEX_EDGE_VORONOI_PLANE_H
#endif
