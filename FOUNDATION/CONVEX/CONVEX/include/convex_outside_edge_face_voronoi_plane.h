#ifndef CONVEX_OUTSIDE_EDGE_FACE_VORONOI_PLANE_H
#define CONVEX_OUTSIDE_EDGE_FACE_VORONOI_PLANE_H

#include <tiny_vector_functions.h>

#include <cassert>

namespace convex
{
  
  /**
   * Test if Point is outside Edge Face Voronoi Plane.
   *
   * @param p     A point which we want to know whether it lies outside a face voronoi
   *              region. A face-voronoi region is given by a collection of edge-face voronoi
   *              planes and the face-plane.
   * @param A     The first vertex of the edge.
   * @param B     The second vertex of the edge.
   * @param C     The opposing vertex of the triangle containing the edge running from
   *              A to B. As such the C-vertex defines the plane with respect to which we want
   *              to test p.
   *
   * @return     If point is outside or on the plane then the return value is true otherwise it is false.
   */
  template< typename V >
  inline bool outside_edge_face_voronoi_plane(
                                       V const & p
                                       , V const & A
                                       , V const & B
                                       , V const & C
                                       )
  {
    typedef typename V::value_traits    value_traits;
    typedef typename V::real_type      T;
    
    V const m = tiny::cross( A-C, B-C );
    assert( tiny::inner_prod( m, m ) > value_traits::zero() || !"outside_edge_face_voronoi_plane(): Degenerate triangle encountered");
    
    V const n      = tiny::cross( B-A, m );
    T const sign_p = tiny::inner_prod( n, p-B );
    T const sign_C = tiny::inner_prod( n, C-B );
    
    assert( is_number( sign_p ) || !"outside_edge_face_voronoi_plane(): Not a Number encountered");
    assert( is_number( sign_C ) || !"outside_edge_face_voronoi_plane(): Not a Number encountered");
    
    return (sign_p*sign_C) <= value_traits::zero();
  }

} // namespace convex

// CONVEX_OUTSIDE_EDGE_FACE_VORONOI_PLANE_H
#endif
