#ifndef CONVEX_SIGNED_DISTANCE_TO_EDGE_FACE_VORONOI_PLANE_H
#define CONVEX_SIGNED_DISTANCE_TO_EDGE_FACE_VORONOI_PLANE_H

#include <tiny_vector_functions.h>
#include <tiny_is_number.h>

#include <cmath>
#include <cassert>

namespace convex
{
  
  /**
   * Signed Distance between a point and a Edge Face Voronoi Plane.
   *
   * @param p A point which we want to know whether it lies outside a face voronoi
   * region. A face-voronoi region is given by a collection of edge-face voronoi
   * planes and the face-plane.
   * @param A    The first vertex of the edge.
   * @param B    The second vertex of the edge.
   * @param C The opposing vertex of the triangle containing the edge running from
   * A to B. As such the C-vertex defines the plane with respect to which we want
   * to test p.
   *
   * @return     The signed distance of p to the edge-face voronoi plane.
   */
  template< typename V >
  inline typename V::real_type signed_distance_to_edge_face_voronoi_plane(
                                                                   V const & p
                                                                   , V const & A
                                                                   , V const & B
                                                                   , V const & C
                                                                   )
  {
    using std::fabs;
    
    typedef typename V::value_traits    VT;
    typedef typename V::real_type       T;
    
    V m      = tiny::cross( A-C, B-C );
    
    assert( tiny::inner_prod( m, m ) > VT::zero() || !"signed_distance_to_edge_face_voronoi_plane(): Degenerate triangle encountered");
    
    V l      = tiny::cross( B-A, m );
    V n      = tiny::unit( l );
    
    T sign_p = tiny::inner_prod( n, p-B );
    T sign_C = tiny::inner_prod( n, C-B );
    T abs_p  = fabs( sign_p );
    
    assert( is_number( sign_p ) || !"signed_distance_to_edge_face_voronoi_plane(): Not a Number encountered");
    assert( is_number( sign_C ) || !"signed_distance_to_edge_face_voronoi_plane(): Not a Number encountered");
    assert( is_number( abs_p )  || !"signed_distance_to_edge_face_voronoi_plane(): Not a Number encountered");
    
    bool in_front = ( (sign_p*sign_C) <= VT::zero() );
    
    return in_front ? abs_p : -abs_p;
  }

} // namespace convex

// CONVEX_SIGNED_DISTANCE_TO_EDGE_FACE_VORONOI_PLANE_H
#endif
