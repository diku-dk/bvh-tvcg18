#ifndef CONVEX_SIGNED_DISTANCE_TO_TRIANGLE_H
#define CONVEX_SIGNED_DISTANCE_TO_TRIANGLE_H

#include <tiny_vector_functions.h>
#include <tiny_is_number.h>

#include <cmath>
#include <cassert>

namespace convex
{
  
  /**
   * Signed Distance between a point and a Triangle.
   *
   * @param p     A point which we want to know whether it lies on the other
   *              side of the triangle that the point q.
   * @param A     The first vertex of the triangle.
   * @param B     The second vertex of the triangle.
   * @param C     The third vertex of the triangle.
   * @param q     A point that is known to lie on the back-side of the triangle
   *
   *@return       The signed distance of p to the triangle given by vertices A, B, and C. The
   *              point q is used to specify the back side half-plane of the triange.
   */
  template< typename V >
  inline typename V::real_type signed_distance_to_triangle(
                                                    V const & p
                                                    , V const & A
                                                    , V const & B
                                                    , V const & C
                                                    , V const & q
                                                    )
  {
    using std::fabs;
    
    typedef typename V::value_traits    VT;
    typedef typename V::real_type       T;
    
    V m = tiny::cross( A-B, C-B );
    
    assert( tiny::inner_prod( m, m ) > VT::zero() || !"signed_distance_to_triangle(): Degenerate triangle encountered");
    
    V n = tiny::unit( m );
    
    T sign_p = tiny::inner_prod( n, p-B );
    T sign_q = tiny::inner_prod( n, q-B );
    T abs_p  = fabs( sign_p );
    
    assert( sign_q < VT::zero() || sign_q > VT::zero() || !"signed_distance_to_triangle(): q was in plane, can  not be used to determine sign");
    
    assert( is_number( sign_p ) || !"signed_distance_to_triangle(): Not a Number encountered");
    assert( is_number( sign_q ) || !"signed_distance_to_triangle(): Not a Number encountered");
    assert( is_number( abs_p )  || !"signed_distance_to_triangle(): Not a Number encountered");
    
    bool in_front = ( (sign_p*sign_q) <= VT::zero() );
    
    return  in_front ? abs_p : - abs_p;
  }

} // namespace convex

// CONVEX_SIGNED_DISTANCE_TO_TRIANGLE_H
#endif
