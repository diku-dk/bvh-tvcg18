#ifndef CONVEX_OUTSIDE_TRIANGLE_H
#define CONVEX_OUTSIDE_TRIANGLE_H

#include <tiny_vector_functions.h>

#include <cassert>

namespace convex
{
  
  /**
   * Test if point is outside a Triangle.
   *
   * @param p     A point which we want to know whether it lies on the other
   *              side of the triangle that the point q.
   * @param A     The first vertex of the triangle.
   * @param B     The second vertex of the triangle.
   * @param C     The third vertex of the triangle.
   * @param q     A point that is known to lie on the back-side of the triangle
   *
   *@return       If p is outside or on the face plane then the return value is true otherwise it is false.
   */
  template< typename V >
  inline bool outside_triangle(
                        V const & p
                        , V const & A
                        , V const & B
                        , V const & C
                        , V const & q
                        )
  {
    typedef typename V::value_traits    value_traits;
    typedef typename V::real_type       T;
    
    V const n = tiny::cross( A-B, C-B );
    
    assert( tiny::inner_prod( n, n ) > value_traits::zero() || !"outside_triangle(): Degenerate triangle encountered");
    
    T const sign_p = tiny::inner_prod( n, p-B );
    T const sign_q = tiny::inner_prod( n, q-B );
    
    assert( is_number( sign_p ) || !"outside_triangle(): Not a Number encountered");
    assert( is_number( sign_q ) || !"outside_triangle(): Not a Number encountered");
    
    assert( sign_q < value_traits::zero() || sign_q > value_traits::zero() || !"outside_triangle(): q was in plane, can  not be used to determine sign");
    
    return (sign_p*sign_q) <= value_traits::zero();
  }

} // namespace convex

// CONVEX_OUTSIDE_TRIANGLE_H
#endif
