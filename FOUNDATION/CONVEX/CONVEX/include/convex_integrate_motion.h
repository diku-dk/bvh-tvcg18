#ifndef CONVEX_INTEGRATE_MOTION_H
#define CONVEX_INTEGRATE_MOTION_H

#include <cmath>
#include <cassert>

namespace convex
{
  
  /**
   * Integrate Motion.
   *
   * @param X      The current coordinate transformation of the object.
   * @param tau    The time step into the future where the coordinate transformation of the object should be computed.
   * @param v      The current linear velocity of the object.
   * @param omega  The current angular velocity of the object.
   *
   * @return       Upon return this argument holds the coordinate transformation of the object at time tau.
   */
  template< typename M>
  inline typename M::coordsys_type integrate_motion(
                                             typename M::coordsys_type const & X
                                             , typename M::real_type const & tau
                                             , typename M::vector3_type const & v
                                             , typename M::vector3_type const & omega
                                             )
  {
    typedef typename M::coordsys_type    C;
    typedef typename M::real_type        T;
    typedef typename M::vector3_type     V;
    typedef typename M::quaternion_type  Q;
    typedef typename M::value_traits     VT;
    
    assert( tau >= VT::zero() || !"integrate_motion(): Tau must be non-negative");
    
    T const radian           = tau * tiny::norm( omega );
    V const axis             = tiny::unit( omega );
    
    assert( is_number( radian )  || !"integrate_motion(): NaN encountered");
    assert( is_number( axis(0) ) || !"integrate_motion(): NaN encountered");
    assert( is_number( axis(1) ) || !"integrate_motion(): NaN encountered");
    assert( is_number( axis(2) ) || !"integrate_motion(): NaN encountered");
    
    Q dq;
    V dv;
    dq = Q::Ru( radian, axis);
    dv = v*tau;
    
    assert( is_number( dv(0) ) || !"integrate_motion(): NaN encountered");
    assert( is_number( dv(1) ) || !"integrate_motion(): NaN encountered");
    assert( is_number( dv(2) ) || !"integrate_motion(): NaN encountered");
    
    return C( dv + X.T(), tiny::prod(dq, X.Q()) );
  }

} // namespace convex

// CONVEX_INTEGRATE_MOTION_H
#endif
