#ifndef CONVEX_COMPUTE_VELOCITIES_H
#define CONVEX_COMPUTE_VELOCITIES_H

#include <cmath>
#include <cassert>

namespace convex
{
  
  /**
   * Compute Velocities.
   * This function tries to convert two poses from the motion of an object into equivalent velocities.
   *
   * The function works under the assumption that the object in questions moves at constant
   * linear and angular velocities between the two given poses. The duration of the motion
   * is assumed to be one unit (second).
   *
   * @param T_from    A coordinate transformation indicating the starting pose of the motion.
   * @param T_to      A coordinate transformation indicating the ending pose of the motion.
   * @param delta_tau The time between from pose and to pose.
   * @param v         Upon return this argument holds the value of the constant linear velocity.
   * @param omega     Upon return this argument holds the value of the constant angular velocity.
   */
  template< typename M >
  inline void compute_velocities(
                          typename M::coordsys_type  const & T_from
                          , typename M::coordsys_type  const & T_to
                          , typename M::real_type const & delta_tau
                          , typename M::vector3_type  & v
                          , typename M::vector3_type & omega
                          )
  {
    using std::atan2;
    
    typedef typename M::real_type         T;
    typedef typename M::vector3_type      V;
    typedef typename M::value_traits      VT;
    
    assert(  delta_tau > VT::zero() || !"compute_velocities(): time step must be positive");
    
    // Translation is straightforward
    v = (T_to.T() - T_from.T()) / delta_tau;
    
    T theta;
    V n;
    tiny::get_axis_angle(
                         tiny::prod( T_to.Q(), tiny::conj( T_from.Q() ) )   // Change in orientation from ``from'' to ''to'', ie. R = T_to * T_from^{-1}
                         , n
                         , theta
                         );
    omega = (theta/ delta_tau)*n;
  }

} // namespace convex

// CONVEX_COMPUTE_VELOCITIES_H
#endif