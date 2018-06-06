#ifndef CONVEX_CONSERVATIVE_ADVANCEMENT_H
#define CONVEX_CONSERVATIVE_ADVANCEMENT_H

#include <cassert>

namespace convex
{
  
  /**
   * Conservative Advancement.
   * This function tries to determine whether two objects have impacted
   * during their motion. The function assumes that the objects will move
   * continuously in the future with contact linear and angular velocities.
   *
   *
   * @param r_A          Initial position of shape A.
   * @param q_A          Initial quaternion orientation of shape A.
   * @param v_A          Linear velocity of shape A.
   * @param w_A          Angular velocity of shape A.
   * @param A            Pointer to convex shape A.
   * @param r_max_A      Maximum radius of the shape of object A.
   *
   * @param r_B          Initial position of shape B.
   * @param q_B          Initial quaternion orientation  of shape B.
   * @param v_B          Linear velocity of shape B.
   * @param w_B          Angular velocity of shape B.
   * @param B            Pointer to convex shape B.
   * @param r_max_B      Maximum radius of the shape of object B.
   *
   * @param p_A             Upon return this argument holds the cloest point on object A in case of an impact.
   * @param p_B             Upon return this argument holds the cloest point on object B in case of an impact.
   * @param time_of_impact  Upon return if an impact is found then this argument holds the estimated value of the time of impact.
   * @param iterations      Upon return this argument holds the number of used iterations by the function. If the value is equal to the max_iterations argument then the function did not converge to an answer.
   * @param epsilon         The size of the collision envelope. That is the smallest separation distance between A and B where we consider A and B to be in touching contact.
   * @param max_tau         The maximum time into the future that the function will look for a time of impact.
   * @param max_iterations  The maximum number of allowed iterations that the function can take.
   *
   * @return                If an impact is found then the return value is true otherwise it is false.
   */
  template<typename M>
  inline bool conservative_advancement(
                                typename M::coordsys_type const & X_A
                                , typename M::vector3_type const & v_A
                                , typename M::vector3_type const & w_A
                                , geometry::SupportMapping<typename M::vector3_type> const * A
                                , typename M::real_type const & r_max_A
                                , typename M::coordsys_type const & X_B
                                , typename M::vector3_type const & v_B
                                , typename M::vector3_type const & w_B
                                , geometry::SupportMapping<typename M::vector3_type> const * B
                                , typename M::real_type const & r_max_B
                                , typename M::vector3_type & p_A
                                , typename M::vector3_type & p_B
                                , typename M::real_type & time_of_impact
                                , size_t & iterations
                                , typename M::real_type const & epsilon
                                , typename M::real_type const & max_tau
                                , size_t const & max_iterations
                                )
  {
    typedef typename M::value_traits  VT;
    typedef typename M::vector3_type  V;
    typedef typename M::coordsys_type C;
    typedef typename M::real_type     T;
    
    assert( r_max_A > VT::zero()              || !"conservative_advancement(): maximum distance of object A must be positive");
    assert( r_max_B > VT::zero()              || !"conservative_advancement(): maximum distance of object B must be positive");
    assert( max_tau > VT::zero()              || !"conservative_advancement(): maximum time-step must be positive");
    assert( epsilon > VT::zero()              || !"conservative_advancement(): collision envelope must be positive");
    assert( max_iterations > 0u               || !"conservative_advancement(): maximum iterations must be positive");
    assert(epsilon >= VT::numeric_cast(1e-2)  || !"conservative_advancement(): Too aggressive setting of epsilon, compute_closest_points uses tolerance 10e4");

    T tau = VT::zero();
    
    for(iterations=1u; iterations <= max_iterations; ++iterations)
    {
      // Compute the coordinate transformations corresponding to the current tau value
      C T_A = integrate_motion<M>( X_A, tau, v_A, w_A );
      C T_B = integrate_motion<M>( X_B, tau, v_B, w_B );
      
      // Compute the closest points at the time tau
      compute_closest_points<M>( T_A, A, T_B, B, p_A, p_B );
      
      // Estimate normal direction and current minimum distance between A and B
      V v = p_A - p_B;
      
      // 2015-11-19 Kenny: If GJK did not converge completely then p_A and p_B will
      //                   be slightly off... this means the distance we compute here
      //                   is in fact a little larger than the "true" minimum distance. We
      //                   counter this by using the treshold test below. However, if one
      //                   is too aggressive then one might overstep the true TOI estimate
      //                   by a tiny fraction.

      T min_distance = tiny::norm(v);

      if( min_distance <= epsilon )
      {
        time_of_impact = tau;
        return true;
      }
      
      // 2015-11-19 Kenny: If GJK did not converge completely then p_A and p_B will
      //                   be slightly off... this means the "n" direction can be off. The
      //                   problem is worsen as objects come close, as n will be determined
      //                   by "substracting" numbers that gets smaller and smaller....
      //
      //                   At some point the impression will dominate and determine the
      //                   direction more than the "closes" point will dominate the
      //                   direction.
      //
      //                   One should use epislon to avoid getting into this case... too
      //                   agressive epsilon means that max_velocity will be badly estimated
      //                   (too low than the true value.
      //

      V n = tiny::unit( v );
      
      // Estimate maximum relative normal velocity between any two points from A and B
      
      T max_velocity = tiny::inner_prod(v_B - v_A, n) + tiny::norm(w_A)*r_max_A + tiny::norm(w_B)*r_max_B;

      if (max_velocity <= VT::zero() )
        return false;
      
      // Compute conservative lower bound for when A and B could impact
      T delta_tau = min_distance / max_velocity;
      tau += delta_tau;
      
      if ( tau > max_tau )
        return false;
    }
    
    // not enough iterations to determine what goes on! We give up
    return false;
  }

} // namespace convex

// CONVEX_CONSERVATIVE_ADVANCEMENT_H
#endif
