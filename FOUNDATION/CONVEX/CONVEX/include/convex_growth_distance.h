#ifndef CONVEX_GROWTH_DISTANCES_H
#define CONVEX_GROWTH_DISTANCES_H

#include <convex_compute_closest_points.h>

#include <tiny_quaternion_functions.h>
#include <tiny_vector_functions.h>
#include <tiny_coordsys_functions.h>

#include <cassert>
#include <cmath>    // needed for std::sqrt

namespace convex
{
  
  /**
   * Growth Distance Algorithm.
   * This function tries to find the growth scale that will make the initially
   * overlapping shapes A and B come into touching contact.
   *
   * @param X_A             The world placement of shape A.
   * @param A               A pointer to the shape A.
   * @param X_B             The world placement of shape B.
   * @param B               A pointer to the shape B.
   * @param p_A             Upon return this argument holds the closest point on shrinked shape of A.
   * @param p_B             Upon return this argument holds the closest point on shrinked shape of B.
   * @param growth_scale    Upon return holds the largest growth scale that make the two objects non-penetrating.
   * @param iterations      Upon return this argument holds the number of used iterations by the function. If the value is equal to the max_iterations argument then the function did not converge to an answer.
   * @param epsilon         The size of the collision envelope. That is the smallest separation distance between A and B where we consider A and B to be in touching contact.
   * @param max_iterations  The maximum number of allowed iterations that the function can take.
   *
   * @return                If a growth distance is found then the return value is true otherwise it is false.
   */
  template< typename M>
  inline bool growth_distance(
                       typename M::coordsys_type const & X_A
                       , geometry::SupportMapping<typename M::vector3_type> const * A
                       , typename M::coordsys_type const & X_B
                       , geometry::SupportMapping<typename M::vector3_type> const * B
                       , typename M::vector3_type & p_A
                       , typename M::vector3_type & p_B
                       , typename M::real_type & growth_scale
                       , size_t & iterations
                       , typename M::real_type const & epsilon
                       , size_t const & max_iterations
                       )
  {
    
    using std::sqrt;
    
    typedef typename M::value_traits    VT;
    typedef typename M::vector3_type    V;
    typedef typename M::coordsys_type   C;
    typedef typename M::real_type       T;
    
    V const v = X_A.T() - X_B.T();
    
    assert( epsilon > VT::zero() || !"growth_distance(): collision envelope must be positive");
    assert( max_iterations > 0u  || !"growth_distance(): maximum iterations must be positive");
    
    T const d_min       = VT::two()*epsilon;
    T const d_min_sqrd  = VT::four()*epsilon*epsilon;
    T const v2          = v*v;
    
    assert( v2 > VT::zero() || !"growth_distance(): internal error growth centers are bad");
    
    // compute the support point: \vec p = S_{\set A - \set B}(- \vec v)
    V s_a = tiny::rotate( tiny::conj( X_A.Q() ), - v  );
    V s_b = tiny::rotate( tiny::conj( X_B.Q() ),   v  );
    
    V w_a = A->get_support_point( s_a );
    V w_b = B->get_support_point( s_b );
    
    w_a = tiny::rotate( X_A.Q(), w_a ) + X_A.T();
    w_b = tiny::rotate( X_B.Q(), w_b ) + X_B.T();
    
    assert( is_number( w_a(0) ) || !"growth_distance(): NaN encountered");
    assert( is_number( w_a(1) ) || !"growth_distance(): NaN encountered");
    assert( is_number( w_a(2) ) || !"growth_distance(): NaN encountered");
    
    assert( is_number( w_b(0) ) || !"growth_distance(): NaN encountered");
    assert( is_number( w_b(1) ) || !"growth_distance(): NaN encountered");
    assert( is_number( w_b(2) ) || !"growth_distance(): NaN encountered");
    
    V const p = w_a - w_b;
    
    T tau = p*v/v2 - d_min/sqrt(v2);
    
    for(iterations=1u; iterations <= max_iterations; ++iterations)
    {
      // Compute the coordinate transformations corresponding to the current tau value
      V dv = v*tau;
      C T_B = C( dv + X_B.T(), X_B.Q() );
      
      // Compute the closest points at the time tau
      {
        // 2015-12-11 Kenny code review: These controls are hard-wired... for
        // some shapes these works well.... For other (in particular curved
        // shapes) GJK simply is not accurate enough with these seetings... It
        // often means that the final p_A and p_B can substantially off

        size_t const max_iterations       = 100u;
        T      const absolute_tolerance   = VT::numeric_cast(10e-4);
        T      const relative_tolerance   = VT::numeric_cast(10e-4);
        T      const stagnation_tolerance = VT::numeric_cast(10e-4);
        size_t       iterations           = 0u;
        size_t       status               = 0u;
        T            distance             = VT::infinity();

        compute_closest_points<M>(
                                  X_A, A, T_B, B, p_A, p_B
                                  , distance
                                  , iterations
                                  , status
                                  , absolute_tolerance
                                  , relative_tolerance
                                  , stagnation_tolerance
                                  , max_iterations
                                  );
      }


      // Compute separation vector
      V s = p_A - p_B;
      
      // Test to see if separation is small enough
      T const d_squared = tiny::inner_prod(s,s);
      if( d_squared < d_min_sqrd )
      {
        // Convert ray cast length to growth scale
        growth_scale = - VT::one() / (tau - VT::one());

        // Convert closest points into "growth-scale" set-up
        p_A = growth_scale * (p_A - X_A.T()) + X_A.T();
        p_B = growth_scale * (p_B - T_B.T()) + X_B.T();

        return true;
      }
      
      // Separation were too big so we will step closer
      tau = tau + 0.9f*(s*v)/v2;//dampening the tau update value
      
    }
    
    // not enough iterations to determine what goes on! We give up
    return false;
  }

} // namespace convex

// CONVEX_GROWTH_DISTANCES_H
#endif
