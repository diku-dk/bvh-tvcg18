#ifndef CONVEX_MOTION_INTERPOLATION_H
#define CONVEX_MOTION_INTERPOLATION_H

#include <tiny.h>

namespace convex
{
  
  /**
   * Motion Interpolation.
   * This function tries to determine whether two objects have impacted
   * during their motion. The motion of the objects is specified by giving
   * initial and final positions of the objects. From these discrete poses
   * the function will reconstruct the continuous inbetween motion under the
   * assumption that the objects moved at contact linear and angular velocities
   * between their starting and final positions.
   *
   * @param r_A_from        Initial position of shape A.
   * @param q_A_from        Initial quaternion orientation of shape A.
   * @param r_A_to          Final position of shape A.
   * @param q_A_to          Final quaternion orientation of shape A.
   * @param A               The shape/geometry of object A.
   * @param r_max_A         Maximum radius of the shape of object A.
   *
   * @param r_B_from        Initial position of shape B.
   * @param q_B_from        Initial quaternion orientation of shape B.
   * @param r_B_to          Final position of shape B.
   * @param q_B_to          Final quaternion orientation of shape B.
   * @param B               The shape/geometry of object B.
   * @param r_max_B         Maximum radius of the shape of object B.
   *
   * @param p_A             Upon return this argument holds the cloest point on object A in case of an impact.
   * @param p_B             Upon return this argument holds the cloest point on object B in case of an impact.
   * @param time_of_impact  Upon return if an impact is found then this argument holds the estimated value of the time of impact.
   * @param iterations      Upon return this argument holds the number of used iterations by the function. If the value is equal to the max_iterations argument then the function did not converge to an answer.
   * @param epsilon         The size of the collision envelope. That is the smallest separation distance between A and B where we consider A and B to be in touching contact.
   * @param max_iterations  The maximum number of allowed iterations that the function can take.
   *
   * @return                If an impact is found then the return value is true otherwise it is false.
   */
  template<typename M>
  inline bool motion_interpolation(
                            typename M::coordsys_type const & X_A_from
                            , typename M::coordsys_type const & X_A_to
                            , geometry::SupportMapping<typename M::vector3_type> const * A
                            , typename M::real_type const & r_max_A
                            , typename M::coordsys_type const & X_B_from
                            , typename M::coordsys_type const & X_B_to
                            , geometry::SupportMapping<typename M::vector3_type> const * B
                            , typename M::real_type const & r_max_B
                            , typename M::vector3_type & p_A
                            , typename M::vector3_type & p_B
                            , typename M::real_type & time_of_impact
                            , size_t & iterations
                            , typename M::real_type const & epsilon
                            , size_t const & max_iterations
                            )
  {
    typedef typename M::vector3_type   V;
    typedef typename M::value_traits   VT;
    
    V v_A;
    V w_A;
    V v_B;
    V w_B;
    
    compute_velocities<M>( X_A_from, X_A_to, VT::one(), v_A, w_A );
    compute_velocities<M>( X_B_from, X_B_to, VT::one(), v_B, w_B );
    
    return conservative_advancement<M>(
                                       X_A_from
                                       , v_A
                                       , w_A
                                       , A
                                       , r_max_A
                                       , X_B_from
                                       , v_B
                                       , w_B
                                       , B
                                       , r_max_B
                                       , p_A
                                       , p_B
                                       , time_of_impact
                                       , iterations
                                       , epsilon
                                       , VT::one()
                                       , max_iterations
                                       );
  }

} // namespace convex

// CONVEX_MOTION_INTERPOLATION_H
#endif
