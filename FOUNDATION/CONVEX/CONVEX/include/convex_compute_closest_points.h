#ifndef CONVEX_COMPUTE_CLOSEST_POINTS_H
#define CONVEX_COMPUTE_CLOSEST_POINTS_H

#include <convex_simplex.h>
#include <convex_constants.h>
#include <convex_reduce_simplex.h>

#include <types/geometry_support_mapping.h>

#include <tiny_quaternion_functions.h>
#include <tiny_vector_functions.h>
#include <tiny_coordsys_functions.h>

#include <cmath>
#include <stdexcept>
#include <cassert>

namespace convex
{
  /**
   * Compute Closest Points and Distance between two Convex Sets.
   * This function uses an iterative algorithm. In each iteration a simplex
   * is updated to best approximate the Minikowsky difference between two
   * given convex objects \f$A\f$ and \f$B\f$. The Minikowsky Difference set
   * is implicitly represented by the support functions of the two objects.
   * The following text tries to give a short introduction to the concepts.
   *
   * Given two objects represented by the two sets \f$A\f$ and \f$B\f$
   * then the two objects overlap if there exist at least one
   * \f$a \in A\f$ and one \f$b \in B\f$ such that
   *
   *   \f[
   *     a - b = 0
   *   \f]
   *
   * Thus one way to test for intersection of the two objects is to form the set
   *
   *   \f[
   *     A \ominus B = \{ a - b | a \in A, b \in B \}
   *   \f]
   *
   * and see if
   *
   *   \f[
   *     0 \in A \ominus B
   *   \f]
   *
   * The set \f$A \ominus B\f$ is called the Minokowsky difference. Clearly if the two
   * objects are separated one must have that
   *
   *   \f[
   *     a - b \neq 0
   *   \f]
   *
   * holds for all \f$a \in A\f$ and all \f$b \in B\f$, or equivalently that
   *
   *   \f[
   *     0 \notin A \ominus B
   *   \f]
   *
   * Assume that the two objects are separated then one may want to know the minimum
   * distance between the two objects. That is one wants to find
   *
   *   \f[
   *     (a^*, b^*) = \min_{a \in A, b \in B} \norm{(a-b)} = \min_{y \in A \ominus B} \norm{(y)}
   *   \f]
   *
   * or equivalently
   *
   *   \f[
   *     y^* = \min_{y \in A \ominus B} \norm{(y)}
   *   \f]
   *
   * Thus seeking the minimum distance between the two object is equivalent to
   * finding the minimum norm point in the set \f$A \ominus B\f$. Or said differently to
   * find a point in \f$A \ominus B\f$ that is closest to zero.
   *
   * Observe that if we find such a point \f$y^* \in A \ominus B\f$ then we implicitly
   * also know the two closest points between \f$A\f$ and \f$B\f$, since \f$y^*\f$ is defined as
   * \f$y^* = a^* - b^*\f$ for some \f$a^* \in A\f$ and some \f$b^* \in B\f$.
   *
   * The important thing to realise is that the problem of finding the minimum
   * distance between two sets \f$A\f$ and \f$B\f$ is equivalent to the problem of finding
   * the distance between a point and the set \f$A \ominus B\f$. Thus we have replaced
   * the original problem with a simple one.
   *
   * One should notice that the solution may not be unique since there could exist
   * multiple \f$y^* \in A \ominus B\f$ that yield the same minimum distance. Further a
   * solution may not exist if \f$0 \in A \ominus B\f$.
   *
   * @param r_A          Current position of shape A.
   * @param q_A          Current quaternion orientation of shape A.
   * @param A            Pointer to convex shape A.
   * @param r_B          Current position of shape B.
   * @param q_B          Current quaternion orientation of shape B.
   * @param B            Pointer to convex shape B.
   *
   * @param p_A          Upon return this argument holds the closest point on the first convex set.
   * @param p_B          Upon return this argument holds the closest point on the second convex set.
   *
   * @param distance             Upon return this argument holds the distance between the point and the convex set.
   *
   * @param iterations           Upon return this argument holds the number of iterations used.
   * @param status               The status code of the algorithm.
   * @param absolute_tolerance
   * @param relative_tolerance
   * @param stagnation_tolerance
   * @param max_iterations
   *
   */
  template<typename M>
  inline void compute_closest_points(
                              typename M::coordsys_type const & X_A
                              , geometry::SupportMapping<typename M::vector3_type> const * A
                              , typename M::coordsys_type const & X_B
                              , geometry::SupportMapping<typename M::vector3_type> const * B
                              , typename M::vector3_type & p_A
                              , typename M::vector3_type & p_B
                              , typename M::real_type & distance
                              , size_t & iterations
                              , size_t & status
                              , typename M::real_type const & absolute_tolerance
                              , typename M::real_type const & relative_tolerance
                              , typename M::real_type const & stagnation_tolerance
                              , size_t const & max_iterations
                              )
  
  {
    using std::sqrt;
    using std::fabs;
    using std::max;
    
    typedef typename M::vector3_type  V;
    typedef typename M::real_type     T;
    typedef typename M::value_traits  VT;
    typedef          Simplex<V>       simplex_type;
    
    if( absolute_tolerance < VT::zero() )
      throw std::invalid_argument( "absolute tolerance must be non-negative" );
    if( relative_tolerance < VT::zero() )
      throw std::invalid_argument( "relative tolerance must be non-negative" );
    if( stagnation_tolerance < VT::zero() )
      throw std::invalid_argument( "stagnation tolerance must be non-negative" );
    if( max_iterations <= 0u )
      throw std::invalid_argument( "max_iterations must be positive" );
    
    distance   = VT::infinity();
    status     = ITERATING;
    iterations = 0u;
    
    T    const squared_absolute_tolerance = absolute_tolerance*absolute_tolerance;
    T          squared_distance           = VT::infinity();
    
    // Simplex approximation to convex set C
    simplex_type sigma;
    
    // Initially we use a 0-simplex corresponding to some point
    // in C. We do this by seeding the initial closest point to
    // be the zero-vector.
    V v = V::make( VT::zero(), VT::zero(), VT::zero() );
    
    // Lower error bound on distance from origin to closest point
    T mu = VT::zero();
    
    // We use a maximum iteration count to guard against infinite loops.
    for(iterations=1u; iterations<=max_iterations; ++iterations)
    {
      // Find another point w that is hopefully closer to the origin.
      //
      // That means the search direction should point towards the origin, ie. s = -v
      //
      //   S_{ T_A(A) - T_B(B) }(s) = S_{ T_A(A) } (-v) - S_{ T_B(B) }(v)
      //                            = T_A( S_A(- R_A^T v) ) - T_B( S_B(R_B^T v)
      //
      V s_a = tiny::rotate( tiny::conj( X_A.Q() ), - v  );
      V s_b = tiny::rotate( tiny::conj( X_B.Q() ),   v  );
      
      V w_a = A->get_support_point( s_a );
      V w_b = B->get_support_point( s_b );
      
      w_a = tiny::rotate( X_A.Q(), w_a ) + X_A.T();
      w_b = tiny::rotate( X_B.Q(), w_b ) + X_B.T();
      
      assert( is_number( w_a(0) ) || !"compute_closest_points(): NaN encountered");
      assert( is_number( w_a(1) ) || !"compute_closest_points(): NaN encountered");
      assert( is_number( w_a(2) ) || !"compute_closest_points(): NaN encountered");
      
      assert( is_number( w_b(0) ) || !"compute_closest_points(): NaN encountered");
      assert( is_number( w_b(1) ) || !"compute_closest_points(): NaN encountered");
      assert( is_number( w_b(2) ) || !"compute_closest_points(): NaN encountered");
      
      V w    = w_a - w_b;
      
      assert( is_number( w(0) ) || !"compute_closest_points(): NaN encountered");
      assert( is_number( w(1) ) || !"compute_closest_points(): NaN encountered");
      assert( is_number( w(2) ) || !"compute_closest_points(): NaN encountered");
      
      // Test if the new point is already part of the current simplex
      if ( is_point_in_simplex ( w, sigma ) )
      {
        assert( iterations > 1u || !"compute_closest_points(): simplex should be empty in first iteration?");
        // if so it means we can not find any points in C that is
        // closer to p and we are done
        distance = sqrt( squared_distance );
        status = SIMPLEX_EXPANSION_FAILED;
        return;
      }
      
      // Update lower error bound
      distance = sqrt(squared_distance);
      mu = max( mu, ( tiny::inner_prod(v,w) / distance) ); // check the minus sign with theory
      // Test relative stopping criteria proposed by Gino van den Bergen!
      if(distance - mu <= distance * relative_tolerance)
      {
        status = LOWER_ERROR_BOUND_CONVERGENCE;
        return;
      }
      
      if (is_degenerate_point(w, sigma))
      {
        status = DEGENERATE_SIMPLEX_ADDITION;
        return;
      }
      
      // Extend the simplex with a new vertex
      add_point_to_simplex(w, w_a, w_b, sigma);
      
      // Compute the point, v, on the simplex that is closest to the origin and
      // Reduce simplex to lowest dimensional face on the boundary
      // containing the closest point.
      // 2010-02-13 mrtn: p_a and p_b may be on the inside of A-B in case of penetrations
      v = reduce_simplex( sigma, p_A, p_B );
      
      assert( is_number( v(0) ) || !"compute_closest_points(): NaN encountered");
      assert( is_number( v(1) ) || !"compute_closest_points(): NaN encountered");
      assert( is_number( v(2) ) || !"compute_closest_points(): NaN encountered");
      
      assert( is_number( p_A(0) ) || !"compute_closest_points(): NaN encountered");
      assert( is_number( p_A(1) ) || !"compute_closest_points(): NaN encountered");
      assert( is_number( p_A(2) ) || !"compute_closest_points(): NaN encountered");
      
      assert( is_number( p_B(0) ) || !"compute_closest_points(): NaN encountered");
      assert( is_number( p_B(1) ) || !"compute_closest_points(): NaN encountered");
      assert( is_number( p_B(2) ) || !"compute_closest_points(): NaN encountered");
      
      // Test if simplex is a full tetrahedron. In this case the closest
      // point must be inside the tetrahedron and equal to the origin. Thus
      // we clearly have an intersection.
      if( is_full_simplex( sigma) )
      {
        distance = VT::zero(); //2010-02-11 mrtn: should penetration depth be computed? And possibly a better approximation of the contact point?
        //2011-11-12 Kenny: This function computes separation distance, if we got a full simplex we have penetration and separation distance is not meaningful. We have other algorithms that is used for processing the penetration case.
        status = INTERSECTION;
        return;
      }
      
      T const old_squared_distance = squared_distance;
      squared_distance = tiny::inner_prod( v, v);
      
      assert( is_number( squared_distance ) || !"compute_closest_points(): NaN encountered");
      
      // Test that closest distance are non-increasing
      if(squared_distance > old_squared_distance)
      {
        // This means that the closest distance is increasing
        distance = sqrt( squared_distance );
        status = NON_DESCEND_DIRECTION;
        return;
      }
      
      // Test absolute stopping criteria
      if( squared_distance <= squared_absolute_tolerance )
      {
        // This basically means that p are so close to the
        // convex set that we consider the distance to be zero.
        distance = sqrt( squared_distance );
        status = ABSOLUTE_CONVERGENCE;
        return;
      }
      
      // Test relative stopping criteria, so see if we do not make enough
      // progress toward the ``solution''
      if( (old_squared_distance - squared_distance) <= (relative_tolerance*old_squared_distance) )
      {
        // If relative test succedes then it means that this is as good as it
        // gets and we consider the algorithm to have converged.
        distance = sqrt( squared_distance );
        status = RELATIVE_CONVERGENCE;
        return;
      }
      
      // Test for stagnation of the solution
      if(fabs( old_squared_distance - squared_distance ) <= stagnation_tolerance )
      {
        distance = sqrt( squared_distance );
        status = STAGNATION;
        return;
      }
      
    }
    
    // If this point of the code is reached it means that we did
    // not converge with the maximum number of iterations.
    distance = sqrt( squared_distance );
    status = EXCEEDED_MAX_ITERATIONS_LIMIT;
  }

  template<typename M>
  inline void compute_closest_points(
                                     typename M::coordsys_type const & X_A
                                     , geometry::SupportMapping<typename M::vector3_type> const * A
                                     , typename M::coordsys_type const & X_B
                                     , geometry::SupportMapping<typename M::vector3_type> const * B
                                     , typename M::vector3_type & p_A
                                     , typename M::vector3_type & p_B
                                     )
  {
    typedef typename M::value_traits    value_traits;
    typedef typename M::real_type       T;

    size_t const max_iterations       = 100u;
    T      const absolute_tolerance   = value_traits::numeric_cast(10e-4);
    T      const relative_tolerance   = value_traits::numeric_cast(10e-4);
    T      const stagnation_tolerance = value_traits::numeric_cast(10e-4);
    size_t       iterations           = 0u;
    size_t       status               = 0u;
    T            distance             = value_traits::infinity();

    compute_closest_points<M>(
                              X_A
                              , A
                              , X_B
                              , B
                              , p_A
                              , p_B
                              , distance
                              , iterations
                              , status
                              , absolute_tolerance
                              , relative_tolerance
                              , stagnation_tolerance
                              , max_iterations
                              );
  }

} // namespace convex

//CONVEX_COMPUTE_CLOSEST_POINTS_H
#endif
