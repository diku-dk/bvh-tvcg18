#ifndef PROX_GJK_ELLIPSOID_H
#define PROX_GJK_ELLIPSOID_H

#include <prox_math_policy.h>
#include <geometry.h>   // needed for geometry::Point
#include <convex.h>
#include <tiny_is_number.h>
#include <cassert>

namespace prox
{
    namespace detail
    {
        
        /**
         *
         * Complete general purpose force-law.
         *
         *
         * This implementation uses the CONVEX algorithm for
         * determining the proximal point.
         *
         * This is actually pretty cool, because if we in the
         * future want non-ellipsoid contact laws then the
         * implementation already supports this!
         */
        template <typename T>
        inline static void gjk_ellipsoid(
                                              T const & z_s,   
                                              T const & z_t,
                                              T const & z_tau,
                                              T const & mu_s,
                                              T const & mu_t,
                                              T const & mu_tau,
                                              T const & lambda_n,
                                              T & lambda_s,
                                              T & lambda_t,
                                              T & lambda_tau
                                              )
        {
            typedef prox::MathPolicy<T>					              math_policy; 
            typedef typename math_policy::vector3_type        vector3_type;
            typedef typename math_policy::quaternion_type     quaternion_type;
            typedef typename math_policy::real_type           real_type;
            typedef typename math_policy::coordsys_type       transformation_type;
            typedef typename math_policy::value_traits        value_traits;
            
            if ( lambda_n <= value_traits::zero() )    
            {
                lambda_s   = value_traits::zero();
                lambda_t   = value_traits::zero();
                lambda_tau = value_traits::zero();
                return;
            }
            
            real_type const a = mu_s*lambda_n;
            real_type const b = mu_t*lambda_n;
            real_type const c = mu_tau*lambda_n;
            
            assert( is_number( a ) || !"gjk_ellipsoid(): a was not a number");
            assert( is_number( b ) || !"gjk_ellipsoid(): b was not a number");
            assert( is_number( c ) || !"gjk_ellipsoid(): c was not a number");
            assert( a > value_traits::zero()       || !"gjk_ellipsoid(): a non-positive");
            assert( b > value_traits::zero()       || !"gjk_ellipsoid(): a non-positive");
            assert( c > value_traits::zero()       || !"gjk_ellipsoid(): a non-positive");
            
            geometry::Point<vector3_type>                       point;
            convex::Ellipsoid<typename math_policy::base_type>  ellipsoid;
            
            point.coord() = vector3_type::make( z_s, z_t, z_tau);
            
            // TODO: Chek theory
            ellipsoid.scale() = vector3_type::make(a,b,c);  // TODO check this is how to setup the scale!
            
            size_t    const max_iterations       = 100u;
            real_type const absolute_tolerance   = value_traits::numeric_cast(10e-6); 
            real_type const relative_tolerance   = value_traits::numeric_cast(10e-6);
            real_type const stagnation_tolerance = value_traits::numeric_cast(10e-15);
            
            transformation_type transformA;
            transformation_type transformB;
            transformA.T().clear();
            transformA.Q() = quaternion_type::identity();//may cause issues on Windows/VS platform
            transformB.T().clear();
            transformB.Q() = quaternion_type::identity();
            
            vector3_type pa;
            vector3_type pb;
            size_t iterations     = 0u;
            size_t status         = 0u;
            real_type distance    = value_traits::infinity();
            
            convex::compute_closest_points<typename math_policy::base_type>(
                                                                            transformA
                                                                            , &point
                                                                            , transformB
                                                                            , &ellipsoid
                                                                            , pa
                                                                            , pb
                                                                            , distance
                                                                            , iterations
                                                                            , status
                                                                            , absolute_tolerance
                                                                            , relative_tolerance
                                                                            , stagnation_tolerance
                                                                            , max_iterations
                                                                            );
            
            assert( status != convex::ITERATING                     || !"gjk_ellipsoid(): gjk internal error");
            assert( status != convex::EXCEEDED_MAX_ITERATIONS_LIMIT || !"gjk_ellipsoid(): gjk internal error");
            assert( status != convex::NON_DESCEND_DIRECTION         || !"gjk_ellipsoid(): gjk internal error");
            
            lambda_s = pb(0);
            lambda_t = pb(1);
            lambda_tau = pb(2);
        }
        
    } // namespace detail
} // namespace prox

// PROX_GJK_ELLIPSOID_H
#endif
