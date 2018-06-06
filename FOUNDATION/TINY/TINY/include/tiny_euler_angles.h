#ifndef TINY_EULER_ANGLES_H
#define TINY_EULER_ANGLES_H

#include <tiny_quaternion.h> // for quaternion
#include <tiny_functions.h>  // for clamp
#include <tiny_is_number.h>  // for is_number
#include <cmath>            // for cos, atan2, and asin
#include <cassert>

namespace tiny
{
  
  /**
   * Computes Euler Angles.
   * The resulting order is: xyz, i.e. first x, then y and finally z
   *
   *
   * @param R    A rotation matrix
   *
   * @param rx   Upon return holds the rotation around x-axis in radians.
   * @param ry   Upon return holds the rotation around y-axis in radians.
   * @param rz   Upon return holds the rotation around z-axis in radians.
   *
   * @return     If return value is true then we have computed unique euler angle
   *             values otherwise our solution might not be unique
   */
  // 2010-05-17 Kenny code review: unit test of this function appears to be missing?
  template<typename matrix3x3_type>
  inline bool XYZ_euler_angles(matrix3x3_type const & R, typename matrix3x3_type::real_type & rx, typename matrix3x3_type::real_type & ry, typename matrix3x3_type::real_type & rz)
  {
    using std::atan2;
    using std::asin;
    
    typedef typename matrix3x3_type::real_type     real_type;
    typedef typename matrix3x3_type::value_traits  value_traits;
    
    real_type const & m00 = R(0,0);
    real_type const & m01 = R(0,1);
    real_type const & m02 = R(0,2);
    real_type const & m10 = R(1,0);
    real_type const & m20 = R(2,0);
    real_type const & m21 = R(2,1);
    real_type const & m22 = R(2,2);
    // rot =  cy*cz           cz*sx*sy-cx*sz  cx*cz*sy+sx*sz
    //        cy*sz           cx*cz+sx*sy*sz -cz*sx+cx*sy*sz
    //       -sy              cy*sx           cx*cy
    if ( m20 < value_traits::one() )
    {
      if ( m20 > -value_traits::one() )
      {
        rz = value_traits::numeric_cast( atan2(m10,m00) );
        ry = value_traits::numeric_cast( asin(-m20)     );
        rx = value_traits::numeric_cast( atan2(m21,m22) );
        return true;
      }
      
      // WARNING.  Not unique.  ZA - XA = -atan2(r01,r02)
      rz = value_traits::numeric_cast( - atan2(m01,m02) );
      ry = value_traits::pi_2();
      rx = value_traits::zero();
      return false;
    }
    
    // WARNING.  Not unique.  ZA + XA = atan2(-r01,-r02)
    rz = value_traits::numeric_cast( atan2(-m01,-m02) );
    ry = value_traits::pi_2();
    rx = value_traits::zero();
    return false;
  }
  
  /**
   * Computes ZYZ Euler Angles.
   *
   * @param Q    A quaternion representing some given orientation for which we wish to find the corresponding ZYZ Euler parameters.
   *
   * @param phi    The first rotation around the Z-axis.
   * @param psi    The rotation around the Y-axis.
   * @param theta  The first rotation around the Z-axis.
   *
   * @return     If return value is true then we have computed unique euler angle
   *             values otherwise our solution might not be unique
   */
  template<typename quaternion_type>
  inline void ZYZ_euler_angles(quaternion_type const & Q, typename quaternion_type::real_type & phi, typename quaternion_type::real_type & psi, typename quaternion_type::real_type & theta  )
  {
    using std::atan2;
    using std::cos;
    
    typedef typename quaternion_type::vector3_type    vector3_type;
    typedef typename quaternion_type::value_traits    value_traits;
    typedef typename quaternion_type::real_type       real_type;
    
    phi   = value_traits::zero();
    psi   = value_traits::zero();
    theta = value_traits::zero();
    
    // Here phi, psi and theta defines the relative rotation, Q, such that
    //
    // Q ~ Rz( phi )*Ry( psi )*Rz(theta);
    //
    // Our taks is to find phi, psi, and theta given Q
    //
    // We exploit the following idea below to reduce the problem. We
    // use a clever test-vector, k = [0 0 1]^T and try to rotate this
    // vector with the given rotation. That is
    //
    // Q k Q^*  = Rz( phi )*Ry( psi )*Rz(theta) k =  Rz( phi )*Ry( psi ) k;
    //
    // denoting Q k Q^* = u, a unit vector, we no longer need to worry about theta.
    vector3_type const u = rotate( Q, vector3_type::k() );
    // Now we must have
    //
    //  u = Rz(phi) Ry(psi) [0 0 1]^T
    //
    // From this we must have
    //
    //  | u_x |   |  cos(phi) -sin(phi) 0 | |  cos(psi) 0 sin(psi) |   |0|
    //  | u_y | = |  sin(phi)  cos(phi) 0 | |  0        1 0        |   |0|
    //  | u_z |   |  0         0        1 | | -sin(psi) 0 cos(psi) |   |1|
    //
    // Multipling once
    //
    //  | u_x |   |  cos(phi) -sin(phi) 0 | |  sin(psi) |  
    //  | u_y | = |  sin(phi)  cos(phi) 0 | |   0       | 
    //  | u_z |   |  0         0        1 | |  cos(psi) |  
    //
    // Multipling twice
    //
    //  | u_x |   |  cos(phi)  sin(psi) |  
    //  | u_y | = |  sin(phi)  sin(psi) | 
    //  | u_z |   |  cos(psi)           |  
    //
    // From the third equation we solve
    //
    //  psi = acos( u_z )
    //
    // This forces psi to always be in the internval [0..pi].
    //
    real_type const u_z = clamp( u(2), -value_traits::one(), value_traits::one());
    assert(is_number(u_z) || !"ZYZ_euler_angles(): not an number encountered");
    assert(u_z <= value_traits::one() || !"ZYZ_euler_angles(): u_z was too big");
    assert(u_z >= -value_traits::one() || !"ZYZ_euler_angles(): u_z was too small");
    
    psi = value_traits::numeric_cast( acos(u_z)   );
    assert(is_number(psi) || !"ZYZ_euler_angles(): psi was not an number encountered");
    assert(psi <= value_traits::pi() || !"ZYZ_euler_angles(): psi was too big");
    assert(psi >= value_traits::zero() || !"ZYZ_euler_angles(): psi was too small");
    //
    // We know that sin(psi) is allways going to be positive, which mean
    // that we can divide the second equation by the first equation and
    // obtain
    //
    //  sin(phi)/cos(phi) = tg(phi) = u_y/u_x
    //
    // From this we have
    //
    //  phi = arctan2( u_y, u_x )
    //
    // That means that phi will always be in the interval [-pi..pi].
    //
    // Observe that if psi is zero then u_y and u_x is both zero and our
    // approach will alway compute phi to be the value zero. The case
    // is actually worse than it seems. Because with psi is zero ZYZ are
    // in a gimbal lock where the two Z-axis transformations are completely
    // aligned.
    //
    //
    real_type const too_small = value_traits::numeric_cast( 0.0001 );
    if(psi<too_small)
    {
      //
      // Our solution is to use another clever test vector 
      //
      vector3_type const w = rotate( Q, vector3_type::i());
      
      real_type const w_x = w(0);
      real_type const w_y = w(1);
      assert(is_number(w_x) || !"ZYZ_euler_angles(): w_x was not an number encountered");
      assert(is_number(w_y) || !"ZYZ_euler_angles(): w_y not an number encountered");
      phi = value_traits::numeric_cast( atan2(w_y,w_x) );
      assert(is_number(phi) || !"ZYZ_euler_angles(): phi was not an number encountered");
      assert(phi <=  value_traits::pi() || !"ZYZ_euler_angles(): phi was too big");
      assert(phi >= -value_traits::pi() || !"ZYZ_euler_angles(): phi was too small");
      
      //
      // psi was too close to zero so we are in a gimbal lock, we simply keep theta zero
      //
      return;
    }
    else
    {
      // We are not close to gimbal lock, so we can safely
      real_type const u_x = u(0);
      real_type const u_y = u(1);
      assert(is_number(u_x) || !"ZYZ_euler_angles(): u_x was not an number encountered");
      assert(is_number(u_y) || !"ZYZ_euler_angles(): u_y not an number encountered");
      phi = value_traits::numeric_cast( atan2(u_y,u_x) );
      assert(is_number(phi) || !"ZYZ_euler_angles(): phi was not an number encountered");
      assert(phi <=  value_traits::pi() || !"ZYZ_euler_angles(): phi was too big");
      assert(phi >= -value_traits::pi() || !"ZYZ_euler_angles(): phi was too small");
    }
    
    
    //
    // So now we have 
    //
    //   Qzy =~ Rz( phi )*Ry( psi );
    //
    // and from this we know
    //
    //   Q = Qzy Qz(theta);
    //
    // so
    //
    //  (Qzy^* Q) = Qz(theta)
    //
    // We also know
    //
    //  (Qzy^* Q) = [s,v] = [ cos(theta/2) , sin(theta/2) * k ]
    //
    // where s is a value_type and v is a 3-vector. k is a unit z-axis and
    // theta is a rotation along that axis.
    //
    // we can get theta/2 by:
    //
    //   theta/2 = atan2 ( sin(theta/2) , cos(theta/2) )
    //
    // but we can not get sin(theta/2) directly, only its absolute value:
    //
    //   |v| = |sin(theta/2)| * |k|  = |sin(theta/2)|
    //
    quaternion_type Qy;
    quaternion_type Qz;
    quaternion_type H;
    Qy = quaternion_type::Ry(psi);
    Qz = quaternion_type::Rz(phi);
    H = prod( conj( prod( Qz , Qy ) ), Q );
    
    vector3_type const w = rotate(H,vector3_type::i());
    
    real_type const w_x = w(0);
    real_type const w_y = w(1);
    assert(is_number(w_x) || !"ZYZ_euler_angles(): w_x was not an number encountered");
    assert(is_number(w_y) || !"ZYZ_euler_angles(): w_y not an number encountered");
    theta = value_traits::numeric_cast( atan2(w_y,w_x) );
    assert(is_number(theta) || !"ZYZ_euler_angles(): phi was not an number encountered");
    assert(theta <=  value_traits::pi() || !"ZYZ_euler_angles(): phi was too big");
    assert(theta >= -value_traits::pi() || !"ZYZ_euler_angles(): phi was too small");
    
    //T ct2 = Q.s();       //---   cos(theta/2)
    //T st2 = norm( v ); //---  |sin(theta/2)|
    
    //// First try positive choice of sin(theta/2)
    //theta = value_traits::two()* atan2(st2,ct2);
    
    return;
  }
  
} // namespace tiny

//TINY_EULER_ANGLES_H
#endif
