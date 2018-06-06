#ifndef GEOMETRY_CONTACTS_SPHERE_SPHERE_H
#define GEOMETRY_CONTACTS_SPHERE_SPHERE_H

#include <types/geometry_sphere.h>

#include <contacts/geometry_contacts_callback.h>

#include <tiny_is_number.h>
#include <tiny_is_finite.h>

#include <cassert>

namespace geometry
{

  /**
   * Sphere Sphere Contact Point Generation
   *
   * @param A          First sphere.
   * @param B          Second sphere.
   * @param envelope   The size of the collision envelope.
   * @param callback   A pointer to a callback interface which is used to add contact point information
   *
   */
  template<typename V>
  inline void contacts_sphere_sphere(
                                    Sphere<V> const & A
                                    , Sphere<V> const & B
                                    , typename V::real_type const & envelope
                                    , ContactsCallback<V> & callback
                                    )
  {
    using std::sqrt;

    typedef typename V::real_type       T;
    typedef typename V::value_traits    VT;

    V const & p_a = A.center();
    T const & rA  = A.radius();
    V const & p_b = B.center();
    T const & rB  = B.radius();

    //--- Make a vector between the two sphere centers.
    V const d  = p_b - p_a;
    T const rs = rA + rB;

    assert( is_number(rs) || !"contacts_sphere_sphere(): nan");
    assert( is_finite(rs) || !"contacts_sphere_sphere(): inf");
    assert( rs>VT::zero() || !"contacts_sphere_sphere(): radius sum non-positive");

    //--- If the distance between the two center spheres are greater than
    //--- the sum of their radius' then there can be no contact between
    //--- the two spheres.
    //---
    //--- The tests is performed using squared distances to avoid the
    //--- square root in the distance computations.
    T const r2eps = (rs + envelope)*(rs + envelope);
    T const d2    = inner_prod(d,d);

    if(d2 > r2eps)
    {
      // either make sure this never happens or make sure the returned p, n and distance is never used!!!
      return;
    }


    //--- The contact normal is simply the normalized vector between
    //--- the sphere centers.
    //real_type squared_radius_sum = radius_sum*radius_sum;
    T const lgh = sqrt(d2);

    assert( is_number(lgh) || !"contacts_sphere_sphere(): nan");
    assert( is_finite(lgh) || !"contacts_sphere_sphere(): inf");
    assert( lgh>VT::zero() || !"contacts_sphere_sphere(): lgh non-positive");

    V const n      = d/lgh;

    //--- The contact point is kind of a midpoint weighted by the radius of the
    //--- spheres. That is the contact point, p, is computed  according to the
    //--- proportionality equation
    //---
    //---  |p-p_a| /|p-p_b| = rA / rB
    //---
    //--- We know
    //---
    //---   dA = |p-p_a|
    //---   dB = |p-p_b|
    //---    d = |p_b-p_a| = dA + dB
    //---
    //---        dA = dB (rA/rB)
    //---   dB + dA = dB + dB (rA/rB)   /*  add dB to both sides */
    //---         d = dB(1+rA/rB)       /*  use d=dA+dB          */
    //---
    //---        dB = d/(1+rA/rB)
    //---      d-dA = d/(1+rA/rB)       /*  use d=dA+dB          */
    //---        dA = d - d/(1+rA/rB)
    //---
    T const dB     = (lgh / ( (rA/rB) + VT::one()));
    T const dA     = lgh - dB;
    T const depth  = lgh - rs;

    assert( is_number(dB)    || !"contacts_sphere_sphere(): nan");
    assert( is_finite(dB)    || !"contacts_sphere_sphere(): inf");
    assert( is_number(dA)    || !"contacts_sphere_sphere(): nan");
    assert( is_finite(dA)    || !"contacts_sphere_sphere(): inf");
    assert( is_number(depth) || !"contacts_sphere_sphere(): nan");
    assert( is_finite(depth) || !"contacts_sphere_sphere(): inf");

    V const p       = n*dA + p_a;

    assert( is_number(p(0)) || !"contacts_sphere_sphere(): nan");
    assert( is_finite(p(0)) || !"contacts_sphere_sphere(): inf");
    assert( is_number(p(1)) || !"contacts_sphere_sphere(): nan");
    assert( is_finite(p(1)) || !"contacts_sphere_sphere(): inf");
    assert( is_number(p(2)) || !"contacts_sphere_sphere(): nan");
    assert( is_finite(p(2)) || !"contacts_sphere_sphere(): inf");

    if( depth <=  envelope  )
      callback(p,n,depth);
  }

}//namespace geometry

//GEOMETRY_CONTACTS_SPHERE_SPHERE_H
#endif