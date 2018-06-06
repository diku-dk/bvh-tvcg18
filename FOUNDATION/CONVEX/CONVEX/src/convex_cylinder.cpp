#include <convex_shapes.h>

#include <tiny_math_types.h>
#include <tiny_is_finite.h>

#include <cassert>
#include <cmath>    // needed for std::min and std::sqrt

namespace convex
{
  
  template<typename M>
  typename M::real_type const & Cylinder<M>::half_height() const { return this->m_half_height; }
  
  template<typename M>
  typename M::real_type & Cylinder<M>::half_height() { return this->m_half_height; }
  
  template<typename M>
  typename M::real_type const & Cylinder<M>::radius() const { return this->m_radius; }
  
  template<typename M>
  typename M::real_type & Cylinder<M>::radius() { return this->m_radius; }
  
  template<typename M>
  Cylinder<M>::Cylinder()
  : m_half_height( M::value_traits::one() )
  , m_radius( M::value_traits::one() )
  {}
  
  template<typename M>
  typename M::vector3_type Cylinder<M>::get_support_point(typename M::vector3_type const & v) const
  {
    using std::sqrt;
    
    typedef typename M::real_type     T;
    typedef typename M::vector3_type  V;
    typedef typename M::value_traits VT;
    
    T const & vx = v(0);
    T const & vy = v(1);
    T const & vz = v(2);
    
    assert( is_number(vx) || !"NAN encountered");
    assert( is_number(vy) || !"NAN encountered");
    assert( is_number(vz) || !"NAN encountered");
    assert( is_finite(vx) || !"INF encountered");
    assert( is_finite(vy) || !"INF encountered");
    assert( is_finite(vz) || !"INF encountered");
    
    assert( is_number(this->m_half_height)    || !"NAN encountered");
    assert( is_finite(this->m_half_height)    || !"INF encountered");
    assert( this->m_half_height >= VT::zero() || !"Negative half height");
    assert( is_number(this->m_radius)         || !"NAN encountered");
    assert( is_finite(this->m_radius)         || !"INF encountered");
		assert( this->m_radius >= VT::zero()      || !"Negative radius");
    
    T const norm_sigma     = sqrt( vx*vx + vy*vy );
    assert( is_number(norm_sigma)    || !"NAN encountered");
    assert( is_finite(norm_sigma)    || !"INF encountered");
    assert( norm_sigma >= VT::zero() || !"Norm can not be negative");
    
    T const h = vz > VT::zero() ? this->m_half_height : -this->m_half_height;
    assert( is_number( h ) || !"NAN encountered");
    assert( is_finite( h ) || !"INF encountered");
    
    T px = VT::zero();
    T py = VT::zero();
    T pz = VT::zero();
    
    // Test if search direction has any radial component
    if(norm_sigma > VT::zero() )
    {
      px = ((this->m_radius) * vx) /norm_sigma;
      py = ((this->m_radius) * vy) /norm_sigma;
      pz =  h;
    }
    else
    {
      // Search direction is parallel with z-axis
      //
      // Or search direction is zero in which case we just some point as the support point.
      px = VT::zero();
      py = VT::zero();
      pz = h;
    }
    
    assert( is_number(px) || !"NAN encountered");
    assert( is_number(py) || !"NAN encountered");
    assert( is_number(pz) || !"NAN encountered");
    assert( is_finite(px) || !"INF encountered");
    assert( is_finite(py) || !"INF encountered");
    assert( is_finite(pz) || !"INF encountered");
    
    return V::make(px,py,pz);
  }
  
  template<typename M>
  typename M::real_type Cylinder<M>::get_scale() const
  {
    using std::min;
    
    typedef typename M::real_type     T;
    typedef typename M::value_traits VT;
    
    assert( is_number(this->m_half_height)    || !"NAN encountered");
    assert( is_finite(this->m_half_height)    || !"INF encountered");
    assert( this->m_half_height >= VT::zero() || !"Negative half height");
    assert( is_number(this->m_radius)         || !"NAN encountered");
    assert( is_finite(this->m_radius)         || !"INF encountered");
		assert( this->m_radius >= VT::zero()      || !"Negative radius");
    
    T const d = VT::two() * ((this->m_radius > VT::zero()) ? this->m_radius : VT::infinity());
    T const h = VT::two() * ((this->m_half_height > VT::zero()) ? this->m_half_height : VT::infinity());
    
    return min(h, d);
  }
  
  typedef tiny::MathTypes<float>  Mf;
  typedef tiny::MathTypes<double> Md;
  
  template class Cylinder<Mf>;
  template class Cylinder<Md>;
  
} // namespace convex
