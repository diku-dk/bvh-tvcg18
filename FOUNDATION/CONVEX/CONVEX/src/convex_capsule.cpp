#include <convex_shapes.h>

#include <types/geometry_sphere.h>

#include <tiny_math_types.h>
#include <tiny_is_finite.h>

#include <cassert>
#include <cmath>   // needed for std::min and std::sqrt

namespace convex
{
  
  template<typename M>
  typename M::real_type const & Capsule<M>::half_height() const { return this->m_half_height; }
  
  template<typename M>
  typename M::real_type & Capsule<M>::half_height() { return this->m_half_height; }
  
  template<typename M>
  typename M::real_type const & Capsule<M>::radius() const { return this->m_radius; }
  
  template<typename M>
  typename M::real_type & Capsule<M>::radius() { return this->m_radius; }
  
  template<typename M>
  Capsule<M>::Capsule()
  : m_half_height( M::value_traits::one() )
  , m_radius( M::value_traits::one() )
  {}
  
  template<typename M>
  typename M::vector3_type Capsule<M>::get_support_point( typename M::vector3_type const & v) const
  {
    using std::sqrt;
    
    typedef typename M::vector3_type V;
    typedef typename M::value_traits VT;
        
    assert( is_number(v(0)) || !"NAN encountered");
    assert( is_number(v(1)) || !"NAN encountered");
    assert( is_number(v(2)) || !"NAN encountered");
    assert( is_finite(v(0)) || !"INF encountered");
    assert( is_finite(v(1)) || !"INF encountered");
    assert( is_finite(v(2)) || !"INF encountered");
    
    assert( is_number(this->m_half_height)    || !"NAN encountered");
    assert( is_finite(this->m_half_height)    || !"INF encountered");
    assert( this->m_half_height >= VT::zero() || !"Negative half height");
    assert( is_number(this->m_radius)         || !"NAN encountered");
    assert( is_finite(this->m_radius)         || !"INF encountered");
    assert( this->m_radius >= VT::zero()      || !"Negative radius");
    
    geometry::Sphere<V> S;
    S.radius() = this->m_radius;
    
    // Get the support point of the sphere
    V p = S.get_support_point(v);
    
    // Cut the sphere into two halves and displace them along the z-axis.
    if( v(2) > VT::zero() )
      p(2) += this->m_half_height;
    else if( v(2) < VT::zero() )
      p(2) -= this->m_half_height;
    
    assert( is_number(p(0)) || !"NAN encountered");
    assert( is_number(p(1)) || !"NAN encountered");
    assert( is_number(p(2)) || !"NAN encountered");
    assert( is_finite(p(0)) || !"INF encountered");
    assert( is_finite(p(1)) || !"INF encountered");
    assert( is_finite(p(2)) || !"INF encountered");
    
    return p;
  }
  
  template<typename M>
  typename M::real_type Capsule<M>::get_scale() const
  {
    using std::min;
    
    typedef typename M::real_type    T;
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
    
  template class Capsule<Mf>;
  template class Capsule<Md>;
  
} // namespace convex
