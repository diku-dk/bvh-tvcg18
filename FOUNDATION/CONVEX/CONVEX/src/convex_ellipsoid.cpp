#include <convex_shapes.h>

#include <tiny_math_types.h>
#include <tiny_is_finite.h>

#include <cassert>
#include <cmath>     // needed for std::min and std::sqrt

namespace convex
{
  
  template<typename M>
  typename M::vector3_type const & Ellipsoid<M>::scale() const { return this->m_scale; }
  
  template<typename M>
  typename M::vector3_type & Ellipsoid<M>::scale() { return this->m_scale; }
    
  template<typename M>
  Ellipsoid<M>::Ellipsoid()
  : m_scale( M::value_traits::one() )
  {}
  
  template<typename M>
  typename M::vector3_type Ellipsoid<M>::get_support_point(typename M::vector3_type const & v) const
  {
    using std::sqrt;
    
    typedef typename M::real_type    T;
    typedef typename M::vector3_type V;
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
    
    T const & sx = this->m_scale(0);
    T const & sy = this->m_scale(1);
    T const & sz = this->m_scale(2);
    
    assert( sx  || !"NAN encountered");
    assert( sx  || !"INF encountered");
    assert( sy  || !"NAN encountered");
    assert( sy  || !"INF encountered");
    assert( sz  || !"NAN encountered");
    assert( sz  || !"INF encountered");
    assert( sx >= VT::zero() || !"Negative scale encountered");
    assert( sy >= VT::zero() || !"Negative scale encountered");
    assert( sz >= VT::zero() || !"Negative scale encountered");
    
    /*
    // An ellipsoid, E, is simply a scaled unit ball, B, and a scale is a linear
    // transformation, T. We can write it in a general way as
    //
    //  E = T(B)
    //
    // That means we can create a support function, S, of an ellipsoid from
    // that of a unit sphere ball by
    //
    //  S_E(v) = S_{T(B)}(v)
    //
    // Further we know that for any affine transformation, T(v) = R v + t, we have
    //
    //   S_{T(B)}(v)  =   T(  S_B( R^T v )  )
    //
    // In our particular case R = R^T = D, where D = diag(s_0,s_1,s_3) and t=0. Here
    // the s_i's are the axes scales repsectively. Putting it all together we have
    //
    //   S_{E}(v)  =   D(  S_B( D v )  )
    //
    // This is the formula implemented by this functor.
    */
    
    T const vv = vx*vx + vy*vy + vz*vz;
    
    assert( is_number(vv) || !"NAN encountered");
    assert( is_finite(vv) || !"INF encountered");
    
    T px = VT::zero();
    T py = VT::zero();
    T pz = VT::zero();
    
    if (vv > VT::zero() )
    {
      T const wx = vx * sz;
      T const wy = vy * sy;
      T const wz = vz * sz;
      T const ww  = wx*wx + wy*wy + wz*wz;
      
      assert( is_number(ww) || !"NAN encountered");
      assert( is_finite(ww) || !"INF encountered");
      
      T const tmp = VT::one() / sqrt(ww);
      
      assert( is_number(tmp) || !"NAN encountered");
      assert( is_finite(tmp) || !"INF encountered");
      
      px = wx * tmp * sx;
      py = wy * tmp * sy;
      pz = wz * tmp * sz;
    }
    else
    {
      px = sx;
      py = VT::zero();
      pz = VT::zero();
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
  typename M::real_type Ellipsoid<M>::get_scale() const
  {
    using std::min;
  
    typedef typename M::real_type    T;
    typedef typename M::value_traits VT;
    
    T const & sx = this->m_scale(0);
    T const & sy = this->m_scale(1);
    T const & sz = this->m_scale(2);
    
    assert( is_number(sx ) || !"NAN encountered");
    assert( is_finite(sx ) || !"INF encountered");
    assert( is_number(sy ) || !"NAN encountered");
    assert( is_finite(sy ) || !"INF encountered");
    assert( is_number(sz ) || !"NAN encountered");
    assert( is_finite(sz ) || !"INF encountered");
    assert( sx >= VT::zero() || !"Negative scale encountered");
    assert( sy >= VT::zero() || !"Negative scale encountered");
    assert( sz >= VT::zero() || !"Negative scale encountered");
    
    T const w = VT::two() * ((sx > VT::zero()) ? sx : VT::infinity());
    T const h = VT::two() * ((sy > VT::zero()) ? sy : VT::infinity());
    T const d = VT::two() * ((sz > VT::zero()) ? sz : VT::infinity());
    
    return min(w, min(h, d));
  }
  
  typedef tiny::MathTypes<float>  Mf;
  typedef tiny::MathTypes<double> Md;
  
  template class Ellipsoid<Mf>;
  template class Ellipsoid<Md>;
  
} // namespace convex
