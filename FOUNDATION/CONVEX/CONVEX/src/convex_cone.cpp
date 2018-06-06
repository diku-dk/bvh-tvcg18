#include <convex_shapes.h>

#include <tiny_math_types.h>
#include <tiny_is_finite.h>

#include <cassert>
#include <cmath>     // needed for std::min and std::sqrt

namespace convex
{
  template<typename M>
  typename M::real_type const & Cone<M>::half_height() const { return this->m_half_height; }
  
  template<typename M>
  typename M::real_type       & Cone<M>::half_height()       { return this->m_half_height; }
  
  template<typename M>
  typename M::real_type const & Cone<M>::base_radius() const { return this->m_base_radius; }
  
  template<typename M>
	typename M::real_type       & Cone<M>::base_radius()       { return this->m_base_radius; }
  
  template<typename M>
  Cone<M>::Cone()
  : m_half_height( M::value_traits::one() )
  , m_base_radius( M::value_traits::one() )
  {}
  
  template<typename M>
  typename M::vector3_type Cone<M>::get_support_point(typename M::vector3_type const& v) const
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
    assert( is_number(this->m_base_radius)    || !"NAN encountered");
    assert( is_finite(this->m_base_radius)    || !"INF encountered");
    assert( this->m_base_radius >= VT::zero() || !"Negative base radius");
    
    T const vv = vx*vx + vy*vy + vz*vz;
    assert( is_number(vv)    || !"NAN encountered");
    assert( is_finite(vv)    || !"INF encountered");
    
    T px = VT::zero();
    T py = VT::zero();
    T pz = VT::zero();
    
    // Test if we have a valid search direction. If not we simply return some boundary point of the cone
    if (vv <= VT::zero() )
    {
      px = VT::zero();
      py = VT::zero();
      pz = -this->m_half_height;
      
      assert( is_number(px) || !"NAN encountered");
      assert( is_number(py) || !"NAN encountered");
      assert( is_number(pz) || !"NAN encountered");
      assert( is_finite(px) || !"INF encountered");
      assert( is_finite(py) || !"INF encountered");
      assert( is_finite(pz) || !"INF encountered");
      
      return V::make(px,py,pz);
    }
    
    /*
    // Now we know that we have a valid search direction. We proceed by
    // performing a case-by-case analysis. The theoretical back-ground
    // of the analysis is outlined below.
    
    // Since a Cone is radially symmetric around the z-axis we choose to
    // perform the analysis in the radial half-plane orthogonal to the
    // search direction.
    //
    // In this radial half plane the cone is simply a triangle. Looking like this
    //
    //                                  //
    //  z=+h --+        apex            //
    //         |\                       //
    //         | \                      //
    //  z=0 ---+  \                     //
    //         |   \                    //
    //         |    \                   //
    //  z=-h --+-----+---->             //
    //         |     |
    //        r=0   r=R
    //          base
    //
    // The height can be computed as
    //
    //   H = 2*h
    //
    // The angle, alpha at the apex, can be found from the
    // trigonometric relation
    //
    //   sin(alpha) = \frac{R}{H}
    //
    // If the appex is the support point then it must mean
    // that the search direction, s, is contained in the voronoi
    // region of the appex. By geometric relations we observe
    // that the voroni region is defined by the alpha-angle.
    //
    //                                _                             //
    //         |    /                 /|  search direction          //
    //         |  /                  /                              //
    //         |/  alpha            /                               //
    //  apex   +----------         / gamma                          //
    //         |\                 +-------                          //
    //         | \                                                  //
    //  z=0 ---+  \                                                 //
    //         |   \                                                //
    //         |    \                                               //
    //         +-----+---->                                         //
    //
    // So we need to test if angle, gamma, of the search direction is
    // larger than the apex angle. That is
    //
    //   gamma >= alpha   (*1)
    //
    // From geometry we find
    //
    // sine(gamma) =  \frac{s_z}{\norm{s}}
    //
    // Fortunately the sine is a monotome function of the
    // interval [0..pi/2] so (*1) can stated as
    //
    //    sine(gamma) >= sine(alpha)
    //
    // By substitution of trigonometric terms this can be rewritten as
    //
    //   s_z >= \norm{s} \frac{B}{H}
    //
    // Observe that we have included the voronoi region of the inclined
    // cone side into the voronoi region of the appex. Therefor the next
    // voroni-region to consider is the base corner.
    //
    // If the first test fails then it suffices to test if the search
    // direction has any component in the radial direction. If this is
    // the case then the base corner must be a support point.
    //
    // If the two previous test-case both fails it means we have
    // a downward search direction and any point on the base of the
    // cone is a support point.
    */
    
    T const sine_alpha = this->m_base_radius / (VT::two()*this->m_half_height);
    assert( is_number(sine_alpha)    || !"NAN encountered");
    assert( is_finite(sine_alpha)    || !"INF encountered");
    assert( sine_alpha >= VT::zero() || !"sine(alpha) can not be negative");
    assert( sine_alpha <= VT::one()  || !"sine(alpha) can not be larger than one");
    
    T const norm_v     = sqrt(vv);
    assert( is_number(norm_v)    || !"NAN encountered");
    assert( is_finite(norm_v)    || !"INF encountered");
    assert( norm_v >= VT::zero() || !"Norm can not be negative");
    
    // Test if search direction is in Apex voronoi region
    if( vz >= norm_v*sine_alpha)
    {
      px = VT::zero();
      py = VT::zero();
      pz = this->m_half_height;
      
      assert( is_number(px) || !"NAN encountered");
      assert( is_number(py) || !"NAN encountered");
      assert( is_number(pz) || !"NAN encountered");
      assert( is_finite(px) || !"INF encountered");
      assert( is_finite(py) || !"INF encountered");
      assert( is_finite(pz) || !"INF encountered");
      
      return V::make(px,py,pz);
    }
    
    T const norm_sigma     = sqrt( vx*vx + vy*vy );
    assert( is_number(norm_sigma)    || !"NAN encountered");
    assert( is_finite(norm_sigma)    || !"INF encountered");
    assert( norm_sigma >= VT::zero() || !"Cone::operator(): Norm can not be negative");
    
    // Test if search direction has any radial component
    if(norm_sigma > VT::zero() )
    {
      px = this->m_base_radius*vx/norm_sigma;
      py = this->m_base_radius*vy/norm_sigma;
      pz = -(this->m_half_height);
      
      assert( is_number(px) || !"NAN encountered");
      assert( is_number(py) || !"NAN encountered");
      assert( is_number(pz) || !"NAN encountered");
      assert( is_finite(px) || !"INF encountered");
      assert( is_finite(py) || !"INF encountered");
      assert( is_finite(pz) || !"INF encountered");
      
      return V::make(px,py,pz);
    }
    
    // Search direction must be straight down
    px = VT::zero();
    py = VT::zero();
    pz = -this->m_half_height;
    
    assert( is_number(px) || !"NAN encountered");
    assert( is_number(py) || !"NAN encountered");
    assert( is_number(pz) || !"NAN encountered");
    assert( is_finite(px) || !"INF encountered");
    assert( is_finite(py) || !"INF encountered");
    assert( is_finite(pz) || !"INF encountered");
    
    return V::make(px,py,pz);
  }
  
  template<typename M>
  typename M::real_type Cone<M>::get_scale() const
  {
    using std::min;
    
    typedef typename M::real_type     T;
    typedef typename M::value_traits VT;
    
    assert( is_number(this->m_half_height)    || !"NAN encountered");
    assert( is_finite(this->m_half_height)    || !"INF encountered");
    assert( this->m_half_height >= VT::zero() || !"Negative half height");
    assert( is_number(this->m_base_radius)    || !"NAN encountered");
    assert( is_finite(this->m_base_radius)    || !"INF encountered");
    assert( this->m_base_radius >= VT::zero() || !"Negative base radius");
    
    T const d = VT::two() * ((this->m_base_radius > VT::zero()) ? this->m_base_radius : VT::infinity());
    T const h = VT::two() * ((this->m_half_height > VT::zero()) ? this->m_half_height : VT::infinity());
    
    return min(h, d);
  }
  
  typedef tiny::MathTypes<float> Mf;
  typedef tiny::MathTypes<double> Md;
  
  template class Cone<Mf>;
  template class Cone<Md>;
  
} // namespace convex
