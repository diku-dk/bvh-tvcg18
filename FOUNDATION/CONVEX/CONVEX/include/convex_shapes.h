#ifndef CONVEX_SHAPES_H
#define CONVEX_SHAPES_H

#include <types/geometry_support_mapping.h>

#include <vector>

namespace convex
{

  /**
   * The cylinder axis is by default equal to the z-axis.
   */
  template<typename M>
  class Cylinder
  : public geometry::SupportMapping<typename M::vector3_type>
  {
  public:
    
    typedef typename M::real_type     T;
    typedef typename M::vector3_type  V;
    
  protected:
    
    T    m_half_height;     ///< The half height of the cylinder. Default value is one.
    T    m_radius;          ///< The radius of the cylinder. The default value is one.
        
  public:
    
    T const & half_height() const;
    T       & half_height();
    T const & radius() const;
    T       & radius();
    
  public:
    
    Cylinder();
    
  public:
    
    V get_support_point( V const & v ) const;
    
    T get_scale() const;
     
  };
  
  /**
   * The capsule axis is by default equal to the z-axis.
   */
  template<typename M>
  class Capsule
  : public geometry::SupportMapping<typename M::vector3_type>
  {
  public:
    
    typedef typename M::real_type     T;
    typedef typename M::vector3_type  V;
    
  protected:
    
    T m_half_height;     ///< The half height of the Capsule. Default value is one.
    T m_radius;          ///< The radius of the Capsule. The default value is one.
    
  public:
    
    T const & half_height() const;
    T       & half_height();
    T const & radius()      const;
    T       & radius();
    
  public:
    
    Capsule();
    
  public:
    
    V get_support_point( V const & v ) const;
    
    T get_scale() const;
 
  };
  
  template<typename M>
  class Ellipsoid
  : public geometry::SupportMapping<typename M::vector3_type>
  {
  public:
    
    typedef typename M::real_type     T;
    typedef typename M::vector3_type  V;
    
  protected:
    
    V m_scale; /// The scaling of the unit sphere along the x, y and z axes.
        
  public:
    
    V const & scale() const;
    V       & scale();
    
  public:
    
    Ellipsoid();
    
  public:
    
    V get_support_point( V const & v ) const;
    
    T get_scale() const;

  };
  
  /**
   * A Cone.
   * A Cone is placed with its apex on positive z-axis and
   * the base orthogonal to the negative z-axis.
   * 
   * The apex and base are placed equidistant wrt. x-y plane and
   * the distance is given by the half-height of the cone.
   *
   * The base is a circular disk of a specified radius. From the radius
   * and height of the cone one can compute the cone angle at the apex.
   * This angle is denoted alpha.
   */
  template<typename M>
  class Cone
  : public geometry::SupportMapping<typename M::vector3_type>
  {
  public:
    
    typedef typename M::real_type     T;
    typedef typename M::vector3_type  V;
    
  protected:
    
    T    m_half_height;   ///< The half height of the cone. Default value is one.
    T    m_base_radius;   ///< The radius of the circular base of the cone. Default value is one.
        
  public:
    
    T const & half_height() const;
    T       & half_height();
    T const & base_radius() const;
    T       & base_radius();
    
  public:
    
    Cone();
    
  public:
    
    V get_support_point( V const & v ) const;
    
    T get_scale() const;

  };
  
  template<typename M>
  class ConvexHull
  : public geometry::SupportMapping<typename M::vector3_type>
  {
  public:
    
    typedef typename M::real_type     T;
    typedef typename M::vector3_type  V;
    
  protected:
    
    std::vector<V> m_points;
        
  public:
    
    void add_point( V const & p);
    
    V const & get_point( std::size_t const & idx) const;
    
    std::size_t size() const;
    
    void clear();
    
  public:
    
    ConvexHull();
    
  public:
    
    V get_support_point( V const & v ) const;
    
    T get_scale() const;

  };
  
} // namespace convex

// CONVEX_SHAPES_H
#endif
