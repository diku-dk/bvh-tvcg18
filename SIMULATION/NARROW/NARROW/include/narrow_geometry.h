#ifndef NARROW_GEOMETRY_H
#define NARROW_GEOMETRY_H

#include <narrow_shape_types.h>

#include <tiny_quaternion_functions.h>
#include <tiny_coordsys_functions.h>

#include <vector>
#include <algorithm>

namespace narrow
{
  
  /**
   * Narrow Phase Geometry Type.
   * The geometry of an object consists of a collection of shapes. The object defines
   * a local coordinate system in which the shapes live. Ie. each shape has an associated
   * transform that specifies how the shape is placed in the local object frame. When an
   * object is simulated then the simulator essientially moves the local object frame
   * to a new placement. However, the shapes rigidly follows the local object frames as it
   * moves around.
   *
   * This class offers an interface for easy manipulation of the shapes that make up the geometry
   * of an object.
   *
   * @tparam M     The Math types used.
   */
  template<typename M>
  class Geometry
  {
  protected:
    
    typedef typename M::vector3_type                      V;
    typedef typename M::value_traits                      VT;
    typedef typename M::real_type                         T;
    typedef typename M::coordsys_type                     C;
    
  public:
    
    typedef          detail::ShapeTypes<M>           shape_types;
    
    typedef typename shape_types::Box                box_type;
    typedef typename shape_types::Sphere             sphere_type;
    typedef typename shape_types::ConvexHull         convex_type;
    typedef typename shape_types::Tetramesh          tetramesh_type;

    typedef std::vector<box_type>                         box_container;
    typedef std::vector<sphere_type>                      sphere_container;
    typedef std::vector<convex_type>                      convex_container;

  public:
    
    box_container           m_boxes;
    sphere_container        m_spheres;
    convex_container        m_hulls;
    tetramesh_type          m_tetramesh;

    T                       m_radius;

  public:
    
    Geometry()
    : m_boxes()
    , m_spheres()
    , m_hulls()
    , m_tetramesh()
    , m_radius( VT::zero() )
    {
    }
    
    Geometry( Geometry const & geo){ *this = geo; }
    
    Geometry& operator=( Geometry const & geo)
    {
      if( this != &geo)
      {
        this->m_boxes        = geo.m_boxes;
        this->m_spheres      = geo.m_spheres;
        this->m_hulls        = geo.m_hulls;
        this->m_tetramesh    = geo.m_tetramesh;

        this->m_radius       = geo.m_radius;
      }
      return *this;
    }
    
    ~Geometry()
    {
      this->clear();
    }
    
  protected:
    
    template< typename container_type, typename shape_type>
    inline void add_shape( container_type & container, shape_type const & shape )
    {
      container.push_back( shape );
    }
    
  public:
    
    /**
     * Add Shape to geoemtry.
     *
     * @warning   This method will create a copy of the added shape inside the geometry class.
     *            Thus, subsequent changes to the shape argument instance after invokation
     *            has not effect on the geometry.
     */
    void add_shape( box_type       const & box    )  {  this->add_shape( this->m_boxes, box );          }
    void add_shape( sphere_type    const & sphere )  {  this->add_shape( this->m_spheres, sphere );     }
    void add_shape( convex_type    const & hull   )  {  this->add_shape( this->m_hulls, hull );         }
    void add_shape( tetramesh_type const & mesh   )  {  this->m_tetramesh = mesh;                       }

    size_t number_of_boxes()         const {  return this->m_boxes.size();                       }
    size_t number_of_spheres()       const {  return this->m_spheres.size();                     }
    size_t number_of_hulls()         const {  return this->m_hulls.size();                       }
    size_t number_of_tetrameshes()   const {  return this->m_tetramesh.has_data() ? 1 : 0;       }


    void clear()
    {
      this->m_boxes.clear();
      this->m_spheres.clear();
      this->m_hulls.clear();
      this->m_tetramesh.clear();
    }
    
  public:
    
    void update_radius()
    {
      using std::sqrt;
      using std::max;
      
      this->m_radius = VT::zero();
      
      for (size_t i = 0; i < number_of_boxes(); ++i)
      {
        V const & p = this->m_boxes[i].transform().T();
        V const & e = this->m_boxes[i].half_extent();
        
        this->m_radius = max ( this->m_radius, norm(e) + norm(p) );
      }

      for (size_t i = 0; i < number_of_spheres(); ++i)
      {
        V const p = this->m_spheres[i].transform().T();
        T const r = this->m_spheres[i].radius();
        
        this->m_radius = max ( this->m_radius, r + norm(p) );
      }
      
      for (size_t i = 0; i < number_of_hulls(); ++i)
      {
        C const X = this->m_hulls[i].transform();

        for (size_t j=0u; j < this->m_hulls[i].data().size();++j)
        {
          V const p = this->m_hulls[i].data().get_point(j);
          V const q = tiny::xform_point(X,p);

          this->m_radius = max ( this->m_radius, norm(q) );
        }
      }
    }
    
    T const & get_radius() const
    {
      using std::max;
      return max(this->m_radius, this->m_tetramesh.m_mesh_radius);
    }
    
  };
  
} //namespace narrow

// NARROW_GEOMETRY_H
#endif 
