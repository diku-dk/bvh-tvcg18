#ifndef NARROW_UPDATE_KDOP_BVH_H
#define NARROW_UPDATE_KDOP_BVH_H

#include <kdop_refit_tree.h>

#include <mesh_array.h>

#include <narrow_geometry.h>
#include <narrow_object.h>
#include <narrow_tags.h>

#include <tiny_quaternion_functions.h>

#include <util_profiling.h>

#include <cassert>
#include <vector>

namespace narrow
{

  /**
   * This class encapsulates all information necessary for batch processing
   * of KDOP-BVH updates of objects.
   *
   * @tparam M a typebinder of math types.
   */
  template< typename M >
  class KDopBvhUpdateWorkItem
  {
  public:
    
    typedef typename M::vector3_type     V;
    typedef typename M::quaternion_type  Q;
    
  protected:

    Object<M>    * m_object;
    Geometry<M> const * m_geometry;
    V              m_p;
    Q              m_q;

  private:

    bool is_valid() const
    {
      if (this->m_object == 0)
        return false;
      if (this->m_geometry == 0)
        return false;
      return true;
    }

  public:

    Object<M> & object() const
    {
      assert(this->is_valid() || "KDopBvhUpdateWorkItem::object(): null pointer");

      return *(this->m_object);
    }

    Geometry<M> const & geometry() const
    {
      assert(this->is_valid() || "KDopBvhUpdateWorkItem::geometry() : null pointer");

      return *(this->m_geometry);
    }

    V const & p() const
    {
      assert(this->is_valid() || "KDopBvhUpdateWorkItem::p(): null pointer");

      return this->m_p;
    }

    Q const & q() const
    {
      assert(this->is_valid() || "KDopBvhUpdateWorkItem::q(): null pointer");

      return this->m_q;
    }

  public:
    
    KDopBvhUpdateWorkItem()
    : m_object(0)
    , m_geometry(0)
    , m_p()
    , m_q()
    {}

    ~KDopBvhUpdateWorkItem()
    {}

    KDopBvhUpdateWorkItem(  Object<M> & object
                          , Geometry<M> const & geometry
                          , V const & p
                          , Q const & q
                          )
    : m_object(&object)
    , m_geometry( &geometry   )
    , m_p(p)
    , m_q(q)
    {}

    KDopBvhUpdateWorkItem( KDopBvhUpdateWorkItem<M> const & item )
    {
      *this = item;
    }

    KDopBvhUpdateWorkItem<M> & operator=( KDopBvhUpdateWorkItem<M> const & item )
    {
      if( this != &item )
      {
        this->m_object   = item.m_object;
        this->m_geometry = item.m_geometry;
        this->m_p        = item.m_p;
        this->m_q        = item.m_q;
      }
      return *this;
    }

  };
  
  template<typename M>
  inline void update_kdop_bvh(  std::vector< KDopBvhUpdateWorkItem< M > > & work_pool
                              , sequential const & /* tag */
                              )
  {
    assert( !work_pool.empty() || "update_kdop_bvh : update_kdop_bvh_objects are empty" );
    
    typedef typename M::real_type                                      T;
    typedef typename M::vector3_type                                   V;
    typedef typename M::quaternion_type                                Q;
    typedef typename std::vector< KDopBvhUpdateWorkItem< M > >::iterator work_item_iterator;
    
    work_item_iterator current = work_pool.begin();
    work_item_iterator end     = work_pool.end();
    
    START_TIMER("refit_tree");
    
    for (; current != end; ++current)
    {
      Object<M>         & object   = current->object();
      Geometry<M> const & geometry = current->geometry();
      V           const   p        = current->p();
      Q           const   q        = current->q();
      size_t      const   N        = geometry.m_tetramesh.m_mesh.vertex_size();

      if( N <= 0u)
        continue;
      
      for(size_t n = 0u; n < N;++n)
      {
        mesh_array::Vertex const & v = geometry.m_tetramesh.m_mesh.vertex(n);
        
        V const r0 = V::make( geometry.m_tetramesh.m_X0(v), geometry.m_tetramesh.m_Y0(v), geometry.m_tetramesh.m_Z0(v) );
        
        V const r = tiny::rotate(q, r0) + p;
        
        object.m_X(v) = r(0);
        object.m_Y(v) = r(1);
        object.m_Z(v) = r(2);
      }
            
      kdop::refit_tree<V,8,T>(  object.m_tree
                              , geometry.m_tetramesh.m_mesh
                              , object.m_X, object.m_Y, object.m_Z
                              , kdop::sequential()
                              );
    }
    
    STOP_TIMER("refit_tree");
    
  }
  
} // namespace narrow

// NARROW_UPDATE_KDOP_BVH_H
#endif