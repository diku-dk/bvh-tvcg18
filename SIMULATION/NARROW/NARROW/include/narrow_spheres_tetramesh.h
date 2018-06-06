#ifndef NARROW_SPHERES_TETRAMESH_H
#define NARROW_SPHERES_TETRAMESH_H

#include <narrow_object.h>
#include <narrow_geometry.h>

#include <kdop_single_traversal.h>

namespace narrow
{

  namespace detail
  {

    template<typename M>
    inline void spheres_tetramesh(
                                    typename Geometry<M>::sphere_container const & A
                                  , typename M::vector3_type const & tA
                                  , typename M::quaternion_type const & qA
                                  , Object<M> const & objB
                                  , Geometry<M> const & geoB
                                  , typename geometry::ContactsCallback<typename M::vector3_type> & callback
                                  , bool const & should_flip
                                 )
    {
      typedef typename M::value_traits    VT;
      typedef typename M::coordsys_type   C;
      typedef typename M::vector3_type    V;
      typedef typename M::real_type       T;

      typedef typename Geometry<M>::sphere_container::const_iterator sphere_iterator;

      if( A.empty() )
        return;

      C const bodyAtoWCS = C(tA, qA);

      for( sphere_iterator a = A.begin(); a!=A.end(); ++a )
      {
        C const shapeAtobodyA = C(a->transform().T(), a->transform().Q());

        C const shapeAtoWCS = tiny::prod(shapeAtobodyA, bodyAtoWCS);

        geometry::Sphere<V> const sphere = geometry::make_sphere(shapeAtoWCS.T(), a->radius());

        kdop::single_traversal<V, 8, T>(
                                          sphere
                                        , objB.m_tree
                                        , geoB.m_tetramesh.m_mesh
                                        , objB.m_X
                                        , objB.m_Y
                                        , objB.m_Z
                                        , geoB.m_tetramesh.m_surface_map
                                        , callback
                                        , should_flip
                                        );
      }
    }
    
  } // namespace detail
  
} //namespace narrow

// NARROW_SPHERES_TETRAMESH_H
#endif
