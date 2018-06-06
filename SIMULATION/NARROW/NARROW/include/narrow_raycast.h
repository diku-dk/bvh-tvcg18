#ifndef NARROW_RAYCAST_H
#define NARROW_RAYCAST_H

#include <narrow_object.h>
#include <narrow_geometry.h>

#include <tiny.h>

#include <geometry.h>

#include <kdop_raycast.h>

namespace narrow
{

  template<typename M>
  inline bool raycast(
                        geometry::Ray<typename M::vector3_type> const & ray
                      , Object<M>                               const & objA
                      , Geometry<M>                             const & geoA
                      , typename M::vector3_type                const & tA
                      , typename M::quaternion_type             const & qA
                      , typename M::vector3_type                      & point
                      , typename M::real_type                         & distance
                      )
  {
    typedef typename M::value_traits    VT;
    typedef typename M::vector3_type    V;
    typedef typename M::real_type       T;
    typedef typename M::coordsys_type   C;

    typedef typename Geometry<M>::box_container::const_iterator      box_iterator;
    typedef typename Geometry<M>::sphere_container::const_iterator   sphere_iterator;
    typedef typename Geometry<M>::convex_container::const_iterator   hull_iterator;

    point    = V::zero();
    distance = VT::infinity();

    if ( geoA.m_tetramesh.has_data() )
    {
      return kdop::raycast<V, 8>(
                                 ray
                                 , objA.m_tree
                                 , geoA.m_tetramesh.m_mesh
                                 , objA.m_X
                                 , objA.m_Y
                                 , objA.m_Z
                                 , geoA.m_tetramesh.m_surface_map
                                 , point
                                 , distance
                                 );

    }
    else
    {
      C const bodyAtoWCS = C(tA, qA);

      if (! geoA.m_boxes.empty() )
      {
        for( box_iterator a = geoA.m_boxes.begin(); a!= geoA.m_boxes.end(); ++a )
        {
          C const shapeAtobodyA = C(a->transform().T(), a->transform().Q());
          C const shapeAtoWCS   = tiny::prod(shapeAtobodyA, bodyAtoWCS);

          geometry::OBB<M> const obb  = geometry::make_obb<M>( shapeAtoWCS.T(), shapeAtoWCS.Q(), a->half_extent());

          T local_distance = VT::infinity();
          V local_point    = V::zero();

          bool const did_hit = geometry::compute_raycast_obb(ray, obb, local_point, local_distance);

          distance = did_hit ? local_distance : distance;
          point    = did_hit ? local_point    : point;
        }
      }
      if (! geoA.m_spheres.empty() )
      {
        for( sphere_iterator a = geoA.m_spheres.begin(); a!= geoA.m_spheres.end(); ++a )
        {
          C const shapeAtobodyA = C(a->transform().T(), a->transform().Q());
          C const shapeAtoWCS   = tiny::prod(shapeAtobodyA, bodyAtoWCS);

          geometry::Sphere<V> const sphere = geometry::make_sphere( shapeAtoWCS.T(), a->radius());

          T local_distance = VT::infinity();
          V local_point    = V::zero();

          bool const did_hit = geometry::compute_raycast_sphere(ray, sphere, local_point, local_distance);

          distance = did_hit ? local_distance : distance;
          point    = did_hit ? local_point    : point;
        }
      }
      if (! geoA.m_hulls.empty() )
      {
        for( hull_iterator a = geoA.m_hulls.begin(); a!= geoA.m_hulls.end(); ++a )
        {
          C const shapeAtobodyA = C(a->transform().T(), a->transform().Q());
          C const shapeAtoWCS   = tiny::prod(shapeAtobodyA, bodyAtoWCS);

          assert(false || !"not implemented yet");
          //          geometry::ConvexHull<V> const hull = geometry::make_convex_hull( shapeAtoWCS.T(), a->radius());

          T local_distance = VT::infinity();
          V local_point    = V::zero();

          bool const did_hit = false; //geometry::compute_raycast_convex(ray, hull, local_point, local_distance);

          distance = did_hit ? local_distance : distance;
          point    = did_hit ? local_point    : point;

        }
      }

    }

    return distance < VT::infinity();

  }

} //namespace narrow
  
// NARROW_RAYCAST_H
#endif
