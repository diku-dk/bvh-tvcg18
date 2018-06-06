#ifndef GEOMETRY_CONTACTS_SPHERE_TETRAHEDRON_H
#define GEOMETRY_CONTACTS_SPHERE_TETRAHEDRON_H

#include <types/geometry_sphere.h>
#include <types/geometry_tetrahedron.h>
#include <types/geometry_plane.h>
#include <types/geometry_triangle.h>
#include <types/geometry_line.h>
#include <closest_points/geometry_closest_point_on_line.h>
#include <closest_points/geometry_closest_point_on_plane.h>

#include <contacts/geometry_contacts_callback.h>

#include <vector>
#include <cassert>

namespace geometry
{

  /**
   *
   * @param flip          By default the contact normal points from A to B.
   *                      If this flag is set to true then the normal
   *                      points from B to A.
   *
   * @param surface_map   This 4-entry large array tells which faces of
   *                      the tetrahedron that are on the surface of the
   *                      "object". For a single isolated tetrahedron all faces
   *                      are surface-faces. However, for tetrahedra in a tetrahedra
   *                      mesh, there might be fewer of the triangle faces
   *                      lying on the actual surface of the object represented
   *                      by the tetrahedral mesh.
   */
  template<typename V>
  inline bool contacts_sphere_tetrahedron(
                                          Sphere<V> const & A
                                          , Tetrahedron<V> const & B
                                          , ContactsCallback<V> & callback
                                          , bool const flip
                                          , std::vector<bool> const & surface_map
                                          )
  {
    assert(surface_map.size() == 4u                                                || !"contacts_sphere_tetrahedron(): internal error, must have four surface map values");
    assert( (surface_map[0] || surface_map[1] || surface_map[2] || surface_map[3]) || !"contacts_sphere_tetrahedron(): internal error, tetrahedron must have at least one surface face");

    typedef typename V::real_type    T;
    typedef typename V::value_traits VT;

    std::vector<T>            d(4);
    std::vector<Triangle<V> > triangle(4);
    std::vector<Plane<V> >    plane(4);

    for (unsigned int k = 0u; k < 4u; ++k)
    {
      triangle[k] = get_opposite_face( k, B );
      plane[k]    = make_plane(triangle[k]);
      d[k]        = get_signed_distance( A.center(), plane[k] );

      // Sphere too far from tetrahedron face to ever come in contact
      if (d[k] > A.radius())
        return false;
    }

    // Test if sphere center is inside tetrahedron
    if( d[0] < VT::zero() && d[1] < VT::zero() && d[2] < VT::zero() && d[3] < VT::zero() )
    {

      // Find the tetrahedron plane with smallest penetration depth
      for (unsigned int k = 0u; k < 4u; ++k)
      {
        if( d[k] < d[(k+1) % 4] )
          continue;

        if( d[k] < d[(k+2) % 4] )
          continue;

        if( d[k] < d[(k+3) % 4] )
          continue;

        V const n = flip ? plane[k].n() : - plane[k].n();

        V const p = closest_point_on_plane(A.center(), plane[k]);

        callback( p, n, d[k] );

        return true;
      }
    }

    // Test if the sphere center lies in a voronoi region of a vertex
    for (unsigned int i=0u; i<4u; ++i)
    {
      if( ! ( surface_map[ (i+1)%4 ] || surface_map[ (i+2)%4 ] || surface_map[ (i+3)%4 ] ) )  // Test if vertex is part of the surface
        continue;

      Plane<V> const vp_01 = make_plane( tiny::unit( B.p(i)-B.p( (i+1)%4) ), B.p(i) );
      Plane<V> const vp_02 = make_plane( tiny::unit( B.p(i)-B.p( (i+2)%4) ), B.p(i) );
      Plane<V> const vp_03 = make_plane( tiny::unit( B.p(i)-B.p( (i+3)%4) ), B.p(i) );

      T const d_vp_01 = get_signed_distance( A.center(), vp_01 );
      T const d_vp_02 = get_signed_distance( A.center(), vp_02 );
      T const d_vp_03 = get_signed_distance( A.center(), vp_03 );

      if(d_vp_01 >= VT::zero() && d_vp_02 >= VT::zero() && d_vp_03 >= VT::zero())
      {
        V const m =  A.center() - B.p(i);

        T const d = tiny::norm( m );

        if (d > A.radius() )
          return false;

        V const n = flip ? tiny::unit(m) : - tiny::unit(m);

        callback( B.p(i), n, d - A.radius() );

        return true;
      }
    }

    // Test if the sphere center lies in a voronoi region of an edge
    {
      unsigned int edge_voronoi_region[6][4] = {
        {0, 1,  2, 3}   // edge 0-1 with opposing vertices 2 and 3 (left/right)
        ,{0, 2, 3, 1}  // edge 0-2 with opposing vertices 1 and 3  (left/right)
        ,{0, 3, 1, 2}  // ... ditto
        ,{1, 2, 0, 3}
        ,{1, 3, 2, 0}
        ,{2, 3, 0, 1}
      };

      for (unsigned int v=0u; v < 6u; ++v)
      {
        unsigned int const & i = edge_voronoi_region[v][0];
        unsigned int const & j = edge_voronoi_region[v][1];
        unsigned int const & k = edge_voronoi_region[v][2];
        unsigned int const & m = edge_voronoi_region[v][3];

        if( !(surface_map[k] || surface_map[m]))  // Test if edge is part of the surface
          continue;

        V const  d  = B.p(j) - B.p(i);
        V const n_k = tiny::unit( tiny::cross(  d, plane[k].n()) );
        V const n_m = tiny::unit( tiny::cross( -d, plane[m].n()) );

        Plane<V> const vp_k = make_plane( n_k, B.p(i) );
        Plane<V> const vp_m = make_plane( n_m, B.p(i) );

        T const d_vp_k = get_signed_distance( A.center(), vp_k );
        T const d_vp_m = get_signed_distance( A.center(), vp_m );

        if(d_vp_k >= VT::zero() && d_vp_m >= VT::zero())
        {
          V const p = closest_point_on_line(A.center(), make_line( B.p(i), B.p(j) ) );

          V const m =  A.center() - p;

          T const d = tiny::norm( m );

          if (d > A.radius() )
            return false;

          V const n = flip ? tiny::unit(m) : - tiny::unit(m);

          callback( p, n, d - A.radius() );

          return true;
        }
      }
    }

    // Test if sphere center lies in a voronoi region of a face
    for (unsigned int v=0u; v < 4 ; ++v)
    {
      if( !surface_map[v])  // Test if face is part of the surface
        continue;

      T const d = get_signed_distance( A.center(), plane[v] );

      if( d < VT::zero() )
        continue;

      bool const inside = inside_triangle( A.center(), triangle[v], false );
      
      if (inside)
      {
        V const n = flip ? plane[v].n() : - plane[v].n();

        V const p = closest_point_on_plane(A.center(), plane[v]);

        callback( p, n,  d - A.radius() );
        
        return true;
      }
    }

    return false;
  }

  template<typename V>
  inline bool contacts_sphere_tetrahedron(
                                          Sphere<V> const & A
                                          , Tetrahedron<V> const & B
                                          , ContactsCallback<V> & callback
                                          , bool const flip = false
                                        )
  {
    std::vector<bool> const surface_map(4u,true);
    return contacts_sphere_tetrahedron(A, B, callback, flip, surface_map);
  }

}// namespace geometry

// GEOMETRY_CONTACTS_SPHERE_TETRAHEDRON_H
#endif