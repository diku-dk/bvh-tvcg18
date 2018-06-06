#ifndef CONTENT_ADD_VISUAL_MESH_H
#define CONTENT_ADD_VISUAL_MESH_H

#include <content.h>

#include <mesh_array.h>

#include <tiny.h>

namespace content
{

  inline void add_visual_mesh_of_boxes(
                                       size_t const & gid
                                       , content::API * engine
                                       , mesh_array::T3Mesh & mesh
                                       , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & X
                                       , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & Y
                                       , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & Z
                                       )
  {
    typedef tiny::MathTypes<float>       MT;
    typedef typename MT::vector3_type    V;
    typedef typename MT::quaternion_type Q;

    for(size_t j=0u; j < engine->get_number_of_boxes(gid) ; ++j)
    {
      float width;
      float height;
      float depth;
      engine->get_box_shape( gid, j , width, height, depth );

      mesh_array::T3Mesh submesh;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subX;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subY;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subZ;

      mesh_array::make_box<MT>( width, height, depth, submesh, subX, subY, subZ);

      float x;
      float y;
      float z;
      engine->get_box_position( gid, j , x, y, z );

      V const trans = V::make(x, y, z);

      float qs, qx, qy, qz;
      engine->get_box_orientation( gid, j, qs, qx, qy, qz );

      Q const rot = Q(qs,qx,qy,qz);

      mesh_array::transform<MT>(trans, rot, submesh, subX, subY, subZ);

      mesh_array::concatenation<MT>( submesh, subX, subY, subZ, mesh, X, Y, Z );
    }
  }

  inline void add_visual_mesh_of_capsules(
                                          size_t const & gid
                                          , content::API * engine
                                          , mesh_array::T3Mesh & mesh
                                          , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & X
                                          , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & Y
                                          , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & Z
                                          )
  {
    typedef tiny::MathTypes<float>       MT;
    typedef typename MT::vector3_type    V;
    typedef typename MT::quaternion_type Q;

    for(size_t j=0u; j < engine->get_number_of_capsules(gid) ; ++j)
    {
      float radius;
      float height;
      engine->get_capsule_shape( gid, j , radius, height );

      mesh_array::T3Mesh submesh;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subX;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subY;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subZ;

      mesh_array::make_capsule<MT>( radius, height, 12, 12, submesh, subX, subY, subZ );

      float x;
      float y;
      float z;
      engine->get_capsule_position( gid, j , x, y, z );

      V const trans = V::make(x, y, z);

      float qs, qx, qy, qz;
      engine->get_capsule_orientation( gid, j, qs, qx, qy, qz );

      Q const rot = Q(qs,qx,qy,qz);

      mesh_array::transform<MT>(trans, rot, submesh, subX, subY, subZ);

      mesh_array::concatenation<MT>( submesh, subX, subY, subZ, mesh, X, Y, Z );
    }
  }

  inline void add_visual_mesh_of_cones(
                                       size_t const & gid
                                       , content::API * engine
                                       , mesh_array::T3Mesh & mesh
                                       , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & X
                                       , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & Y
                                       , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & Z
                                       )
  {
    typedef tiny::MathTypes<float>       MT;
    typedef typename MT::vector3_type    V;
    typedef typename MT::quaternion_type Q;

    for(size_t j=0u; j < engine->get_number_of_cones(gid) ; ++j)
    {
      float radius;
      float height;
      engine->get_cone_shape( gid, j , radius, height );

      mesh_array::T3Mesh submesh;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subX;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subY;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subZ;

      mesh_array::make_cone<MT>( radius, height, 12, submesh, subX, subY, subZ );

      float x;
      float y;
      float z;
      engine->get_cone_position( gid, j , x, y, z );

      V const trans = V::make(x, y, z);

      float qs, qx, qy, qz;
      engine->get_cone_orientation( gid, j, qs, qx, qy, qz );

      Q const rot = Q(qs,qx,qy,qz);

      mesh_array::transform<MT>(trans, rot, submesh, subX, subY, subZ);

      mesh_array::concatenation<MT>( submesh, subX, subY, subZ, mesh, X, Y, Z );
    }
  }

  inline void add_visual_mesh_of_convexes(
                                          size_t const & gid
                                          , content::API * engine
                                          , mesh_array::T3Mesh & mesh
                                          , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & X
                                          , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & Y
                                          , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & Z
                                          )
  {
    typedef tiny::MathTypes<float>       MT;
    typedef typename MT::vector3_type    V;
    typedef typename MT::quaternion_type Q;

    for(size_t j=0u; j < engine->get_number_of_convexes(gid) ; ++j)
    {
      size_t no_points = 0u;

      engine->get_convex_shape( gid, j , no_points );

      std::vector<float> coordinates;
      coordinates.resize(no_points*3u);

      engine->get_convex_shape( gid, j, &coordinates[0] );

      std::vector<V> vertices;
      vertices.resize(no_points);

      for(size_t k=0u;k<no_points;++k)
      {
        vertices[k](0) = coordinates[3*k];
        vertices[k](1) = coordinates[3*k+1];
        vertices[k](2) = coordinates[3*k+2];
      }

      mesh_array::T3Mesh submesh;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subX;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subY;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subZ;

      mesh_array::make_convex<MT>( vertices, submesh, subX, subY,subZ );

      float x;
      float y;
      float z;
      engine->get_convex_position( gid, j , x, y, z );

      V const trans = V::make(x, y, z);

      float qs, qx, qy, qz;
      engine->get_convex_orientation( gid, j, qs, qx, qy, qz );

      Q const rot = Q(qs,qx,qy,qz);

      mesh_array::transform<MT>(trans, rot, submesh, subX, subY, subZ);

      mesh_array::concatenation<MT>( submesh, subX, subY, subZ, mesh, X, Y, Z );
    }
  }

  inline void add_visual_mesh_of_cylinders(
                                           size_t const & gid
                                           , content::API * engine
                                           , mesh_array::T3Mesh & mesh
                                           , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & X
                                           , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & Y
                                           , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & Z
                                           )
  {
    typedef tiny::MathTypes<float>       MT;
    typedef typename MT::vector3_type    V;
    typedef typename MT::quaternion_type Q;

    for(size_t j=0u; j < engine->get_number_of_cylinders(gid) ; ++j)
    {
      float radius;
      float height;
      engine->get_cylinder_shape( gid, j , radius, height );

      mesh_array::T3Mesh submesh;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subX;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subY;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subZ;

      mesh_array::make_cylinder<MT>( radius, height, 12, submesh, subX, subY, subZ );

      float x;
      float y;
      float z;
      engine->get_cylinder_position( gid, j , x, y, z );

      V const trans = V::make(x, y, z);

      float qs, qx, qy, qz;
      engine->get_cylinder_orientation( gid, j, qs, qx, qy, qz );

      Q const rot = Q(qs,qx,qy,qz);

      mesh_array::transform<MT>(trans, rot, submesh, subX, subY, subZ);

      mesh_array::concatenation<MT>( submesh, subX, subY, subZ, mesh, X, Y, Z );
    }
  }

  inline void add_visual_mesh_of_ellipsoids(
                                            size_t const & gid
                                            , content::API * engine
                                            , mesh_array::T3Mesh & mesh
                                            , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & X
                                            , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & Y
                                            , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & Z
                                            )
  {
    typedef tiny::MathTypes<float>       MT;
    typedef typename MT::vector3_type    V;
    typedef typename MT::quaternion_type Q;

    for(size_t j=0u; j < engine->get_number_of_ellipsoids(gid) ; ++j)
    {
      float a;
      float b;
      float c;
      engine->get_ellipsoid_shape( gid, j , a, b, c );

      mesh_array::T3Mesh submesh;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subX;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subY;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subZ;

      mesh_array::make_ellipsoid<MT>( a, b, c, 12, 12, submesh, subX, subY, subZ );

      float x;
      float y;
      float z;
      engine->get_ellipsoid_position( gid, j , x, y, z );

      V const trans = V::make(x, y, z);

      float qs, qx, qy, qz;
      engine->get_ellipsoid_orientation( gid, j, qs, qx, qy, qz );

      Q const rot = Q(qs,qx,qy,qz);

      mesh_array::transform<MT>(trans, rot, submesh, subX, subY, subZ);

      mesh_array::concatenation<MT>( submesh, subX, subY, subZ, mesh, X, Y, Z );
    }
  }

  inline void add_visual_mesh_of_spheres(
                                         size_t const & gid
                                         , content::API * engine
                                         , mesh_array::T3Mesh & mesh
                                         , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & X
                                         , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & Y
                                         , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & Z
                                         )
  {
    typedef tiny::MathTypes<float>       MT;
    typedef typename MT::vector3_type    V;
    typedef typename MT::quaternion_type Q;

    for(size_t j=0u; j < engine->get_number_of_spheres(gid) ; ++j)
    {
      float radius;
      engine->get_sphere_shape( gid, j , radius );

      mesh_array::T3Mesh submesh;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subX;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subY;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subZ;

      mesh_array::make_sphere<MT>( radius, 12, 12, submesh, subX, subY, subZ );

      float x;
      float y;
      float z;
      engine->get_sphere_position( gid, j , x, y, z );

      V const trans = V::make(x, y, z);

      float qs, qx, qy, qz;
      engine->get_sphere_orientation( gid, j, qs, qx, qy, qz );

      Q const rot = Q(qs,qx,qy,qz);

      mesh_array::transform<MT>(trans, rot, submesh, subX, subY, subZ);

      mesh_array::concatenation<MT>( submesh, subX, subY, subZ, mesh, X, Y, Z );
    }
  }

  inline void add_visual_mesh_of_tetrameshes(
                                             size_t const & gid
                                             , content::API * engine
                                             , mesh_array::T3Mesh & mesh
                                             , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & X
                                             , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & Y
                                             , mesh_array::VertexAttribute<float, mesh_array::T3Mesh> & Z
                                             )
  {
    typedef tiny::MathTypes<float>       MT;

    for(size_t j=0u; j < engine->get_number_of_tetrameshes(gid) ; ++j)
    {
      // Get raw data from engine
      size_t N = 0u; ///< Number of vertices
      size_t K = 0u; ///< Number of tetrahedra

      engine->get_tetramesh_shape(  gid, N, K);

      if(N==0 || K==0)  // Test if empty mesh and skip if this is the case
        continue;

      std::vector<size_t> vertices(N);
      std::vector<size_t> tetrahedra(K*4);
      std::vector<float>  coordinates(3*N);

      engine->get_tetramesh_shape(  gid
                                  , &vertices[0]
                                  , &tetrahedra[0]
                                  , &coordinates[0]
                                  );
      
      
      // Convert from raw data to a T4Mesh
      mesh_array::T4Mesh tetmesh;
      mesh_array::VertexAttribute<float, mesh_array::T4Mesh> tetX;
      mesh_array::VertexAttribute<float, mesh_array::T4Mesh> tetY;
      mesh_array::VertexAttribute<float, mesh_array::T4Mesh> tetZ;
      
      mesh_array::convert(N, K, &vertices[0], &tetrahedra[0], &coordinates[0], tetmesh, tetX, tetY, tetZ);
      
      // Extract T3Mesh surface of T4Mesh
      mesh_array::T3Mesh submesh;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subX;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subY;
      mesh_array::VertexAttribute<float, mesh_array::T3Mesh> subZ;
      
      mesh_array::make_t3mesh( tetmesh, tetX, tetY, tetZ, submesh, subX, subY,subZ );
      
      mesh_array::concatenation<MT>( submesh, subX, subY, subZ, mesh, X, Y, Z );
    }
    
  }
  
}//namespace content

// CONTENT_ADD_VISUAL_MESH_H
#endif
