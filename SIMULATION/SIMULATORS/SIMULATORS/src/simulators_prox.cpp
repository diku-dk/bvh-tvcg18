#include <simulators_prox.h>
#include <simulators_prox_data.h>

#include <util_string_helper.h>
#include <util_config_file.h>
#include <util_log.h>

#include <mesh_array.h>

namespace simulators
{

  typedef tiny::MathTypes<float> MT;
  typedef MT::vector3_type       V;
  typedef MT::quaternion_type    Q;
  typedef MT::value_traits       VT;
  typedef MT::real_type          T;
  
  ProxEngine::ProxEngine()
  {
    m_data = new ProxData();
    
    assert( m_data || !"ProxEngine(): internal error, null pointer");
  }
  
  ProxEngine::ProxEngine(  std::string const & solver
                         , std::string const & normal_solver
                         , std::string const & friction_solver
                         , std::string const & time_stepper
                         , std::string const & r_factor_strategy
                         )
  {
    m_data = new ProxData();
    
    assert( m_data || !"ProxEngine(): internal error, null pointer");
    
    set_parameter(PARAM_SOLVER,          solver             );
    set_parameter(PARAM_NORMAL_SOLVER,   normal_solver      );
    set_parameter(PARAM_FRICTION_SOLVER, friction_solver    );
    set_parameter(PARAM_TIME_STEPPER,    time_stepper       );
    set_parameter(PARAM_R_FACTOR,        r_factor_strategy  );
  }
  
  ProxEngine::~ProxEngine()
  {
    if (m_data)
    {
      m_data->clear();
      delete m_data;
    }
  }
  
  bool ProxEngine::simulate( float const & dt )
  {
    using std::min;
    
    assert( m_data    || !"simulate(): internal error, null pointer");
    assert( dt > 0.0f || !"simulate(): it does not make sense to take zero time step");
    
    
    float dt_left     = dt;
    float script_time = m_data->m_time;
    
    while(dt_left > 0.0f)
    {
      float const ddt = min( m_data->m_time_step, dt_left );
      
      script_time += ddt;
      for (size_t i = 0u; i < m_data->m_all_scripted_bodies.size(); ++i)
      {
        
        size_t const body_idx = m_data->m_all_scripted_bodies[i];
        
        ProxData::rigid_body_type & body = m_data->m_bodies[body_idx];
        
        ProxData::ScriptedMotion * motion = m_data->m_motion_callbacks.at(body_idx);
        
        motion->update(script_time, body);
      }
      
      m_data->step_simulation( ddt );
      
      dt_left -= ddt;
    }
    
    m_data->m_time += dt;
    
    return true;
  }
  
  void ProxEngine::clear()
  {
    assert( m_data || !"clear(): internal error, null pointer");
    if (m_data)
      m_data->clear();
  }

  size_t ProxEngine::create_rigid_body( std::string const & name )
  {
    assert( m_data || !"create_rigid_body(): internal error, null pointer");
    ProxData::rigid_body_type B = ProxData::rigid_body_type();
    B.set_name( name );
    m_data->m_bodies.push_back(B);
    return m_data->m_bodies.size()-1;
  }
  
  void ProxEngine::set_rigid_body_position(  size_t const & body_idx
                                           , float const & x
                                           , float const & y
                                           , float const & z
                                           )
  {
    assert( m_data                                         || !"set_rigid_body_position(): null pointer");
    assert( body_idx < m_data->m_bodies.size()             || !"set_rigid_body_position(): no such rigid body");
    assert( (is_number(x) && is_number(y) && is_number(z)) || !"set_rigid_body_position(): NaN or inf value");
    
    m_data->m_bodies[ body_idx ].set_position( ProxData::V::make( x, y, z ) );
  }
  
  void ProxEngine::set_rigid_body_orientation(  size_t const & body_idx
                                              , float const & Qs
                                              , float const & Qx
                                              , float const & Qy
                                              , float const & Qz
                                              )
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_idx < m_data->m_bodies.size() || !"internal error: no such rigid body");
    assert( (is_number(Qs) && is_number(Qx) && is_number(Qy) && is_number(Qz)) || !"internal error: NaN or inf value");
    
    ProxData::MT::quaternion_type Q;
    
    Q.real()    = Qs;
    Q.imag()(0) = Qx;
    Q.imag()(1) = Qy;
    Q.imag()(2) = Qz;
    
    m_data->m_bodies[ body_idx ].set_orientation( Q );
  }
  
  void ProxEngine::set_rigid_body_velocity(
                                           size_t const & body_idx
                                           , float const & vx
                                           , float const & vy
                                           , float const & vz
                                           )
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_idx < m_data->m_bodies.size() || !"internal error: no such rigid body");
    assert( (is_number(vx) && is_number(vy) && is_number(vz)) || !"internal error: NaN or inf value");
    
    m_data->m_bodies[ body_idx ].set_velocity( ProxData::V::make( vx, vy, vz ) );
  }
  
  void ProxEngine::set_rigid_body_spin(
                                       size_t const & body_idx
                                       , float const & wx
                                       , float const & wy
                                       , float const & wz
                                       )
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_idx < m_data->m_bodies.size() || !"internal error: no such rigid body");
    assert( (is_number(wx) && is_number(wy) && is_number(wz)) || !"internal error: NaN or inf value");
    
    m_data->m_bodies[ body_idx ].set_spin( ProxData::V::make( wx, wy, wz ) );
  }
  
  void ProxEngine::set_rigid_body_mass( size_t const & body_idx, float const & mass)
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_idx < m_data->m_bodies.size() || !"internal error: no such rigid body");
    assert( is_number(mass) || !"internal error: NaN or inf value");
    
    m_data->m_bodies[ body_idx ].set_mass( mass );
  }
  
  void ProxEngine::set_rigid_body_inertia(
                                          size_t const & body_idx
                                          , float const & Ixx
                                          , float const & Iyy
                                          , float const & Izz
                                          )
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_idx < m_data->m_bodies.size() || !"internal error: no such rigid body");
    assert( (is_number(Ixx) && is_number(Iyy) && is_number(Izz)) || !"internal error: NaN or inf value");
    assert( ((0 <= Ixx) && (0 <= Iyy) && (0 <= Izz)) || !"internal error: negative value");
    
    ProxData::MT::matrix3x3_type I = ProxData::MT::matrix3x3_type::make(
                                                                        Ixx, 0.0f, 0.0f,
                                                                        0.0f, Iyy, 0.0f,
                                                                        0.0f, 0.0f, Izz
                                                                        );
    
    m_data->m_bodies[ body_idx ].set_inertia_bf( I );
  }
  
  void ProxEngine::set_rigid_body_active( size_t const & body_idx, bool const & active )
  {
    //not used
  }
  
  void ProxEngine::set_rigid_body_fixed( size_t const & body_idx, bool const & fixed )
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_idx < m_data->m_bodies.size() || !"internal error: no such rigid body");
    
    m_data->m_bodies[ body_idx ].set_fixed( fixed );
  }
  
  void ProxEngine::set_rigid_body_material( size_t const & body_idx, size_t const & material_idx)
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_idx < m_data->m_bodies.size() || !"internal error: no such rigid body");
    
    m_data->m_bodies[ body_idx ].set_material_idx( material_idx );
  }
  
  void ProxEngine::connect_collision_geometry(size_t body_idx, size_t geometry_index )
  {
    assert( m_data                             || !"internal error: null pointer");
    assert( body_idx < m_data->m_bodies.size() || !"internal error: no such rigid body");
    
    ProxData::rigid_body_type & body     = m_data->m_bodies[ body_idx ];
    ProxData::geometry_type   & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    body.set_geometry_idx( geometry_index );
    
    if( geometry.m_tetramesh.has_data())
    {
      narrow::make_kdop_bvh( m_data->m_narrow.params(), body, geometry);
    }
  }
  
  /// materials
  size_t ProxEngine::create_material( std::string const & name )
  {
    
    assert( m_data || !"internal error: null pointer");
    m_data->m_materials.push_back( name );
    size_t idx = m_data->m_materials.size() - 1u;
    
    // m_number_of_materials is a #define in prox.h, not pretty I know...
    assert( idx < m_number_of_materials || !"internal error: more materials created than is allocated space for in material table");
    return idx;
  }
  
  void ProxEngine::create_material_property(  size_t const & first_idx
                                            , size_t const & second_idx)
  {
    assert( m_data || !"internal error: null pointer");
    assert( first_idx  < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    assert( second_idx < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    
    
    bool exist = m_data->m_exist_property[first_idx][second_idx];
    
    assert( !exist || !"internal error: material was already created?");
    
    ProxData::property_type P = ProxData::property_type();
    
    m_data->m_properties[first_idx][second_idx] = P;
    m_data->m_properties[second_idx][first_idx] = P;
    
    m_data->m_exist_property[first_idx][second_idx] = true;
    
    m_data->m_property_counter++;
  }
  
  void ProxEngine::set_master(  size_t const & first_idx
                              , size_t const & second_idx
                              , size_t master_idx)
  {
    assert( m_data || !"internal error: null pointer");
    assert( first_idx  < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    assert( second_idx < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    assert( master_idx < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    
    assert((master_idx==first_idx || master_idx==second_idx) || !"set_master(): invalid master idx");
    
    m_data->m_properties[first_idx][second_idx].set_master_material_idx( master_idx );
    m_data->m_properties[second_idx][first_idx].set_master_material_idx( master_idx );
  }
  
  void ProxEngine::set_friction(  size_t const & first_idx
                                , size_t const & second_idx
                                , float const & mu_x
                                , float const & mu_y
                                , float const & mu_z
                                )
  {
    assert( m_data || !"internal error: null pointer");
    assert( first_idx  < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    assert( second_idx < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    assert( (is_number(mu_x) && is_number(mu_y) && is_number(mu_z)) || !"internal error: NaN or inf value");
    
    m_data->m_properties[first_idx][second_idx].set_friction_coefficients( mu_x, mu_y, mu_z );
    m_data->m_properties[second_idx][first_idx].set_friction_coefficients( mu_x, mu_y, mu_z );
  }
  
  void ProxEngine::set_master_direction(  size_t const & first_idx
                                        , size_t const & second_idx
                                        , float const & dir_x
                                        , float const & dir_y
                                        , float const & dir_z
                                        )
  {
    assert( m_data || !"internal error: null pointer");
    assert( first_idx  < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    assert( second_idx < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    assert( (is_number(dir_x) && is_number(dir_y) && is_number(dir_z)) || !"internal error: NaN or inf value");
    
    m_data->m_properties[first_idx][second_idx].set_s_vector( dir_x, dir_y, dir_z );
    m_data->m_properties[second_idx][first_idx].set_s_vector( dir_x, dir_y, dir_z );
  }
  
  void ProxEngine::set_restitution(  size_t const & first_idx
                                   , size_t const & second_idx
                                   , float const & e
                                   )
  {
    assert( m_data || !"internal error: null pointer");
    assert( first_idx  < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    assert( second_idx < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    assert( is_number(e) || !"internal error: NaN or inf value");
    
    m_data->m_properties[first_idx][second_idx].set_restitution_coefficient( e );
    m_data->m_properties[second_idx][first_idx].set_restitution_coefficient( e );
  }
  /// materials
  
  /// collision geometries
  size_t ProxEngine::create_collision_geometry( std::string const & name )
  {
    assert( m_data || !"internal error: null pointer");
    
    size_t const gid = m_data->m_narrow.create_geometry();
    
    m_data->m_geometry_names.push_back( name );
    
    return gid;
  }
  
  size_t ProxEngine::create_box_shape( size_t const & geometry_index )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    if (m_data->m_use_only_tetrameshes)
    {
      assert( !geometry.m_tetramesh.has_data() || !"create_box_shape(): Internal error, tetramesh shape already created");
      return 0u;
    }
    
    ProxData::geometry_type::box_type box = ProxData::geometry_type::box_type();
    
    geometry.add_shape( box );
    
    return geometry.number_of_boxes()-1u;
  }
  
  void ProxEngine::set_box_shape(  size_t const & geometry_index
                                 , size_t const & box_number
                                 , float const & width
                                 , float const & height
                                 , float const & depth
                                 )
  {
    assert( m_data || !"internal error: null pointer");
    assert( (is_number(width) && is_number(height) && is_number(depth)) || !"internal error: NaN or inf value");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    if (m_data->m_use_only_tetrameshes)
    {
      mesh_array::T3Mesh                                 surface;
      mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_X;
      mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_Y;
      mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_Z;
      
      mesh_array::make_box<MT>(
                               width
                               , height
                               , depth
                               , surface
                               , surface_X
                               , surface_Y
                               , surface_Z
                               );
      
      m_data->make_tetramesh_geoemtry(
                                      geometry
                                      , surface
                                      , surface_X
                                      , surface_Y
                                      , surface_Z
                                      );
      
      return;
    }
    
    assert( box_number < geometry.number_of_boxes() || !"internal error: no such geometry");
    
    geometry.m_boxes[box_number].half_extent() = V::make(width, height, depth)/VT::two();
  }
  
  void ProxEngine::set_box_position(  size_t const & geometry_index
                                    , size_t const & box_number
                                    , float const & x
                                    , float const & y
                                    , float const & z
                                    )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( (is_number(x) && is_number(y) && is_number(z)) || !"internal error: NaN or inf value");
    
    assert( box_number < geometry.number_of_boxes() || !"internal error: no such geometry");
    
    geometry.m_boxes[box_number].transform().T() = V::make( x, y, z );
  }
  
  void ProxEngine::set_box_orientation(  size_t const & geometry_index
                                       , size_t const & box_number
                                       , float const & Qs
                                       , float const & Qx
                                       , float const & Qy
                                       , float const & Qz
                                       )
  {
    assert( m_data || !"internal error: null pointer");
    assert( (is_number(Qs) && is_number(Qx) && is_number(Qy) && is_number(Qz)) || !"internal error: NaN or inf value");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( box_number < geometry.number_of_boxes() || !"internal error: no such geometry");
    
    geometry.m_boxes[box_number].transform().Q() = Q( Qs, Qx, Qy, Qz );
  }
  
  size_t ProxEngine::create_capsule_shape( size_t const & geometry_index )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( geometry.m_tetramesh.has_data() || !"create_capsule_shape(): Internal error, tetramesh shape already created");
    
    return 0;
  }
  
  void ProxEngine::set_capsule_shape(  size_t const & geometry_index
                                     , size_t const & capsule_number
                                     , float const & radius
                                     , float const & height
                                     )
  {
    assert( m_data      || !"internal error: null pointer");
    assert( (is_number(radius) && is_number(height)) || !"internal error: NaN or inf value");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    mesh_array::T3Mesh                                 surface;
    mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_X;
    mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_Y;
    mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_Z;
    
    size_t const segments = 12u;
    size_t const slices   = 12u;
    
    mesh_array::make_capsule<MT>(
                                 radius
                                 , height
                                 , slices
                                 , segments
                                 , surface
                                 , surface_X
                                 , surface_Y
                                 , surface_Z
                                 );
    
    m_data->make_tetramesh_geoemtry(
                                    geometry
                                    , surface
                                    , surface_X
                                    , surface_Y
                                    , surface_Z
                                    );
    
  }
  
  void ProxEngine::set_capsule_position(  size_t const & geometry_index
                                        , size_t const & capsule_number
                                        , float const & x
                                        , float const & y
                                        , float const & z
                                        )
  {
    assert(!"not implemented");
  }
  
  void ProxEngine::set_capsule_orientation(  size_t const & geometry_index
                                           , size_t const & capsule_number
                                           , float const & Qs
                                           , float const & Qx
                                           , float const & Qy
                                           , float const & Qz
                                           )
  {
    assert(!"not implemented");
  }
  
  size_t ProxEngine::create_cone_shape( size_t const & geometry_index )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    // So idea is to use tetramesh to fake a convex shape
    assert( geometry.m_tetramesh.has_data() || !"create_cone_shape(): Internal error, tetramesh shape already created");
    
    return 0u;
  }
  
  void ProxEngine::set_cone_shape(  size_t const & geometry_index
                                  , size_t const & cone_number
                                  , float const & radius
                                  , float const & height
                                  )
  {
    assert( m_data      || !"internal error: null pointer");
    assert( (is_number(radius) && is_number(height)) || !"internal error: NaN or inf value");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    mesh_array::T3Mesh                                 surface;
    mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_X;
    mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_Y;
    mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_Z;
    
    size_t const slices = 12u;
    
    mesh_array::make_cone<MT>(
                              radius
                              , height
                              , slices
                              , surface
                              , surface_X
                              , surface_Y
                              , surface_Z
                              );
    
    m_data->make_tetramesh_geoemtry(
                                    geometry
                                    , surface
                                    , surface_X
                                    , surface_Y
                                    , surface_Z
                                    );
  }
  
  void ProxEngine::set_cone_position(  size_t const & geometry_index
                                     , size_t const & cone_number
                                     , float const & x
                                     , float const & y
                                     , float const & z
                                     )
  {
    assert(!"not implemented");
  }
  
  void ProxEngine::set_cone_orientation(  size_t const & geometry_index
                                        , size_t const & cone_number
                                        , float const & Qs
                                        , float const & Qx
                                        , float const & Qy
                                        , float const & Qz
                                        )
  {
    assert(!"not implemented");
  }
  
  size_t ProxEngine::create_convex_shape( size_t const & geometry_index )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    if (m_data->m_use_only_tetrameshes)
    {
      assert( !geometry.m_tetramesh.has_data() || !"create_box_shape(): Internal error, tetramesh shape already created");
      //return 0u;  // 2015-01-26 Kenny: Exception we ignore this for grain simulation
    }
    
    ProxData::geometry_type::convex_type hull = ProxData::geometry_type::convex_type();
    
    geometry.add_shape( hull );
    
    return geometry.number_of_hulls()-1u;
  }
  
  void ProxEngine::set_convex_shape(  size_t const & geometry_index
                                    , size_t const & convex_number
                                    , size_t const & N
                                    , float const * coordinates
                                    )
  {
    assert( m_data      || !"internal error: null pointer");
    assert( coordinates || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    std::vector<V> p;
    p.resize(N);
    for(size_t i = 0u; i < N; ++i)
    {
      p[i](0) = coordinates[3u*i+0u];
      p[i](1) = coordinates[3u*i+1u];
      p[i](2) = coordinates[3u*i+2u];
    }
    
    if (m_data->m_use_only_tetrameshes)
    {
      
      mesh_array::T3Mesh                                 surface;
      mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_X;
      mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_Y;
      mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_Z;
      
      mesh_array::make_convex<MT>(p
                                  , surface
                                  , surface_X
                                  , surface_Y
                                  , surface_Z
                                  );
      
      
      m_data->make_tetramesh_geoemtry(
                                      geometry
                                      , surface
                                      , surface_X
                                      , surface_Y
                                      , surface_Z
                                      );
      
      //      return;  // 2015-01-26 Kenny: Exception we use both tetrameshes and convex hulls here
    }
    
    assert( convex_number < geometry.number_of_hulls() || !"internal error: no such geometry");
    
    for(size_t i=0u; i< p.size() ;++i)
      geometry.m_hulls[convex_number].data().add_point( p[i] );
  }
  
  void ProxEngine::set_convex_position(  size_t const & geometry_index
                                       , size_t const & convex_number
                                       , float const & x
                                       , float const & y
                                       , float const & z
                                       )
  {
    
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( (is_number(x) && is_number(y) && is_number(z)) || !"internal error: NaN or inf value");
    
    assert( convex_number < geometry.number_of_hulls() || !"internal error: no such geometry");
    
    geometry.m_hulls[convex_number].transform().T() = V::make( x, y, z );
  
  }
  
  void ProxEngine::set_convex_orientation(  size_t const & geometry_index
                                          , size_t const & convex_number
                                          , float const & Qs
                                          , float const & Qx
                                          , float const & Qy
                                          , float const & Qz
                                          )
  {
    assert( m_data || !"internal error: null pointer");
    assert( (is_number(Qs) && is_number(Qx) && is_number(Qy) && is_number(Qz)) || !"internal error: NaN or inf value");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( convex_number < geometry.number_of_hulls() || !"internal error: no such geometry");
    
    geometry.m_hulls[convex_number].transform().Q() = Q( Qs, Qx, Qy, Qz );
  }
  
  size_t ProxEngine::create_cylinder_shape( size_t const & geometry_index )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    // So idea is to use tetramesh to fake a convex shape
    assert( geometry.m_tetramesh.has_data() || !"create_cylinder_shape(): Internal error, tetramesh shape already created");
    
    return 0u;
    
  }
  
  void ProxEngine::set_cylinder_shape(  size_t const & geometry_index
                                      , size_t const & cylinder_number
                                      , float const & radius
                                      , float const & height
                                      )
  {
    assert( m_data      || !"internal error: null pointer");
    assert( (is_number(radius) && is_number(height)) || !"internal error: NaN or inf value");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    mesh_array::T3Mesh                                 surface;
    mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_X;
    mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_Y;
    mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_Z;
    
    size_t const slices = 12u;
    
    mesh_array::make_cylinder<MT>(
                                  radius
                                  , height
                                  , slices
                                  , surface
                                  , surface_X
                                  , surface_Y
                                  , surface_Z
                                  );
    
    m_data->make_tetramesh_geoemtry(
                                    geometry
                                    , surface
                                    , surface_X
                                    , surface_Y
                                    , surface_Z
                                    );
  }
  
  void ProxEngine::set_cylinder_position(  size_t const & geometry_index
                                         , size_t const & cylinder_number
                                         , float const & x
                                         , float const & y
                                         , float const & z
                                         )
  {
    assert(!"not implemented");
  }
  
  void ProxEngine::set_cylinder_orientation(  size_t const & geometry_index
                                            , size_t const & cylinder_number
                                            , float const & Qs
                                            , float const & Qx
                                            , float const & Qy
                                            , float const & Qz
                                            )
  {
    assert(!"not implemented");
  }
  
  size_t ProxEngine::create_ellipsoid_shape( size_t const & geometry_index )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    // So idea is to use tetramesh to fake a convex shape
    assert( geometry.m_tetramesh.has_data() || !"create_ellipsoid_shape(): Internal error, tetramesh shape already created");
    
    return 0u;
  }
  
  void ProxEngine::set_ellipsoid_shape(  size_t const & geometry_index
                                       , size_t const & ellipsoid_number
                                       , float const & sx
                                       , float const & sy
                                       , float const & sz
                                       )
  {
    
    assert( m_data      || !"internal error: null pointer");
    assert( (is_number(sx) && is_number(sy) && is_number(sz)) || !"internal error: NaN or inf value");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    mesh_array::T3Mesh                                 surface;
    mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_X;
    mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_Y;
    mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_Z;
    
    size_t const segments = 12u;
    size_t const slices   = 12u;
    
    mesh_array::make_ellipsoid<MT>(
                                   sx
                                   , sy
                                   , sz
                                   , slices
                                   , segments
                                   , surface
                                   , surface_X
                                   , surface_Y
                                   , surface_Z
                                   );
    
    m_data->make_tetramesh_geoemtry(
                                    geometry
                                    , surface
                                    , surface_X
                                    , surface_Y
                                    , surface_Z
                                    );
  }
  
  
  void ProxEngine::set_ellipsoid_position( size_t const & geometry_index
                                          , size_t const & ellipsoid_number
                                          , float const & x
                                          , float const & y
                                          , float const & z
                                          )
  {
    assert(!"not implemented");
  }
  void ProxEngine::set_ellipsoid_orientation( size_t const & geometry_index
                                             , size_t const & ellipsoid_number
                                             , float const & Qs
                                             , float const & Qx
                                             , float const & Qy
                                             , float const & Qz
                                             )
  {
    assert(!"not implemented");
  }
  
  size_t ProxEngine::create_sphere_shape( size_t const & geometry_index )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    if(m_data->m_use_only_tetrameshes)
    {
      assert( !geometry.m_tetramesh.has_data() ||
             !"create_sphere_shape(): Internal error, tetramesh shape already created");
      
      return 0u;
    }
    ProxData::geometry_type::sphere_type sphere = ProxData::geometry_type::sphere_type();
    geometry.add_shape( sphere );
    
    return geometry.number_of_spheres()-1u;
  }
  
  void ProxEngine::set_sphere_shape(  size_t const & geometry_index
                                    , size_t const & sphere_number
                                    , float const & radius
                                    )
  {
    assert( m_data || !"internal error: null pointer");
    assert( is_number(radius) || !"internal error: NaN or inf value");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    if (m_data->m_use_only_tetrameshes)
    {
      mesh_array::T4Mesh                                 volume;
      mesh_array::VertexAttribute<T, mesh_array::T4Mesh> volume_X;
      mesh_array::VertexAttribute<T, mesh_array::T4Mesh> volume_Y;
      mesh_array::VertexAttribute<T, mesh_array::T4Mesh> volume_Z;
      
      mesh_array::T3Mesh                                 surface;
      mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_X;
      mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_Y;
      mesh_array::VertexAttribute<T, mesh_array::T3Mesh> surface_Z;
      
      size_t const segments = 12u;
      size_t const slices   = 12u;
      
      mesh_array::make_sphere<MT>(
                                  radius
                                  , slices
                                  , segments
                                  , surface
                                  , surface_X
                                  , surface_Y
                                  , surface_Z
                                  );
      
      m_data->make_tetramesh_geoemtry(
                                      geometry
                                      , surface
                                      , surface_X
                                      , surface_Y
                                      , surface_Z
                                      );
      return;
    }
    
    assert( sphere_number < geometry.number_of_spheres()
           || !"internal error: no such geometry");
    
    
    geometry.m_spheres[ sphere_number ].radius() = radius;
  }
  
  void ProxEngine::set_sphere_position(  size_t const & geometry_index
                                       , size_t const & sphere_number
                                       , float const & x
                                       , float const & y
                                       , float const & z
                                       )
  {
    assert( m_data || !"internal error: null pointer");
    assert( (is_number(x) && is_number(y) && is_number(z)) || !"internal error: NaN or inf value");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( sphere_number < geometry.number_of_spheres() || !"internal error: no such geometry");
    
    geometry.m_spheres[sphere_number].transform().T() = V::make( x, y, z );
    
  }
  
  void ProxEngine::set_sphere_orientation(  size_t const & geometry_index
                                          , size_t const & sphere_number
                                          , float const & Qs
                                          , float const & Qx
                                          , float const & Qy
                                          , float const & Qz
                                          )
  {
    assert( m_data || !"internal error: null pointer");
    assert( (is_number(Qs) && is_number(Qx) && is_number(Qy) && is_number(Qz)) || !"internal error: NaN or inf value");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( sphere_number < geometry.number_of_spheres() || !"internal error: no such geometry");
    
    geometry.m_spheres[sphere_number].transform().Q() = Q( Qs, Qx, Qy, Qz );
  }
  
  size_t ProxEngine::create_tetramesh_shape( size_t const & geometry_index )
  {
    assert( m_data || !"internal error: null pointer");
    
    //ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    // since we only allow a single mesh per body, always return 0
    return 0u;
  }
  
  void ProxEngine::set_tetramesh_shape(  size_t const & geometry_index
                                       , size_t const & N
                                       , size_t const & K
                                       , size_t const * vertices
                                       , size_t const * tetrahedra
                                       , float  const * coordinates
                                       )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( vertices    || !"internal error: null pointer");
    assert( tetrahedra  || !"internal error: null pointer");
    assert( coordinates || !"internal error: null pointer");
    
    mesh_array::T4Mesh mesh;
    mesh_array::VertexAttribute<T,mesh_array::T4Mesh> X;
    mesh_array::VertexAttribute<T,mesh_array::T4Mesh> Y;
    mesh_array::VertexAttribute<T,mesh_array::T4Mesh> Z;
    
    mesh_array::convert(N, K, vertices, tetrahedra, coordinates, mesh, X, Y,Z );

    if(kdop::SelectContactPointAlgorithm::is_using_closest_point())
    {
      mesh_array::shrink<MT>(
                             VT::numeric_cast(0.99)
                             , mesh
                             , X
                             , Y
                             , Z
                             );
//      std::vector<V> normals;
//
//      mesh_array::compute_vertex_normals<MT>(mesh, X, Y, Z, normals);
//
//      T const distance = VT::numeric_cast(-0.01);
//
//      mesh_array::displace_vertices<MT>(mesh, X, Y, Z, distance, normals);
    }

    geometry.m_tetramesh.set_tetramesh_shape(
                                             mesh
                                             , X
                                             , Y
                                             , Z
                                             );
    
  }
  
  float ProxEngine::get_collision_envelope()
  {
    assert( m_data || !"internal error: null pointer");
    
    return m_data->m_narrow.params().get_envelope();
  }
  
  float ProxEngine::get_time_step()
  {
    assert( m_data || !"internal error: null pointer");
    
    return m_data->m_time_step;
  }
  
  float ProxEngine::get_time()
  {
    assert( m_data || !"internal error: null pointer");
    
    return m_data->m_time;
  }
  
  std::string ProxEngine::get_material_name(size_t const & material_index)
  {
    assert( m_data || !"internal error: null pointer");
    return m_data->m_materials[ material_index ];
  }
  
  size_t ProxEngine::get_number_of_materials()
  {
    assert( m_data || !"internal error: null pointer");
    return m_data->m_materials.size();
  }
  
  void ProxEngine::get_material_indices( size_t * index_array )
  {
    assert( m_data || !"internal error: null pointer");
    for (size_t i = 0; i < m_data->m_materials.size() ; ++i)
    {
      index_array[i] = i;
    }
  }
  
  size_t ProxEngine::get_number_of_properties()
  {
    assert( m_data || !"internal error: null pointer");
    return m_data->m_property_counter;
  }
  
  void ProxEngine::get_material_property_indices(  size_t * first_index_array
                                                 , size_t * second_index_array
                                                 )
  {
    assert( m_data || !"internal error: null pointer");
    
    size_t p = 0;
    for (size_t first_idx = 0; first_idx < m_number_of_materials; ++first_idx)
    {
      for (size_t second_idx = 0; second_idx < m_number_of_materials; ++second_idx)
      {
        if (m_data->m_exist_property[first_idx][second_idx])
        {
          first_index_array[p]  = first_idx;
          second_index_array[p] = second_idx;
          
          ++p;
        }
      }
    }
  }
  
  size_t ProxEngine::get_master( size_t const & first_index
                                , size_t const & second_index
                                )
  {
    assert( m_data || !"internal error: null pointer");
    assert( first_index  < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    assert( second_index < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    
    return m_data->m_properties[first_index][second_index].get_master_material_idx();
  }
  
  void ProxEngine::get_friction( size_t const & first_index
                                , size_t const & second_index
                                , float & x
                                , float & y
                                , float & z
                                )
  {
    
    assert( m_data || !"internal error: null pointer");
    assert( first_index  < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    assert( second_index < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    
    m_data->m_properties[first_index][second_index].get_friction_coefficients( x, y, z );
  }
  
  void ProxEngine::get_master_direction( size_t const & first_index
                                        , size_t const & second_index
                                        , float & x
                                        , float & y
                                        , float & z
                                        )
  {
    assert( m_data || !"internal error: null pointer");
    assert( first_index  < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    assert( second_index < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    
    m_data->m_properties[first_index][second_index].get_s_vector( x, y, z );
  }
  
  float ProxEngine::get_restitution( size_t const & first_index
                                    , size_t const & second_index
                                    )
  {
    assert( m_data || !"internal error: null pointer");
    assert( first_index  < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    assert( second_index < m_number_of_materials || !"internal error: index excedes allocated space for in material table");
    
    return m_data->m_properties[first_index][second_index].get_restitution_coefficient();
  }
  
  size_t ProxEngine::get_number_of_geometries()
  {
    assert( m_data || !"internal error: null pointer");
    return m_data->m_narrow.size();
  }
  
  void ProxEngine::get_geometry_indices( size_t * index_array )
  {
    assert( m_data || !"internal error: null pointer");
    
    for (size_t i = 0; i < m_data->m_narrow.size() ; ++i)
    {
      index_array[i] = i;
    }
  }
  
  std::string ProxEngine::get_geometry_name(size_t const & geometry_index)
  {
    assert( m_data || !"internal error: null pointer");
    
    return m_data->m_geometry_names[ geometry_index ];
  }
  
  size_t ProxEngine::get_number_of_boxes( size_t const & geometry_index )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    return geometry.number_of_boxes();
  }
  
  void ProxEngine::get_box_shape( size_t const & geometry_index
                                 , size_t const & box_number
                                 , float & width
                                 , float & height
                                 , float & depth
                                 )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( box_number < geometry.number_of_boxes() || !"internal error: no such geometry");
    
    V const ext = geometry.m_boxes[ box_number ].half_extent()*VT::two();
    
    width  = ext(0);
    height = ext(1);
    depth  = ext(2);
  }
  
  void ProxEngine::get_box_position( size_t const & geometry_index
                                    , size_t const & box_number
                                    , float & x
                                    , float & y
                                    , float & z
                                    )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( box_number < geometry.number_of_boxes() || !"internal error: no such geometry");
    
    x = geometry.m_boxes[box_number].transform().T()[0];
    y = geometry.m_boxes[box_number].transform().T()[1];
    z = geometry.m_boxes[box_number].transform().T()[2];
  }
  
  void ProxEngine::get_box_orientation( size_t const & geometry_index
                                       , size_t const & box_number
                                       , float & Qs
                                       , float & Qx
                                       , float & Qy
                                       , float & Qz
                                       )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( box_number < geometry.number_of_boxes() || !"internal error: no such geometry");
    
    Qs = geometry.m_boxes[box_number].transform().Q().real();
    Qx = geometry.m_boxes[box_number].transform().Q().imag()(0);
    Qy = geometry.m_boxes[box_number].transform().Q().imag()(1);
    Qz = geometry.m_boxes[box_number].transform().Q().imag()(2);
  }
  
  size_t ProxEngine::get_number_of_cones( size_t const & geometry_index )
  {
    return 0u;
  }
  
  void ProxEngine::get_cone_shape( size_t const & geometry_index
                                  , size_t const & cone_number
                                  , float & radius
                                  , float & height
                                  )
  {
    assert(!"not implemented");
  }
  
  void ProxEngine::get_cone_position(  size_t const & geometry_index
                                     , size_t const & cone_number
                                     , float & x
                                     , float & y
                                     , float & z
                                     )
  {
    assert(!"not implemented");
  }
  
  void ProxEngine::get_cone_orientation(  size_t const & geometry_index
                                        , size_t const & cone_number
                                        , float & Qs
                                        , float & Qx
                                        , float & Qy
                                        , float & Qz
                                        )
  {
    assert(!"not implemented");
  }
  
  size_t ProxEngine::get_number_of_capsules( size_t const & geometry_index )
  {
    return 0u;
  }
  
  void ProxEngine::get_capsule_shape(  size_t const & geometry_index
                                     , size_t const & capsule_number
                                     , float & radius
                                     , float & height
                                     )
  {
    assert(!"not implemented");
  }
  
  void ProxEngine::get_capsule_position(  size_t const & geometry_index
                                        , size_t const & capsule_number
                                        , float & x
                                        , float & y
                                        , float & z
                                        )
  {
    assert(!"not implemented");
  }
  
  void ProxEngine::get_capsule_orientation(size_t const & geometry_index
                                           , size_t const & capsule_number
                                           , float & Qs
                                           , float & Qx
                                           , float & Qy
                                           , float & Qz
                                           )
  {
    assert(!"not implemented");
  }
  
  size_t ProxEngine::get_number_of_cylinders( size_t const & geometry_index )
  {
    return 0u;
  }
  
  void ProxEngine::get_cylinder_shape( size_t const & geometry_index
                                      , size_t const & cylinder_number
                                      , float & radius
                                      , float & height
                                      )
  {
    assert(!"not implemented");
  }
  
  void ProxEngine::get_cylinder_position(size_t const & geometry_index
                                         , size_t const & cylinder_number
                                         , float & x
                                         , float & y
                                         , float & z
                                         )
  {
    assert(!"not implemented");
  }
  
  void ProxEngine::get_cylinder_orientation(size_t const & geometry_index
                                            , size_t const & cylinder_number
                                            , float & Qs
                                            , float & Qx
                                            , float & Qy
                                            , float & Qz
                                            )
  {
    assert(!"not implemented");
  }
  
  size_t ProxEngine::get_number_of_ellipsoids( size_t const & geometry_index )
  {
    return 0u;
  }
  
  void ProxEngine::get_ellipsoid_shape( size_t const & geometry_index
                                       , size_t const & ellipsoid_number
                                       , float & scale_x
                                       , float & scale_y
                                       , float & scale_z
                                       )
  {
    assert(!"not implemented");
  }
  
  void ProxEngine::get_ellipsoid_position(size_t const & geometry_index
                                          , size_t const & ellipsoid_number
                                          , float & x
                                          , float & y
                                          , float & z
                                          )
  {
    assert(!"not implemented");
  }
  
  void ProxEngine::get_ellipsoid_orientation(  size_t const & geometry_index
                                             , size_t const & ellipsoid_number
                                             , float & Qs
                                             , float & Qx
                                             , float & Qy
                                             , float & Qz
                                             )
  {
    assert(!"not implemented");
  }
  
  size_t ProxEngine::get_number_of_spheres( size_t const & geometry_index )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    return geometry.number_of_spheres();
  }
  
  void ProxEngine::get_sphere_shape( size_t const & geometry_index
                                    , size_t const & sphere_number
                                    , float & radius
                                    )
  {
    assert( m_data || !"internal error: null pointer");
    
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( sphere_number < geometry.number_of_spheres() || !"internal error: no such geometry");
    
    radius = geometry.m_spheres[ sphere_number ].radius();
  }
  
  void ProxEngine::get_sphere_position( size_t const & geometry_index
                                       , size_t const & sphere_number
                                       , float & x
                                       , float & y
                                       , float & z
                                       )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( sphere_number < geometry.number_of_spheres() || !"internal error: no such geometry");
    
    x = geometry.m_spheres[sphere_number].transform().T()[0];
    y = geometry.m_spheres[sphere_number].transform().T()[1];
    z = geometry.m_spheres[sphere_number].transform().T()[2];
  }
  
  void ProxEngine::get_sphere_orientation( size_t const & geometry_index
                                          , size_t const & sphere_number
                                          , float & Qs
                                          , float & Qx
                                          , float & Qy
                                          , float & Qz
                                          )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( sphere_number < geometry.number_of_spheres() || !"internal error: no such geometry");
    
    Qs = geometry.m_spheres[sphere_number].transform().Q().real();
    Qx = geometry.m_spheres[sphere_number].transform().Q().imag()(0);
    Qy = geometry.m_spheres[sphere_number].transform().Q().imag()(1);
    Qz = geometry.m_spheres[sphere_number].transform().Q().imag()(2);
  }
  
  size_t ProxEngine::get_number_of_tetrameshes( size_t const & geometry_index )
  {
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);

    return geometry.number_of_tetrameshes();
  }
  
  void ProxEngine::get_tetramesh_shape(  size_t const & geometry_index
                                       , size_t       & N
                                       , size_t       & K
                                       )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    N = geometry.m_tetramesh.m_mesh.vertex_size();
    K = geometry.m_tetramesh.m_mesh.tetrahedron_size();
  }
  
  void ProxEngine::get_tetramesh_shape(  size_t const & geometry_index
                                       , size_t * vertices
                                       , size_t * tetrahedra
                                       , float  * coordinates
                                       )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    size_t const N = geometry.m_tetramesh.m_mesh.vertex_size();
    size_t const K = geometry.m_tetramesh.m_mesh.tetrahedron_size();
    
    for (size_t m = 0u; m < N; ++m)
    {
      mesh_array::Vertex const v = geometry.m_tetramesh.m_mesh.vertex(m);
      
      vertices[m] = v.idx();
      
      coordinates[3u*m+0u] = geometry.m_tetramesh.m_X0(v);
      coordinates[3u*m+1u] = geometry.m_tetramesh.m_Y0(v);
      coordinates[3u*m+2u] = geometry.m_tetramesh.m_Z0(v);
    }
    
    for (size_t k = 0u; k < K; ++k)
    {
      mesh_array::Tetrahedron const t = geometry.m_tetramesh.m_mesh.tetrahedron(k);
      
      tetrahedra[4u*k+0u] = t.i();
      tetrahedra[4u*k+1u] = t.j();
      tetrahedra[4u*k+2u] = t.k();
      tetrahedra[4u*k+3u] = t.m();
    }
  }
  
  size_t ProxEngine::get_number_of_convexes( size_t const & geometry_index )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    return geometry.m_hulls.size();
  }
  
  void ProxEngine::get_convex_shape( size_t const & geometry_index
                                    , size_t const & convex_number
                                    , size_t & no_points
                                    )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( convex_number < geometry.number_of_hulls() || !"internal error: no such geometry");
    
    no_points = geometry.m_hulls[convex_number].data().size();
  }
  
  void ProxEngine::get_convex_shape( size_t const & geometry_index
                                    , size_t const & convex_number
                                    , float * coordinates
                                    )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( convex_number < geometry.number_of_hulls() || !"internal error: no such geometry");
    
    size_t const N = geometry.m_hulls[convex_number].data().size();
    
    for(size_t i=0u; i< N ;++i)
    {
      V const p = geometry.m_hulls[convex_number].data().get_point( i );
      
      coordinates[3*i + 0 ] = p(0);
      coordinates[3*i + 1 ] = p(1);
      coordinates[3*i + 2 ] = p(2);
    }
  }
  
  void ProxEngine::get_convex_position(size_t const & geometry_index
                                       , size_t const & convex_number
                                       , float & x
                                       , float & y
                                       , float & z
                                       )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( convex_number < geometry.number_of_hulls() || !"internal error: no such geometry");
    
    x = geometry.m_hulls[convex_number].transform().T()[0];
    y = geometry.m_hulls[convex_number].transform().T()[1];
    z = geometry.m_hulls[convex_number].transform().T()[2];
  }
  
  void ProxEngine::get_convex_orientation(size_t const & geometry_index
                                          , size_t const & convex_number
                                          , float & Qs
                                          , float & Qx
                                          , float & Qy
                                          , float & Qz
                                          )
  {
    assert( m_data || !"internal error: null pointer");
    
    ProxData::geometry_type & geometry = m_data->m_narrow.get_geometry(geometry_index);
    
    assert( convex_number < geometry.number_of_hulls() || !"internal error: no such geometry");
    
    Qs = geometry.m_hulls[convex_number].transform().Q().real();
    Qx = geometry.m_hulls[convex_number].transform().Q().imag()(0);
    Qy = geometry.m_hulls[convex_number].transform().Q().imag()(1);
    Qz = geometry.m_hulls[convex_number].transform().Q().imag()(2);
  }
  
  size_t ProxEngine::get_number_of_rigid_bodies()
  {
    assert( m_data || !"internal error: null pointer");
    return m_data->m_bodies.size();
  }
  
  void ProxEngine::get_rigid_body_indices( size_t * index_array )
  {
    assert( m_data || !"internal error: null pointer");
    for (size_t i = 0; i < m_data->m_bodies.size() ; ++i)
    {
      index_array[i] = i;
    }
  }
  
  std::string ProxEngine::get_rigid_body_name( size_t const & body_index )
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_index < m_data->m_bodies.size() || !"internal error: no such rigid body");
    
    return m_data->m_bodies[ body_index ].get_name();
  }
  
  void ProxEngine::get_rigid_body_position( size_t const & body_index
                                           , float & x
                                           , float & y
                                           , float & z
                                           )
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_index < m_data->m_bodies.size() || !"internal error: no such rigid body");
    
    ProxData::V pos = m_data->m_bodies[ body_index ].get_position();
    
    assert( (is_number(pos[0]) && is_number(pos[1]) && is_number(pos[2])) || !"internal error: NaN or inf value");
    
    x = pos[0];
    y = pos[1];
    z = pos[2];
  }
  
  void ProxEngine::get_rigid_body_orientation( size_t const & body_index
                                              , float & Qs
                                              , float & Qx
                                              , float & Qy
                                              , float & Qz
                                              )
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_index < m_data->m_bodies.size() || !"internal error: no such rigid body");
    
    ProxData::MT::quaternion_type Q = m_data->m_bodies[ body_index ].get_orientation();
    
    assert( (is_number(Q.real()) && is_number(Q.imag()(0)) && is_number(Q.imag()(1)) && is_number(Q.imag()(2))) || !"internal error: NaN or inf value");
    
    Qs = Q.real();
    Qx = Q.imag()(0);
    Qy = Q.imag()(1);
    Qz = Q.imag()(2);
  }
  
  void ProxEngine::get_rigid_body_velocity( size_t const & body_index
                                           , float & x
                                           , float & y
                                           , float & z
                                           )
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_index < m_data->m_bodies.size() || !"internal error: no such rigid body");
    
    ProxData::V vel = m_data->m_bodies[ body_index ].get_velocity();
    
    assert( (is_number(vel[0]) && is_number(vel[1]) && is_number(vel[2])) || !"internal error: NaN or inf value");
    
    x = vel[0];
    y = vel[1];
    z = vel[2];
  }
  
  void ProxEngine::get_rigid_body_spin( size_t const & body_index
                                       , float & x
                                       , float & y
                                       , float & z
                                       )
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_index < m_data->m_bodies.size() || !"internal error: no such rigid body");
    
    ProxData::V spin = m_data->m_bodies[ body_index ].get_spin();
    
    assert( (is_number(spin[0]) && is_number(spin[1]) && is_number(spin[2])) || !"internal error: NaN or inf value");
    
    x = spin[0];
    y = spin[1];
    z = spin[2];
  }
  
  void ProxEngine::get_rigid_body_inertia( size_t const & body_index
                                          , float & xx
                                          , float & yy
                                          , float & zz
                                          )
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_index < m_data->m_bodies.size() || !"internal error: no such rigid body");
    
    ProxData::MT::matrix3x3_type in = m_data->m_bodies[ body_index ].get_inertia_bf();
    
    assert( (is_number(in(0,0)) && is_number(in(1,1)) && is_number(in(2,2))) || !"internal error: NaN or inf value");
    
    // BF -> WCS??
    xx = in(0,0);
    yy = in(1,1);
    zz = in(2,2);
  }
  
  float ProxEngine::get_rigid_body_mass( size_t const & body_index )
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_index < m_data->m_bodies.size() || !"internal error: no such rigid body");
    
    float mass = m_data->m_bodies[ body_index ].get_mass();
    
    assert( is_number(mass) || !"internal error: NaN or inf value");
    
    return mass;
  }
  
  bool ProxEngine::get_rigid_body_active( size_t const & body_index )
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_index < m_data->m_bodies.size() || !"internal error: no such rigid body");
    
    return true;
  }
  
  bool ProxEngine::get_rigid_body_fixed( size_t const & body_index )
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_index < m_data->m_bodies.size() || !"internal error: no such rigid body");
    
    return m_data->m_bodies[ body_index ].is_fixed();
  }
  
  size_t ProxEngine::get_rigid_body_material( size_t const & body_index )
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_index < m_data->m_bodies.size() || !"internal error: no such rigid body");
    
    return m_data->m_bodies[ body_index ].get_material_idx();
  }
  
  size_t ProxEngine::get_rigid_body_collision_geometry(size_t const & body_index)
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_index < m_data->m_bodies.size() || !"internal error: no such rigid body");
    
    return m_data->m_bodies[ body_index ].get_geometry_idx();
  }
  

  void ProxEngine::get_rigid_body_bounding_box(
                                               size_t const & body_index
                                               , float & min_x
                                               , float & min_y
                                               , float & min_z
                                               , float & max_x
                                               , float & max_y
                                               , float & max_z
                                               )
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_index < m_data->m_bodies.size() || !"internal error: no such rigid body");
    
    return m_data->m_bodies[ body_index ].get_box(min_x, min_y, min_z, max_x, max_y, max_z);
  }
  
  void ProxEngine::set_gravity_up(
                                  float const & x
                                  , float const & y
                                  , float const & z
                                  )
  {
    assert( m_data || !"internal error: null pointer");
    
    m_data->m_gravity.up() = V::make(x,y,z);
  }
  
  void ProxEngine::set_gravity_acceleration( float const & acceleration )
  {
    assert( m_data         || !"internal error: null pointer");
    assert(acceleration>=0 || !"set_gravity_acceleration(): acceleartion damping must be non-negative");
    
    m_data->m_gravity.acceleration() = acceleration;
  }
  
  void ProxEngine::set_damping_parameters( float const & linear, float const & angular )
  {
    assert( m_data    || !"internal error: null pointer");
    assert(linear>=0  || !"set_damping_parameters(): linear damping must be non-negative");
    assert(angular>=0 || !"set_damping_parameters(): angular damping must be non-negative");
    
    m_data->m_damping.linear()  = linear;
    m_data->m_damping.angular() = angular;
  }
  
  void ProxEngine::get_gravity_up(
                                  float & x
                                  , float & y
                                  , float & z
                                  )
  {
    assert( m_data || !"internal error: null pointer");
    
    x = m_data->m_gravity.up()(0);
    y = m_data->m_gravity.up()(1);
    z = m_data->m_gravity.up()(2);
  }
  
  void ProxEngine::get_gravity_acceleration( float & acceleration )
  {
    assert( m_data || !"internal error: null pointer");
    
    acceleration = m_data->m_gravity.acceleration();
  }
  
  void ProxEngine::get_damping_parameters( float & linear, float & angular )
  {
    assert( m_data || !"internal error: null pointer");
    
    linear  = m_data->m_damping.linear();
    angular = m_data->m_damping.angular();
  }

  size_t ProxEngine::get_number_of_pin_forces() const
  {
    assert( m_data || !"internal error: null pointer");
    
    return m_data->m_pin_forces.size();
  }
  
  void ProxEngine::get_pin_force_indices( size_t * index_array )
  {
    assert( m_data || !"internal error: null pointer");
    
    assert(index_array || !"internal error: null pointer");
    
    for( size_t i = 0; i < m_data->m_pin_forces.size();++i)
    {
      index_array[i] = m_data->m_pin_forces[i].get_idx();
    }
  }
  
  void ProxEngine::get_pin_tau(size_t const & force_idx, float & tau )
  {
    assert( m_data || !"internal error: null pointer");
    
    tau = m_data->m_pin_forces[force_idx].tau();
  }
  
  void ProxEngine::get_pin_target(size_t const & force_idx, float  & x, float  & y, float  & z )
  {
    assert( m_data || !"internal error: null pointer");
    
    x = m_data->m_pin_forces[force_idx].target()(0);
    y = m_data->m_pin_forces[force_idx].target()(1);
    z = m_data->m_pin_forces[force_idx].target()(2);
  }
  
  void ProxEngine::get_pin_anchor(size_t const & force_idx, float  & x, float  & y, float  & z )
  {
    assert( m_data || !"internal error: null pointer");
    
    x = m_data->m_pin_forces[force_idx].anchor()(0);
    y = m_data->m_pin_forces[force_idx].anchor()(1);
    z = m_data->m_pin_forces[force_idx].anchor()(2);
  }
  
  size_t ProxEngine::create_pin_force() const
  {
    assert( m_data || !"internal error: null pointer");
    
    size_t const idx = m_data->m_force_callbacks.size();
    
    m_data->m_pin_forces[idx] = prox::Pin<ProxData::MT>();
    
    m_data->m_pin_forces[idx].set_idx(idx);
    
    m_data->m_force_callbacks.push_back( &m_data->m_pin_forces[idx] );
    
    return m_data->m_pin_forces.size() - 1;
  }
  
  void ProxEngine::set_pin_tau(size_t const & force_idx, float const & tau )
  {
    assert( m_data || !"internal error: null pointer");
    
    m_data->m_pin_forces[force_idx].tau() = tau;
  }
  
  void ProxEngine::set_pin_target(size_t const & force_idx, float const & x, float const & y, float const & z )
  {
    assert( m_data || !"internal error: null pointer");
    
    m_data->m_pin_forces[force_idx].target()(0) = x;
    m_data->m_pin_forces[force_idx].target()(1) = y;
    m_data->m_pin_forces[force_idx].target()(2) = z;
  }
  
  void ProxEngine::set_pin_anchor(size_t const & force_idx, float const & x, float const & y, float const & z )
  {
    assert( m_data || !"internal error: null pointer");
    
    m_data->m_pin_forces[force_idx].anchor()(0) = x;
    m_data->m_pin_forces[force_idx].anchor()(1) = y;
    m_data->m_pin_forces[force_idx].anchor()(2) = z;
  }
  
  size_t ProxEngine::get_number_of_connected_forces(size_t const & body_index)
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_index < m_data->m_bodies.size() || !"internal error: no such rigid body");
    
    return m_data->m_bodies[body_index].get_force_callbacks().size();
  }
  
  void ProxEngine::get_connected_force_indices(  size_t const & body_index, size_t * index_array )
  {
    assert( m_data || !"internal error: null pointer");
    assert( body_index < m_data->m_bodies.size() || !"internal error: no such rigid body");
    
    size_t const N = m_data->m_bodies[body_index].get_force_callbacks().size();
    
    for(size_t i =0u; i< N; ++i)
    {
      index_array[i] = m_data->m_bodies[body_index].get_force_callbacks()[i]->get_idx();
    }
  }
  
  void ProxEngine::connect_force( size_t body_idx, size_t force_idx )
  {
    assert( m_data                                       || !"internal error: null pointer");
    assert( body_idx < m_data->m_bodies.size()           || !"internal error: no such rigid body");
    assert( force_idx < m_data->m_force_callbacks.size() || !"internal error: no such force callback");
    
    m_data->m_bodies[body_idx].get_force_callbacks().push_back( m_data->m_force_callbacks[force_idx]  );
  }
  
  void ProxEngine::connect_scripted_motion( size_t const & body_idx, size_t const & motion_idx )
  {
    assert( m_data                                        || !"connect_scripted_motion(): Data was null");
    assert( body_idx < m_data->m_bodies.size()            || !"connect_scripted_motion(): No such rigid body exist");
    assert( !(m_data->m_bodies[ body_idx ].is_scripted()) || !"connect_scripted_motion(): Rigid body is already scripted");
    assert( m_data->find_motion(motion_idx)!=0            || !"connect_scripted_motion(): No such motion exist");
    
    m_data->m_bodies[ body_idx ].set_scripted( true );
    
    m_data->m_all_scripted_bodies.push_back(body_idx);
    
    ProxData::ScriptedMotion * motion = m_data->find_motion(motion_idx);
    
    m_data->m_motion_callbacks[body_idx] = motion;
  }
  
  void ProxEngine::set_scripted_key_position(
                                             size_t const & motion_index
                                             , float const & time
                                             , float const & x
                                             , float const & y
                                             , float const & z
                                             )
  {
    assert( m_data || !"internal error: null pointer");
    
    std::map<size_t, ProxData::KeyframeMotion>::iterator lookup = m_data->m_keyframe_motions.find(motion_index);
    
    assert( lookup  != m_data->m_keyframe_motions.end() || !"set_scripted_key_position(): Motion did not exist");
    
    lookup->second.add_position(time, x, y, z);
  }
  
  void ProxEngine::set_scripted_key_orientation(
                                                size_t const & motion_index
                                                , float const & time
                                                , float const & qs
                                                , float const & qx
                                                , float const & qy
                                                , float const & qz
                                                )
  {
    assert( m_data || !"internal error: null pointer");
    
    std::map<size_t, ProxData::KeyframeMotion>::iterator lookup = m_data->m_keyframe_motions.find(motion_index);
    
    assert( lookup  != m_data->m_keyframe_motions.end() || !"set_scripted_key_position(): Motion did not exist");
    
    lookup->second.add_orientation(time, qs, qx, qy, qz);
  }
  
  bool ProxEngine::has_scripted_motion( size_t const & body_index )
  {
    assert( m_data || !"has_scripted_motion(): data null pointer");
    assert( body_index < m_data->m_bodies.size() || !"has_scripted_motion(): no such rigid body");
    
    return m_data->m_bodies[ body_index ].is_scripted();
  }
  
  size_t ProxEngine::get_scripted_motion( size_t const & body_index )
  {
    assert( m_data || !"get_scripted_motion(): data null pointer");
    assert( body_index < m_data->m_bodies.size() || !"get_scripted_motion(): no such rigid body");
    assert( m_data->m_bodies[ body_index ].is_scripted() || !"get_scripted_motion(): rigid body was not scripted");
    
    std::map<size_t, ProxData::ScriptedMotion * > ::iterator lookup = m_data->m_motion_callbacks.find(body_index);
    
    assert(lookup!=m_data->m_motion_callbacks.end() || !"get_scripted_motion(): No scripted motion was connected to this body");
    
    size_t const motion_idx = lookup->second->m_index;
    
    return motion_idx;
  }
  
  size_t ProxEngine::create_key_frame_scripted_motion()
  {
    ProxData::KeyframeMotion motion;
    
    motion.m_index = ProxData::ScriptedMotion::get_next_index();
    
    ++(ProxData::ScriptedMotion::get_next_index());
    
    m_data->m_keyframe_motions[motion.m_index] = motion;
    
    return motion.m_index;
  }
  
  size_t ProxEngine::create_oscilation_scripted_motion()
  {
    ProxData::OscillationMotion motion;
    
    motion.m_index = ProxData::ScriptedMotion::get_next_index();
    
    ++(ProxData::ScriptedMotion::get_next_index());
    
    m_data->m_oscillation_motions[motion.m_index] = motion;
    
    return motion.m_index;
  }
  
  void ProxEngine::set_scripted_oscilation_paramters(
                                                     size_t const & motion_index
                                                     , float const & amplitude
                                                     , float const & frequency
                                                     , float const & phase
                                                     , float const & dir_x
                                                     , float const & dir_y
                                                     , float const & dir_z
                                                     , float const & ref_x
                                                     , float const & ref_y
                                                     , float const & ref_z
                                                     )
  {
    assert( m_data || !"set_scripted_oscilation_paramters(): data was null pointer");
    
    std::map<size_t, ProxData::OscillationMotion>::iterator lookup = m_data->m_oscillation_motions.find(motion_index);
    
    assert( lookup  != m_data->m_oscillation_motions.end() || !"set_scripted_oscilation_paramters(): Motion did not exist");
    
    assert(amplitude > 0.0f || !"set_scripted_oscilation_paramters(): Amplitude must be positive");
    assert(frequency > 0.0f || !"set_scripted_oscilation_paramters(): frequency must be positive");
    
    lookup->second.m_amplitude = amplitude;
    lookup->second.m_frequency = frequency;
    lookup->second.m_phase     = phase;
    
    assert(( dir_x!=0.0f || dir_y!=0.0f || dir_z!= 0.0f) || !"set_scripted_oscilation_paramters(): direction was zero vector");
    
    lookup->second.m_direction = tiny::unit( V::make(dir_x, dir_y, dir_z) );
    lookup->second.m_origin    = V::make(ref_x,ref_y,ref_z);
  }
  
  size_t ProxEngine::get_number_of_scripted_motions()
  {
    assert( m_data || !"set_scripted_oscilation_paramters(): data was null pointer");
    
    return m_data->m_oscillation_motions.size() + m_data->m_keyframe_motions.size();
  }
  
  void ProxEngine::get_scripted_motion_indices( size_t * index_array )
  {
    assert( m_data || !"get_scripted_motion_indices(): data was null pointer");
    
    size_t count = 0;
    
    {
      std::map<size_t, ProxData::OscillationMotion>::const_iterator oscilation = m_data->m_oscillation_motions.begin();
      std::map<size_t, ProxData::OscillationMotion>::const_iterator end        = m_data->m_oscillation_motions.end();
      
      for(;oscilation!=end;++oscilation,++count)
      {
        index_array[count] = oscilation->second.m_index;
      }
    }
    {
      std::map<size_t, ProxData::KeyframeMotion>::const_iterator keyframe = m_data->m_keyframe_motions.begin();
      std::map<size_t, ProxData::KeyframeMotion>::const_iterator end      = m_data->m_keyframe_motions.end();
      
      for(;keyframe!=end;++keyframe,++count)
      {
        index_array[count] = keyframe->second.m_index;
      }
    }
  }
  
  bool ProxEngine::is_scripted_motion_keyframe( size_t const & motion_index )
  {
    assert( m_data || !"is_scripted_motion_keyframe(): data was null pointer");
    
    std::map<size_t, ProxData::KeyframeMotion>::iterator lookup = m_data->m_keyframe_motions.find(motion_index);
    
    return lookup  != m_data->m_keyframe_motions.end();
  }
  
  bool ProxEngine::is_scripted_motion_oscilation( size_t const & motion_index )
  {
    assert( m_data || !"is_scripted_motion_oscilation(): data was null pointer");
    
    std::map<size_t, ProxData::OscillationMotion>::iterator lookup = m_data->m_oscillation_motions.find(motion_index);
    
    return lookup  != m_data->m_oscillation_motions.end();
  }
  
  void ProxEngine::get_scripted_oscilation_paramters(
                                                     size_t const & motion_index
                                                     , float & amplitude
                                                     , float & frequency
                                                     , float & phase
                                                     , float & dir_x
                                                     , float & dir_y
                                                     , float & dir_z
                                                     )
  {
    assert( m_data || !"get_scripted_oscilation_paramters(): data was null pointer");
    
    std::map<size_t, ProxData::OscillationMotion>::iterator lookup = m_data->m_oscillation_motions.find(motion_index);
    
    assert( lookup  != m_data->m_oscillation_motions.end() || !"get_scripted_oscilation_paramters(): motion was not oscilation scripted motion");
    
    amplitude = lookup->second.m_amplitude;
    frequency = lookup->second.m_frequency;
    phase     = lookup->second.m_phase;
    dir_x     = lookup->second.m_direction(0);
    dir_y     = lookup->second.m_direction(1);
    dir_z     = lookup->second.m_direction(2);
  }
  
  size_t ProxEngine::get_number_of_key_frame_positions( size_t const & motion_index)
  {
    assert( m_data || !"get_number_of_key_frame_positions(): data was null pointer");
    
    std::map<size_t, ProxData::KeyframeMotion>::iterator lookup = m_data->m_keyframe_motions.find(motion_index);
    
    assert( lookup  != m_data->m_keyframe_motions.end() || !"get_number_of_key_frame_positions(): motion was not keyframe scripted motion");
    
    return lookup->second.m_positions.size();
  }
  
  size_t ProxEngine::get_number_of_key_frame_orientations( size_t const & motion_index)
  {
    assert( m_data || !"get_number_of_key_frame_orientations(): data was null pointer");
    
    std::map<size_t, ProxData::KeyframeMotion>::iterator lookup = m_data->m_keyframe_motions.find(motion_index);
    
    assert( lookup  != m_data->m_keyframe_motions.end() || !"get_number_of_key_frame_positions(): motion was not keyframe scripted motion");
    
    return lookup->second.m_orientations.size();
  }
  
  void ProxEngine::get_key_frame_positions(
                                           size_t const & motion_index
                                           , float * time_array
                                           , float * x_array
                                           , float * y_array
                                           , float * z_array
                                           )
  {
    assert( m_data || !"get_key_frame_positions(): data was null pointer");
    
    std::map<size_t, ProxData::KeyframeMotion>::iterator lookup = m_data->m_keyframe_motions.find(motion_index);
    
    assert( lookup  != m_data->m_keyframe_motions.end() || !"get_number_of_key_frame_positions(): motion was not keyframe scripted motion");
    
    std::vector<ProxData::KeyPosition>::const_iterator key = lookup->second.m_positions.begin();
    std::vector<ProxData::KeyPosition>::const_iterator end = lookup->second.m_positions.end();
    
    size_t count = 0u;
    for(;key!=end;++key,++count)
    {
      time_array[count] = key->m_time;
      x_array[count] = key->m_value(0);
      y_array[count] = key->m_value(1);
      z_array[count] = key->m_value(2);
    }
  }
  
  void ProxEngine::get_key_frame_orientations(
                                              size_t const & motion_index
                                              , float * time_array
                                              , float * qs_array
                                              , float * qx_array
                                              , float * qy_array
                                              , float * qz_array
                                              )
  {
    assert( m_data || !"get_key_frame_orientations(): data was null pointer");
    
    std::map<size_t, ProxData::KeyframeMotion>::iterator lookup = m_data->m_keyframe_motions.find(motion_index);
    
    assert( lookup  != m_data->m_keyframe_motions.end() || !"get_key_frame_orientations(): motion was not keyframe scripted motion");
    
    std::vector<ProxData::KeyOrientation>::const_iterator key = lookup->second.m_orientations.begin();
    std::vector<ProxData::KeyOrientation>::const_iterator end = lookup->second.m_orientations.end();
    
    size_t count = 0u;
    for(;key!=end;++key,++count)
    {
      time_array[count] = key->m_time;
      qs_array[count]   = key->m_value.real();
      qx_array[count]   = key->m_value.imag()(0);
      qy_array[count]   = key->m_value.imag()(1);
      qz_array[count]   = key->m_value.imag()(2);
    }
  }

  bool ProxEngine::compute_raycast(
                                   float const & p_x
                                   , float const & p_y
                                   , float const & p_z
                                   , float const & ray_x
                                   , float const & ray_y
                                   , float const & ray_z
                                   , size_t & body_idx
                                   , float & hit_x
                                   , float & hit_y
                                   , float & hit_z
                                   , float & distance
                                   )
  {
    assert( m_data || !"compute_raycast(): data was null pointer");

    return m_data->compute_raycast(
                                   p_x, p_y, p_z,
                                   ray_x, ray_y, ray_z,
                                   body_idx,
                                   hit_x, hit_y, hit_z,
                                   distance
                                   );
  }

}// namespace simulators
