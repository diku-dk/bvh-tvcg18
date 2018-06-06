#include <content_io_reader.h>
#include <content_io_utils.h>

#include <content_io_base.h>

#include <cassert>
#include <stdexcept>  // needed for std::runtime_error

namespace content
{
  namespace details
  {
    
    bool read_point(TiXmlElement const * tag, Cache & data )
    {
      assert( tag || !"missing point tag");
      
      Point P;
      
      P.m_x = read_float( tag, "x", 0.0f, false );
      P.m_y = read_float( tag, "y", 0.0f, false );
      P.m_z = read_float( tag, "z", 0.0f, false );
      
      data.add_point( P ); 
      return true;
    }
    
    bool read_box(TiXmlElement const * tag, Cache & data )
    {
      assert( tag || !"missing box tag");
      size_t geometry_idx = data.get_idx();
      
      // read width height and depth of box
      float w = read_float( tag, "width",  1.0f, false );
      float h = read_float( tag, "height", 1.0f, false );
      float d = read_float( tag, "depth",  1.0f, false );
      
      size_t box_number = data.input()->create_box_shape( geometry_idx );
      data.input()->set_box_shape( geometry_idx, box_number,  w, h, d);
      TiXmlElement const * transform = get_singleton_child(tag,"transform", false);
      if( transform)
      {
        Transform T = read_transform( transform, data );
        data.input()->set_box_position( geometry_idx, box_number,  T.m_x, T.m_y, T.m_z);
        data.input()->set_box_orientation( geometry_idx, box_number, T.m_qs, T.m_qx, T.m_qy, T.m_qz );
      }      
      return true;
    }
    
    bool read_capsule(TiXmlElement const * tag, Cache & data )
    {
      assert( tag || !"missing capsule tag");
      size_t geometry_idx = data.get_idx();
      
      // Read height and radius of capsule
      float h = read_float( tag, "height", 1.0f, false );
      float r = read_float( tag, "radius", 1.0f, false );
      
      size_t capsule_number = data.input()->create_capsule_shape( geometry_idx );
      data.input()->set_capsule_shape( geometry_idx, capsule_number,  r, h);
      TiXmlElement const * transform = get_singleton_child(tag,"transform", false);
      if( transform)
      {
        Transform T = read_transform( transform, data );
        data.input()->set_capsule_position( geometry_idx, capsule_number,  T.m_x, T.m_y, T.m_z);
        data.input()->set_capsule_orientation( geometry_idx, capsule_number, T.m_qs, T.m_qx, T.m_qy, T.m_qz );
      }      
      return true;
    }
    
    bool read_cone(TiXmlElement const * tag, Cache & data )
    {
      assert( tag || !"missing cone tag");
      size_t geometry_idx = data.get_idx();
      
      // Read height of cone and base radius
      float h = read_float( tag, "height", 1.0f, false );
      float r = read_float( tag, "radius", 1.0f, false );
      
      size_t cone_number = data.input()->create_cone_shape( geometry_idx );
      data.input()->set_cone_shape( geometry_idx, cone_number,  r, h);
      TiXmlElement const * transform = get_singleton_child(tag,"transform", false);
      if( transform)
      {
        Transform T = read_transform( transform, data );
        data.input()->set_cone_position( geometry_idx, cone_number,  T.m_x, T.m_y, T.m_z);
        data.input()->set_cone_orientation( geometry_idx, cone_number, T.m_qs, T.m_qx, T.m_qy, T.m_qz );
      }      
      return true;
    }
    
    bool read_convex(TiXmlElement const * tag, Cache & data )
    {
      assert( tag || !"missing convex tag");
      size_t geometry_idx = data.get_idx();
      
      data.clear_points();
      if ( !read_collection( tag, "point", &read_point, data  ) ) 
        return false;
      
      size_t N = data.size_points();
      std::vector<float> coordinates;
      coordinates.resize(3*N);
      
      Point const * points = data.get_points();
      for(size_t i=0;i<N;++i,++points)
      {
        coordinates[i*3]   = points->m_x;
        coordinates[i*3+1] = points->m_y;
        coordinates[i*3+2] = points->m_z;
      }
      
      size_t convex_number = data.input()->create_convex_shape( geometry_idx );
      data.input()->set_convex_shape( geometry_idx, convex_number,  N, &coordinates[0]);
      TiXmlElement const * transform = get_singleton_child(tag,"transform", false);
      if( transform)
      {
        Transform T = read_transform( transform, data );
        data.input()->set_convex_position( geometry_idx, convex_number,  T.m_x, T.m_y, T.m_z);
        data.input()->set_convex_orientation( geometry_idx, convex_number, T.m_qs, T.m_qx, T.m_qy, T.m_qz );
      }      
      return true;
    }

    bool read_vertex(TiXmlElement const * tag, Cache & data )
    {
      assert( tag || !"missing vertex tag");

      Vertex V;

      V.m_idx = read_integer(tag, "idx", 0, true );
      V.m_x   = read_float( tag, "x", 0.0f, true );
      V.m_y   = read_float( tag, "y", 0.0f, true );
      V.m_z   = read_float( tag, "z", 0.0f, true );

      data.add_vertex( V );
      return true;
    }

    bool read_tetrahedron(TiXmlElement const * tag, Cache & data )
    {
      assert( tag || !"missing vertex tag");

      Tetrahedron T;

      T.m_i = read_integer(tag, "i", 0, true );
      T.m_j = read_integer(tag, "j", 0, true );
      T.m_k = read_integer(tag, "k", 0, true );
      T.m_m = read_integer(tag, "m", 0, true );

      assert( T.m_i != T.m_j || !"read_tetrahedron(): i and j was the same index");
      assert( T.m_i != T.m_k || !"read_tetrahedron(): i and k was the same index");
      assert( T.m_i != T.m_m || !"read_tetrahedron(): i and m was the same index");
      assert( T.m_j != T.m_k || !"read_tetrahedron(): j and k was the same index");
      assert( T.m_j != T.m_m || !"read_tetrahedron(): j and m was the same index");
      assert( T.m_k != T.m_m || !"read_tetrahedron(): k and m was the same index");
      
      data.add_tetrahedron( T );
      return true;
    }

    bool read_tetramesh(TiXmlElement const * tag, Cache & data )
    {
      assert( tag || !"missing tetrahedra tag");
      size_t const geometry_idx = data.get_idx();

      data.clear_vertices();
      if ( !read_collection( tag, "vertex", &read_vertex, data  ) )
        return false;

      data.clear_tetrahedra();
      if ( !read_collection( tag, "tetrahedron", &read_tetrahedron, data  ) )
        return false;

      size_t const N = data.size_vertices();
      size_t const K = data.size_tetrahedra();

      std::vector<size_t> vertices;
      std::vector<size_t> tetrahedra;
      std::vector<float> coordinates;
      vertices.resize(N);
      tetrahedra.resize(4*K);
      coordinates.resize(3*N);

      Vertex const * V = data.get_vertices();
      for(size_t n=0;n<N;++n,++V)
      {
        vertices[n]        = V->m_idx;
        coordinates[n*3]   = V->m_x;
        coordinates[n*3+1] = V->m_y;
        coordinates[n*3+2] = V->m_z;
      }
      Tetrahedron const * T = data.get_tetrahedra();
      for(size_t k=0;k<K;++k,++T)
      {
        tetrahedra[4*k  ] = T->m_i;
        tetrahedra[4*k+1] = T->m_j;
        tetrahedra[4*k+2] = T->m_k;
        tetrahedra[4*k+3] = T->m_m;
      }

      data.input()->create_tetramesh_shape( geometry_idx );
      data.input()->set_tetramesh_shape( geometry_idx, N, K, &vertices[0], &tetrahedra[0], &coordinates[0]);
      return true;
    }

    bool read_cylinder(TiXmlElement const * tag, Cache & data )
    {
      assert( tag || !"missing cylinder tag");
      size_t geometry_idx = data.get_idx();
      
      // Read height and radius of cylinder
      float h = read_float( tag, "height", 1.0f, false );
      float r = read_float( tag, "radius", 1.0f, false );
      
      size_t cylinder_number = data.input()->create_cylinder_shape( geometry_idx );
      data.input()->set_cylinder_shape( geometry_idx, cylinder_number,  r, h);
      TiXmlElement const * transform = get_singleton_child(tag,"transform", false);
      if( transform)
      {
        Transform T = read_transform( transform, data );
        data.input()->set_cylinder_position( geometry_idx, cylinder_number,  T.m_x, T.m_y, T.m_z);
        data.input()->set_cylinder_orientation( geometry_idx, cylinder_number, T.m_qs, T.m_qx, T.m_qy, T.m_qz );
      }      
      return true;
    }
    
    bool read_ellipsoid(TiXmlElement const * tag, Cache & data )
    {
      assert( tag || !"missing ellipsoid tag");
      size_t geometry_idx = data.get_idx();
      
      // Read scales along the principal axes
      TiXmlElement const * scale = get_singleton_child(tag, "scale", false);
      float sx =  1.0f;
      float sy =  1.0f;
      float sz =  1.0f;
      if(scale)
      {        
        sx = read_float( scale, "x", sx, false );
        sy = read_float( scale, "y", sy, false );
        sz = read_float( scale, "z", sz, false );
      }
      
      size_t ellipsoid_number = data.input()->create_ellipsoid_shape( geometry_idx );
      data.input()->set_ellipsoid_shape( geometry_idx, ellipsoid_number,  sx, sy, sz);
      TiXmlElement const * transform = get_singleton_child(tag,"transform", false);
      if( transform)
      {
        Transform T = read_transform( transform, data );
        data.input()->set_ellipsoid_position( geometry_idx, ellipsoid_number,  T.m_x, T.m_y, T.m_z);
        data.input()->set_ellipsoid_orientation( geometry_idx, ellipsoid_number, T.m_qs, T.m_qx, T.m_qy, T.m_qz );
      }      
      return true;
    }
    
    bool read_sphere(TiXmlElement const * tag, Cache & data )
    {
      assert( tag || !"missing sphere tag");
      size_t geometry_idx = data.get_idx();
      
      // read radius of shpere
      float r = read_float( tag, "radius", 1.0f, false );
      
      size_t sphere_number = data.input()->create_sphere_shape( geometry_idx );
      data.input()->set_sphere_shape( geometry_idx, sphere_number,  r);
      TiXmlElement const * transform = get_singleton_child(tag,"transform", false);
      if( transform)
      {
        Transform T = read_transform( transform, data );
        data.input()->set_sphere_position( geometry_idx, sphere_number,  T.m_x, T.m_y, T.m_z);
        data.input()->set_sphere_orientation( geometry_idx, sphere_number, T.m_qs, T.m_qx, T.m_qy, T.m_qz );
      }      
      return true;
    }
    
    bool read_geometry(TiXmlElement const * tag, Cache & data )
    {
      assert( tag || !"missing geometry tag");
      
      std::string name = read_string(tag, "name", true);
      if(name.size()==0)
        return false;
      
      std::string type = read_string(tag, "type");
      
      // Default type if not specified is collection
      if(type.size()==0 || type.compare("collection")==0)
      {
        size_t geometry_idx = data.input()->create_collision_geometry(name);
        data.set_idx_for_name( name ,  geometry_idx );
        data.set_idx( geometry_idx );
        
        if ( !read_collection( tag, "box", &read_box, data  ) ) 
          return false;
        if ( !read_collection( tag, "capsule", &read_capsule, data  ) ) 
          return false;
        if ( !read_collection( tag, "cone", &read_cone, data  ) ) 
          return false;
        if ( !read_collection( tag, "convex", &read_convex, data  ) ) 
          return false;
        if ( !read_collection( tag, "cylinder", &read_cylinder, data  ) ) 
          return false;
        if ( !read_collection( tag, "ellipsoid", &read_ellipsoid, data  ) ) 
          return false;
        if ( !read_collection( tag, "sphere", &read_sphere, data  ) )
          return false;

        TiXmlElement const * tetramesh  = get_singleton_child( tag, "tetramesh" );
        if( tetramesh )
        {
          if(!read_tetramesh(tetramesh, data ) )
            return false;
        }

        return true;
      }
      
      // no geometry shape information was found?
      return false;
    }
    
    bool read_property(TiXmlElement const * tag, Cache & data )
    {
      // get restitution coefficient
      float e     = read_float(tag, "restitution", 0.0f, false );
      
      // get planar friction coefficients (x,y) and torque coefficient (z)
      TiXmlElement const * friction = get_singleton_child(tag, "friction", false);
      float mu_x = 0.0f;
      float mu_y = 0.0f;
      float mu_z = 0.0f;      
      if(friction)
      {
        mu_x = read_float(friction, "x", mu_x, false );
        mu_y = read_float(friction, "y", mu_y, false );
        mu_z = read_float(friction, "z", mu_z, false );
      }
      
      TiXmlElement const * direction = get_singleton_child(tag, "direction", false);
      float dir_x = 0.0f;
      float dir_y = 0.0f;
      float dir_z = 0.0f;      
      if(direction)
      {
        dir_x = read_float(direction, "x", dir_x, false );
        dir_y = read_float(direction, "y", dir_y, false );
        dir_z = read_float(direction, "z", dir_z, false );
      }
      
      std::string materials = read_string(tag,"materials",true);
      if(materials.size()==0)
        return false;
      
      // Now extract the paired material names from the materials-string
      size_t first_pos = 0u;
      size_t last_pos  = (materials.size() -  1u);
      
      char  first_char = materials[first_pos];   // 2009-12-27 kenny: hmm maybe utf8/16 problems?
      char  last_char  = materials[ last_pos];
      size_t comma_pos = materials.find(',', 1);  // 2009-12-27 kenny: hmm maybe utf8/16 problems?
      
      if(first_pos>=last_pos || first_char!='(' || last_char!=')' || comma_pos<=first_pos ||comma_pos>=last_pos )
      {
        throw std::runtime_error("invalid syntax for material pair references");
        return false;
      }
      
      std::string first_material  = materials.substr(          1u, comma_pos - 1u );
      std::string second_material = materials.substr(comma_pos+1u, last_pos - comma_pos - 1u  );
      
      size_t first_idx  = data.get_idx_from_name( first_material ); 
      size_t second_idx = data.get_idx_from_name( second_material ); 
      
      // get master material
      std::string master = read_string(tag, "master", true);

      size_t master_idx = 0u;

      if(master.size()==0u)
      {
        throw std::runtime_error( "Missing master of the paired materials: " + first_material + " or " + second_material );
        return false;
      }
      
      if( !(master.compare( first_material)==0 || master.compare( second_material)==0) )
      {
        throw std::runtime_error( master + " was none one of the paired materials: " + first_material + " or " + second_material );
        return false;
      }
      
      master_idx = data.get_idx_from_name( master);
      
      if( !(master_idx == first_idx || master_idx == second_idx) )
      {
        throw std::runtime_error( " master idx was not equal to first or second idx" );
        return false;
      }
      
      data.input()->create_material_property( first_idx, second_idx );
      data.input()->set_master( first_idx, second_idx, master_idx );
      data.input()->set_friction( first_idx, second_idx, mu_x, mu_y, mu_z );
      data.input()->set_master_direction( first_idx, second_idx, dir_x, dir_y, dir_z );
      data.input()->set_restitution( first_idx, second_idx, e );
      
      return true;
    }
    
    bool read_material(TiXmlElement const * tag, Cache & data )
    {
      assert(tag || !"material tag was missing?");
      std::string name = read_string( tag, "name", true);
      if(name.size()==0)
        return false;
      size_t material_idx = data.input()->create_material(name);
      data.set_idx_for_name( name, material_idx );
      return true;
    }
    
    bool read_gravity(TiXmlElement const * tag, Cache & data )
    {
      assert(tag || !"gravity tag was missing");

      float const x   = read_float(tag, "x", 0.0f, false);
      float const y   = read_float(tag, "y", 1.0f, false);
      float const z   = read_float(tag, "z", 0.0f, false);
      float const acc = read_float(tag, "acc", 9.81f, false);

      data.input()->set_gravity_up(x ,y, z);
      data.input()->set_gravity_acceleration(acc);

      return true;
    }

    bool read_damping(TiXmlElement const * tag, Cache & data )
    {
      assert(tag || !"damping tag was missing");

      float const linear  = read_float(tag, "linear", 0.01f, false);
      float const angular = read_float(tag, "angular", 0.01f, false);

      data.input()->set_damping_parameters(linear,angular);

      return true;
    }
    
    bool read_apply_force(TiXmlElement const * tag, Cache & data )
    {
      assert(tag || !"apply force tag was missing");
      
      std::string ref = read_string(tag, "ref", true);

      if(ref.size()==0)
        return false;
      
      size_t force_idx = data.get_idx_from_name( ref );
      size_t body_idx  = data.get_idx();
      
      data.input()->connect_force( body_idx, force_idx );
      return true;
    } 
    
    RigidBodyState read_state (TiXmlElement const * tag, Cache & data )
    {
      assert(tag || !"state tag was missing");
      
      RigidBodyState value;
      
      // check to see if this state references a global state
      std::string ref = read_string(tag,"ref", false);
      if(ref.size() != 0)
      {
        value = data.get_state_from_name(ref);
      }
      
      TiXmlElement const * transform = get_singleton_child(tag, "transform", false);
      if(transform)
        value.m_T = read_transform( transform, data, value.m_T );
      
      TiXmlElement const * motion = get_singleton_child(tag, "motion", false);
      if(motion)
        value.m_M = read_motion( motion, data, value.m_M );
      
      TiXmlElement const * mass = get_singleton_child(tag, "mass", false);
      if(mass)
        value.m_mass = read_float(mass,   "value",   value.m_mass, false );
      
      TiXmlElement const * inertia = get_singleton_child(tag, "inertia", false);
      if(inertia)
      {
        value.m_Ixx = read_float(inertia, "xx", value.m_Ixx, false );
        value.m_Iyy = read_float(inertia, "yy", value.m_Iyy, false );
        value.m_Izz = read_float(inertia, "zz", value.m_Izz, false );
      }
      
      value.m_active = read_bool( tag, "active", value.m_active, false );
      value.m_fixed  = read_bool( tag, "fixed" ,  value.m_fixed, false );

      value.m_scripted = read_bool( tag, "scripted" ,  value.m_scripted, false );
      std::string motion_name = read_string(tag, "scripted_motion", value.m_scripted);
      if(motion_name.size()>0u)
        value.m_motion_idx = data.get_idx_from_name( motion_name );

      bool mandatory = ref.size() == 0;
      std::string material = read_string(tag, "material", mandatory);
      if(material.size()>0u)
        value.m_material_idx = data.get_idx_from_name( material );
      
      return value;
    }
    
    bool read_global_state (TiXmlElement const * tag, Cache & data )
    {
      assert( tag || !"state tag was missing");
      
      std::string name = read_string(tag, "name", true);
      if(name.size() == 0)
        return false;
      
      RigidBodyState value = read_state( tag, data);
      data.set_state_for_name(name, value);
      
      return true;
    }      
    
    bool read_global_transform (TiXmlElement const * tag, Cache & data )
    {
      assert(tag || !"transform tag was missing");
      
      std::string name = read_string(tag,"name", true);
      if(name.size() == 0)
        return false;
      
      Transform value = read_transform( tag, data);
      data.set_transform_for_name(name, value);
      
      return true;
    }      
    
    bool read_global_motion (TiXmlElement const * tag, Cache & data )
    {
      assert(tag || !"motion tag was missing");
      
      std::string name = read_string(tag,"name", true);
      if(name.size() == 0)
        return false;
      
      Motion value = read_motion( tag, data);
      data.set_motion_for_name(name, value);
      
      return true;
    }      
    
    bool read_object (TiXmlElement const * tag, Cache & data )
    {
      assert(tag || !"tag was null");
      
      std::string name = read_string(tag, "name", true);
      if(name.size()==0)
        return false;
      
      std::string type = read_string(tag, "type");
      
      if(type.size()==0 || type.compare("rigid")==0)
      {
        RigidBodyState state = read_state( get_singleton_child(tag,"state", true), data ); 
        
        size_t body_idx = data.input()->create_rigid_body( name );
        data.input()->set_rigid_body_position(    body_idx, state.m_T.m_x,  state.m_T.m_y , state.m_T.m_z                  );
        data.input()->set_rigid_body_orientation( body_idx, state.m_T.m_qs, state.m_T.m_qx, state.m_T.m_qy, state.m_T.m_qz );
        data.input()->set_rigid_body_velocity(    body_idx, state.m_M.m_vx, state.m_M.m_vy, state.m_M.m_vz                 );
        data.input()->set_rigid_body_spin(        body_idx, state.m_M.m_wx, state.m_M.m_wy, state.m_M.m_wz                 );
        data.input()->set_rigid_body_inertia(     body_idx, state.m_Ixx,    state.m_Iyy,    state.m_Izz                    );
        data.input()->set_rigid_body_mass(        body_idx, state.m_mass);
        data.input()->set_rigid_body_active(      body_idx, state.m_active);
        data.input()->set_rigid_body_fixed(       body_idx, state.m_fixed);
        data.input()->set_rigid_body_material(    body_idx, state.m_material_idx);

        if( state.m_scripted && state.m_motion_idx!=UNDEFINED)
          data.input()->connect_scripted_motion(body_idx, state.m_motion_idx);

        data.set_idx_for_name( name , body_idx );
        data.set_idx( body_idx );
        
        // Get reference to the collision shape
        std::string geometry_ref = read_string(get_singleton_child(tag,"shape", true), "ref", true);
        size_t geometry_idx = data.get_idx_from_name( geometry_ref );
        data.input()->connect_collision_geometry(body_idx, geometry_idx );
        
        // Get all applied external forces on the object
        TiXmlElement const * forces = get_singleton_child(tag,"forces",false);
        if(forces)
        {
          if ( !read_collection( forces, "apply", &read_apply_force , data  ) )
            return false;
        }

        return true;
      }
      
      throw std::runtime_error( "object " + name + " of " + type + " can not be processed" );
      
      return false;
    }
    
    bool read_params( TiXmlElement const * tag, Cache & data  )
    {
      assert(tag || !"tag was null");
      
      TiXmlElement const * envelope = get_singleton_child(tag,"envelope",false);
      if(envelope)
      {
        float value = read_float( envelope,  "value", 0.01f );
        data.input()->set_parameter("collision_envelope", value );
      }
      
      TiXmlElement const * timestep = get_singleton_child(tag,"timestep",false);
      if(timestep)  
      {
        float value = read_float( timestep,  "value", 0.01f );      
        data.input()->set_parameter("time_step", value );
      }     
      return true;
    }

    bool read_pin_force( TiXmlElement const * tag, Cache & data  )
    {
      assert(tag || !"tag was null");

      std::string name = read_string(tag,"name", true);
      if(name.size() == 0)
        return false;

      float const tau      = read_float( tag,  "tau", 0.1f );
      float const anchor_x = read_float( tag,  "anchor_x", 0.0f );
      float const anchor_y = read_float( tag,  "anchor_y", 0.0f );
      float const anchor_z = read_float( tag,  "anchor_z", 0.0f );
      float const target_x = read_float( tag,  "target_x", 0.0f );
      float const target_y = read_float( tag,  "target_y", 0.0f );
      float const target_z = read_float( tag,  "target_z", 0.0f );

      size_t const pin_idx = data.input()->create_pin_force();

      data.set_idx_for_name( name , pin_idx );

      data.input()->set_pin_tau(pin_idx, tau);
      data.input()->set_pin_anchor(pin_idx, anchor_x, anchor_y, anchor_z);
      data.input()->set_pin_target(pin_idx, target_x, target_y, target_z);

      return true;
    }

    bool read_forces( TiXmlElement const * tag, Cache & data  )
    {
      assert(tag || !"tag was null");

      TiXmlElement const * gravity = get_singleton_child(tag,"gravity",false);
      if(gravity)
      {
        read_gravity(gravity, data);
      }

      TiXmlElement const * damping = get_singleton_child(tag,"damping",false);
      if(damping)
      {
        read_damping(damping, data);
      }

      if ( !read_collection( tag,     "pin",     &read_pin_force, data ) ) return false;


      return true;
    }

    bool read_position_keytick( TiXmlElement const * tag, Cache & data  )
    {
      assert(tag || !"tag was null");

      float const time  = read_float( tag,  "time", 0.0f );
      float const x     = read_float( tag,  "x", 0.0f );
      float const y     = read_float( tag,  "y", 0.0f );
      float const z     = read_float( tag,  "z", 0.0f );

      size_t const motion_idx  = data.get_idx();

      data.input()->set_scripted_key_position(motion_idx, time, x, y, z);

      return true;
    }

    bool read_orientation_keytick( TiXmlElement const * tag, Cache & data  )
    {
      assert(tag || !"tag was null");

      float const time  = read_float( tag,  "time", 0.0f );
      float const qs     = read_float( tag,  "qs", 0.0f );
      float const qx     = read_float( tag,  "qx", 0.0f );
      float const qy     = read_float( tag,  "qy", 0.0f );
      float const qz     = read_float( tag,  "qz", 0.0f );

      size_t const motion_idx  = data.get_idx();

      data.input()->set_scripted_key_orientation(motion_idx, time, qs, qx, qy, qz);

      return true;
    }

    bool read_keyframe_scripted_motion( TiXmlElement const * tag, Cache & data  )
    {
      assert(tag || !"tag was null");

      std::string name = read_string(tag,"name", true);
      if(name.size() == 0)
        return false;

      size_t const motion_idx = data.input()->create_key_frame_scripted_motion();

      data.set_idx_for_name( name , motion_idx );

      data.set_idx(motion_idx);

      if ( !read_collection( tag,     "position",     &read_position_keytick, data ) ) return false;
      if ( !read_collection( tag,     "orientation",     &read_orientation_keytick, data ) ) return false;

      return true;
    }

    bool read_oscilation_scripted_motion( TiXmlElement const * tag, Cache & data  )
    {
      assert(tag || !"tag was null");

      std::string name = read_string(tag,"name", true);
      if(name.size() == 0)
        return false;

      float const amplitude = read_float( tag,  "amplitude", 1.0f );
      float const frequency = read_float( tag,  "frequency", 1.0f );
      float const phase     = read_float( tag,  "phase", 0.0f );
      float const dir_x     = read_float( tag,  "dir_z", 0.0f );
      float const dir_y     = read_float( tag,  "dir_y", 1.0f );
      float const dir_z     = read_float( tag,  "dir_z", 0.0f );
      float const ref_x     = read_float( tag,  "ref_x", 0.0f );
      float const ref_y     = read_float( tag,  "ref_y", 0.0f );
      float const ref_z     = read_float( tag,  "ref_z", 0.0f );

      size_t const motion_idx = data.input()->create_oscilation_scripted_motion();

      data.set_idx_for_name( name , motion_idx );

      data.input()->set_scripted_oscilation_paramters(
                                                      motion_idx
                                                      , amplitude
                                                      , frequency
                                                      , phase
                                                      , dir_x
                                                      , dir_y
                                                      , dir_z
                                                      , ref_x
                                                      , ref_y
                                                      , ref_z
                                                      );
      return true;
    }

    bool read_scripted_motions( TiXmlElement const * tag, Cache & data  )
    {
      assert(tag || !"tag was null");

      if ( !read_collection( tag,     "keyframe",     &read_keyframe_scripted_motion, data ) ) return false;

      if ( !read_collection( tag,     "oscilation",     &read_oscilation_scripted_motion, data ) ) return false;

      return true;
    }

    bool read_configuration ( TiXmlElement const * tag, Cache & data  )
    {
      assert(tag || !"tag was null");
      
      if ( !read_collection( tag, "transform", &read_global_transform, data ) ) return false;
      if ( !read_collection( tag,    "motion",    &read_global_motion, data )    ) return false;
      if ( !read_collection( tag,     "state",     &read_global_state, data ) ) return false;
      
      if ( !read_collection( get_singleton_child( tag, "geometries" ), "geometry", &read_geometry, data  ) ) return false;

      if ( !read_forces( get_singleton_child( tag, "forces" ), data  ) ) return false;

      if ( !read_scripted_motions( get_singleton_child( tag, "scripted_motions" ), data  ) ) return false;

      if ( !read_collection( get_singleton_child( tag, "objects" )   , "object"  , &read_object  , data  ) ) return false;
      
      return true;
    }
    
    bool read_physics ( TiXmlElement const * tag, Cache & data  )
    {
      assert(tag || !"physics tag was missing");
      
      data.input()->clear();
      
      if( !read_params ( get_singleton_child( tag, "params" ), data ) ) 
        return false;
      
      TiXmlElement const * materials  = get_singleton_child( tag, "materials" );
      if( materials )
      {
        TiXmlElement const * properties = get_singleton_child( materials, "properties" );
        
        if ( !read_collection( materials , "material", &read_material, data  ) ) 
          return false;
        
        assert(properties || !"properties tag was missing");
        
        if ( !read_collection( properties , "property", &read_property, data  ) ) 
          return false;
      }
      
      if ( !read_configuration ( get_singleton_child( tag, "configuration" ), data ) )  
        return false;
      
      return true;
    }
    
  } // namespace details
  
  bool xml_read( std::string const & filename, content::Input * input )
  {
    assert( input || !"manager was null");
    
#ifdef TIXML_USE_STL
    TiXmlDocument xml_document ( filename );
#else
    TiXmlDocument xml_document ( filename.c_str() );
#endif
    if ( !xml_document.LoadFile() )
    {
      throw std::invalid_argument("read(...): Error " + filename  + " not found!");
      return false;
    }
    TiXmlHandle document_tag ( &xml_document );
    details::Cache data( input );
    
    TiXmlElement const * physics = details::get_singleton_child( document_tag.Node(), "physics", true );
    if ( !details::read_physics( physics, data ) )
    {
      return false;
    }
    return true;
  }
  
}// namespace content
