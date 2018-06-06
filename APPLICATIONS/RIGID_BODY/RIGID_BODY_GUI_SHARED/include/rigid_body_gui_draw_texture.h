#ifndef RIGID_BODY_GUI_DRAW_TEXTURE_H
#define RIGID_BODY_GUI_DRAW_TEXTURE_H

#include <gl3_texture_2d.h>
#include <gl3_shader.h>
#include <gl3_program.h>
#include <gl3_vbo.h>
#include <gl3_vao.h>

namespace rigid_body
{
  namespace gui
  {

    inline void draw_texture(
                             gl3::Program program
                             , gl3::Texture2D const & texture
                             , unsigned int const & x
                             , unsigned int const & y
                             , unsigned int const & width
                             , unsigned int const & height
                             )
    {
      gl3::check_errors("draw_texture(program, texture) invoked");

      gl3::VBO     vbo     = gl3::make_quad_vbo();

      gl3::VAO     vao     = gl3::make_vao( vbo, program, "position");

      GLint viewport_params[4];
      glGetIntegerv(GL_VIEWPORT, &viewport_params[0]);

      glViewport(x, y, width, height);

      GLboolean depth_param;

      glGetBooleanv(GL_DEPTH_TEST, &depth_param);
      glDisable(GL_DEPTH_TEST);

      program.use();

      glActiveTexture( GL_TEXTURE0 );          // Active texture unit 0
      texture.bind();                          // Bind this texture to texture unit 0
      program.set_uniform( "uvmap", 0 );       // Tell glsl to use texture unit 0

      vao.bind();
      vbo.draw();
      vao.unbind();

      texture.unbind();

      program.stop();

      glViewport(viewport_params[0], viewport_params[1], viewport_params[2], viewport_params[3]);

      if (depth_param)
        glEnable(GL_DEPTH_TEST);
    }

    inline void draw_texture(
                              std::string const & shader_path
                             , gl3::Texture2D const & texture
                             , unsigned int const & x
                             , unsigned int const & y
                             , unsigned int const & width
                             , unsigned int const & height
                             )
    {
      gl3::check_errors("draw_texture() invoked");

      std::string const vertex_shader_filename   = shader_path + "/show_texture/tex2d_vertex.glsl";
      std::string const fragment_shader_filename = shader_path + "/show_texture/tex2d_fragment.glsl";

      gl3::Shader  vs = gl3::make_shader_from_file(vertex_shader_filename, gl3::Shader::vertex_shader);
      gl3::Shader  fs = gl3::make_shader_from_file(fragment_shader_filename, gl3::Shader::fragment_shader);

      gl3::Program program = gl3::make_program(vs,fs);

      draw_texture(program, texture, x, y, width, height);

      vs.clear();
      fs.clear();
      program.clear();
    }

    inline void draw_red_texture(
                                 std::string const & shader_path
                                 , gl3::Texture2D const & texture
                                 , unsigned int const & x
                                 , unsigned int const & y
                                 , unsigned int const & width
                                 , unsigned int const & height
                                 )
    {
      gl3::check_errors("draw_texture() invoked");

      std::string const vertex_shader_filename   = shader_path + "/show_texture/red_vertex.glsl";
      std::string const fragment_shader_filename = shader_path + "/show_texture/red_fragment.glsl";

      gl3::Shader  vs = gl3::make_shader_from_file(vertex_shader_filename, gl3::Shader::vertex_shader);
      gl3::Shader  fs = gl3::make_shader_from_file(fragment_shader_filename, gl3::Shader::fragment_shader);

      gl3::Program program = gl3::make_program(vs,fs);

      draw_texture(program, texture, x, y, width, height);

      vs.clear();
      fs.clear();
      program.clear();
    }

    inline void draw_depth_texture(
                                   std::string const & shader_path
                                   , gl3::Texture2D const & texture
                                   , float const & z_near
                                   , float const & z_far
                                   , unsigned int const & x
                                   , unsigned int const & y
                                   , unsigned int const & width
                                   , unsigned int const & height
                                   )
    {
      gl3::check_errors("draw_depth_texture() invoked");

      std::string const vertex_shader_filename   = shader_path + "/show_texture/depth_vertex.glsl";
      std::string const fragment_shader_filename = shader_path + "/show_texture/depth_fragment.glsl";

      gl3::Shader  vs = gl3::make_shader_from_file(vertex_shader_filename, gl3::Shader::vertex_shader);
      gl3::Shader  fs = gl3::make_shader_from_file(fragment_shader_filename, gl3::Shader::fragment_shader);

      gl3::Program program = gl3::make_program(vs,fs);

      program.use();

      program.set_uniform("z_near", z_near);
      program.set_uniform("z_far", z_far);

      draw_texture(program, texture, x, y, width, height);

      vs.clear();
      fs.clear();
      program.clear();
    }

    inline void draw_normal_texture(
                                    std::string const & shader_path
                                    , gl3::Texture2D const & texture
                                    , unsigned int const & x
                                    , unsigned int const & y
                                    , unsigned int const & width
                                    , unsigned int const & height
                                    )
    {
      gl3::check_errors("draw_normal_texture() invoked");

      std::string const vertex_shader_filename   = shader_path + "/show_texture/normal_vertex.glsl";
      std::string const fragment_shader_filename = shader_path + "/show_texture/normal_fragment.glsl";

      gl3::Shader  vs = gl3::make_shader_from_file(vertex_shader_filename, gl3::Shader::vertex_shader);
      gl3::Shader  fs = gl3::make_shader_from_file(fragment_shader_filename, gl3::Shader::fragment_shader);

      gl3::Program program = gl3::make_program(vs,fs);

      draw_texture(program, texture, x, y, width, height);

      vs.clear();
      fs.clear();
      program.clear();
    }


    inline void draw_position_texture(
                                      std::string const & shader_path
                                      , gl3::Texture2D const & texture
                                      , float const & min_x
                                      , float const & min_y
                                      , float const & min_z
                                      , float const & max_x
                                      , float const & max_y
                                      , float const & max_z
                                      , unsigned int const & x
                                      , unsigned int const & y
                                      , unsigned int const & width
                                      , unsigned int const & height
                                      )
    {
      gl3::check_errors("draw_position_texture() invoked");

      std::string const vertex_shader_filename   = shader_path + "/show_texture/position_vertex.glsl";
      std::string const fragment_shader_filename = shader_path + "/show_texture/position_fragment.glsl";

      gl3::Shader  vs = gl3::make_shader_from_file(vertex_shader_filename, gl3::Shader::vertex_shader);
      gl3::Shader  fs = gl3::make_shader_from_file(fragment_shader_filename, gl3::Shader::fragment_shader);
      
      gl3::Program program = gl3::make_program(vs,fs);
      
      program.use();
      
      program.set_uniform("min_x", min_x);
      program.set_uniform("min_y", min_y);
      program.set_uniform("min_z", min_z);
      program.set_uniform("max_x", max_x);
      program.set_uniform("max_y", max_y);
      program.set_uniform("max_z", max_z);
      
      draw_texture(program, texture, x, y, width, height);
      
      vs.clear();
      fs.clear();
      program.clear();
    }
    
  }//namespace gui
}//namespace rigid_body

// RIGID_BODY_GUI_DRAW_TEXTURE_H
#endif