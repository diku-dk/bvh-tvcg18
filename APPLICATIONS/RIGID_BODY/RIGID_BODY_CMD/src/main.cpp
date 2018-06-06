#include <tiny.h>
#include <mass.h>
#include <geometry.h>
#include <mesh_array.h>
#include <simulators.h>
#include <procedural.h>

#include <util_config_file.h>
#include <util_string_helper.h>
#include <util_log.h>
#include <util_timestamp.h>

#include <boost/algorithm/string.hpp>

#include <csignal> // for handling SIGINT
#include <cstdlib> // for handling exit
#include <functional> // for passing a member function to a signal handler
#include <sstream>
#include <string>

namespace rigid_body
{
  namespace cmd
  {

    class Application
    {
    public:

      typedef tiny::MathTypes<float> MT;
      typedef MT::real_type          T;
      typedef MT::vector3_type       V;
      typedef MT::matrix3x3_type     M;
      typedef MT::quaternion_type    Q;
      typedef MT::coordsys_type      C;
      typedef MT::value_traits       VT;

    protected:

      T             m_time;
      T             m_time_step;
      T             m_total_time;
      bool          m_profiling;
      bool          m_xml_record;
      bool          m_xml_load;

      std::string   m_matlab_file;
      std::string   m_procedural_scene;
      std::string   m_xml_save_scene_file;
      std::string   m_xml_load_scene_file;
      std::string   m_xml_save_channel_file;
      std::string   m_xml_load_channel_file;

      std::string                       m_obj_path;
      std::string                       m_output_path;
      std::string                       m_working_directory;

      util::ConfigFile                    m_config_file;
      procedural::MaterialInfo<T>         m_mat_info;
      simulators::ProxEngine              m_engine;
      content::ChannelStorage             m_channel_storage;
      size_t                              m_key_idx;
      size_t                              m_max_keys;

    protected:

      void clear()
      {
        m_obj_path                 = "";
        m_output_path              = "";
        m_working_directory        = "";

        m_time                     = VT::zero();
        m_time_step                = VT::numeric_cast(0.01f);

        m_profiling                = false;
        m_xml_record               = false;
        m_xml_load                 = false;

        m_matlab_file              = "out.m";
        m_procedural_scene         = "wall";
        m_xml_save_channel_file    = "out_channels.xml";
        m_xml_load_channel_file    = "in_channels.xml";
        m_xml_save_scene_file      = "out_scene.xml";
        m_xml_load_scene_file      = "in_scene.xml";

        m_engine.clear();
        m_config_file.clear();
      }

    public:

      void load_xml_file()
      {
        using std::max;

        util::Log logging;

        content::xml_read(m_xml_load_scene_file, &m_engine);
        content::xml_read(m_xml_load_channel_file, m_channel_storage);

        logging << "Read XML data to " << m_xml_save_scene_file << " and " << m_xml_save_channel_file << util::Log::newline();


        //--- Determine the maximum number of keys that was loaded and initialize bodies with first key frame
        m_key_idx  = 0u;
        m_max_keys = 0u;

        for(size_t channel_idx = 0u; channel_idx < m_channel_storage.get_number_of_channels(); ++ channel_idx )
        {
          size_t const body_idx = m_channel_storage.get_channel_id( channel_idx);
          {
            float x= 0.0f;
            float y= 0.0f;
            float z= 0.0f;
            m_channel_storage.get_key_position( channel_idx, m_key_idx, x, y, z);
            m_engine.set_rigid_body_position( body_idx, x, y, z);
          }
          {
            float qs = 0.0f;
            float qx = 0.0f;
            float qy = 0.0f;
            float qz = 0.0f;
            m_channel_storage.get_key_orientation( channel_idx, m_key_idx, qs, qx, qy, qz);
            m_engine.set_rigid_body_orientation( body_idx, qs, qx, qy, qz);
          }

          m_max_keys = max (m_max_keys, m_channel_storage.get_number_of_keys(channel_idx) );
        }

        logging << "Maximum number of keys =  " << m_max_keys << util::Log::newline();

      }

      void save_xml_file()
      {
        content::xml_write(m_output_path + m_xml_save_scene_file, &m_engine);
        content::xml_write(m_output_path + m_xml_save_channel_file, m_channel_storage);
      }

      void load_config_file(std::string const & cfg_file)
      {
        clear();

        {
          util::Log logging;

          logging << "loading cfg: " << cfg_file << util::Log::newline();
        }

        bool cfg_loaded = m_config_file.load(cfg_file);

        assert(cfg_loaded || !"load_config_file(): cfg file could not be loaded");

        util::LogInfo::on()            = util::to_value<bool>(m_config_file.get_value("logging","true"));
        util::LogInfo::console()       = util::to_value<bool>(m_config_file.get_value("console","true"));
        util::LogInfo::filename()      = m_config_file.get_value("log_file","log.txt");

        {
          util::Log logging;

          logging << "### " << util::timestamp() << util::Log::newline();
        }

        m_obj_path               = m_config_file.get_path( "obj_path",                ""                   );
        m_working_directory      = m_config_file.get_path( "working_directory",       ""                   );
        m_output_path            = m_config_file.get_path( "output_path",             ""                   );

        m_matlab_file            = m_config_file.get_value( "matlab_file",             "output.m"           );
        m_procedural_scene       = m_config_file.get_value( "procedural_scene",        "wall"               );
        m_xml_save_scene_file    = m_config_file.get_value( "xml_save_scene_file",     "out_scene.xml"      );
        m_xml_load_scene_file    = m_config_file.get_value( "xml_load_scene_file",     "in_scene.xml"       );
        m_xml_save_channel_file  = m_config_file.get_value( "xml_save_channel_file",   "out_channels.xml"   );
        m_xml_load_channel_file  = m_config_file.get_value( "xml_load_channel_file",   "in_channels.xml"    );

        m_xml_record             = util::to_value<bool>(  m_config_file.get_value("xml_record", "false"   ) );
        m_xml_load               = util::to_value<bool>(  m_config_file.get_value("xml_load",   "false"   ) );
        m_total_time             = util::to_value<float>( m_config_file.get_value("total_time",  "3.0"    ) );
        m_time_step              = util::to_value<float>( m_config_file.get_value("time_step",   "0.01"   ) );
        m_profiling              = util::to_value<bool>(  m_config_file.get_value("profiling",   "false"  ) );

        m_engine.set_parameters_from_config_file( cfg_file );

        if(m_xml_load)
        {
          load_xml_file();
        }
        else
        {
          procedural::Noise::on()    = util::to_value<bool>(   m_config_file.get_value("procedural_noise_on",    "false") );
          procedural::Noise::scale() = util::to_value<double>( m_config_file.get_value("procedural_noise_scale", "0.001") );

          procedural::make_scene<MT>(m_procedural_scene, m_obj_path, &m_engine, m_config_file);
        }

        if(m_xml_record)
        {
          // Prepare xml channel storage for recording
          std::vector<size_t> rids;
          size_t const N = m_engine.get_number_of_rigid_bodies();
          rids.resize(N);
          m_engine.get_rigid_body_indices( &rids[0] );

          m_channel_storage.clear();

          for(size_t i = 0u; i < N; ++i)
          {
            size_t const id = rids[i];

            std::string const name = m_engine.get_rigid_body_name(id);

            m_channel_storage.create_channel( id, name );
          }
        }
      }

      void signal_handler(int signum) {
        if (m_xml_record) {
          save_xml_file();
        }

        if(m_profiling)
        {
          m_engine.write_profiling( m_output_path + m_matlab_file );
        }
      }

      int run()
      {
        while (m_time < m_total_time)
        {
          {
            util::Log logging;

            logging << "Application::run(): Time : " << m_time << "/" << m_total_time
                    << " (" << util::timestamp() << ")" << util::Log::newline();
            logging.flush();
          }

          m_engine.simulate(m_time_step);

          m_time = m_time + m_time_step;

          if (m_xml_record)
          {
            //--- record xml motion channel data -----------------------------

            float const time = m_engine.get_time();

            for(size_t channel_idx = 0u; channel_idx < m_channel_storage.get_number_of_channels(); ++channel_idx )
            {
              size_t const body_idx = m_channel_storage.get_channel_id( channel_idx);
              size_t const key_idx  = m_channel_storage.create_key( channel_idx, time);
              {
                float x= 0.0f;
                float y= 0.0f;
                float z= 0.0f;

                m_engine.get_rigid_body_position( body_idx, x, y, z);

                m_channel_storage.set_key_position( channel_idx, key_idx, x, y, z);
              }

              {
                float qs = 0.0f;
                float qx = 0.0f;
                float qy = 0.0f;
                float qz = 0.0f;

                m_engine.get_rigid_body_orientation( body_idx, qs, qx, qy, qz);

                m_channel_storage.set_key_orientation( channel_idx, key_idx, qs, qx, qy, qz);
              }
            }
          }

        }

        {
          util::Log logging;

          logging << "Application::run(): Total time reached "
          << m_total_time
          << util::Log::newline();
          logging.flush();
        }

        if(m_profiling)
        {
          m_engine.write_profiling( m_output_path + m_matlab_file );
        }

        if(m_xml_record)
        {
          save_xml_file();
        }

        return 0;
      }

    public:

      Application()
      {
      }
      
      virtual ~Application()
      {
        clear();
      }
      
    };
    
  }// end of namespace cmd
}// end of namespace rigid_body

rigid_body::cmd::Application g_app;

auto g_signal_handler = std::bind(std::mem_fn(&rigid_body::cmd::Application::signal_handler), &g_app, std::placeholders::_1);
void signal_handler(int signum) {
  g_signal_handler(signum);
  if (signum == SIGINT) {
    exit(signum);
  }
}

void exit_handler(void) {
  signal_handler(SIGUSR1);
}

int main(int argc, char **argv)
{
  std::signal(SIGINT, signal_handler);
  std::signal(SIGUSR1, signal_handler);

  atexit(exit_handler);
  
  std::string cfg_file = "";
  
  if (argv[1])
    cfg_file = argv[1];
  else
    cfg_file = "default.cfg";
  
  g_app.load_config_file(cfg_file);
  
  return g_app.run();
}
