#ifndef NARROW_PARAMS_H
#define NARROW_PARAMS_H

namespace narrow
{
  template<typename M>
  class Params
  {
  public:

    typedef typename M::real_type    T;
    typedef typename M::value_traits VT;

  protected:
    
    bool   m_use_open_cl;
    size_t m_open_cl_platform;
    size_t m_open_cl_device;
    bool   m_use_gproximity;
    bool   m_use_batching;
    T      m_envelope;              ///< Procentage of scale of smallest object size to be used as collision envelope
    size_t m_chunk_bytes;

  public:
    
    bool   const & use_open_cl()       const { return this->m_use_open_cl;        }
    size_t const & open_cl_platform()  const { return this->m_open_cl_platform;   }
    size_t const & open_cl_device()    const { return this->m_open_cl_device;     }
    bool   const & use_gproximity()    const { return this->m_use_gproximity;     }
    bool   const & use_batching()      const { return this->m_use_batching;       }
    T      const & get_envelope()      const { return this->m_envelope;           }
    size_t const & get_chunk_bytes()   const { return this->m_chunk_bytes;        }


  public:      
    
    void set_use_open_cl(bool const & value)        { this->m_use_open_cl    = value;   }
    void set_open_cl_platform(size_t const & value) { this->m_open_cl_platform = value; }
    void set_open_cl_device(size_t const & value)   { this->m_open_cl_device = value;   }
    void set_use_gproximity(bool const & value)     { this->m_use_gproximity = value;   }
    void set_use_batching(bool const & value)       { this->m_use_batching   = value;   }
    void set_envelope(T const & value)              { this->m_envelope       = value;   }
    void set_chunk_bytes(size_t const & value)      { this->m_chunk_bytes    = value;   }

  public:
    
    Params()
    : m_use_open_cl( false )
    , m_open_cl_platform( 0 )
    , m_open_cl_device( 0 )
    , m_use_gproximity( false )
    , m_use_batching( true )
    , m_envelope(VT::numeric_cast(0.01))
    , m_chunk_bytes(8000)
    {}
  };
  
} // namespace narrow

// NARROW_PARAMS_H
#endif 
