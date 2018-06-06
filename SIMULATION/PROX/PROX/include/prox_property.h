#ifndef PROX_PROPERTY_H
#define PROX_PROPERTY_H

#include <cstdlib>  // for std::size_t

namespace prox
{
  
  template< typename math_policy >
  class Property 
  {
  public:
    
    typedef typename math_policy::real_type       real_type;
    typedef typename math_policy::vector3_type    vector3_type;
    typedef typename math_policy::value_traits    value_traits;
    
  protected:
    
    vector3_type     m_mu;
    real_type        m_e;
    
    //given in local frame of the master matrial body
    vector3_type     m_s;  
    size_t           m_master_material_idx;
    
  public:
    
    Property()
    : m_mu(vector3_type::make(value_traits::half(),value_traits::half(),value_traits::half()))
    , m_e(value_traits::zero())
    , m_s(vector3_type::make(value_traits::one(),value_traits::zero(),value_traits::zero()))
    , m_master_material_idx(0u)
    {}
    
    real_type    const & get_restitution_coefficient()    const {  return this->m_e; }
    
    void set_restitution_coefficient(real_type const & e)       {  this->m_e = e; }
    
    vector3_type const & get_friction_coefficients() const {  return this->m_mu; }
    
    void get_friction_coefficients(real_type& mux, real_type& muy, real_type& muz) const 
    {
      mux = this->m_mu[0];
      muy = this->m_mu[1];
      muz = this->m_mu[2];
    }
    
    void set_friction_coefficients(vector3_type const & mu) {  this->m_mu = mu; }
    
    void set_friction_coefficients(real_type const & mux, real_type const & muy, real_type const & muz) 
    { 
      this->m_mu[0] = mux; 
      this->m_mu[1] = muy; 
      this->m_mu[2] = muz; 
    }
    
    size_t       const & get_master_material_idx() const {  return this->m_master_material_idx; }
    
    void set_master_material_idx(size_t const & i)       {  this->m_master_material_idx = i; }
    
    vector3_type const & get_s_vector() const {  return this->m_s; }
    
    void get_s_vector(real_type& sx, real_type& sy, real_type& sz) const 
    {
      sx = this->m_s[0];
      sy = this->m_s[1];
      sz = this->m_s[2];
    }
    
    void set_s_vector(vector3_type const & s_) {  this->m_s = s_; }
    
    void set_s_vector(real_type const & sx, real_type const & sy, real_type const & sz) 
    { 
      this->m_s[0] = sx; 
      this->m_s[1] = sy; 
      this->m_s[2] = sz; 
    }
    
    bool is_isotropic() const
    {
      if (m_mu(0) != m_mu(1))
        return false;
      return true;
    }
    
  };
} // namespace prox

// PROX_PROPERTY_H
#endif 
