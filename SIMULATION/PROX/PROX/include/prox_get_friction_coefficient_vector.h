#ifndef PROX_GET_FRICTION_COEFFICIENT_VECTOR_H
#define PROX_GET_FRICTION_COEFFICIENT_VECTOR_H

namespace prox
{
  
  template< typename contact_iterator, typename math_policy >
  inline void get_friction_coefficient_vector( 
                                                contact_iterator begin
                                              , contact_iterator end
                                              , std::vector< std::vector< Property< math_policy > > > const &  properties
                                              , typename math_policy::vector4_type & mu
                                              , math_policy const & /*tag*/
                                              , size_t const K
                                              ) 
  {
    typedef typename math_policy::block4x1_type block4x1_type;
    typedef typename math_policy::real_type     real_type;
    typedef typename math_policy::value_traits  value_traits;
    
    mu.resize(  K );
    
    size_t k = 0u;
    for(contact_iterator contact = begin;contact!=end;++contact, ++k) 
    {
      size_t const material_i = contact->get_body_i()->get_material_idx();
      size_t const material_j = contact->get_body_j()->get_material_idx();
      
      real_type const mu_s   = properties[material_i][material_j].get_friction_coefficients()(0);
      real_type const mu_t   = properties[material_i][material_j].get_friction_coefficients()(1);
      real_type const mu_tau = properties[material_i][material_j].get_friction_coefficients()(2);
      
      block4x1_type & b = mu( k );
      
      b(0) = value_traits::zero();
      b(1) = mu_s;
      b(2) = mu_t;
      b(3) = mu_tau;    
    }
  }
  
  
} // namespace prox
// PROX_GET_FRICTION_COEFFICIENT_VECTOR_H
#endif 
