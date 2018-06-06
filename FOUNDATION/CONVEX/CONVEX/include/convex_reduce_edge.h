#ifndef CONVEX_REDUCE_EDGE_H
#define CONVEX_REDUCE_EDGE_H

#include <convex_outside_vertex_edge_voronoi_plane.h>

#include <barycentric/geometry_barycentric.h> 
#include <tiny_vector_functions.h>


namespace convex
{
  
  /**
   * Reduce Edge S.
   * This function implements the case where a simplex
   * is an edge. The function will compute the closest
   * point to p on the simplex and try to reduce the simplex
   * to the lowest dimensional face on the boundary of the
   * simplex containing the closest point.
   *
   * @param p         The test point.
   * @param simplex   Initially this argument holds the edge simplex. Upon
   *                  return the argument holds the reduced simplex.
   */
  template< typename V >
  inline void reduce_edge( V const & p_in, Simplex<V> & S)
  {
    typedef typename V::real_type     T;
    typedef typename V::value_traits  VT;
    
    int bit_A = 0;
    int bit_B = 0;
    size_t idx_A = 0u;
    size_t idx_B = 0u;
    get_used_indices( S.m_bitmask, idx_A, bit_A, idx_B, bit_B );
    
    T const scale = tiny::norm(S.m_v[idx_A]) > tiny::norm(S.m_v[idx_B]) ? tiny::norm(S.m_v[idx_A]) : tiny::norm(S.m_v[idx_B]);
    
    V const & A = S.m_v[idx_A]/scale;// scale so that A lies within [0;1] on all three axis
    V const & B = S.m_v[idx_B]/scale;// scale so that B lies within [0;1] on all three axis
    V const & p = p_in/scale;
    
    bool const outside_AB = outside_vertex_edge_voronoi_plane( p, A, B );
    bool const outside_BA = outside_vertex_edge_voronoi_plane( p, B, A );
    
    if(outside_AB)
    {
      // Simplex is A
      S.m_bitmask = bit_A;
      S.m_v[idx_B].clear();
      S.m_a[idx_B].clear();
      S.m_b[idx_B].clear();
      S.m_w[idx_A] = VT::one();
      S.m_w[idx_B] = VT::zero();
      return;
    }
    if(outside_BA)
    {
      // Simplex is B
      S.m_bitmask = bit_B;
      S.m_v[idx_A].clear();
      S.m_a[idx_A].clear();
      S.m_b[idx_A].clear();
      S.m_w[idx_A] = VT::zero();
      S.m_w[idx_B] = VT::one();
      return;
    }
    if(!outside_AB && !outside_BA)
    {
      // because barycentric coords are invariant to uniform scaling, we don't need to rescale A and B
      geometry::barycentric(A,B,p,S.m_w[idx_A],S.m_w[idx_B]);
      return;
    }
    
  }

} // namespace convex

// CONVEX_REDUCE_EDGE_H
#endif
