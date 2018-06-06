#ifndef MESH_ARRAY_COMPUTE_DISTANCE_MAP_H
#define MESH_ARRAY_COMPUTE_DISTANCE_MAP_H

#include <mesh_array_t4mesh.h>
#include <mesh_array_constants.h>
#include <mesh_array_adjacency_info_t4mesh.h>
#include <mesh_array_vertex_ring_t4mesh.h>
#include <mesh_array_vertex_attribute.h>
#include <mesh_array_tetrahedron_attribute_t4mesh.h>

#include <tiny_value_traits.h>

#include <cmath> // needed for std::sqrt

namespace mesh_array
{

  namespace details
  {

    template<typename T>
    bool compare_pairs( std::pair<size_t, T> const & a
                       , std::pair<size_t, T> const & b
                       )
    {
      return (a.second > b.second); //we want a min_heap
    }
    
    template<typename T>
    T distance( mesh_array::VertexAttribute<T,mesh_array::T4Mesh> const & X
               , mesh_array::VertexAttribute<T,mesh_array::T4Mesh> const & Y
               , mesh_array::VertexAttribute<T,mesh_array::T4Mesh> const & Z
               , size_t const & a
               , size_t const & b
               )
    {
      using std::sqrt;

      return sqrt(( X[a] - X[b] )*( X[a] - X[b] ) +
                  ( Y[a] - Y[b] )*( Y[a] - Y[b] ) +
                  ( Z[a] - Z[b] )*( Z[a] - Z[b] ) );
    }
    
  } // namespace details
  
  
  /**
   *
   * Phi is a per-vertex value, zero on the surface vertices and increase in
   * value as you move deeper inside the object.
   *
   */
  template<typename T>
  inline void compute_distance_map(
                                   T4Mesh const & mesh
                                   , VertexAttribute<T, T4Mesh> const & X
                                   , VertexAttribute<T, T4Mesh> const & Y
                                   , VertexAttribute<T, T4Mesh> const & Z
                                   , VertexAttribute<T, T4Mesh>  & phi
                                   )
  {
    using std::min;

    typedef typename tiny::ValueTraits<T> VT;
    
    VertexRing<T4Mesh>    ring(mesh);
    AdjacencyInfo<T4Mesh> A(ring);
    
    t4_vertex_bool_attribute      on_heap(mesh);
    t4_vertex_bool_attribute      seen(mesh);
    t4_tetrahedron_bool_attribute queued(mesh);
    
    phi.bind(mesh);
    
    for (size_t i = 0u; i < mesh.vertex_size(); ++i)
    {
      phi(     mesh.vertex(i)) = VT::infinity();
      seen(    mesh.vertex(i)) = false;
      on_heap( mesh.vertex(i)) = false;
    }
    
    std::vector<std::pair<size_t, T> > h;
    std::make_heap(h.begin(), h.end(), details::compare_pairs<T>);
    
    // initializing
    // mark all boundary vertices as seen and set phi = 0
    for ( size_t idx = 0u; idx < mesh.tetrahedron_size(); ++idx)
    {
      size_t const idx_adj_i  = A.i(  idx ) ;
      size_t const idx_adj_j  = A.j(  idx ) ;
      size_t const idx_adj_k  = A.k(  idx ) ;
      size_t const idx_adj_m  = A.m(  idx ) ;
      
      Tetrahedron const tet = mesh.tetrahedron( idx);
      
      size_t const v_i  = tet.i();
      size_t const v_j  = tet.j();
      size_t const v_k  = tet.k();
      size_t const v_m  = tet.m();
      
      // no tetrathedron opposite vertex i so j,k,m are on the boundary
      if ( idx_adj_i == UNASSIGNED() )
      {
        phi(v_j) = VT::zero();
        phi(v_k) = VT::zero();
        phi(v_m) = VT::zero();
        seen(v_j) = true;
        seen(v_k) = true;
        seen(v_m) = true;
        
        if (!seen(v_i) && !on_heap(v_i))
        {
          h.push_back(std::make_pair(v_i, phi(v_i)));
          std::push_heap(h.begin(), h.end());
          on_heap(v_i) = true;
        }
        
      }
      
      // no tetrathedron opposite vertex i so j,k,m are on the boundary
      if ( idx_adj_j == UNASSIGNED() )
      {
        phi(v_i) = VT::zero();
        phi(v_k) = VT::zero();
        phi(v_m) = VT::zero();
        seen(v_i) = true;
        seen(v_k) = true;
        seen(v_m) = true;
        
        if (!seen(v_j) && !on_heap(v_j))
        {
          h.push_back(std::make_pair(v_j, phi(v_j)));
          std::push_heap(h.begin(), h.end());
          on_heap(v_j) = true;
        }
      }
      
      // no tetrathedron opposite vertex i so j,k,m are on the boundary
      if ( idx_adj_k == UNASSIGNED() )
      {
        phi(v_i) = VT::zero();
        phi(v_j) = VT::zero();
        phi(v_m) = VT::zero();
        seen(v_i) = true;
        seen(v_j) = true;
        seen(v_m) = true;
        
        if (!seen(v_k) && !on_heap(v_k))
        {
          h.push_back(std::make_pair(v_k, phi(v_k)));
          std::push_heap(h.begin(), h.end());
          on_heap(v_k) = true;
        }
      }
      
      // no tetrathedron opposite vertex i so j,k,m are on the boundary
      if ( idx_adj_m == UNASSIGNED() )
      {
        phi(v_i) = VT::zero();
        phi(v_j) = VT::zero();
        phi(v_k) = VT::zero();
        seen(v_i) = true;
        seen(v_j) = true;
        seen(v_k) = true;
        
        if (!seen(v_m) && !on_heap(v_m))
        {
          h.push_back(std::make_pair(v_m, phi(v_m)));
          std::push_heap(h.begin(), h.end());
          on_heap(v_m) = true;
        }
      }
    }
    
    // still initializing
    for ( size_t i = 0u; i < h.size(); ++i)
    {
      size_t const v = h[i].first;
      
      for (size_t n = ring.offset()[v]; n < ring.offset()[v+1]; ++n)
      {
        Tetrahedron const tet = mesh.tetrahedron(ring.V2T()[n].second);
        
        size_t idx = tet.i();
        if (v != idx && seen(idx))
        {
          phi(v) = min(phi(v), phi(idx) + details::distance(X, Y, Z, v, idx));
        }
        
        idx = tet.j();
        if (v != idx && seen(idx))
        {
          phi(v) = min(phi(v), phi(idx) + details::distance(X, Y, Z, v, idx));
        }
        
        idx = tet.k();
        if (v != idx && seen(idx))
        {
          phi(v) = min(phi(v), phi(idx) + details::distance(X, Y, Z, v, idx));
        }
        
        idx = tet.m();
        if (v != idx && seen(idx))
        {
          phi(v) = min(phi(v), phi(idx) + details::distance(X, Y, Z, v, idx));
        }
      }
    }
    
    // real action here
    while ( h.begin()!=h.end() )
    {
      //vertex on heap with smallest phi
      size_t const v = h.front().first;
      
      //take it off heap
      std::pop_heap(h.begin(), h.end(), details::compare_pairs<T>);
      h.pop_back();
      
      //mark as seen
      seen(    v) = true;
      on_heap( v) = false;
      
      //look up all neighbors
      for (size_t n = ring.offset()[v]; n < ring.offset()[v+1]; ++n)
      {
        Tetrahedron const tet = mesh.tetrahedron(ring.V2T()[n].second);
        
        //add !seen neighbors to heap
        size_t idx = tet.i();
        if (v != idx && !seen(idx))
        {
          phi(idx) = min(phi(idx), phi(v) + details::distance(X, Y, Z, v, idx));
          if (!on_heap(idx))
          {
            h.push_back(std::make_pair(idx, phi(idx)));
            std::push_heap(h.begin(), h.end());
            on_heap(idx) = true;
          }
        }
        
        idx = tet.j();
        if (v != idx && !seen(idx))
        {
          phi(idx) = min(phi(idx), phi(v) + details::distance(X, Y, Z, v, idx));
          if (!on_heap(idx))
          {
            h.push_back(std::make_pair(idx, phi(idx)));
            std::push_heap(h.begin(), h.end());
            on_heap(idx) = true;
          }
        }
        
        idx = tet.k();
        if (v != idx && !seen(idx))
        {
          phi(idx) = min(phi(idx), phi(v) + details::distance(X, Y, Z, v, idx));
          if (!on_heap(idx))
          {
            h.push_back(std::make_pair(idx, phi(idx)));
            std::push_heap(h.begin(), h.end());
            on_heap(idx) = true;
          }
        }
        
        idx = tet.m();
        if (v != idx && !seen(idx))
        {
          phi(idx) = min(phi(idx), phi(v) + details::distance(X, Y, Z, v, idx));
          if (!on_heap(idx))
          {
            h.push_back(std::make_pair(idx, phi(idx)));
            std::push_heap(h.begin(), h.end());
            on_heap(idx) = true;
          }
        }
      }
    }
    
  }
  
}// namespace kdop

// MESH_ARRAY_COMPUTE_DISTANCE_MAP_H
#endif