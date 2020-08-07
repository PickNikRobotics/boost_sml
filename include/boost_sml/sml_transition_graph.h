/*
 * Desc: Class to generate SML diagrams using boost.Graph library
 */

#pragma once

#include <string>
// [boost].SML
#include <boost_sml/sml.hpp>
// Boost.Graph
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/breadth_first_search.hpp>


namespace sml_transition_graph {

// A bundle for the state's name see https://www.boost.org/doc/libs/1_49_0/libs/graph/doc/bundles.html
struct State
{
  std::string name;
};
using graph_t = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, State>;

class SmlTransitionGraph: public graph_t
{
public:
  using edge_t = std::pair<std::string, std::string>;
  const vertex_descriptor NIL = std::numeric_limits<vertex_descriptor>::max();

  template <class SM>
  SmlTransitionGraph(const SM& sm)
  {
    construct_graph(sm);
  }
  void write_graphiz(std::ostream& out = std::cout) const
  {
    boost::write_graphviz(out, *this, boost::make_label_writer(boost::get(&State::name, *this)));
  }

  void write_all_reachable_states(const vertex_descriptor& start_vertex, std::ostream& out = std::cout)
  {
    const auto predecessors = find_predecessors(start_vertex);
    write_graphviz_predecessors(out, predecessors);
  }

  // TODO: write all paths https://github.com/networkx/networkx/blob/master/networkx/algorithms/simple_paths.py
  void write_path_between_two_states(const vertex_descriptor& start_vertex,
                                     const vertex_descriptor& goal_vertex, std::ostream& out = std::cout)
  {
    auto path = find_path(start_vertex, goal_vertex);
    write_graphviz_path(out, path);
  }

  std::vector<vertex_descriptor> find_path(const vertex_descriptor& start_vertex,
                                                                const vertex_descriptor& goal_vertex)
  {
    if(start_vertex == goal_vertex)
      return {};

    const auto predecessors = find_predecessors(start_vertex);

    std::vector<vertex_descriptor> path = {goal_vertex};
    for (auto current_vertex = predecessors[goal_vertex]; current_vertex != NIL; current_vertex = predecessors[current_vertex])
      path.push_back(current_vertex);

    return path;
  }

  std::vector<vertex_descriptor> find_predecessors(const vertex_descriptor& start_vertex)
  {
    std::vector<vertex_descriptor> predecessors(boost::num_vertices(*this), NIL);
    boost::breadth_first_search(
        *this, start_vertex,
        boost::visitor(boost::make_bfs_visitor(boost::record_predecessors(&predecessors[0], boost::on_tree_edge()))));
    return predecessors;
  }

  vertex_descriptor get_vertex_index(const std::string& vertex_name) const
  {
      return vertex_name_to_descriptor_map_.at(vertex_name);
  }
  std::string get_vertex_name(const vertex_descriptor& vertex_index) const
  {
    return (*this)[vertex_index].name;
  }

private:
  template <class T>
  void construct_edge(std::vector<edge_t>& edges) noexcept
  {
    auto src_state = std::string{ boost::sml::aux::string<typename T::src_state>{}.c_str() };
    auto dst_state = std::string{ boost::sml::aux::string<typename T::dst_state>{}.c_str() };
    if (T::initial)
      edges.push_back({ "sml_entry_point", src_state });
    // Check if dst_state starts with prefix
    if (dst_state.rfind("boost::sml::", 0) == 0)
      return;
    edges.push_back({ src_state, dst_state });
  }

  template <template <class...> class T, class... Ts>
  void construct_edges_impl(const T<Ts...>&, std::vector<edge_t>& edges) noexcept
  {
    int _[]{ 0, (construct_edge<Ts>(edges), 0)... };
    (void)_;
  }

  template <class SM>
  std::vector<edge_t> construct_edges(const SM& /* sm */) noexcept
  {
    std::vector<edge_t> edges;
    construct_edges_impl(typename SM::transitions{}, edges);
    return edges;
  }

  template <class SM>
  void construct_graph(const SM& sm) noexcept
  {
    auto edges = construct_edges(sm);
    // Create unique vertices
    std::unordered_set<std::string> vertices;
    for (const auto& edge : edges)
      vertices.insert({ edge.first, edge.second });
    for (const auto& vertex : vertices)
    {
      auto vertex_descriptor = boost::add_vertex(*this);
      (*this)[vertex_descriptor].name = vertex;
      vertex_name_to_descriptor_map_.insert({ vertex, vertex_descriptor });
    }
    for (const auto& edge : edges)
      boost::add_edge(vertex_name_to_descriptor_map_[edge.first], vertex_name_to_descriptor_map_[edge.second], *this);
  }

  void write_graphviz_predecessors(std::ostream& out, const std::vector<vertex_descriptor>& predecessors)
  {
    out << "digraph G {\n";
    edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = edges(*this); ei != ei_end; ++ei)
    {
      edge_descriptor e = *ei;
      vertex_descriptor u = boost::source(e, *this), v = boost::target(e, *this);
      out << (*this)[u].name << " -> " << (*this)[v].name << "[label=\" \"";
      if (predecessors[v] == u)
        out << ", color=\"black\"";
      else
        out << ", color=\"grey\"";
      out << "];\n";
    }
    out << "}\n";
  }

  void write_graphviz_path(std::ostream& out, const std::vector<vertex_descriptor>& path)
  {
    out << "digraph G {\n";
    edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = edges(*this); ei != ei_end; ++ei)
    {
      edge_descriptor e = *ei;
      vertex_descriptor u = boost::source(e, *this), v = boost::target(e, *this);
      out << (*this)[u].name << " -> " << (*this)[v].name << "[label=\" \"";
      if (std::find(path.cbegin(), path.cend(), u) != path.end() &&
          std::find(path.cbegin(), path.cend(), v) != path.end())
        out << ", color=\"black\"";
      else
        out << ", color=\"grey\"";
      out << "];\n";
    }
    out << "}\n";
  }


  std::unordered_map<std::string, vertex_descriptor> vertex_name_to_descriptor_map_;
};
} // namespace sml_transition_graph
