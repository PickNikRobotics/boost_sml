/*
 * Desc: Class to generate SML diagrams using boost.Graph library
 */

#pragma once

#include <string>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost_sml/sml.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/breadth_first_search.hpp>

template <class SM>
class SmlTransitionGraph
{
public:
  // A bundle for the state's name see https://www.boost.org/doc/libs/1_49_0/libs/graph/doc/bundles.html
  struct State
  {
    std::string name;
  };
  using graph_t = boost::adjacency_list<boost::vecS, boost::vecS, boost::directedS, State>;
  using edge_t = std::pair<std::string, std::string>;
  using edge_iterator_t = typename boost::graph_traits<graph_t>::edge_iterator;
  using edge_descriptor_t = typename boost::graph_traits<graph_t>::edge_descriptor;
  using vertex_descriptor_t = typename boost::graph_traits<graph_t>::vertex_descriptor;
  static constexpr vertex_descriptor_t NIL = std::numeric_limits<vertex_descriptor_t>::max();

  SmlTransitionGraph()
  {
    construct_graph();
  }
  void write_graphiz(std::ostream& out = std::cout) const
  {
    boost::write_graphviz(out, graph_, boost::make_label_writer(boost::get(&State::name, graph_)));
  }

  void write_all_reachable_states(const vertex_descriptor_t& start_vertex, std::ostream& out = std::cout)
  {
    const auto predecessors = find_predecessors(start_vertex);
    write_graphviz_predecessors(out, predecessors);
  }

  // TODO: write all paths https://github.com/networkx/networkx/blob/master/networkx/algorithms/simple_paths.py
  void write_path_between_two_states(const vertex_descriptor_t& start_vertex,
                                     const vertex_descriptor_t& goal_vertex, std::ostream& out = std::cout)
  {
    auto path = find_path(start_vertex, goal_vertex);
    write_graphviz_path(out, path);
  }

  std::vector<vertex_descriptor_t> find_path(const vertex_descriptor_t& start_vertex,
                                                                const vertex_descriptor_t& goal_vertex)
  {
    if(start_vertex == goal_vertex)
      return {};

    const auto predecessors = find_predecessors(start_vertex);

    std::vector<vertex_descriptor_t> path = {goal_vertex};
    for (auto current_vertex = predecessors[goal_vertex]; current_vertex != NIL; current_vertex = predecessors[current_vertex])
      path.push_back(current_vertex);

    return std::move(path);
  }

  std::vector<vertex_descriptor_t> find_predecessors(const vertex_descriptor_t& start_vertex)
  {
    std::vector<vertex_descriptor_t> predecessors(boost::num_vertices(graph_),NIL  );
    boost::breadth_first_search(
        graph_, start_vertex,
        boost::visitor(boost::make_bfs_visitor(boost::record_predecessors(&predecessors[0], boost::on_tree_edge()))));
    return std::move(predecessors);
  }

  vertex_descriptor_t get_vertex_index(const std::string& vertex_name) const
  {
      return vertex_name_to_descriptor_map_.at(vertex_name);
  }
  std::string get_vertex_name(const vertex_descriptor_t& vertex_index) const
  {
    return graph_[vertex_index].name;
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

  std::vector<edge_t> construct_edges() noexcept
  {
    std::vector<edge_t> edges;
    construct_edges_impl(typename SM::transitions{}, edges);
    return edges;
  }

  void construct_graph() noexcept
  {
    auto edges = construct_edges();
    // Create unique vertices
    std::unordered_set<std::string> vertices;
    for (const auto& edge : edges)
      vertices.insert({ edge.first, edge.second });
    for (const auto& vertex : vertices)
    {
      auto vertex_descriptor = boost::add_vertex(graph_);
      graph_[vertex_descriptor].name = vertex;
      vertex_name_to_descriptor_map_.insert({ vertex, vertex_descriptor });
    }
    for (const auto& edge : edges)
      boost::add_edge(vertex_name_to_descriptor_map_[edge.first], vertex_name_to_descriptor_map_[edge.second], graph_);
  }

  void write_graphviz_predecessors(std::ostream& out, const std::vector<vertex_descriptor_t>& predecessors)
  {
    out << "digraph G {\n";
    edge_iterator_t ei, ei_end;
    for (boost::tie(ei, ei_end) = edges(graph_); ei != ei_end; ++ei)
    {
      edge_descriptor_t e = *ei;
      vertex_descriptor_t u = boost::source(e, graph_), v = boost::target(e, graph_);
      out << graph_[u].name << " -> " << graph_[v].name << "[label=\" \"";
      if (predecessors[v] == u)
        out << ", color=\"black\"";
      else
        out << ", color=\"grey\"";
      out << "];\n";
    }
    out << "}\n";
  }

  void write_graphviz_path(std::ostream& out, const std::vector<vertex_descriptor_t>& path)
  {
    out << "digraph G {\n";
    edge_iterator_t ei, ei_end;
    for (boost::tie(ei, ei_end) = edges(graph_); ei != ei_end; ++ei)
    {
      edge_descriptor_t e = *ei;
      vertex_descriptor_t u = boost::source(e, graph_), v = boost::target(e, graph_);
      out << graph_[u].name << " -> " << graph_[v].name << "[label=\" \"";
      if (std::find(path.cbegin(), path.cend(), u) != path.end() &&
          std::find(path.cbegin(), path.cend(), v) != path.end())
        out << ", color=\"black\"";
      else
        out << ", color=\"grey\"";
      out << "];\n";
    }
    out << "}\n";
  }


  graph_t graph_;
  std::unordered_map<std::string, vertex_descriptor_t> vertex_name_to_descriptor_map_;
};
