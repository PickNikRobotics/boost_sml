#include <boost_sml/sml_transition_graph.h>
#include <boost_sml/example.h>

using StateMachine = boost::sml::sm<sml_example::StateMachineLogic>;

int main()
{
  SmlTransitionGraph<StateMachine> graph;
  graph.write_graphiz();
  std::cout << "----------------------------------------\n";
  graph.write_all_reachable_states(graph.get_vertex_index("idle"));
  std::cout << "----------------------------------------\n";
  graph.write_path_between_two_states(graph.get_vertex_index("idle"), graph.get_vertex_index("executing"));
  return 0;
}
