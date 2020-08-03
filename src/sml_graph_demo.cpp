#include <boost_sml/sml_transition_graph.h>
#include <boost_sml/example.h>

using StateMachine = boost::sml::sm<sml_example::StateMachineLogic>;

int main()
{
  StateMachine sml;
  sml_transition_graph::SmlTransitionGraph graph(sml);

  const std::string output_filename_dot = "/tmp/sml_transition_diagram.dot";
  const std::string output_filename_png = "/tmp/sml_transition_diagram.png";
  std::ofstream dot_file(output_filename_dot);
  std::cout << "Saving sml tranistion diagram to: `" << output_filename_dot << "`\n";
  graph.write_graphiz(dot_file);
  std::cout << "You can visualize it by running `dot -Tpng " << output_filename_dot << " > " << output_filename_png
            << " && xdg-open " << output_filename_png << "`\n";

  std::cout << "----------------------------------------\n";
  graph.write_graphiz();

  std::cout << "----------------------------------------\n";
  graph.write_all_reachable_states(graph.get_vertex_index("idle"));

  std::cout << "----------------------------------------\n";
  graph.write_path_between_two_states(graph.get_vertex_index("idle"), graph.get_vertex_index("executing"));

  std::cout << "----------------------------------------\n";
  auto path_1 = graph.find_path(graph.get_vertex_index("idle"), graph.get_vertex_index("executing"));
  std::reverse(path_1.begin(), path_1.end());
  for (size_t i = 0; i < path_1.size() - 1; i++)
    std::cout << graph.get_vertex_name(path_1.at(i)) << " -> ";
  std::cout << graph.get_vertex_name(path_1.back()) << "\n";

  std::cout << "----------------------------------------\n";
  auto path_2 = graph.find_path(graph.get_vertex_index("executing"), graph.get_vertex_index("idle"));
  if (path_2.size() == 1)
    std::cout << "There's no path from 'executing' to 'idle'\n";
}
