#include "iNVNt/core/logger.hpp"
#include "iNVNt/core/dag_graph.hpp"

#include <string>
#include <memory>

using namespace nvn;
int main(int argc, char **argv)
{
    auto log = Logger();
    log << "Testing graph/directed_acyclic.hpp\n";

    DirectedAcyclicGraph<std::string> tree;

    log << "Adding nodes to the tree->..\n";
    auto nodeA = tree.addNode(std::string("A"));
    auto nodeB = tree.addNode(std::string("B"), nodeA);
    auto nodeC = tree.addNode(std::string("C"), nodeA);
    auto nodeD = tree.addNode(std::string("D"), nodeB);
    auto nodeE = tree.addNode(std::string("E"), nodeB);
    auto nodeF = tree.addNode(std::string("F"), nodeC);

    log << "Tree structure:\n";
    for (index_t i = 0; i < tree.nodes.size(); ++i)
    {
        log << "Node " << i << ": " << *tree.nodes[i].get()
            << ", Parent: " << tree.node_parent[i]
            << ", Depth: " << tree.node_depth[i] << "\n";
    }

    log << "Checking if nodes exist...\n";
    log << "Node 0 exists: " << tree.hasNode(0) << "\n";
    log << "Node " << tree.nodes.size() << " exists: " << tree.hasNode(tree.nodes.size()) << " (should be false)\n";

    auto path_search = DAGPathSearch<std::string>(&tree);

    log << "Searching for path from node D to node F...\n";
    auto path = path_search(nodeD, nodeF);
    log << "Up: ";
    for (const auto up_node : path.up)
        log << *up_node << " ";
    log << "\nDown: ";
    for (const auto down_node : path.down)
        log << *down_node << " ";
    log << "\n-\n";

    log << "Searching for path from node D to node A...\n";
    auto path2 = path_search(nodeD, nodeA);
    log << "Up: ";
    for (const auto up_node : path2.up)
        log << *up_node << " ";
    log << "\nDown: ";
    for (const auto down_node : path2.down)
        log << *down_node << " ";
    log << "\n-\n";

    log << "Searching for path from node E to node E...\n";
    auto path3 = path_search(nodeE, nodeE);
    log << "Up: ";
    for (const auto up_node : path3.up)
        log << *up_node << " ";
    log << "\nDown: ";
    for (const auto down_node : path3.down)
        log << *down_node << " ";
    log << "\n-\n";

    log << "Testing completed successfully.\n";
    return 0;
}