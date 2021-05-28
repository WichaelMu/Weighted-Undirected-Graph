#include "graph.hpp"

int disregard() {

    Graph<int> g;

    //  Original tests.

    //g.add_vertex("A");
    //g.add_vertex("B");
    //g.add_vertex("C");
    //g.add_vertex("D");
    //g.add_vertex("E");
    //g.add_vertex("F");

    //g.add_edge("A", "B", 7);
    //g.add_edge("A", "C", 2);
    //g.add_edge("C", "D", 4);
    //g.add_edge("C", "E", 8);
    //g.add_edge("B", "E", 10);
    //g.add_edge("A", "E", 6);
    //g.add_edge("B", "C", 3);
    //g.add_edge("B", "F", 5);
    //g.add_edge("E", "F", 10);

    //g.remove_edge("B", "C");
    //g.remove_vertex("F");

    //  Modified tests.

    g.add_vertex("A");
    g.add_vertex("B");
    g.add_vertex("C");
    g.add_vertex("D");
    g.add_vertex("E");
    g.add_vertex("F");
    g.add_vertex("G");
    g.add_vertex("H");
    g.add_vertex("I");
    g.add_vertex("J");
    g.add_vertex("K");
    g.add_vertex("L");
    g.add_vertex("M");
    g.add_vertex("N");

    g.add_edge("A", "B", 2);
    g.add_edge("B", "C", 3);
    g.add_edge("B", "D", 1);
    g.add_edge("C", "D", 1);
    g.add_edge("D", "E", 5);
    g.add_edge("D", "F", 7);
    g.add_edge("D", "G", 6);
    g.add_edge("E", "G", 3);
    g.add_edge("F", "G", 4);
    g.add_edge("D", "H", 9);
    g.add_edge("H", "I", 8);
    g.add_edge("I", "J", 9);
    g.add_edge("F", "A", 2);
    g.add_edge("F", "K", 7);
    g.add_edge("A", "L", 10);
    g.add_edge("B", "M", 6);
    g.add_edge("C", "N", 2);
    g.add_edge("M", "N", 9);
    g.add_edge("H", "N", 4);

    cout << "Number of vertices: " << g.num_vertices() << endl; // should be 5
    cout << "Number of edges: " << g.num_edges() << endl; // should be 6

    cout << "Is vertex A in the graph? " << g.contains("A") << endl; // should be 1 or true
    cout << "Is vertex F in the graph? " << g.contains("F") << endl; // should be 0 or false

    cout << "Is there an edge between A and B? " << g.adjacent("A", "B") << endl; // should be 1 or true
    cout << "Is there an edge between B and C? " << g.adjacent("B", "C") << endl; // should be 0 or false

    cout << "Degree of D: " << g.degree("D") << endl; // should be 1

    cout << "The visiting order of DFS (starting from B):";
    for (string x : g.depth_first_traversal("B")) {
        cout << " " << x;
    }
    cout << endl;

    cout << "The visiting order of BFS (starting from B):";
    for (string x : g.breadth_first_traversal("B")) {
        cout << " " << x;
    }
    cout << endl;

    cout << "Edges in the graph: \n";
    vector<pair<string, string>> edges = g.get_edges();
    for (int i = 0; i < edges.size(); ++i) {
        cout << edges[i].first << ' ' << edges[i].second << "\n";
    }

    cout << "Is there a cycle? " << g.contain_cycles() << "\n";

    cout << "Minimum spanning tree: \n";
    Graph<int> m = g.minimum_spanning_tree();
    vector<pair<string, string>> mst = m.get_edges();
    cout << "Number of edges: " << m.num_edges() << "\n";
    for (int i = 0; i < mst.size(); ++i) {
        cout << i+1 << ' ' << mst[i].first << ' ' << mst[i].second << "\n";
    }

    return 0;
}
