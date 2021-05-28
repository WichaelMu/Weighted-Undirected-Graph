#include "stdc++.h"
#include "node.hpp"
#include "visited.hpp"

using namespace std;

template <typename T> // the template allows the weight of an edge to take any numeric data type (denoted by T).
class Graph {

public:

	/* define your data structure to represent a weighted undirected graph */
	vector<Node<T>> v;
	size_t edge_count = 0;

	/* test1 */
	Graph(); // the contructor function.
	~Graph(); // the destructor function.
	size_t num_vertices(); // returns the total number of vertices in the graph.
	size_t num_edges(); // returns the total number of edges in the graph.

	/* test2 */
	void add_vertex(const string&); // adds a vertex to the graph -- every vertex uses a string as its unique identifier.
	bool contains(const string&); // checks if a vertex is in the graph -- returns true if the graph contains the given vertex; otherwise, returns false.

	/* test3 */
	vector<string> get_vertices(); // returns a vector of all the vertices in the graph.

	/* test4 */
	void add_edge(const string&, const string&, const T&); // adds a weighted edge to the graph -- the two strings represent the incident vertices; the third parameter represents the edge's weight.
	bool adjacent(const string&, const string&); // check if there is an edge between the two vertices in the graph -- returns true if the edge exists; otherwise, returns false.

	/* test5 */
	vector<pair<string, string>> get_edges(); // returns a vector of all the edges in the graph -- each edge is represented by a pair of vertices incident to the edge.

	/* test6 */
	vector<string> get_neighbours(const string&); // returns a vector of all the vertices, each of which is directly connected with the given vertex by an edge.
	size_t degree(const string&); // returns the degree of a vertex.

	/* test7 */
	void remove_edge(const string&, const string&); // removes the edge between two vertices, if it exists.

	/* test8 */
	void remove_vertex(const string&); // delete the given vertex from the graph -- note that, all incident edges of the vertex should be deleted as well.

	/* test9 */
	vector<string> depth_first_traversal(const string&); // returns a vector of all the vertices in the visiting order of a depth-first traversal from the given vertex.

	/* test10 */
	vector<string> breadth_first_traversal(const string&); // returns a vector of all the vertices in the visiting order of a breadth-first traversal from the given vertex.

	/* test11 */
	bool contain_cycles(); // check if the graph contains any cycle -- returns true if there exists a path from a vertex to itself; otherwise, return false.

	/* test12 */
	Graph<T> minimum_spanning_tree(); // returns a spanning tree of the graph -- the returned tree is preferably a minimum spanning tree.

private:
	//  Custom methods and functions.
	Node<T>& find_node(const string&);			//	Returns the corresponding Node<T> of u.
	Node<T>& find_node(const string&, int, int);		//	Recursively finds Node<T> of u in O(log n).
	void connect_vertices(Node<T>&, Node<T>&, const T&);	//	Connects Node<T> l and Node<T> r with T weight.
	T find_edge_weight(const string&, const string&);	//	Returns T weight between Node<T> u and Node<T> v.
	bool is_connected();					//	If this Graph<T> is connected.
};

template <typename T>
Node<T>& Graph<T>::find_node(const string& u, int left_pos, int right_pos) {		//	A binary search on every vertex to find Node u.
	int mid_pos = (left_pos + right_pos) * .5;					//	Returns the default Node<T> if it can't be found.

	if (this->v[mid_pos] == u) {
		return this->v.at(mid_pos);
	}

	if (this->v[mid_pos] < u) {
		return find_node(u, mid_pos, right_pos);
	} else if (this->v[mid_pos] > u) {
		return find_node(u, left_pos, mid_pos);
	}

	return Node<T>::default_node;
}

template <typename T>
Node<T>& Graph<T>::find_node(const string& u) {
	return find_node(u, 0, this->num_vertices());
}

template <typename T>
T Graph<T>::find_edge_weight(const string& u, const string& v) {
	Node<T> node_u = find_node(u);
	for (auto& adj_node : node_u.v_adj) {			//	Finds the corresponding adjacent Node<T> v.
		if (adj_node.first == v) {			//	Returns the T weight connecting
			return adj_node.second;			//	Node<T> u and Node<T> v.
		}
	}

	return -1;
}

template <typename T>
void Graph<T>::connect_vertices(Node<T>& left_node, Node<T>& right_node, const T& weight_t) {
	if (!left_node.is_adj(right_node.node) && !right_node.is_adj(left_node.node)) {		//	Connects Node<T> lhs and Node<T> rhs
		left_node.v_adj.push_back(make_pair(right_node, weight_t));			//	if they are not already connected.
		right_node.v_adj.push_back(make_pair(left_node, weight_t));			//
		this->edge_count++;								//
	}
}

template <typename T>
bool Graph<T>::is_connected() {
	vector<string> v_DFS = depth_first_traversal(this->v[0].node);	//	A Graph<T> is connected if a DFT
									//	is the same size as the total number
	return v_DFS.size() == this->num_vertices();			//	of vertices.
}

/* test1 */

template <typename T>
Graph<T>::Graph() {
	this->v = vector<Node<T>>();
}

template <typename T>
Graph<T>::~Graph() {}

template <typename T>
size_t Graph<T>::num_vertices() {
	return this->v.size();
}

template <typename T>
size_t Graph<T>::num_edges() {
	return this->edge_count;
}

/* test2 */

template <typename T>
void Graph<T>::add_vertex(const string& u) {
	if (!contains(u)) {
		Node<T> node_u;
		node_u.node = u;

		this->v.push_back(node_u);

		sort(this->v.begin(), this->v.end());
	}
}

template <typename T>
bool Graph<T>::contains(const string& u) {
	for (auto& node : this->v) {                       //
		if (node == u) {				//
			return true;                    //
		}
	}

	return false;
}

/* test3 */

template <typename T>
vector<string> Graph<T>::get_vertices() {
	vector<string> vertex_node;

	for (auto& n : this->v) {                       //
		vertex_node.push_back(n.node);            //
	}

	return vertex_node;
}

/* test4 */

template <typename T>
void Graph<T>::add_edge(const string& u, const string& v, const T& weight) {
	connect_vertices(find_node(u), find_node(v), weight);
}

template <typename T>
bool Graph<T>::adjacent(const string& u, const string& v) {
	return find_node(u).is_adj(v);;
}

/* test5 */

template <typename T>
vector<pair<string, string>> Graph<T>::get_edges() {
	vector<pair<string, string>> v_edges;
	visited<string> v_visited;

	for (int i = 0; i < this->num_vertices(); ++i) {                                			//  Goes through all Nodes and their adjacent Nodes.
		for (auto& adj_node : this->v[i].v_adj) {                                 			//
			if (!v_visited.is_visited(adj_node.first.node)) {					//  Adds this Node to the vt<p<s, s>> answer
				v_edges.push_back(make_pair(this->v[i].node, adj_node.first.node));		//  if it is not, yet, connected.
			}
		}

		v_visited.visit(this->v[i].node);
	}

	return v_edges;
}

/* test6 */

template <typename T>
vector<string> Graph<T>::get_neighbours(const string& u) {
	vector<string> v_neighbours;

	for (auto& adj_node : find_node(u).v_adj) {		//
		v_neighbours.push_back(adj_node.first.node);		//
	}

	return v_neighbours;
}

template <typename T>
size_t Graph<T>::degree(const string& u) {
	return find_node(u).v_adj.size();
}

/* test7 */

template <typename T>
void Graph<T>::remove_edge(const string& u, const string& v) {
	int removed_edges = 0;

	for (int i = 0; i < this->num_vertices() && removed_edges != 2; ++i) {                            	//	Need to remove two edges in an undirected Graph<T>.

		if (this->v[i] == u || this->v[i] == v) {							//	Look for either Node<T> in undirected Graph<T>.

			for (int k = 0; k < this->degree(this->v[i].node); ++k) {               		//	Once found, go through that Node<T>'s adjaceny.

				if (this->v[i].v_adj[k].first == v || this->v[i].v_adj[k].first == u) {     	//	If we found the other Node<T>
					this->v[i].v_adj.erase(this->v[i].v_adj.begin() + k);       		//	Remove it.
					removed_edges++;                                           		//
					break;                                                  		//	Exit and find the other Node<T>.x
				}
			}
		}
	}

	this->edge_count--;
}

/* test8 */

template <typename T>
void Graph<T>::remove_vertex(const string& u) {
	for (int i = 0; i < this->num_vertices(); ++i) {            	//
		if (this->v[i] == u) {
			this->v.erase(this->v.begin() + i);             //  Removes Node<T> u (at index i) from vertices.
			break;                                          //
		}
	}

	for (auto& n : this->v) {                               //  Loops through every Node.
		for (auto& adj_node : n.v_adj) {                     //  Loops through Node<T> n's adjacent Nodes.
			if (adj_node.first == u) {			//
				remove_edge(n.node, u);		//  Remove any adjacent Node<T> to Node<T> u.
				break;				//
			}
		}
	}
}

/* test9 */

template <typename T>
vector<string> Graph<T>::depth_first_traversal(const string& u) {
	vector<string> v_DFS;
	stack<Node<T>> s_DFS;
	visited<string> v_visited;

	s_DFS.push(find_node(u));                           //  Begin with the root Node.

	while (!s_DFS.empty()) {                            //
		Node<T> top_node = s_DFS.top();             //  Evaluate the top Node<T> of the stack.
		s_DFS.pop();                                //

		if (!v_visited.is_visited(top_node.node)) {                                       //
			v_visited.visit(top_node.node);                                           //  Add ndTop to visited.
			v_DFS.push_back(top_node.node);                                           //  Add to DFS answer.

			for (int i = this->degree(top_node.node) - 1; i >= 0; --i) {			//	Add any Node<T> to ndTop
				if (!v_visited.is_visited(top_node.v_adj[i].first.node)) {		//	that has not been visited
					s_DFS.push(find_node(top_node.v_adj[i].first.node));		//	by the DFS stack.
				}
			}
		}
	}

	return v_DFS;
}

/* test10 */

template <typename T>
vector<string> Graph<T>::breadth_first_traversal(const string& u) {
	vector<string> v_BFS;
	queue<Node<T>> q_BFS;
	visited<string> v_visited;

	q_BFS.push(find_node(u));                           //  Begin with the root Node.

	while (!q_BFS.empty()) {                            //
		Node<T> front_node = q_BFS.front();         //  Search the first Node<T> in the queue.
		q_BFS.pop();                                //

		if (!v_visited.is_visited(front_node.node)) {                                     //
			v_visited.visit(front_node.node);                                         //  Add ndFront to visited.
			v_BFS.push_back(front_node.node);                                         //  Add to BFS answer.

			for (auto& adj_node : front_node.v_adj) {					//	Add any adjacent Node<T> to ndFront
				if (!v_visited.is_visited(adj_node.first.node)) {			//	to the queue if it is unvisited.
					q_BFS.push(find_node(adj_node.first.node));
				}
			}
		}
	}

	return v_BFS;
}

/* test11 */

template <typename T>
bool Graph<T>::contain_cycles() {
	return !(this->edge_count == this->num_vertices() - 1);	//	For a connected Graph<T>, a cycle must exist
								//	if the number of edges is not the number of
								//	vertices - 1.
}

/* test12 */

template <typename T>
Graph<T> Graph<T>::minimum_spanning_tree() {
	vector<pair<string, string>> v_edges = this->get_edges();
	vector<T> v_weights;

	for (auto& edge : v_edges) {
		v_weights.push_back(find_edge_weight(edge.first, edge.second));
	}

	while (this->edge_count != this->num_vertices() - 1) {		//	A tree is defined as e = v - 1
		pair<string, string> p_edge;
		T max_weight = INT_MIN;
		int max_weight_pos = -1;

		for (int i = 0; i < v_weights.size(); ++i) {		//	Finds the maximum weight of every edge.
			if (v_weights[i] > max_weight) {		//	Sets the edge with the maximum weight to
				max_weight = v_weights[i];		//	pEdge.
				p_edge = v_edges[i];			//
				max_weight_pos = i;			//
			}
		}

		v_weights.erase(v_weights.begin() + max_weight_pos);	//	Removes the maximum.
		v_edges.erase(v_edges.begin() + max_weight_pos);	//

		this->remove_edge(p_edge.first, p_edge.second);

		if (!this->is_connected()) {						//	If the edge is no longer connected
			this->add_edge(p_edge.first, p_edge.second, max_weight);	//	then it is a part of the MST.
		}
	}

	return *this;
}
