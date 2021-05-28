#include "stdc++.h"

using namespace std;

template <typename T>
struct Node {                           	//  The Node struct to store the weighted, undirected graph.

	static Node<T> default_node;

	string node;
	vector<pair<Node<T>, T>> v_adj;       	//  A vector of pairs of Nodes with their corresponding weights.
						//  .first = Node<T> adjacent, .second = T weight

	bool is_adj(const string& u) {      		//  If this Node is adjacent to Node u.
		for (auto adj_node : this->v_adj)        	//
			if (adj_node.first.node == u)      	//
				return true;            //
		return false;                   	//
	}

	bool operator<  (const Node<T>& n) { return this->node < n.node; }
	bool operator>  (const Node<T>& n) { return this->node > n.node; }
	bool operator<  (const string& u)  { return this->node < u; }
	bool operator>  (const string& u)  { return this->node > u; }
	bool operator== (const string& u)  { return this->node == u; }
};

template <typename T>
Node<T> Node<T>::default_node;
