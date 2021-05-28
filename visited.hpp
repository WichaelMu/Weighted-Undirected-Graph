#include "stdc++.h"

using namespace std;

template <typename T>
class visited {				//  The class to store what T has been visited.

public:
	vector<T> v_visited;		//  A vector of T that stores what has been visited.

	void visit(T u);		//  Visits T u.
	bool is_visited(T u);		//  Checks if T u has been visited.
};

template <typename T>
void visited<T>::visit(T u) {
	this->v_visited.push_back(u);
}

template <typename T>
bool visited<T>::is_visited(T u) {
	for (T& type : this->v_visited)
		if (type == u)
			return true;
	return false;
}
