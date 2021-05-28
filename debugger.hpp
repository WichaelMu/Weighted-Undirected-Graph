#if DEBUG

#include "stdc++.h"
#include <chrono>

struct Timer {

	std::chrono::duration<double> time;
	std::chrono::steady_clock::time_point first, second;

public:
	void start_time() {
		first = std::chrono::high_resolution_clock::now();
	}

	void print_time(const string& message = "") {
		if (message == "") {
			cout << std::chrono::duration_cast<std::chrono::milliseconds>(this->end_time()).count() << "ms.\n";
		}
		else {
			cout << message << " took: " << std::chrono::duration_cast<std::chrono::microseconds>(this->end_time()).count() << "us.\n";
		}
	}

private:
	std::chrono::duration<double> end_time() {
		second = std::chrono::high_resolution_clock::now();

		time = second - first;
		return time;
	}

};

#endif
