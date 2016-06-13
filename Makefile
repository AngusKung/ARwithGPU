all:
	g++ -std=c++11 loadpng.cpp AR.cpp -pedantic -Wall -Wextra -O3
	./a.out desk_1 desk_1_result
