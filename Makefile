all:
	g++ loadpng.cpp AR.cpp -ansi -pedantic -Wall -Wextra -O3
	./a.out desk_1 desk_1_result
