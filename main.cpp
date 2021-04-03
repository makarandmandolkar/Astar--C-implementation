#include "utilityfunction.hpp"

// https://en.wikipedia.org/wiki/A*_search_algorithm
// Main function to initiate the code. 
// Map in provided by a text file, where 1 is not blocked and 0 is blocked

int main(){
    // Created an object astar
    Astarplanner astar; 

    //Provide the path to the txt file
    string path = "map.txt";
    auto Grid = astar.read_map(path); 

	//Provide the source and destination position
    vector<int> start = {0,0};
    vector<int> dest = {5,3};

    //For the heuristic component  
    int weight = 10;   

    // Function call to give the path
	astar.path_finder(Grid, start, dest, weight);

	return (0);
}
