#include<iostream>
#include <cstring>
#include <queue>
#include <set>
#include <stack>
#include "math.h"
#include <array>
#include <chrono>
#include <tuple>
#include <sstream>
#include <fstream>

using namespace std;

//Defining a simple container
class values {
	public:
		values (int x, int y) : x(x), y(y) {}
		int x; int y;
	
};

// A structure to hold the neccesary parameters such as cells parent info, f,g,h cost.
struct Position_Cell {

	//Variables of the position cell
	Position_Cell(): fcost(-1), gcost(-1), hcost(-1), Parent(-1,-1) {}

	// F cost, g cost and h cost
	double fcost, gcost, hcost;
	values Parent;

};

//Class defination with necessary functions
class Astarplanner{
  
 	public:
		void path_finder(const vector<vector<int>>& grid, const vector<int>& src, const vector<int>& dest, int& weight);
		vector<vector<int>> read_map(string &path);
	
	private:
		void pathfound(const vector<vector<Position_Cell>> & TrackerCell, const vector<int>& dest,const vector<int>& src );
		bool CellValidityCheck(const vector<vector<int>>& grid, const vector<int>& position,int &ROW, int &COL);
		bool BlockCheck(const vector<vector<int>>& grid, const vector<int>& position,int &ROW, int &COL);
		double Heuristic(const vector<int> src, const vector<int>& dest, int &weight);
		bool DestinationCheck(const vector<int>&  position, const vector<int>&  dest);
		bool check_validity(const vector<vector<int>>& grid,const vector<int> src,const vector<int>& dest,int &ROW, int &COL);
				
		vector<vector<Position_Cell>>  TrackerCell;
		vector<int> dest;
		vector<int> src;
		vector<vector<int>> grid;
		vector<int> position;
		int ROW; 
		int COL;
		int weight;
		string path;
		std::vector<std::vector<int>> List_Closed(int ROW, std::vector<int>(int COL));

};

