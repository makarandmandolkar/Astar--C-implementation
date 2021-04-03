#pragma once
#include "givepath.hpp"

// Supporting utility functions
// To check if the cell value is withtin the range or not
bool Astarplanner::CellValidityCheck(const vector<vector<int>>& grid, const vector<int>& position,int &ROW, int &COL)
{ 
	if (ROW > 0 && COL > 0)
		return (position[0] >= 0) && (position[0] < ROW) && (position[1] >= 0) && (position[1] < COL);
	return false;
}

// To check if the cell is blocked or not. O is block and 1 is path available
bool Astarplanner::BlockCheck(const vector<vector<int>>& grid, const vector<int>& position,int &ROW, int &COL)
{
	return CellValidityCheck(grid, position, ROW, COL) && grid[position[0]][position[1]] == 1;
}

// To check if we have reached the destination
bool Astarplanner::DestinationCheck(const vector<int>&  position, const vector<int>&  dest)
{
	return dest == position;
}

// To calculate the heuristic value from current cell to the destination
double Astarplanner::Heuristic(const vector<int> src, const vector<int>& dest, int &weight)
{
	return weight * (sqrt(pow((src[0] - dest[0]), 2.0) + pow((src[1] - dest[1]), 2.0)));
}

// Function to check the validity of the source and destination provided by the user
bool Astarplanner::check_validity(const vector<vector<int>>& grid,const vector<int> src,const vector<int>& dest,int &ROW, int &COL){

	// To check of the source in within the range of the grid
	if (!CellValidityCheck(grid, src, ROW, COL)) {
		cout<<endl<<"Insert another value, The source in not valid"<<endl;
		return false;
	}
   

	// If the destination is not in the range
	if (!CellValidityCheck(grid, dest,ROW, COL)) {
		cout<<endl<<"Insert another value, the destination in not valid"<<endl;
		return false;
	}

     
	// To check if the position available for use
	if (!BlockCheck(grid, src, ROW,COL)) {
		cout<<endl<<"Insert another value, source is blocked"<<endl;
		return false;
	}

	// To check if the position available for use
	if (!BlockCheck(grid, dest, ROW, COL)) {
		cout<<endl<<"Insert another value, destination is blocked"<<endl;
		return false;
	}

	// If the destination cell is the same as source cell
	if (DestinationCheck(src, dest)) {
		cout<<endl<<"Insert another value, we are already at the destination"<<endl;
		return false;
	}
	return true;
}



// The main function which find the path  
void Astarplanner::path_finder(const vector<vector<int>>& grid, const vector<int>& src, const vector<int>& dest, int& weight)
{   
	// Initialization
	int ROW = grid.size();
	int COL = grid[0].size();
	vector<int> source = src;

    // Closed list equal to size of grid 
	int List_Closed[ROW][COL];

	// Initialized to 0 since no cell has been added yet
    for (int i = 0; i < grid.size(); i++){
        for (int j = 0; j < grid[1].size(); j++){
            List_Closed[i][j] = 0;  
        }
    }

    // To check if the given source and destination are valid to use
    if (!check_validity(grid, src, dest, ROW, COL)){
        return;
	}

    // Setting the values of the start node with 2D vector to store information
	vector<vector<Position_Cell>> TrackerCell( ROW ,vector<Position_Cell> (COL));
	int i, j;
	i = src[0]; j = src[1];
	TrackerCell[i][j].Parent = { i, j };
	TrackerCell[i][j].fcost = 0.0;
	TrackerCell[i][j].gcost = 0.0;
	TrackerCell[i][j].hcost = 0.0;
	
	
	// Using priority queue to set information of f cost, and position information
	std::priority_queue<vector<double>, std::vector<vector<double>>, std::greater<vector<double>>> Openlist;
	Openlist.emplace(vector<double> {0.0, i*1.0, j*1.0}); 


	// while loop to do the necessary computation and return the path
	while (!Openlist.empty()) {	
		// Taking the top element from the openlist and adding its position to closedlist
		const vector<double>& p = Openlist.top();
		i = p[1]; 
		j = p[2];
		List_Closed[i][j] = true;

		// Removing the element from the openlist
		Openlist.pop();
		
		// Checking the position of the neibouring cells of the current cells
		for (int x_neighbour = -1; x_neighbour <= 1; x_neighbour++) {

			for (int y_neighbour = -1; y_neighbour <= 1; y_neighbour++) {

				vector<int> next_cell {i + x_neighbour, j + y_neighbour};
				
				int p = next_cell[0]; 
				int q = next_cell[1];
				
				// Checking if the cells is good to use
				if (CellValidityCheck(grid, next_cell, ROW, COL)) {
					
					// If destination is found, setting the parent of the destination cell
					if (DestinationCheck(next_cell, dest)) { 
						cout<<"The destination cell is found! \n";
						TrackerCell[p][q].Parent = { i, j };
						pathfound(TrackerCell, dest, source);
						return;
					}
					// If available to use and not in closed list, do the following
					else if (!List_Closed[p][q] && BlockCheck(grid,next_cell, ROW, COL)) {
						double G_cost, H_cost, F_cost;
						G_cost = TrackerCell[i][j].gcost + 1.0;
						H_cost = Heuristic(next_cell, dest, weight);
						F_cost = G_cost + H_cost;
						
						// If better path by lower fcost and update also if not in open list, add it in open list
						if (TrackerCell[p][q].fcost == -1 || TrackerCell[p][q].fcost > F_cost) {
	
							Openlist.emplace(vector<double>{{F_cost, next_cell[0]*1.0, next_cell[1]*1.0}});
							TrackerCell[p][q].Parent = { i, j };
							TrackerCell[p][q].gcost = G_cost;
							TrackerCell[p][q].hcost = H_cost;
							TrackerCell[p][q].fcost = F_cost;
									
						}
					}
				}
			}
		}
	}


    //when there is no way to the destination or the open list is emply,
	//we can conculde that we failed to reach the destination
	std::cout<<"\nFailed to find the Destination Cell.\n";
}




