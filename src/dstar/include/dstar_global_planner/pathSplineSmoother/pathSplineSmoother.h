
/*
    SRL D* Lite ROS Package
    Copyright (C) 2015, Andrey Rudenko, Palmieri Luigi, palmieri@informatik.uni-freiburg.de

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>

 */



#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <vector>
#include <time.h>
#include <math.h>

#include <dstar_global_planner/pathSplineSmoother/realPoint.h>
#include <ros/ros.h>
#include <ros/console.h>

class PathSplineSmoother
{
	public:
		
		// Constructor with default sigma
		PathSplineSmoother();
		// Constructor with user's sigma input
		PathSplineSmoother(double sigma);
		// Destructor (currently nothing is there)
		~PathSplineSmoother();
		
		// Get sigma value
		double getSigma();
		// Set sigma value
		void setSigma(double sigma);
		
		// Copy input path from a RealPoint vector
		bool readPathFromStruct(std::vector<RealPoint> path);
		// Read input path from file
		bool readPathFromFile(std::string fileName);
		// Print input/filtered/smoothed path
		void printOriginalPath();

		void printFilteredPath();

		void printSmoothPath();
		// Get input/filtered/smoothed path
		std::vector<RealPoint> getOriginalPath();

		std::vector<RealPoint> getFilteredPath();

		std::vector<RealPoint> getSmoothPath();
		// Clear input/filtered/smoothed path
		void deleteOriginalPath();

		void deleteFilteredPath();

		void deleteSmoothPath();
		// Filter path
		// (remove some points from dense places, required for correct spline smoothing)
		// P parameter is the avgSegmentLength multiplier, set p to accept every
		// next point located further than p*avgSegmentLength.
		bool filterPath(double p);
		// Places additional points on segments of the input path to equilibrate
		// segment lengths. Parameter p is how densely the additional points are placed.
		bool placeAdditionalPoints(double p);
		// Smooth path
		bool smoothPath();
		// 1D spline smoothing subroutine for smoothPath() function
		void splineSmoothing(double input[], double x[], int n, double sig);
		// Direct (in-place) 2D path smoothing
		bool smoothPath2D();
		//TODO distance between first, last points, Frechet, what else?
		bool smoothWhileDistanceLessThan(double max_displacement, double sigma_div);
		
		// Iteratively increase amount of smoothing as long as smooth path's
		// start and goal displacement is smaller than acceptable max_displacement.
		bool smoothWhileSGDistanceLessThan(double max_displacement, double sigma_div);
		// Fix large displacement of start and goal points of the smooth path
		bool fixStartGoalDisplacement(int numPointsToRemove, double max_displacement, double sigma_div);

	   	// Compute how much the first and last points of the smooth path shifted
		// from the original.
		double distanceBetweenStartingPoints();

		// Checks if path is a straight line
		// In that case no smoothing is required
		bool isPathLine(std::vector<RealPoint> path);

		double distanceBetweenEndingPoints();

		// Compute how much a point on the path has shifted while smoothing
		double maxDisplacement();
		// TODO maybe this displacement measure is enough? Frechet is length^2 complexity...
		void writeFiles();
	
	private:
		// Sigma value
		double sigma_;
		// Default sigma constant value
		const double defSigma_ = 0.9;
		// Input path
		std::vector<RealPoint> path_;
		// Filtered path
		std::vector<RealPoint> pathC_;
		// Smoothed path
		std::vector<RealPoint> pathS_;
};
