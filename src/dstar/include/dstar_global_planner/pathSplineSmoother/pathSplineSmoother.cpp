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
#include <dstar_global_planner/pathSplineSmoother/pathSplineSmoother.h>

using namespace std;
PathSplineSmoother::PathSplineSmoother()
{
	sigma_ = defSigma_;
}

PathSplineSmoother::PathSplineSmoother(double sigma)
{
	if(sigma >= 1 || sigma <= 0)
	{
		ROS_WARN("PathSplineSmoother : [warning] Input sigma is invalid, setting default value...");
		sigma_ = defSigma_;
	}
	else sigma_ = sigma;
}

PathSplineSmoother::~PathSplineSmoother()
{
	
}

double PathSplineSmoother::getSigma()
{
	return sigma_;
}

void PathSplineSmoother::setSigma(double sigma)
{
	if(sigma >= 1 || sigma <= 0)
	{
		ROS_WARN("PathSplineSmoother : [warning] Input sigma is invalid, keeping the old sigma value...");
	}
	else sigma_ = sigma;
}

bool PathSplineSmoother::readPathFromStruct(vector<RealPoint> path)
{	
	path_ = path;
	return true;
}

bool PathSplineSmoother::readPathFromFile(string fileName)
{
	ifstream myfile (fileName);
	
	if (myfile.is_open())
	{
		string line;

    	while ( getline (myfile,line) )
    	{
      		//cout << line << '\n';
      		
      		istringstream iss(line);
    		while (iss)
    		{	
    			RealPoint point;
        		string sub;
        		iss >> sub;
        		if(sub.length() == 0) continue;
        		//cout << "Substring: " << sub << endl;
        		point.x = stod(sub);
        		iss >> sub;
        		if(sub.length() == 0) continue;
        		//cout << "Substring: " << sub << endl;
        		point.y = stod(sub);
        		iss >> sub;
        		if(sub.length() == 0) continue;
        		//cout << "Substring: " << sub << endl;
        		point.theta = stod(sub);
        		
        		path_.push_back(point);
    		}
    	}
    	
    	myfile.close();
	}
	else
    {	
    	ROS_ERROR(" PathSplineSmoother : [ERROR] Cannot open input file. Path reading failed.");
    	return false;
    }
	
	return true;
}

void PathSplineSmoother::printOriginalPath()
{
	int l = path_.size();
	ROS_DEBUG(" PathSplineSmoother :  Original path is:");
	ROS_DEBUG("PathSplineSmoother : x, y, theta");
	for(int i = 0;i<l;i++)
	{
		ROS_DEBUG("(%f, %f, %f)", path_.at(i).x, path_.at(i).y, path_.at(i).theta);
	}
}

void PathSplineSmoother::printFilteredPath()
{
	int l = pathC_.size();
	ROS_DEBUG("PathSplineSmoother : Filtered path is:");
	ROS_DEBUG("PathSplineSmoother : x, y, theta");
	for(int i = 0;i<l;i++)
	{
		ROS_DEBUG("(%f, %f, %f)", pathC_.at(i).x, pathC_.at(i).y, pathC_.at(i).theta);
	}
}

void PathSplineSmoother::printSmoothPath()
{
	int l = pathS_.size();
	ROS_DEBUG("PathSplineSmoother : Smooth path is:");
	ROS_DEBUG("PathSplineSmoother : x, y, theta");
	for(int i = 0;i<l;i++)
	{
		ROS_DEBUG("(%f, %f, %f)", pathS_.at(i).x, pathS_.at(i).y, pathS_.at(i).theta);

	}
}

vector<RealPoint> PathSplineSmoother::getOriginalPath()
{
	return path_;
}

vector<RealPoint> PathSplineSmoother::getFilteredPath()
{
	return pathC_;
}

vector<RealPoint> PathSplineSmoother::getSmoothPath()
{
	return pathS_;
}

void PathSplineSmoother::deleteOriginalPath()
{
	path_.clear();
}

void PathSplineSmoother::deleteFilteredPath()
{
	pathC_.clear();
}

void PathSplineSmoother::deleteSmoothPath()
{
	pathS_.clear();
}

bool PathSplineSmoother::filterPath(double p)
{
	int N = path_.size();
	
	if(N>0)
	{
		if(N<5)
		{
			ROS_WARN("PathSplineSmoother : [Warning] Path is too short for filtering, running placeAdditionalPoints(0.2) instead.");
			placeAdditionalPoints(0.2);
			return true;
		}
		
		//cout << N << endl;
	
		//------------------------------------------------------------------------------------
		// Segment lengths computation
	
		double avgSegmentLength;
		double segments[N-1];
	
		for(int i=0;i<N-1;i++)
		{
			segments[i] = sqrt(pow((path_.at(i+1).x - path_.at(i).x),2) + pow((path_.at(i+1).y - path_.at(i).y),2));
			avgSegmentLength+=segments[i];
		}
		
		avgSegmentLength = avgSegmentLength/(N-1);
		avgSegmentLength*=p;
	
		//------------------------------------------------------------------------------------
		// Path filtering
		
		deleteFilteredPath();
	
		double lengthSinceLastPoint = 0;
	
		pathC_.push_back(path_.at(0));
	
		for(int i=1;i<N-1;i++)
		{
			lengthSinceLastPoint+=segments[i-1];
			if(lengthSinceLastPoint>=avgSegmentLength)
			{
				pathC_.push_back(path_.at(i));
				lengthSinceLastPoint=0;
			}
		}
	
		int M = pathC_.size();
		if(pathC_.at(M-1).x != path_.at(N-1).x ||
			pathC_.at(M-1).y != path_.at(N-1).y ||
			pathC_.at(M-1).theta != path_.at(N-1).theta)
				pathC_.push_back(path_.at(N-1));
		
		if(M<5)
		{
			ROS_WARN("PathSplineSmoother : [Warning] Path is too short for filtering, running placeAdditionalPoints(0.2) instead.");
			placeAdditionalPoints(0.2);
		}
	
		return true;
	}
	else 
	{
		ROS_ERROR("PathSplineSmoother : [ERROR] The stored input path is empty. Path filtering failed.");
		return false;
	}
}

bool PathSplineSmoother::placeAdditionalPoints(double p)
{
	int N = path_.size();
	
	if(N>0)
	{
		if(N<5 && p>0.2)
		{
			ROS_ERROR("PathSplineSmoother : [Warning] Path is too short, for correct smoothing running placeAdditionalPoints(0.2).");
			p = 0.2;
		}
		
		//cout << N << endl;
	
		//------------------------------------------------------------------------------------
		// Segment lengths computation
	
		double minSegmentLength = 0;
		double segments[N-1];
		double dx[N-1];
		double dy[N-1];
	
		for(int i=0;i<N-1;i++)
		{
			dx[i] = path_.at(i+1).x - path_.at(i).x;
			dy[i] = path_.at(i+1).y - path_.at(i).y;
			segments[i] = sqrt(pow((dx[i]),2) + pow((dy[i]),2));
			if(segments[i] > 0 && minSegmentLength == 0) minSegmentLength = segments[i];
			else if(segments[i] > 0 && segments[i] < minSegmentLength) minSegmentLength = segments[i];
		}
		
		minSegmentLength*=p;
	
		//------------------------------------------------------------------------------------
		// Path filtering
		
		deleteFilteredPath();
	
		pathC_.push_back(path_.at(0));
	
		for(int i=0;i<N-1;i++)
		{
			if(segments[i] > 0)
			{
				int numNewPoints = round(segments[i]/minSegmentLength);
				
				//cout << "i = " << i << ", segments[i] = " << segments[i] 
				//	<< ", minSegmentLength = " << minSegmentLength << ", num = " <<  numNewPoints << endl;
				
				if(numNewPoints >= 2)
				{
					for(int j=1;j<numNewPoints;j++)
					{
						RealPoint point;
						point.x = path_.at(i).x + (j*dx[i])/numNewPoints;
						point.y = path_.at(i).y + (j*dy[i])/numNewPoints;
						point.theta = path_.at(i).theta;
						pathC_.push_back(point);
					}
				}
				
				pathC_.push_back(path_.at(i+1));
			}
		}
	
		//pathC_.push_back(path_.at(N-1));
	
		return true;
	}
	else 
	{
		ROS_ERROR("PathSplineSmoother : [ERROR] The stored input path is empty. Path filtering failed.");
		return false;
	}
}

bool PathSplineSmoother::isPathLine(std::vector<RealPoint> path)
{
	int N = path.size();
	
	if(N>2)
	{
		for(int i=1;i<N-1;i++)
		{
			double dx1 = path.at(i).x - path.at(i-1).x;
			double dy1 = path.at(i).y - path.at(i-1).y;
			double dx2 = path.at(i+1).x - path.at(i).x;
			double dy2 = path.at(i+1).y - path.at(i).y;
			double alpha1 = atan2(dy1,dx1);
			double alpha2 = atan2(dy2,dx2);
			if((dx1==0 && dy1==0) || (dx2==0 && dy2==0)) continue;
			if(alpha1!=alpha2)
			{
				return false;
			}
		}
		
		return true;
	}
	else if(N>0)
	{
		return true;
	}
	else
	{
		ROS_ERROR("PathSplineSmoother :[ERROR] Function input is empty. Path checking failed.");
		return false;
	}
}

bool PathSplineSmoother::smoothPath()
{
	int M = pathC_.size();
	
	if(M>=5)
	{	
		if(isPathLine(getFilteredPath()))
		{
			ROS_DEBUG("PathSplineSmoother : [Warning] Path is a line. Skipping smoothing.");
			pathS_ = pathC_;
			return true;
		}
			
		deleteSmoothPath();
	
		double x[M];
		double y[M];
		double smoothX[M];
		double smoothY[M];
	
		for(int i=0;i<M;i++)
		{
			x[i] = pathC_.at(i).x;
			y[i] = pathC_.at(i).y;
		}
	
		splineSmoothing(x,smoothX,M,sigma_);
		splineSmoothing(y,smoothY,M,sigma_);
		
		RealPoint point;
		
		for(int i=0;i<M;i++)
		{
			point.x = smoothX[i];
			point.y = smoothY[i];
			point.theta = 0;
			pathS_.push_back(point);
		}
		
		for(int i=1;i<M-1;i++)
		{
			double dx1 = pathS_.at(i).x - pathS_.at(i-1).x;
			double dy1 = pathS_.at(i).y - pathS_.at(i-1).y;
			double dx2 = pathS_.at(i+1).x - pathS_.at(i).x;
			double dy2 = pathS_.at(i+1).y - pathS_.at(i).y;
			double alpha1 = atan2(dy1,dx1);
			double alpha2 = atan2(dy2,dx2);
			if(alpha1<0) alpha1+=2*M_PI;
			if(alpha2<0) alpha2+=2*M_PI;
			pathS_.at(i).theta = 0.5*(alpha1 + alpha2);
			if(pathS_.at(i).theta>M_PI) pathS_.at(i).theta-=2*M_PI;
		}
		pathS_.at(0).theta = pathC_.at(0).theta;
		pathS_.at(M-1).theta = pathS_.at(M-2).theta;
		
		return true;
	}
	else if(M>0)
	{
		ROS_WARN("PathSplineSmoother : [Warning] Path is too short, correct smoothing is impossible.");
		pathS_ = pathC_;
		
		return true;
	}
	else
	{
		ROS_WARN("PathSplineSmoother : [ERROR] The stored filtered path is empty. Please filter the path before smoothing.");
		return false;
	}
}

void PathSplineSmoother::splineSmoothing(double input[], double x[], int n, double sig)
{	
	int nc = ceil(n/2);
	double e[n-1];
	for(int i = 0;i<n-1;i++)
		e[i] = 0;
	double f[n];
	for(int i = 0;i<n;i++)
	{
		f[i] = 0;
		x[i] = 0;
	}
	double lam = (4*pow(sig,4))/(1-pow(sig,2));
	double a1 = 1 + lam;
	double a2 = 5 + lam;
	double a3 = 6 + lam;

	//Factor the coefficient matrix and solve the first triangular system
	double d = a1;
	f[0] = 1/d;
	x[0] = f[0]*lam*input[0];
	double mu = 2;
	e[0] = mu*f[0];
	d = a2 - mu * e[0]; 
	f[1] = 1/d;
	x[1] = f[1] * (lam * input[1] + mu * x[0]);
	mu = 4 - e[0];
	e[1] = mu * f[1];
	
	for(int j = 2; j<n-2; j++)
	{
	    int m1 = j - 1;
   		int m2 = j - 2;
   		d = a3 - mu * e[m1] - f[m2];
    	f[j] = 1/d;
    	x[j] = f[j] * (lam * input[j] + mu * x[m1] - x[m2]);
    	mu = 4 - e[m1];
    	e[j] = mu * f[j];
	}
	
	d = a2 - mu * e[n - 3] - f[n - 4];
	f[n - 2] = 1/d;
	x[n - 2] = f[n - 2] * (lam * input[n - 2] + mu * x[n - 3] - x[n - 4]);
	mu = 2 - e[n - 3]; 
	e[n - 2] = mu * f[n - 2];
	d = a1 - mu * e[n - 2] - f[n - 3]; 
	f[n-1] = 1/d;
	x[n-1] = f[n-1] * (lam * input[n-1] + mu * x[n - 2] - x[n - 3]);

	//Solve the second triangular system
	x[n - 2] = x[n - 2] + e[n - 2] * x[n-1];
	for(int j = n - 3;j>=0; j--)
    	x[j] = x[j] + e[j] * x[j + 1] - f[j] * x[j + 2];
}

bool PathSplineSmoother::smoothPath2D()
{	
	int n = pathC_.size();
	
	if(n>=5)
	{
		if(isPathLine(getFilteredPath()))
		{
			ROS_WARN("PathSplineSmoother : [Warning] Path is a line. Skipping smoothing.");
			pathS_ = pathC_;
			return true;
		}
	
		deleteSmoothPath();
		pathS_ = pathC_;
		
		// - - - - - - - - - - - - x smoothing - - - - - - - - - - - -
		
		int nc = ceil(n/2);
		double e[n-1];
		for(int i = 0;i<n-1;i++)
			e[i] = 0;
		double f[n];
		for(int i = 0;i<n;i++)
		{
			f[i] = 0;
			pathS_.at(i).x = 0;
		}
		double lam = (4*pow(sigma_,4))/(1-pow(sigma_,2));
		double a1 = 1 + lam;
		double a2 = 5 + lam;
		double a3 = 6 + lam;

		//Factor the coefficient matrix and solve the first triangular system
		double d = a1;
		f[0] = 1/d;
		pathS_.at(0).x = f[0]*lam*pathC_.at(0).x;
		double mu = 2;
		e[0] = mu*f[0];
		d = a2 - mu * e[0]; 
		f[1] = 1/d;
		pathS_.at(1).x = f[1] * (lam * pathC_.at(1).x + mu * pathS_.at(0).x);
		mu = 4 - e[0];
		e[1] = mu * f[1];
	
		for(int j = 2; j<n-2; j++)
		{
			int m1 = j - 1;
			int m2 = j - 2;
			d = a3 - mu * e[m1] - f[m2];
			f[j] = 1/d;
			pathS_.at(j).x = f[j] * (lam * pathC_.at(j).x + mu * pathS_.at(m1).x - pathS_.at(m2).x);
			mu = 4 - e[m1];
			e[j] = mu * f[j];
		}
	
		d = a2 - mu * e[n - 3] - f[n - 4];
		f[n - 2] = 1/d;
		pathS_.at(n-2).x = f[n - 2] * (lam * pathC_.at(n-2).x + mu * pathS_.at(n-3).x - pathS_.at(n-4).x);
		mu = 2 - e[n - 3];
		e[n - 2] = mu * f[n - 2];
		d = a1 - mu * e[n - 2] - f[n - 3]; 
		f[n-1] = 1/d;
		pathS_.at(n-1).x = f[n-1] * (lam * pathC_.at(n-1).x + mu * pathS_.at(n-2).x - pathS_.at(n-3).x);

		//Solve the second triangular system
		pathS_.at(n-2).x = pathS_.at(n-2).x + e[n - 2] * pathS_.at(n-1).x;
		for(int j = n - 3;j>=0; j--)
			pathS_.at(j).x = pathS_.at(j).x + e[j] * pathS_.at(j+1).x - f[j] * pathS_.at(j+2).x;
			
		// - - - - - - - - - - - - y smoothing - - - - - - - - - - - -
		
		nc = ceil(n/2);
		for(int i = 0;i<n-1;i++)
			e[i] = 0;
		for(int i = 0;i<n;i++)
		{
			f[i] = 0;
			pathS_.at(i).y = 0;
		}
		lam = (4*pow(sigma_,4))/(1-pow(sigma_,2));
		a1 = 1 + lam;
		a2 = 5 + lam;
		a3 = 6 + lam;

		//Factor the coefficient matrix and solve the first triangular system
		d = a1;
		f[0] = 1/d;
		pathS_.at(0).y = f[0]*lam*pathC_.at(0).y;
		mu = 2;
		e[0] = mu*f[0];
		d = a2 - mu * e[0]; 
		f[1] = 1/d;
		pathS_.at(1).y = f[1] * (lam * pathC_.at(1).y + mu * pathS_.at(0).y);
		mu = 4 - e[0];
		e[1] = mu * f[1];
	
		for(int j = 2; j<n-2; j++)
		{
			int m1 = j - 1;
			int m2 = j - 2;
			d = a3 - mu * e[m1] - f[m2];
			f[j] = 1/d;
			pathS_.at(j).y = f[j] * (lam * pathC_.at(j).y + mu * pathS_.at(m1).y - pathS_.at(m2).y);
			mu = 4 - e[m1];
			e[j] = mu * f[j];
		}
	
		d = a2 - mu * e[n - 3] - f[n - 4];
		f[n - 2] = 1/d;
		pathS_.at(n-2).y = f[n - 2] * (lam * pathC_.at(n-2).y + mu * pathS_.at(n-3).y - pathS_.at(n-4).y);
		mu = 2 - e[n - 3]; 
		e[n - 2] = mu * f[n - 2];
		d = a1 - mu * e[n - 2] - f[n - 3]; 
		f[n-1] = 1/d;
		pathS_.at(n-1).y = f[n-1] * (lam * pathC_.at(n-1).y + mu * pathS_.at(n-2).y - pathS_.at(n-3).y);

		//Solve the second triangular system
		pathS_.at(n-2).y = pathS_.at(n-2).y + e[n - 2] * pathS_.at(n-1).y;
		for(int j = n - 3;j>=0; j--)
			pathS_.at(j).y = pathS_.at(j).y + e[j] * pathS_.at(j+1).y - f[j] * pathS_.at(j+2).y;
			
		// - - - - - - - - - - - - Add theta coordinates - - - - - - - - - - - -
		
		for(int i=1;i<n-1;i++)
		{
			double dx1 = pathS_.at(i).x - pathS_.at(i-1).x;
			double dy1 = pathS_.at(i).y - pathS_.at(i-1).y;
			double dx2 = pathS_.at(i+1).x - pathS_.at(i).x;
			double dy2 = pathS_.at(i+1).y - pathS_.at(i).y;
			double alpha1 = atan2(dy1,dx1);
			double alpha2 = atan2(dy2,dx2);
			if(alpha1<0) alpha1+=2*M_PI;
			if(alpha2<0) alpha2+=2*M_PI;
			pathS_.at(i).theta = 0.5*(alpha1 + alpha2);
			if(pathS_.at(i).theta>M_PI) pathS_.at(i).theta-=2*M_PI;
		}
		pathS_.at(0).theta = pathC_.at(0).theta;
		pathS_.at(n-1).theta = pathS_.at(n-2).theta;
			
		return true;
	}
	else if(n>0)
	{
		ROS_WARN("PathSplineSmoother : [Warning] Path is too short, correct smoothing is impossible.");
		pathS_ = pathC_;
		
		return true;
	}
	else
	{
		ROS_ERROR("PathSplineSmoother : [ERROR] The stored filtered path is empty. Please filter the path before smoothing.");
		return false;
	}
}

bool PathSplineSmoother::smoothWhileDistanceLessThan(double max_displacement, double sigma_div)
{
	if(sigma_div <= 1)
	{
		ROS_ERROR("PathSplineSmoother : [ERROR] Input sigma_div <= 1 is invalid, smoothing failed.");
		return false;
	}
	
	int n = pathC_.size();
	
	if(n>=5)
	{
		if(isPathLine(getFilteredPath()))
		{
			ROS_WARN("PathSplineSmoother : [Warning] Path is a line. Skipping smoothing.");
			pathS_ = pathC_;
			return true;
		}
	
		double sigma_old = sigma_;
		sigma_ = 0.99;
		while(true)
		{
			sigma_/=sigma_div;
			ROS_DEBUG("PathSplineSmoother : Sigma = %f ", sigma_);
			deleteSmoothPath();
			smoothPath2D();
			ROS_DEBUG("PathSplineSmoother : Max displacement = %f", maxDisplacement());

			if((maxDisplacement() > max_displacement) || (sigma_ < 0.01))
			{
				deleteSmoothPath();
				sigma_*=sigma_div;
				smoothPath2D();
				break;
			}
		}
		
		//sigma_ = sigma_old;
		return true;
	}
	else if(n>0)
	{
		ROS_WARN("PathSplineSmoother : [warning] Path is too short, correct smoothing is impossible.");
		pathS_ = pathC_;
		
		return true;
	}
	else
	{
		ROS_ERROR("PathSplineSmoother : [ERROR] The stored filtered path is empty. Please filter the path before smoothing.");
		return false;
	}
}

bool PathSplineSmoother::smoothWhileSGDistanceLessThan(double max_displacement, double sigma_div)
{
	if(sigma_div <= 1)
	{
		ROS_ERROR("PathSplineSmoother : [ERROR] Input sigma_div <= 1 is invalid, smoothing failed.");
		return false;
	}
	
	int n = pathC_.size();
	
	if(n>=5)
	{
		if(isPathLine(getFilteredPath()))
		{
			ROS_WARN("PathSplineSmoother : [Warning] Path is a line. Skipping smoothing.");
			pathS_ = pathC_;
			return true;
		}
	
		double sigma_old = sigma_;
		sigma_ = 0.99;
		while(true)
		{
			sigma_/=sigma_div;
			ROS_DEBUG("PathSplineSmoother : Sigma = %f", sigma_);
			deleteSmoothPath();
			smoothPath();
			ROS_DEBUG("PathSplineSmoother : Max displacement = %f", maxDisplacement());
			if((distanceBetweenStartingPoints() > max_displacement) || 
				(distanceBetweenEndingPoints() > max_displacement)
				|| (sigma_ < 0.01)) 
			//if(maxDisplacement() > max_displacement)
			{
				deleteSmoothPath();
				sigma_*=sigma_div;
				smoothPath2D();
				break;
			}
		}
		
		//sigma_ = sigma_old;
		return true;
	}
	else if(n>0)
	{
		ROS_WARN("PathSplineSmoother : [Warning] Path is too short, correct smoothing is impossible.");
		pathS_ = pathC_;
		
		return true;
	}
	else
	{
		ROS_ERROR("PathSplineSmoother : [ERROR] The stored filtered path is empty. Please filter the path before smoothing.");
		return false;
	}
}

bool PathSplineSmoother::fixStartGoalDisplacement(int numPointsToRemove, double max_displacement, double sigma_div)
{
	int M = pathS_.size();
	
	if(M>5)
	{	
		if(distanceBetweenStartingPoints()<max_displacement && distanceBetweenEndingPoints()<max_displacement)
		{
			ROS_WARN("PathSplineSmoother : [Warning] Displacement is below threshold, no fixing required.");
			return true;
		}
		
		if(M<=numPointsToRemove*2)
		{
			ROS_DEBUG_STREAM("PathSplineSmoother : Smoothed path length is " << M << ", numPointsToRemove = " << numPointsToRemove
				<< ", removing " << numPointsToRemove*2 << " points is impossible! Path fixing failed.\n");
			return false;
		}
		

		pathS_.at(0) = pathC_.at(0);
		pathS_.at(M-1) = pathC_.at(M-1);
		double dx1 = pathS_.at(numPointsToRemove).x - pathC_.at(0).x;
		double dy1 = pathS_.at(numPointsToRemove).y - pathC_.at(0).y;
		double dx2 = pathS_.at(M-1-numPointsToRemove).x - pathC_.at(M-1).x;
		double dy2 = pathS_.at(M-1-numPointsToRemove).y - pathC_.at(M-1).y;
		for(int i=1;i<numPointsToRemove;i++)
		{
			RealPoint point;
			point.x = pathC_.at(0).x + (i*dx1)/numPointsToRemove;
			point.y = pathC_.at(0).y + (i*dy1)/numPointsToRemove;
			point.theta = 0;
			pathS_.at(i) = point;
			point.x = pathC_.at(M-1).x + (i*dx2)/numPointsToRemove;
			point.y = pathC_.at(M-1).y + (i*dy2)/numPointsToRemove;
			point.theta = 0;
			pathS_.at(M-1-i) = point;
		}
		
		deleteFilteredPath();
		pathC_ = pathS_;
		deleteSmoothPath();
		
		smoothWhileSGDistanceLessThan(max_displacement,sigma_div);
		
		return true;
	}
	else if(M>0)
	{
		ROS_WARN("PathSplineSmoother : [Warning] Path is too short, correct smoothing is impossible.");
		pathS_ = pathC_;
		
		return true;
	}
	else
	{
		ROS_ERROR("PathSplineSmoother : [ERROR] The stored smooth path is empty. Please smooth the path before fixing.");
		return false;
	}
}

double PathSplineSmoother::distanceBetweenStartingPoints()
{
	int n = pathS_.size();
	if(n > 0)
	{	
		double x1 = path_.at(0).x;
		double x2 = pathS_.at(0).x;
		double y1 = path_.at(0).y;
		double y2 = pathS_.at(0).y;
		return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
	}
	else
	{
		ROS_ERROR("PathSplineSmoother : [ERROR] The stored smoothed path is empty. No distance is available.");
		return -1;
	}
}

double PathSplineSmoother::distanceBetweenEndingPoints()
{
	int n = pathS_.size();
	int m = path_.size();
	if(n > 0)
	{	
		double x1 = path_.at(m-1).x;
		double x2 = pathS_.at(n-1).x;
		double y1 = path_.at(m-1).y;
		double y2 = pathS_.at(n-1).y;
		return sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
	}
	else
	{
		ROS_ERROR("PathSplineSmoother : [ERROR] The stored smoothed path is empty. No distance is available.");
		return -1;
	}
}

double PathSplineSmoother::maxDisplacement()
{
	int n = pathS_.size();
	if(n > 0)
	{	
		double max_dist = 0;
		for(int i=0;i<n;i++)
		{
			double x1 = pathC_.at(i).x;
			double x2 = pathS_.at(i).x;
			double y1 = pathC_.at(i).y;
			double y2 = pathS_.at(i).y;
			double dist = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
			if(dist>max_dist) max_dist = dist;
		}
		return max_dist;
	}
	else
	{
		ROS_ERROR("PathSplineSmoother : [ERROR] The stored smoothed path is empty. No distance is available.");
		return -1;
	}
}

void PathSplineSmoother::writeFiles()
{
	int M = pathS_.size();
	
	ofstream pathS;
  	pathS.open ("pathS.txt");
  	ofstream pathCf;
  	pathCf.open ("pathC.txt");
  	
	for(int i = 0; i<M; i++)
	{
		pathS << pathS_.at(i).x << "\t";
		pathS << pathS_.at(i).y << "\t";
		pathS << pathS_.at(i).theta << endl;
		pathCf << pathC_.at(i).x << "\t";
		pathCf << pathC_.at(i).y << "\t";
		pathCf << pathC_.at(i).theta << endl;
	}
	pathS.close();
	pathCf.close();
}
