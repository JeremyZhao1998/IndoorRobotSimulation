#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <string>
#include <time.h>

//include ros libraries
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseActionGoal.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"

#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/GetPlan.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <boost/foreach.hpp>
//#define forEach BOOST_FOREACH

/** for global path planner interface */
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>

#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>

//#include <pcl_conversions/pcl_conversions.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

#include <set>

#include "rrt.h"

using namespace std;
using std::string;

#ifndef RRT_PLANNER
#define RRT_PLANNER

/**
 * @struct cells
 * @brief A struct that represents a cell and its fCost.
 */

#define RANDOM_MAX 2147483647

namespace rrt_plan
{
struct Cell
{
    int posX;
    int posY;
};

class rrt_planner : public nav_core::BaseGlobalPlanner
{
public:

    rrt_planner (ros::NodeHandle &); //this constructor is may be not needed
    rrt_planner ();
    rrt_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

    ros::NodeHandle ROSNodeHandle;

    /** overriden classes from interface nav_core::BaseGlobalPlanner **/
    void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
    bool makePlan(const geometry_msgs::PoseStamped& start,
                  const geometry_msgs::PoseStamped& goal,
                  std::vector<geometry_msgs::PoseStamped>& plan);

    bool isCellInsideMap(float x, float y);
    void mapToWorld(double mx, double my, double& wx, double& wy);

    bool isStartAndGoalCellsValid(int startCell,int goalCell);
    bool isFree(int CellID); //returns true if the cell is Free
    bool isFree(int i, int j);
    vector <int> findFreeNeighborCell (int CellID);

   // vector<int>   findCellPath(RRT &myRRT, int startCell, int goalCell,vector<vector<float> > gridVal);//使用温度场作为启发因子
    vector<int>   findCellPath(RRT &myRRT, int startCell, int goalCell);//使用温度场作为启发因子
    void convertToPlan(vector<int> bestPath,std::vector<geometry_msgs::PoseStamped>& plan, geometry_msgs::PoseStamped goal,int goalCell);
    void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
    void publishPlan2(const std::vector<geometry_msgs::PoseStamped>& path);
    //void getGridVal(vector<vector<float> >& gridVal, int startRowID, int startColID,int goalRowID,int goalColID);
    
    

    void getIdx(int &x, int &y)
    {
        x = x/mapReduceSize;
        y = y/mapReduceSize;
    }

    void getCorrdinate (float &x, float &y)
    {
        x = x - originX;
        y = y - originY;
    }

    int getCellIndex(int i,int j)//根据cell所处的行和列得到cell的索引值
    {
        return (i*width)+j;
    }

    int getCellRowID(int index)//根据cell的索引值得到行数
    {
        return index/width;
    }

    int getCellColID(int index)//根据索引值得到列数
    {
        return index%width;
    }

    int convertToCellIndex (float x, float y)//根据坐标值得到cell索引值
    {
        int cellIndex;

        float newX = x / resolution;
        float newY = y / resolution;

        cellIndex = getCellIndex(newY, newX);

        return cellIndex;
    }


    void convertToCoordinate(int index, float &x, float &y)//根据cell索引值得到坐标值
    {
        x = getCellColID(index) * resolution;

        y = getCellRowID(index) * resolution;

        x = x + originX;
        y = y + originY;
    }

    bool checkIfOnObstacles(RRT::rrtNode &tempNode);

    bool pointcheck(RRT::rrtNode const & m,RRT::rrtNode const & n);

    void generateTempPoint(RRT::rrtNode &tempNode,int width, int height)//生成一个随机结点
    {
        int x = rand() % width + 1;  //行
        int y = rand() % height + 1;  //列
        tempNode.posX = x;
        tempNode.posY = y;
    }

    void generateRandNodes(RRT &myRRT, vector<RRT::rrtNode> &randNodes,int width, int height)//生成一个随机结点数组
    {
        RRT::rrtNode loopNode;

        for(int i=0; i<randNodeNum; i++)
        {
            int x = rand() % width + 1;  //行
            int y = rand() % height + 1;  //列
          
            loopNode.posX = x;
            loopNode.posY = y;

            randNodes.push_back(loopNode);
        }

    }

    //bool addNewNodetoRRT(RRT &myRRT, vector<RRT::rrtNode> &randNodes,  float goalX, float goalY , int rrtStepSize,vector<vector<float> > gridVal)  //使用温度场作为启发因子
    bool addNewNodetoRRT(RRT &myRRT, vector<RRT::rrtNode> &randNodes,  float goalX, float goalY , int rrtStepSize)  //使用温度场作为启发因子
    {
        vector<RRT::rrtNode> nearestNodes;  //nearest结点数组
        vector<RRT::rrtNode> tempNodes;  //临时结点数组
        RRT::rrtNode loopNode;

        for(int i=0; i<randNodeNum; i++)
        {
            int tempNearestNodeID = myRRT.getNearestNodeID(randNodes[i].posX,randNodes[i].posY);
            RRT::rrtNode tempNearestNode=myRRT.getNode(tempNearestNodeID);
            nearestNodes.push_back(tempNearestNode);
        }

        for(int i=0; i<randNodeNum; i++)
        {
            double theta = atan2(randNodes[i].posY - nearestNodes[i].posY, randNodes[i].posX - nearestNodes[i].posX);//两个点形成的斜率
            loopNode.posX = (int)(nearestNodes[i].posX + (rrtStepSize * cos(theta)));
            loopNode.posY = (int)(nearestNodes[i].posY + (rrtStepSize * sin(theta)));
            loopNode.parentID=nearestNodes[i].nodeID;  //temp结点的父结点是它的nearest结点
            if(pointcheck(nearestNodes[i], loopNode))
            {
                tempNodes.push_back(loopNode);
            }
        }

        vector<float> evaluateCost;  //启发函数估价

        for(int i=0; i<tempNodes.size(); i++)
        {
            int tempIndex=i;
            RRT::rrtNode tempNode=tempNodes[i];

            if(checkIfOnObstacles(tempNode))//先检查temp结点，不在障碍物上
            {
                float costG=nearestNodes[i].depth*rrtStepSize;
                //float costH=gridVal[tempNode.posX/mapReduceSize][tempNode.posY/mapReduceSize];  //温度场作为代价值
                evaluateCost.push_back(costG);
            }
            else  //tempNode结点在障碍物上
            {
                evaluateCost.push_back(infinity);  //障碍物上代价无穷大
            }
        }

        int tempIndex=-1;
        float tempCost=infinity;
        for(int i=0; i<evaluateCost.size(); i++)
        {
            if(evaluateCost[i]<tempCost)
            {
                tempIndex=i;
                tempCost=evaluateCost[i];
            }
        }

        if(tempIndex!=-1)
        {
            RRT::rrtNode successNode=tempNodes[tempIndex];
            RRT::rrtNode nearestNode=nearestNodes[tempIndex];
            float thrshold=15.00,maxvalue=999,maxvalue1;     
            int index=-1;      
            for(int i=0;i<myRRT.getTreeSize();i++)
            {
                float dist=sqrt(pow(successNode.posX-myRRT.getNode(i).posX,2)+pow(successNode.posY-myRRT.getNode(i).posY,2));
                if((dist<thrshold)&&(myRRT.getNode(i).depth*rrtStepSize+dist<maxvalue))
                {
                    maxvalue=myRRT.getNode(i).depth*rrtStepSize+dist;
                    index=i;
                }
            }          
            if((index!=-1)&&(pointcheck(myRRT.getNode(index),successNode)))
            {               
                successNode.nodeID = myRRT.getTreeSize();
                successNode.depth=myRRT.getNode(index).depth+sqrt(pow(successNode.posX-myRRT.getNode(index).posX,2)+pow(successNode.posY-myRRT.getNode(index).posY,2))/rrtStepSize;
                successNode.parentID=myRRT.getNode(index).nodeID;              
            }
            else
            {
                successNode.nodeID = myRRT.getTreeSize();
                successNode.depth=nearestNodes[tempIndex].depth+1; 
            }
            vector<int>index1;
            for(int j=0;j<myRRT.getTreeSize();j++)
            {
                float dist1=sqrt(pow(successNode.posX-myRRT.getNode(j).posX,2)+pow(successNode.posY-myRRT.getNode(j).posY,2));
                if((j!=index)&&(dist1<thrshold)&&(pointcheck(successNode,myRRT.getNode(j) )))
                {              
                    index1.push_back(j);
                }
            }
            if((index1.size()!=0)&&(index!=-1)&&(pointcheck(myRRT.getNode(index),successNode)))
            {
            	    for (int k=0;k<index1.size();k++)
            	    {
                    	RRT::rrtNode tt=myRRT.getNode(index1[k]);
                    	successNode.nodeID = myRRT.getTreeSize();
                    	successNode.depth=myRRT.getNode(index).depth+sqrt(pow(successNode.posX-myRRT.getNode(index).posX,2)+pow(successNode.posY-myRRT.getNode(index).posY,2))/rrtStepSize;   
                    	if((successNode.depth+sqrt(pow(successNode.posX-myRRT.getNode(index1[k]).posX,2)+pow(successNode.posY-myRRT.getNode(index1[k]).posY,2))/rrtStepSize<myRRT.getNode(index1[k]).depth)&&(pointcheck(successNode,myRRT.getNode(index1[k]))))
                    	{          
                    		tt.parentID=successNode.nodeID; 
                    		tt.depth=successNode.depth+sqrt(pow(successNode.posX-myRRT.getNode(index1[k]).posX,2)+pow(successNode.posY-myRRT.getNode(index1[k]).posY,2))/rrtStepSize;
                    		tt.nodeID=myRRT.getTreeSize()+1;
                   		    myRRT.getNode(index1[k])=tt;
                        }
                    }
            }
    
            myRRT.addNewNode(successNode);
            return true;            
        }
        return false;
    }

    bool checkNodetoGoal(int X, int Y, RRT::rrtNode &tempNode)
    {
        double distance = sqrt(pow(X-tempNode.posX,2)+pow(Y-tempNode.posY,2));
        if(distance < rrtStepSize)  //goal与nearest之间距离小于rrtStepSize时即认为到达目标点
        {
            return true;
        }
        return false;
    }

    float originX;
    float originY;
    float resolution;
    costmap_2d::Costmap2DROS* costmap_ros_;
    double step_size_, min_dist_from_robot_;
    costmap_2d::Costmap2D* costmap_;
    //base_local_planner::WorldModel* world_model_; ///< @brief The world model that the controller will use
    bool initialized_;
    int width;
    int height;

    ros::Publisher plan_pub_;
    ros::Publisher plan_pub_2;
};

};
#endif
