#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <string>

#include <time.h>
#include <omp.h>

#include <pluginlib/class_list_macros.h>

#include "inspire_rrt_planner.h"

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_plan::rrt_planner, nav_core::BaseGlobalPlanner)

static const float INFINIT_COST = INT_MAX; //!< cost of non connected nodes

int mapSize;
bool* OGM;
bool* OGM2;
RRT myRRT;//RRT树结构
ofstream irrtlog("/home/ljq/irrtlog.txt");

int clock_gettime(clockid_t clk_id, struct timespect *tp);

long getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv,NULL);
    return tv.tv_sec*1000+tv.tv_usec/1000;
}

timespec diff(timespec start, timespec end)
{
    timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0)
    {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    }
    else
    {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp;
}

namespace rrt_plan
{
//Default Constructor
rrt_planner::rrt_planner():initialized_(false)
{

}
rrt_planner::rrt_planner(ros::NodeHandle &nh)
{
    ROSNodeHandle = nh;
}

rrt_planner::rrt_planner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
    initialize(name, costmap_ros);
}

void rrt_planner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{

    if (!initialized_)
    {
        costmap_ros_ = costmap_ros;
        costmap_ = costmap_ros_->getCostmap();

        ros::NodeHandle private_nh("~/" + name);

        originX = costmap_->getOriginX();
        originY = costmap_->getOriginY();

        width = costmap_->getSizeInCellsX();
        height = costmap_->getSizeInCellsY();
        resolution = costmap_->getResolution();
        mapSize = width*height;

        ROS_INFO("---originX=%f,originY=%f---",originX,originY);

        ROS_INFO("---width=%d,height=%d---",width,height);

        ROS_INFO("---resolution=%f---",resolution);

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
        plan_pub_2 = private_nh.advertise<nav_msgs::Path>("plan2", 1);

        OGM = new bool [mapSize];
        for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
        {
            for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
            {
                unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));

                //if (cost == 0)
                /*if(cost<=80)  // 调节避障条件，范围越大，要求越松散
                    OGM[iy*width+ix]=true;
                else
                    OGM[iy*width+ix]=false;*/
              if(cost<=78)  
                    OGM[iy*width+ix]=true;
                else
                    OGM[iy*width+ix]=false;
            }
        }

        OGM2 = new bool [mapSize];
        for (unsigned int iy = 0; iy < costmap_->getSizeInCellsY(); iy++)
        {
            for (unsigned int ix = 0; ix < costmap_->getSizeInCellsX(); ix++)
            {
                unsigned int cost = static_cast<int>(costmap_->getCost(ix, iy));

                if(cost==254)  // 调节避障条件，范围越大，要求越松散
                    OGM2[iy*width+ix]=false;
                else
                    OGM2[iy*width+ix]=true;
            }
        }
        
        ROS_INFO("---rrt planner1 initialized successfully---");

        initialized_ = true;
    }
    else
    {
        ROS_WARN("---This planner1 has already been initialized... doing nothing---");
    }
}



/*void rrt_planner::getGridVal(vector<vector<float> >& gridVal, int startRowID, int startColID,int goalRowID,int goalColID)  //模拟温度场
{
    int subwidth=width/mapReduceSize,subheight=height/mapReduceSize;
    int i , j , m , n , s;
    float gridMap[subheight][subwidth];

    #pragma opm parallel for private(i,j) schedule(static)
    for (i=0; i<subheight; i++)
    {
        for (j=0; j<subwidth; j++)
        {
            gridMap[i][j] = 1.0f;  //1表示自由区域

            gridVal[i][j] = 0.0f;
            for (m=0; m<mapReduceSize; m++)
            {
                for (n=0; n<mapReduceSize; n++)
                {
                    int cellIndex=getCellIndex(i*mapReduceSize+m,j*mapReduceSize+n);
                    if(!OGM2[cellIndex])
                    {
                        gridMap[i][j] = 0.0f;  //0表示障碍物
                    }
                }
            }
        }
    }
    

    int initIdx[2] = {0}; // index of initial point
    int goalIdx[2] = {0}; // index of goal point

    getIdx(startRowID, startColID);
    getIdx(goalRowID, goalColID);

    initIdx[0]=startRowID;
    initIdx[1]=startColID;
    goalIdx[0]=goalRowID;
    goalIdx[1]=goalColID;

    gridVal[initIdx[0]][initIdx[1]] = initialTemperature;
    gridVal[goalIdx[0]][goalIdx[1]] = -initialTemperature;
    
    
    #pragma omp parallel for private(s,i,j) schedule(static)
    for (s=0; s<iterationTime; s++)
    {
        {
            for (i=1; i<subheight-1; i++)
            {
                for (j=1; j<subwidth-1; j++)
                {
                    if (gridMap[i][j])
                    {
                        float delta = (gridVal[i-1][j]*gridMap[i-1][j]
                                       +gridVal[i][j-1]*gridMap[i][j-1]
                                       +gridVal[i+1][j]*gridMap[i+1][j]
                                       +gridVal[i][j+1]*gridMap[i][j+1])
                                      /(gridMap[i-1][j]+gridMap[i][j-1]+gridMap[i+1][j]+gridMap[i][j+1]);
                        gridVal[i][j] = delta;
                    }
                }
            }
        }
    }
}*/



bool rrt_planner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                           std::vector<geometry_msgs::PoseStamped>& plan)
{
    srand((unsigned)time(NULL));//设置随机数种子

    if (!initialized_)
    {
        ROS_ERROR("---The planner has not been initialized, please call initialize() to use the planner---");
        return false;
    }

    if (goal.header.frame_id != costmap_ros_->getGlobalFrameID())
    {
        ROS_ERROR("---This planner as configured will only accept goals in the %s frame, but a goal was sent in the %s frame.---",
                  costmap_ros_->getGlobalFrameID().c_str(), goal.header.frame_id.c_str());
        return false;
    }


    plan.clear();

    tf::Stamped < tf::Pose > goal_tf;
    tf::Stamped < tf::Pose > start_tf;

    poseStampedMsgToTF(goal, goal_tf);
    poseStampedMsgToTF(start, start_tf);

    // convert the start and goal positions

    float startX = start.pose.position.x;
    float startY = start.pose.position.y;

    float goalX = goal.pose.position.x;
    float goalY = goal.pose.position.y;

    getCorrdinate(startX, startY);
    getCorrdinate(goalX, goalY);

    int startCell;
    int goalCell;

    //ROS_INFO("---startX=%f,startY=%f---",startX,startY);
    //ROS_INFO("---goalX=%f,goalY=%f---",goalX,goalY);

    if (isCellInsideMap(startX, startY) && isCellInsideMap(goalX, goalY))
    {
        startCell = convertToCellIndex(startX, startY);
        goalCell = convertToCellIndex(goalX, goalY);
        cout << startCell <<"\t"<< start.pose.position.x <<"\t"<< start.pose.position.y <<"\t"
            << goalCell <<"\t"<< goal.pose.position.x <<"\t"<< goal.pose.position.y<<"\n";
    }
    else
    {
        ROS_WARN("---the start or goal is out of the map---");
        //rrtlog<<"---the start or goal is out of the map---"<<endl;
        return false;
    }

    //cout<<"startposion:"<<getCellRowID(startCell)<<"  "<<getCellColID(startCell)<<endl;
    //cout<<"goalposion:"<<getCellRowID(goalCell)<<"  "<<getCellColID(goalCell)<<endl;

    vector<int> rrtPath;

    if (isStartAndGoalCellsValid(startCell, goalCell))
    {
     
	        long t1,t2,t3;
	        t1=getCurrentTime();
            //timespec time1, time2, time3;
            /* take current time here */
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time1);

           /* vector<vector<float> > gridVal;
            gridVal.clear();

            gridVal.resize(height/mapReduceSize);
            for(int i=0; i<(height/mapReduceSize); i++)
                gridVal[i].resize(width/mapReduceSize);

            getGridVal(gridVal,getCellRowID(startCell), getCellColID(startCell),getCellRowID(goalCell), getCellColID(goalCell));
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time2);
             t2=getCurrentTime();
            
            
            irrtlog<<"+++++++++++++++++++++++++++++++"<<endl;
            for(int k=0; k<(height/mapReduceSize); k++)
            {
                for(int l=0; l<(width/mapReduceSize-1); l++)
                {
                    irrtlog<<gridVal[k][l]<<" ";
                }
                irrtlog<<gridVal[k][width/mapReduceSize]<<endl;
                //irrtlog<<endl;
            }
            irrtlog<<"-------------------------------"<<endl;*/
            
            

            myRRT.clearRRT();
            RRT temprrt(getCellRowID(startCell),getCellColID(startCell));
            myRRT=temprrt;

            //rrtPath=findCellPath(myRRT, startCell, goalCell, gridVal);
            rrtPath=findCellPath(myRRT, startCell, goalCell);
            //clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time3);
            t2=getCurrentTime();

            
            //cout<<"temperature field time="<<t2-t1<<endl;
            //irrtlog<<"temperature field time="<<t2-t1<<endl;
            //cout<<"planning time="<<t3-t2<<endl;
            //irrtlog<<"planning time="<<t3-t2<<endl;
            cout<<"total time="<<t2-t1<<endl;
    }
    else
    {
        ROS_WARN("------Not valid start or goal------");
        return false;
    }

    //if the global planner find a path
    if (rrtPath.size()>0)
    {
        convertToPlan(rrtPath,plan,goal,goalCell);

        float path_length = 0.0;

        std::vector<geometry_msgs::PoseStamped>::iterator it = plan.begin();

        geometry_msgs::PoseStamped last_pose;
        last_pose = *it;
        it++;
        irrtlog<<"路径位姿点："<<endl;
        for (; it!=plan.end(); ++it)
        {
            irrtlog<<"("<<(*it).pose.position.x<<","<<(*it).pose.position.y<<")"<<endl;
            path_length += hypot(  (*it).pose.position.x - last_pose.pose.position.x,
                                   (*it).pose.position.y - last_pose.pose.position.y );
            last_pose = *it;
        }
        cout <<"------The global path length: "<< path_length<< " meters------"<<endl;
        irrtlog <<"------The global path length: "<< path_length<< " meters------"<<endl;
    }
    else
    {
        ROS_WARN("------The planner failed to find a path, choose other goal position------");
        //return false;
    }

    publishPlan(plan);
    

    //cout<<"+++++++++++++++++++++++++++++++"<<endl;
    irrtlog<<"+++++++++++++++++++++++++++++++"<<endl;
    //cout<<"树中所有结点(格式：结点ID，该结点的父结点ID，x坐标值，y坐标值):"<<endl;
    irrtlog<<"树中所有结点(格式：结点ID，该结点的父结点ID，x坐标值，y坐标值):"<<endl;
    RRT::rrtNode  l;
    for(int m=0; m<myRRT.getTreeSize(); m++)
    {
        l=myRRT.getNode(m);
        //cout<<l.nodeID<<","<<l.parentID<<","<<l.posX* resolution+originX<<","<<l.posY* resolution+originY<<endl;
        irrtlog<<l.nodeID<<","<<l.parentID<<","<<l.posX* resolution+originX<<","<<l.posY* resolution+originY<<endl;
    }
    //cout<<"+++++++++++++++++++++++++++++++"<<endl;
    irrtlog<<"+++++++++++++++++++++++++++++++"<<endl;

    return !plan.empty();
}


void  rrt_planner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path)
{
    if (!initialized_)
    {
        ROS_ERROR(
            "This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if (!path.empty())
    {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++)
    {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}



//vector<int>   rrt_planner::findCellPath(RRT &myRRT, int startCell, int goalCell,vector<vector<float> > gridVal)  //使用温度场作为启发因子
vector<int>   rrt_planner::findCellPath(RRT &myRRT, int startCell, int goalCell) 
{
    vector<int>  rrtPath;

    vector<RRT::rrtNode> randNodes;

    bool addNodeResult = false, nodeToGoal = false;

    while(1)
    {
        randNodes.clear();
        //generateTempPoint(tempNode,width, height);  //一次产生一个随机结点
        generateRandNodes( myRRT,randNodes,width, height);  //一次产生一个随机结点数组

        //addNodeResult = addNewPointtoRRT(myRRT,tempNode,rrtStepSize);  //经典RRT方法添加新结点
        //addNodeResult = addNewNodetoRRT(myRRT,randNodes,getCellRowID(goalCell), getCellColID(goalCell),rrtStepSize,gridVal); //启发函数优化RRT方法
        addNodeResult = addNewNodetoRRT(myRRT,randNodes,getCellRowID(goalCell), getCellColID(goalCell),rrtStepSize); 
        if(addNodeResult)
        {
            //nodeToGoal = checkNodetoGoal(getCellRowID(goalCell), getCellColID(goalCell),tempNode);//经典RRT方法添加新结点

            RRT::rrtNode lastNode=myRRT.getNode(myRRT.getTreeSize()-1);
            nodeToGoal = checkNodetoGoal(getCellRowID(goalCell), getCellColID(goalCell),lastNode);//启发函数优化RRT方法

            if(nodeToGoal)
            {
                //path = myRRT.getRootToEndPath(tempNode.nodeID);//经典RRT方法得到路径

                rrtPath = myRRT.getRootToEndPath(myRRT.getTreeSize()-1);//启发函数优化RRT方法得到路径

                break;
            }
        }
    }

    cout<<"RRT nodeSum="<<myRRT.getTreeSize()<<endl;
    irrtlog<<"RRT nodeSum="<<myRRT.getTreeSize()<<endl;

    return rrtPath;
}

void rrt_planner::convertToPlan(vector<int>rrtPath,std::vector<geometry_msgs::PoseStamped>& plan, geometry_msgs::PoseStamped  goal,int goalCell)
{
    // convert the path
    for (int i = 0; i < rrtPath.size(); i++)
    {

        float x = 0.0;
        float y = 0.0;

        int tempNodeId=rrtPath[i];
        int cellIndex=getCellIndex(myRRT.getPosX(tempNodeId),myRRT.getPosY(tempNodeId));


        convertToCoordinate(cellIndex, x, y);

        //cout<<"--cell"<<index<<": x="<<x<<",y="<<y<<endl;
        //rrtlog<<"--cell"<<index<<": x="<<x<<",y="<<y<<endl;

        geometry_msgs::PoseStamped pose = goal;


        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = 0.0;

        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;

        plan.push_back(pose);
    }


    //处理最后一个结点，即是目标点goal
    float final_x = 0.0;
    float final_y = 0.0;
    int final_index = goalCell;
    convertToCoordinate(final_index, final_x, final_y);
    //cout<<"--cell"<<final_index<<": final_x="<<final_x<<",final_y="<<final_y<<endl;
    //rrtlog<<"--cell"<<final_index<<": final_x="<<final_x<<",final_y="<<final_y<<endl;

    geometry_msgs::PoseStamped final_pose = goal;
    final_pose.pose.position.x = final_x;
    final_pose.pose.position.y = final_y;
    final_pose.pose.position.z = 0.0;
    final_pose.pose.orientation.x = 0.0;
    final_pose.pose.orientation.y = 0.0;
    final_pose.pose.orientation.z = 0.0;
    final_pose.pose.orientation.w = 1.0;
    plan.push_back(final_pose);  //最后一个结点，即是目标点

}

bool rrt_planner::checkIfOnObstacles(RRT::rrtNode &tempNode)
{
    if((tempNode.posX<width)&&(tempNode.posY<height))
    {
        if(OGM[getCellIndex(tempNode.posX,tempNode.posY)])
            return true;
        else
            return false;
    }
    else
        return false;
}
bool rrt_planner::pointcheck(RRT::rrtNode const &m,RRT::rrtNode const &n)
{
    RRT::rrtNode test;
    float p=0.00;
    float dist=sqrt(pow(m.posX-n.posX,2)+pow(m.posY-n.posY,2));
    double theta=atan2(n.posY-m.posY,n.posX-m.posX);
    while(p<=1)
    {
        test.posX=(int)(m.posX+p*dist*cos(theta));
        test.posY=(int)(m.posY+p*dist*sin(theta));
        if(!checkIfOnObstacles(test))
        {
            return false;
	    break;
        }
        p=p+0.2;
    }
}


bool rrt_planner::isCellInsideMap(float x, float y)
{
    bool valid = true;

    if (x > (width * resolution) || y > (height * resolution))
        valid = false;

    return valid;
}

void rrt_planner::mapToWorld(double mx, double my, double& wx, double& wy)
{
    costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    wx = costmap->getOriginX() + mx * resolution;
    wy = costmap->getOriginY() + my * resolution;
}

//verify if the cell(i,j) is free
bool  rrt_planner::isFree(int i, int j)
{
    int CellID = getCellIndex(i, j);
    return OGM[CellID];

}

//verify if the cell(i,j) is free
bool  rrt_planner::isFree(int CellID)
{
    return OGM[CellID];
}

bool rrt_planner::isStartAndGoalCellsValid(int startCell,int goalCell)
{
    bool isvalid=true;
    bool isFreeStartCell=isFree(startCell);
    bool isFreeGoalCell=isFree(goalCell);
    if (startCell==goalCell)
    {
        cout << "------The Start and the Goal cells are the same...------" << endl;
        isvalid = false;
    }
    else
    {
        isvalid=(isFreeStartCell&&isFreeGoalCell);
    }
    return isvalid;
}

}
