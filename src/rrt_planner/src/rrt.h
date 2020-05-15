#ifndef rrt_h
#define rrt_h

#include <vector>
#include <queue>
#include <math.h>
#include <cstddef>
#include <iostream>

using namespace std;

/**
* default constructor for RRT class
* initializes source to 0,0
* adds sorce to rrtTree
*/

float infinity = std::numeric_limits< float >::infinity();
//int rrtStepSize = 13;  //定义生长步长(turtlebot_stage默认地图中能取到的最大值为13)
int rrtStepSize=5;  //RRT随机树步长(步长必须大于等于mapReduceSize，否则找不到解)
int randNodeNum=1;  //一次生成随机结点的个数

int  mapReduceSize=5;  //地图分辨率降低倍数
int  iterationTime=4000;  //模拟温度场扩散迭代次数
float initialTemperature=20000.0f;  //温度场初始温度值

class RRT
{

public:

    RRT();
    RRT(float input_PosX, float input_PosY);

    struct rrtNode
    {
        int nodeID;
        float posX;
        float posY;
        int parentID;
        float depth;//节点深度，同时也是从根结点到该结点的路径长度
        vector<int> children;
    };

    vector<rrtNode> getTree();
    void setTree(vector<rrtNode> input_rrtTree);
    int getTreeSize();

    void addNewNode(rrtNode node);
    rrtNode removeNode(int nodeID);
    rrtNode getNode(int nodeID);

    float getPosX(int nodeID);
    float getPosY(int nodeID);
    void setPosX(int nodeID, float input_PosX);
    void setPosY(int nodeID, float input_PosY);

    rrtNode getParent(int nodeID);
    void setParentID(int nodeID, int parentID);

    void addChildID(int nodeID, int childID);
    vector<int> getChildren(int nodeID);
    int getChildrenSize(int nodeID);

    int getNearestNodeID(float X, float Y);
    int getNearestNodeID(float X, float Y, float goalX, float goalY,  int rrtStepSize);  //启发式方法求nearestNodeID

    vector<int> getRootToEndPath(int endNodeID);
    vector<int> getStartToEndPath(int startNodeID,int endNodeID);

    void clearRRT();

private:
    vector<rrtNode> rrtTree;

    float getEuclideanDistance(float sourceX, float sourceY, float destinationX, float destinationY);
};

RRT::RRT()
{
    RRT::rrtNode newNode;
    newNode.posX = 0;
    newNode.posY = 0;
    newNode.parentID = 0;
    newNode.nodeID = 0;
    newNode.depth=0;  //根结点深度为0
    rrtTree.push_back(newNode);
}

/**
* default constructor for RRT class
* initializes source to input X,Y
* adds sorce to rrtTree
*/
RRT::RRT(float input_PosX, float input_PosY)
{
    RRT::rrtNode newNode;
    newNode.posX = input_PosX;
    newNode.posY = input_PosY;
    newNode.parentID = 0;
    newNode.nodeID = 0;
    newNode.depth=0;
    rrtTree.push_back(newNode);

    //cout<<"---RRT init successfully. posX="<<newNode.posX<<",posY="<<newNode.posY<<"---"<<endl;
}

/**
* Returns the current RRT tree
* @return RRT Tree
*/
vector<RRT::rrtNode> RRT::getTree()
{
    return rrtTree;
}

/**
* For setting the rrtTree to the inputTree
* @param rrtTree
*/
void RRT::setTree(vector<RRT::rrtNode> input_rrtTree)
{
    rrtTree = input_rrtTree;
}

/**
* to get the number of nodes in the rrt Tree
* @return tree size
*/
int RRT::getTreeSize()
{
    return rrtTree.size();
}

/**
* adding a new node to the rrt Tree
*/
void RRT::addNewNode(RRT::rrtNode node)
{
    rrtTree.push_back(node);
}

/**
* removing a node from the RRT Tree
* @return the removed tree
*/
RRT::rrtNode RRT::removeNode(int id)
{
    RRT::rrtNode tempNode = rrtTree[id];
    rrtTree.erase(rrtTree.begin()+id);
    return tempNode;
}

/**
* getting a specific node
* @param node id for the required node
* @return node in the rrtNode structure
*/
RRT::rrtNode RRT::getNode(int id)
{
    return rrtTree[id];
}

/**
* return a node from the rrt tree nearest to the given point
* @param X position in X cordinate
* @param Y position in Y cordinate
* @return nodeID of the nearest Node
*/
int RRT::getNearestNodeID(float X, float Y)
{
    int i, returnID=-1;
    float distance = infinity, tempDistance;

    for(i=0; i<(this->getTreeSize()); i++)
    {
        tempDistance = pow(getPosX(i) - X,2) + pow(getPosY(i) - Y,2);//使用平方距离作为比较值，减少开方运算
        if (tempDistance < distance)
        {
            distance = tempDistance;
            returnID = i;
        }
    }

    return returnID;
}

/**
* returns X coordinate of the given node
*/
float RRT::getPosX(int nodeID)
{
    return rrtTree[nodeID].posX;
}

/**
* returns Y coordinate of the given node
*/
float RRT::getPosY(int nodeID)
{
    return rrtTree[nodeID].posY;
}

/**
* set X coordinate of the given node
*/
void RRT::setPosX(int nodeID, float input_PosX)
{
    rrtTree[nodeID].posX = input_PosX;
}

/**
* set Y coordinate of the given node
*/
void RRT::setPosY(int nodeID, float input_PosY)
{
    rrtTree[nodeID].posY = input_PosY;
}

/**
* returns parentID of the given node
*/
RRT::rrtNode RRT::getParent(int id)
{
    return rrtTree[rrtTree[id].parentID];
}

/**
* set parentID of the given node
*/
void RRT::setParentID(int nodeID, int parentID)
{
    rrtTree[nodeID].parentID = parentID;
}

/**
* add a new childID to the children list of the given node
*/
void RRT::addChildID(int nodeID, int childID)
{
    rrtTree[nodeID].children.push_back(childID);
}

/**
* returns the children list of the given node
*/
vector<int> RRT::getChildren(int id)
{
    return rrtTree[id].children;
}

/**
* returns number of children of a given node
*/
int RRT::getChildrenSize(int nodeID)
{
    return rrtTree[nodeID].children.size();
}

/**
* returns euclidean distance between two set of X,Y coordinates
*/
float RRT::getEuclideanDistance(float sourceX, float sourceY, float destinationX, float destinationY)
{
    return sqrt(pow(destinationX - sourceX,2) + pow(destinationY - sourceY,2));
}

/**
* returns path from root to end node
* @param endNodeID of the end node
* @return path containing ID of member nodes in the vector form
*/
vector<int> RRT::getRootToEndPath(int endNodeID)
{
    vector<int> path;
    path.push_back(endNodeID);
    while(rrtTree[path.front()].nodeID != 0)
    {
        //std::cout<<rrtTree[path.front()].nodeID<<endl;
        path.insert(path.begin(),rrtTree[path.front()].parentID);
    }
    return path;
}

vector<int> RRT::getStartToEndPath(int startNodeID,int endNodeID)
{
    vector<int> path;
    path.push_back(endNodeID);
    while(rrtTree[path.front()].nodeID != startNodeID)
    {
        //std::cout<<rrtTree[path.front()].nodeID<<endl;
        path.insert(path.begin(),rrtTree[path.front()].parentID);
    }
    return path;
}

void RRT::clearRRT()
{
    rrtTree.clear();
    //cout<<"clear the rrtTree successfully."<<endl;
}

#endif
