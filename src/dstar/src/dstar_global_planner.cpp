#include <dstar_global_planner/dstar_global_planner.h>

#include <pluginlib/class_list_macros.h>

using namespace std;

PLUGINLIB_EXPORT_CLASS(dstar_global_planner::DStarPlannerROS, nav_core::BaseGlobalPlanner)

namespace dstar_global_planner{


DStarPlannerROS::DStarPlannerROS()
: costmap_(NULL),  dstar_planner_(), initialized_(false) {}

DStarPlannerROS::DStarPlannerROS(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id)
: costmap_(NULL),  dstar_planner_(), initialized_(false) {
    //initialize the planner
    initialize(name, costmap, frame_id);
}

DStarPlannerROS::~DStarPlannerROS() {
    delete dstar_planner_;
    delete spline_smoother_;
}

void DStarPlannerROS::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) {
    initialize(name, costmap_ros->getCostmap(), costmap_ros->getGlobalFrameID());
}


void DStarPlannerROS::initialize(const std::string& name, costmap_2d::Costmap2D* costmap, std::string frame_id){

    if(!initialized_){

        costmap_ = costmap;
        global_frame_id_ = frame_id;

        dstar_planner_ = new Dstar();
        dstar_planner_->init(0, 0, 10, 10); // First initialization

        spline_smoother_ = new PathSplineSmoother();

        ros::NodeHandle private_nh("~/" + name);

        plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

        private_nh.param("smooth", smooth, true);

        //get the tf prefix
        ros::NodeHandle prefix_nh;
        tf_prefix_ = tf::getPrefixParam(prefix_nh);

        initialized_ = true;
    }
    else
        ROS_WARN("This planner has already been initialized, you can't call it twice, doing nothing");
}

/// This function can not be used from Map3D because it expects an integer cell value!
/// it is preferred to leave the function as it is in Map3D (and not change it to doubles)
void DStarPlannerROS::mapToWorld(double mx, double my, double& wx, double& wy) {
    wx = costmap_->getOriginX() + mx * costmap_->getResolution();
    wy = costmap_->getOriginY() + my * costmap_->getResolution();
}

bool DStarPlannerROS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){

    if(!initialized_){
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return false;
    }


    std::string global_frame = global_frame_id_;
    //until tf can handle transforming things that are way in the past... we'll require the goal to be in our global frame

    if (tf::resolve(tf_prefix_, goal.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {

        ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, goal.header.frame_id).c_str());
        return false;

    }


    if (tf::resolve(tf_prefix_, start.header.frame_id) != tf::resolve(tf_prefix_, global_frame)) {

        ROS_ERROR("The start pose passed to this planner must be in the %s frame.  It is instead in the %s frame.", tf::resolve(tf_prefix_, global_frame).c_str(), tf::resolve(tf_prefix_, start.header.frame_id).c_str());
        return false;
    }


    /// Grid Planning
    std::vector< geometry_msgs::PoseStamped > grid_plan;

    ROS_DEBUG("Start To plan");

    if(getPlan(start, goal, grid_plan)){

        plan.clear();

        for (size_t i = 0; i < grid_plan.size(); i++) {
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = global_frame_id_; /// Check in which frame to publish
            pose.pose.position.x = grid_plan[i].pose.position.x;
            pose.pose.position.y = grid_plan[i].pose.position.y;
            pose.pose.position.z = 0;
            pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,grid_plan[i].pose.position.z);
            plan.push_back(pose);

        }
        ROS_DEBUG("DSTAR_LITE Path found");
        publishPlan(grid_plan);
        return true;

        }else{
            ROS_WARN("NO PATH FOUND FROM THE D* Lite PLANNER");
            return false;
        }

}


void DStarPlannerROS::clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my) {
    if (!initialized_) {
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //set the associated costs in the cost map to be free
    costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
}

bool DStarPlannerROS::getPlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {

    unsigned int mx_start, my_start;
    if(!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start)){
        ROS_WARN("The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?");
        return false;
    }


    dstar_planner_->updateStart(mx_start, my_start);

    unsigned int mx_goal, my_goal;
    if(!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal)){
        ROS_WARN("The goal sent to the planner is off the global costmap. Planning will always fail to this goal.");
        return false;
    }

    dstar_planner_->updateGoal(mx_goal, my_goal);

    //clear the starting cell within the costmap because we know it can't be an obstacle
    tf::Stamped<tf::Pose> start_pose;
    tf::poseStampedMsgToTF(start, start_pose);
    clearRobotCell(start_pose, mx_start, my_start);


    unsigned char* grid = costmap_->getCharMap();
    for(int x=0; x<(int)costmap_->getSizeInCellsX(); x++){
        for(int y=0; y<(int)costmap_->getSizeInCellsY(); y++){
            int index = costmap_->getIndex(x,y);

            double c = (double)grid[index];

            if( c >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
                dstar_planner_->updateCell(x, y, -1);
            else if (c == costmap_2d::FREE_SPACE){
                dstar_planner_->updateCell(x, y, 1);
            }else
            {
                dstar_planner_->updateCell(x, y, c);
            }

        }
    }


    dstar_planner_->replan();

    ROS_DEBUG("Get Path");
    /// 3. Get Path
    list<state> path = dstar_planner_->getPath();



    /// smooth plan
    ///
    /// 4. Returning the path generated
    plan.clear();
    plan.push_back(start);

    if(!smooth){

        double costmap_resolution = costmap_->getResolution();
        double origin_costmap_x = costmap_->getOriginX();
        double origin_costmap_y = costmap_->getOriginY();

        std::list<state>::const_iterator iterator;

        for (iterator = path.begin(); iterator != path.end(); ++iterator) {

            state node = *iterator;

            geometry_msgs::PoseStamped next_node;
            next_node.header.stamp = ros::Time::now();
            next_node.header.frame_id = global_frame_id_;
            next_node.pose.position.x = (node.x+0.5)*costmap_resolution + origin_costmap_x;
            next_node.pose.position.y = (node.y+0.5)*costmap_resolution + origin_costmap_y;

            plan.push_back(next_node);

        }

    }else{

        vector<RealPoint> path_smoothed = SmoothPlan(path);
        int size_path_smoothed = (int)(path_smoothed.size());
        ROS_DEBUG("Size of the smoothed path %d", size_path_smoothed);
        for (int j=0; j<size_path_smoothed; j++){

            geometry_msgs::PoseStamped next_node;
            next_node.header.stamp = ros::Time::now();
            next_node.header.frame_id = global_frame_id_;

            next_node.pose.position.x = path_smoothed[j].x;
            next_node.pose.position.y = path_smoothed[j].y;

            next_node.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, path_smoothed[j].theta);

            plan.push_back(next_node);
        }

    }



    return true;
}

void DStarPlannerROS::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path){
    if(!initialized_){
        ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
        return;
    }

    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    if(!path.empty())
    {
        gui_path.header.frame_id = path[0].header.frame_id;
        gui_path.header.stamp = path[0].header.stamp;
    }

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for(unsigned int i=0; i < path.size(); i++){
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}


/// ==================================================================================
/// SmoothPlan(list<state> path)
/// smoothing using splines
/// ==================================================================================
vector<RealPoint> DStarPlannerROS::SmoothPlan(list<state> path){

        ROS_DEBUG("SmoothPlan / getting costmap infos");
        double costmap_resolution = costmap_->getResolution();
        double origin_costmap_x = costmap_->getOriginX();
        double origin_costmap_y = costmap_->getOriginY();


        /// copying the path in a different format
        int initial_path_size = (int)path.size();
        vector<RealPoint> input_path;
        input_path.clear();
        if(initial_path_size == 0)
        {
            ROS_ERROR("Path not valid for smoothing, returning");
            return input_path;
        }




        ROS_DEBUG("SmoothPlan, filling the points");


        std::list<state>::const_iterator iterator;
        double t, old_x, old_y, old_th, dt;
        dt = 0.1;
        int cnt = 0;
        for (iterator = path.begin(); iterator != path.end(); ++iterator) {

            state node = *iterator;

            /// giving as input path the cartesian path
            double x = (node.x+0.5)*costmap_resolution + origin_costmap_x;
            double y = (node.y+0.5)*costmap_resolution + origin_costmap_y;


            if(cnt>0){

                t = dt;

                while(t<1){
                    RealPoint p_new;
                    p_new.x = ( x - old_x)*t + old_x ;
                    p_new.y = ( y - old_y)*t + old_y ;
                    p_new.theta = 0;
                    input_path.push_back(p_new);
                    ROS_DEBUG("Adding point %f %f ", p_new.x, p_new.y);
                    t=t+dt;
                }
            }else{

                RealPoint p;
                p.x = x;
                p.y = y;
                p.theta = 0;
                input_path.push_back(p);
                ROS_DEBUG("Adding Initial point %f %f of a segment ", x, y);

            }

            old_x = x;
            old_y = y;
            old_th = 0;
            cnt++;

        }

                // do not smooth if the path has not enough points
        if(initial_path_size<3){
          ROS_DEBUG("Returning path, without smoothing it");
          return input_path;

        }

        ROS_DEBUG("SmoothPlan, Providing the path to the smoother");
        spline_smoother_->readPathFromStruct(input_path);
        ROS_DEBUG("SmoothPlan, Filtering path");
        spline_smoother_->filterPath(1);
        ROS_DEBUG("SmoothPlan, Smoothing path");
        spline_smoother_->smoothWhileDistanceLessThan(0.05,1.01);
        ROS_DEBUG("SmoothPlan, getting path");
        vector<RealPoint> smooth_path = spline_smoother_->getSmoothPath();


        return smooth_path;

}

}


