#ifndef DSTAR_PLANNER_CPP
#define DSTAR_PLANNER_CPP

#include <cstdlib>
#include <fstream>
#include <iostream>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <dstar_global_planner/Dstar.h>
#include <dstar_global_planner/pathSplineSmoother/pathSplineSmoother.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>

#include <nav_core/base_global_planner.h>


namespace dstar_global_planner{

  /**
   * @class DStarPlannerROS
   * @brief Plugin to the ros dstar_global_planner. Implements a wrapper for DStarPlanner Method
   */
  class DStarPlannerROS : public nav_core::BaseGlobalPlanner{

    public:
      /**
       * @brief Default constructor for the ros wrapper
       */
      DStarPlannerROS();

      /**
       * @brief Constructs the ros wrapper
       * @param name The name to give this instance of the dstar planner
       * @param tf A pointer to a transform listener
       * @param costmap The cost map to use for assigning costs to trajectories
       */
      DStarPlannerROS(std::string name, costmap_2d::Costmap2D* costmap, std::string frame_id);


      /**
       * @brief  Destructor for the wrapper
       */
      ~DStarPlannerROS();

      /**
          * @brief  Initialization function for the NavFnROS object
          * @param  name The name of this planner
          * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
      void initialize(const std::string& name, costmap_2d::Costmap2D* costmap, std::string frame_id);


      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose
       * @param goal The goal pose
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start,
                    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      bool getPlan(const geometry_msgs::PoseStamped& start,
                   const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);


  protected:

      /**
       * @brief  Publish a path for visualization purposes
       */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
      vector<RealPoint> SmoothPlan(list<state> path);

      std::string global_frame_id_;

      costmap_2d::Costmap2D* costmap_;

      ros::Publisher plan_pub_;


      bool initialized_;
      bool smooth;

    private:

      Dstar *dstar_planner_; ///<  Dstar planner
      PathSplineSmoother *spline_smoother_;
      std::string tf_prefix_;
      int smooth_size_;

      void mapToWorld(double mx, double my, double& wx, double& wy);

      void clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);








  };
}
#endif


