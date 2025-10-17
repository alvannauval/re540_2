#include "control_space_planner/control_space_planner_node.hpp"

/* ----- Class Functions ----- */
MotionPlanner::MotionPlanner(ros::NodeHandle& nh) : nh_(nh)
{
  // Subscriber
  subOccupancyGrid = nh.subscribe("/map/local_map/obstacle",1, &MotionPlanner::CallbackOccupancyGrid, this);
  subEgoOdom = nh.subscribe("/odom",1, &MotionPlanner::CallbackEgoOdom, this);
  subGoalPoint = nh.subscribe("/move_base_simple/goal",1, &MotionPlanner::CallbackGoalPoint, this);
  subGlobalPath = nh.subscribe("/path/global_path", 1, &MotionPlanner::CallbackGlobalPath, this);
  // Publisher
  pubSelectedMotion = nh_.advertise<sensor_msgs::PointCloud2>("/points/selected_motion", 1, true);
  pubMotionPrimitives = nh_.advertise<sensor_msgs::PointCloud2>("/points/motion_primitives", 1, true);
  pubCommand = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1, true);
  pubTruncTarget = nh_.advertise<geometry_msgs::PoseStamped>("/car/trunc_target", 1, true);
  
};

MotionPlanner::~MotionPlanner() 
{    
    ROS_INFO("MotionPlanner destructor.");
}

/* ----- ROS Functions ----- */

void MotionPlanner::CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg)
{
  this->localMap = msg;
  this->origin_x = msg.info.origin.position.x;
  this->origin_y = msg.info.origin.position.y;
  this->frame_id = msg.header.frame_id;
  this->mapResol = msg.info.resolution;
  bGetMap = true;
}

void MotionPlanner::CallbackGoalPoint(const geometry_msgs::PoseStamped& msg)
{
  this->goalPose = msg;
  // - position
  this->goal_x = msg.pose.position.x;
  this->goal_y = msg.pose.position.y;
  // - orientation
  // -- quaternion to RPY (global)
  tf2::Quaternion goal;
  double goal_roll, goal_pitch, goal_yaw;
  // --- copy quaternion from odom
  tf2::convert(msg.pose.orientation, goal);
  // --- get roll pitch yaw
  tf2::Matrix3x3 m_goal(goal);
  m_goal.getRPY(goal_roll, goal_pitch, goal_yaw);
  this->goal_yaw = goal_yaw;
  
  this->bGetGoal = true;
}

void MotionPlanner::CallbackEgoOdom(const nav_msgs::Odometry& msg)
{
  this->egoOdom = msg;
  // - position
  this->ego_x = msg.pose.pose.position.x;
  this->ego_y = msg.pose.pose.position.y;
  // - orientation
  // -- quaternion to RPY (global)
  tf2::Quaternion ego;
  double ego_roll, ego_pitch, ego_yaw;
  // --- copy quaternion from odom
  tf2::convert(msg.pose.pose.orientation, ego);
  // --- get roll pitch yaw
  tf2::Matrix3x3 m_ego(ego);
  m_ego.getRPY(ego_roll, ego_pitch, ego_yaw);
  this->ego_yaw = ego_yaw;
  
  this->bGetEgoOdom = true;
}

void MotionPlanner::PublishSelectedMotion(std::vector<Node> motionMinCost)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  // publish selected motion primitive as point cloud
  for (auto motion : motionMinCost) {
    pcl::PointXYZI pointTmp;
    pointTmp.x = motion.x;
    pointTmp.y = motion.y;
    cloud_in_ptr->points.push_back(pointTmp);
  }

  sensor_msgs::PointCloud2 motionCloudMsg;
  pcl::toROSMsg(*cloud_in_ptr, motionCloudMsg);
  motionCloudMsg.header.frame_id = this->frame_id;
  motionCloudMsg.header.stamp = ros::Time::now();
  pubSelectedMotion.publish(motionCloudMsg);
}

void MotionPlanner::PublishMotionPrimitives(std::vector<std::vector<Node>> motionPrimitives)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  // publish motion primitives as point cloud
  for (auto& motionPrimitive : motionPrimitives) {
    double cost_total = motionPrimitive.back().cost_total;
    for (auto motion : motionPrimitive) {
      pcl::PointXYZI pointTmp;
      pointTmp.x = motion.x;
      pointTmp.y = motion.y;
      pointTmp.z = cost_total;
      pointTmp.intensity = cost_total;
      cloud_in_ptr->points.push_back(pointTmp);
    }
  }
  
  sensor_msgs::PointCloud2 motionPrimitivesCloudMsg;
  pcl::toROSMsg(*cloud_in_ptr, motionPrimitivesCloudMsg);
  motionPrimitivesCloudMsg.header.frame_id = this->frame_id;
  motionPrimitivesCloudMsg.header.stamp = ros::Time::now();
  pubMotionPrimitives.publish(motionPrimitivesCloudMsg);
}

void MotionPlanner::PublishCommand(std::vector<Node> motionMinCost)
{

    geometry_msgs::Twist command;
    
    if (motionMinCost.empty()) {
        command.linear.x = 0.0;
        command.linear.y = 0.0;
        command.angular.z = 0.0;
        pubCommand.publish(command);
        return; 
    }
    const Node& targetNode = motionMinCost.back(); 
    
    double target_x_body = targetNode.x;
    double target_y_body = targetNode.y;
    double dist_to_target = std::sqrt(target_x_body*target_x_body + target_y_body*target_y_body);

    command.linear.x = 0.0;
    command.linear.y = 0.0;
    command.angular.z = 0.0;

    if (dist_to_target > 1e-4) {
        double Kp_Speed = 1.0; 
        double Kp_Angular = 0.1; 
        
        double proportional_speed = Kp_Speed * dist_to_target;
        double desired_yaw_to_pos = std::atan2(target_y_body, target_x_body);
        double yaw_error = desired_yaw_to_pos;

        double actual_speed_magnitude = std::min(proportional_speed, this->MOTION_VEL); 

        command.linear.x = actual_speed_magnitude * (target_x_body / dist_to_target);
        command.linear.y = actual_speed_magnitude * (target_y_body / dist_to_target);
        command.angular.z = Kp_Angular * yaw_error;
        
        command.linear.x = std::max(-this->MOTION_VEL, std::min(command.linear.x, this->MOTION_VEL));
        command.linear.y = std::max(-this->MOTION_VEL, std::min(command.linear.y, this->MOTION_VEL));
        command.angular.z = std::max(-1.0, std::min(command.angular.z, 1.0));
    }


    // arrival rule
    if (bGetGoal && bGetLocalNode) {
      double distToGoal = sqrt(this->localNode.x*this->localNode.x + this->localNode.y*this->localNode.y);

      if (distToGoal < this->ARRIVAL_THRES) {
        command.angular.z = 0.0;
        command.linear.x = 0.0;
      }
      else if (distToGoal < 2*this->ARRIVAL_THRES) {
        command.linear.x = command.linear.x * pow(distToGoal / 2*this->ARRIVAL_THRES, 3.0);
      }
    }

    pubCommand.publish(command);
}

/* ----- Algorithm Functions ----- */

void MotionPlanner::Plan()
{
  // Compute current LOS target pose
  if (this->bGetEgoOdom && this->bGetGoal) {
    Node goalNode;
    goalNode.x = this->goal_x;
    goalNode.y = this->goal_y;
    goalNode.yaw = this->goal_yaw;
    localNode = GlobalToLocalCoordinate(goalNode, this->egoOdom);
    // - compute truncated local node pose within local map
    Node tmpLocalNode;
    memcpy(&tmpLocalNode, &localNode, sizeof(struct Node));
    tmpLocalNode.x = std::max(this->mapMinX, std::min(tmpLocalNode.x, this->mapMaxX));
    tmpLocalNode.y = std::max(this->mapMinY, std::min(tmpLocalNode.y, this->mapMaxY));
    truncLocalNode = tmpLocalNode;

    // for debug
    geometry_msgs::PoseStamped localPose = GlobalToLocalCoordinate(this->goalPose, this->egoOdom);
    localPose.header.frame_id = "base_link";
    pubTruncTarget.publish(localPose);

    this->bGetLocalNode = true;
  }
  // Motion generation
  motionCandidates = GenerateMotionPrimitives(this->localMap);
  
  // Select motion
  std::vector<Node> motionMinCost = SelectMotion(motionCandidates);

  // Publish data
  PublishData(motionMinCost, motionCandidates);
}

std::vector<std::vector<Node>> MotionPlanner::GenerateMotionPrimitives(nav_msgs::OccupancyGrid localMap)
{
  /*
    TODO: Generate motion primitives
    - you can change the below process if you need.
    - you can calculate cost of each motion if you need.
    - TODO: future needs: reverse motion
  */


  // initialize motion primitives
  std::vector<std::vector<Node>> motionPrimitives;

  // compute params w.r.t. uncertainty
  int num_candidates = this->MAX_DELTA*2 / this->DELTA_RESOL; // *2 for considering both left/right direction

  // max progress of each motion
  double maxProgress = this->MAX_PROGRESS;
  for (int i=0; i<num_candidates+1; i++) {
    // current steering delta
    double angle_delta = this->MAX_DELTA - i * this->DELTA_RESOL;

    // init start node
    Node startNode(0, 0, 0, 0, angle_delta, 0, 0, 0, -1, false);
    
    // rollout to generate motion
    std::vector<Node> motionPrimitive = RolloutMotion(startNode, maxProgress, this->localMap);

    // add current motionPrimitive
    motionPrimitives.push_back(motionPrimitive);
  }

  return motionPrimitives;
}

std::vector<Node> MotionPlanner::RolloutMotion(Node startNode,
                                              double maxProgress,
                                              nav_msgs::OccupancyGrid localMap)
{
  /*
    Rollout to generate a motion primitive based on the current steering angle
    - calculate cost terms here if you need
    - check collision / sensor range if you need
    1. Update motion node using current steering angle delta based on the vehicle kinematics equation.
    2. collision checking
    3. range checking
  */

  // Initialize motionPrimitive
  std::vector<Node> motionPrimitive;

  // Check collision and compute traversability cost for each motion node of primitive (in planner coordinate)
  Node currMotionNode(startNode.x, startNode.y, 0, 0, startNode.delta, 0, 0, 0, -1, false);
  double progress = this->DIST_RESOL;

  // for compute closest distance toward goal point. You can use in SelectMotion function to calculate goal distance cost
  double minDistGoal = 987654321;
  if (this->bGetLocalNode) {
    minDistGoal = sqrt((startNode.x-truncLocalNode.x)*(startNode.x-truncLocalNode.x) +
                       (startNode.y-truncLocalNode.y)*(startNode.y-truncLocalNode.y));
  }

  //! 1. Update motion node using current steering angle delta based on the vehicle kinematics equation
  // - while loop until maximum progress of a motion

  // Loop for rollout
  while (progress < maxProgress) {
    double dt = this->TIME_RESOL;
    double dx = this->MOTION_VEL * cos(currMotionNode.yaw) * dt;
    double dy = this->MOTION_VEL * sin(currMotionNode.yaw) * dt;
    double d_omega = currMotionNode.delta;

    // x_t+1   := x_t + x_dot * dt
    // y_t+1   := y_t + y_dot * dt
    // yaw_t+1 := yaw_t + yaw_dot * dt
    currMotionNode.x += dx;
    currMotionNode.y += dy;
    currMotionNode.yaw += d_omega * dt;

    // ROS_INFO("Current Motion Node: Progress = %.2f, x=%.2f, y=%.2f, yaw=%.2f, delta=%.2f", progress, currMotionNode.x, currMotionNode.y, currMotionNode.yaw, currMotionNode.delta);
    
    // Calculate minimum distance toward goal
    if (this->bGetLocalNode) {
      double distGoal = sqrt((currMotionNode.x-truncLocalNode.x)*(currMotionNode.x-truncLocalNode.x) +
                             (currMotionNode.y-truncLocalNode.y)*(currMotionNode.y-truncLocalNode.y));
      if (minDistGoal > distGoal) {
        minDistGoal = distGoal;
      }
    }
    currMotionNode.minDistGoal = minDistGoal; // save current minDistGoal at current node
    
    // collision/range chekcing with "lookahead" concept
    // - lookahead point
    // double aheadYaw = currMotionNode.yaw + this->MOTION_VEL * tan(startNode.delta) / this->WHEELBASE * this->TIME_RESOL;
    double aheadYaw = currMotionNode.yaw;
    double aheadX = currMotionNode.x + this->MOTION_VEL * cos(aheadYaw) * this->TIME_RESOL;
    double aheadY = currMotionNode.y + this->MOTION_VEL * sin(aheadYaw) * this->TIME_RESOL;

    //! 2. collision checking
    // - local to map coordinate transform
    Node collisionPointNode(currMotionNode.x, currMotionNode.y, 0, currMotionNode.yaw, currMotionNode.delta, 0, 0, 0, -1, false);
    Node collisionPointNodeMap = LocalToPlannerCordinate(collisionPointNode);
    // ROS_INFO("MASUK 50");
    if (CheckCollision(collisionPointNodeMap, localMap)) {
      // ROS_INFO("MASUK 100")/;
      // - do some process when collision occurs.
      // - you can save collision information & calculate collision cost here.
      // - you can break and return current motion primitive or keep generate rollout.

      // Check if the primitive has any valid nodes saved before the collision point
      if (!motionPrimitive.empty()) {
          // ROS_INFO("MASUK 200");
          // 1. Set the collision flag on the last *valid* node
          motionPrimitive.back().collision = true;
          
          // 2. Assign a massive collision cost 
          motionPrimitive.back().cost_colli = 999999; 
      }

        return motionPrimitive;
    }
    currMotionNode.cost_colli = 0.0;
    currMotionNode.collision = false;


    //! 3. range checking
    // double LOS_DIST = std::sqrt(currMotionNode.x * currMotionNode.x + currMotionNode.y * currMotionNode.y);
    // double LOS_YAW = std::atan2(currMotionNode.y, currMotionNode.x);

    double LOS_DIST = std::sqrt(currMotionNode.x * currMotionNode.x + currMotionNode.y * currMotionNode.y);
    double LOS_YAW = std::atan2(currMotionNode.y, currMotionNode.x);

    if (LOS_DIST > this->MAX_SENSOR_RANGE || std::abs(LOS_YAW) > this->FOV * 0.5) {
      return motionPrimitive;
    }

    // append collision-free motion in the current motionPrimitive
    currMotionNode.idx = motionPrimitive.size();
    motionPrimitive.push_back(currMotionNode);

    // update progress of motion
    progress += this->DIST_RESOL;
  }
  
  // return current motion
  return motionPrimitive;
}


std::vector<Node> MotionPlanner::SelectMotion(std::vector<std::vector<Node>> motionPrimitives)
{
  /*
    TODO: select the minimum cost motion primitive
  
    1. Calculate cost terms
    2. Calculate total cost (weighted sum of all cost terms)
    3. Compare & Find minimum cost (double minCost) & minimum cost motion (std::vector<Node> motionMinCost)
    4. Return minimum cost motion
  */

  double minCost = 9999999;
  std::vector<Node> motionMinCost; // initialize as odom

  // check size of motion primitives
  if (motionPrimitives.size() != 0) {
    // Iterate all motion primitive (motionPrimitive) in motionPrimitives
    for (auto& motionPrimitive : motionPrimitives) {
      if (motionPrimitive.empty()) {
          continue; 
      }
      //! 1. Calculate cost terms
      const Node& startNode = motionPrimitive.front();
      const Node& endNode = motionPrimitive.back();

      double cost_goal_dist = endNode.minDistGoal; // Goal distance cost
      double cost_control = std::abs(startNode.delta); // Steering control cost
      double cost_collision = endNode.cost_colli; // Collision cost
      double cost_length_penalty = this->MAX_PROGRESS - (endNode.idx + 1) * this->DIST_RESOL; // Progress cost (the longer the progress, the lower the cost)
      // ROS_INFO("endNode.idx: %d", endNode.idx);

      //! 2. Calculate total cost ex) collision cost, goal distance, goal direction, progress cost, steering cost....
      double cost_total = 
          (this->W_COST_TRAVERSABILITY * cost_collision) +  // Prioritize safety (highest weight)
          (this->W_COST_DIRECTION * cost_goal_dist) +      // Drive toward goal
          (this->W_COST_CONTROL * cost_control) +                           // Keep turns smooth (unit weight)
          (this->W_COST_LENGTH_PENALTY * cost_length_penalty * 0);       // Penalize incomplete paths (medium weight)


          // Store the final cost in the last node for visualization (optional but useful)
      motionPrimitive.back().cost_total = cost_total;

      ROS_INFO("Candidate Node no %d: target_x=%.2f, target_y=%.2f, delta=%.2f, cost_control=%.2f, cost_colli=%.2f, cost_goal_dist=%.2f, cost_length_penalty=%.2f, cost_total=%.2f", 
               motionPrimitive.front().idx,
               endNode.x, endNode.y, startNode.delta, 
               cost_control, cost_collision, cost_goal_dist, cost_length_penalty,
               cost_total);

      //! 3. Compare & Find minimum cost & minimum cost motion
      if (cost_total < minCost) {
          motionMinCost = motionPrimitive;
          minCost = cost_total;
      }
    }
  }
  //! 4. Return minimum cost motion
  return motionMinCost;
}

/* ----- Util Functions ----- */

bool MotionPlanner::CheckCollision(Node goalNodePlanner, nav_msgs::OccupancyGrid localMap)
{
  /*
    check collision of the current node
    - the position x of the node should be in a range of [0, map width]
    - the position y of the node should be in a range of [0, map height]
    - check all map values within the inflation area of the current node
  */

  for (int i = 0; i < this->INFLATION_SIZE; i++) {
    for (int j = 0; j < this->INFLATION_SIZE; j++) {
      // Calculate the map coordinates (in cells) of the point to check
      // goalNodePlanner.x and y are the center, already in cell indices
      int tmp_x = (int)goalNodePlanner.x + i - 0.5*this->INFLATION_SIZE; 
      int tmp_y = (int)goalNodePlanner.y + j - 0.5*this->INFLATION_SIZE;

      if (tmp_x >= 0 && tmp_x < localMap.info.width && tmp_y >= 0 && tmp_y < localMap.info.height) {
        // Calculate the 1D index for the map.data array (OccupancyGrid is row-major: index = y * width + x)
        int map_index = tmp_y * localMap.info.width + tmp_x; 
        
        // Get the occupancy value
        int16_t map_value = static_cast<int16_t>(localMap.data[map_index]);
        
        // Check for collision: 
        // If map_value > OCCUPANCY_THRES (confirmed obstacle) 
        // OR map_value < 0 (unknown space, usually treated as an obstacle)
        if (map_value > this->OCCUPANCY_THRES || map_value < 0) {
          return true; // Collision detected
        }
      }
    }
  }

  return false;
  
}

bool MotionPlanner::CheckRunCondition()
{
  if (this->bGetMap && this->bGetGoal) {
  // if (this->bGetMap) {
    return true;
  }
  else {
    std::cout << "Run condition is not satisfied!!!" << "bGetMap : " << bGetMap << " bGetGoal : " << bGetGoal << std::endl;
    return false;
  }
}

Node MotionPlanner::GlobalToLocalCoordinate(Node globalNode, nav_msgs::Odometry egoOdom)
{
  // Coordinate transformation from global to local
  Node tmpLocalNode;
  // - Copy data globalNode to tmpLocalNode
  memcpy(&tmpLocalNode, &globalNode, sizeof(struct Node));

  // - Coordinate transform
  // -- translatioonal transform
  double delX = globalNode.x - egoOdom.pose.pose.position.x;
  double delY = globalNode.y - egoOdom.pose.pose.position.y;
  double delZ = globalNode.z - egoOdom.pose.pose.position.z;

  // -- rotational transform
  tf2::Quaternion q_ego;
  double egoR, egoP, egoY;
  // --- copy quaternion from odom
  tf2::convert(egoOdom.pose.pose.orientation, q_ego);
  // --- get roll pitch yaw
  tf2::Matrix3x3 m_odom(q_ego);
  m_odom.getRPY(egoR, egoP, egoY);

  // - calculate new pose
  double newX = cos(-egoY) * delX - sin(-egoY) * delY;
  double newY = sin(-egoY) * delX + cos(-egoY) * delY;
  double newZ = delZ;
  double newYaw = globalNode.yaw - egoY;

  // - Update pose
  tmpLocalNode.x = newX;
  tmpLocalNode.y = newY;
  tmpLocalNode.z = newZ;
  tmpLocalNode.yaw = newYaw;

  return tmpLocalNode;
}

geometry_msgs::PoseStamped MotionPlanner::GlobalToLocalCoordinate(geometry_msgs::PoseStamped poseGlobal, nav_msgs::Odometry egoOdom)
{
  // Coordinate transformation from global to local
  // - Copy data nodeGlobal to nodeLocal
  geometry_msgs::PoseStamped poseLocal;

  // - Coordinate transform
  // -- translatioonal transform
  double delX = poseGlobal.pose.position.x - egoOdom.pose.pose.position.x;
  double delY = poseGlobal.pose.position.y - egoOdom.pose.pose.position.y;
  double delZ = poseGlobal.pose.position.z - egoOdom.pose.pose.position.z;

  // -- rotational transform
  tf2::Quaternion q_goal, q_ego;
  double goalR, goalP, goalY;
  double egoR, egoP, egoY;
  // --- copy quaternion from odom
  tf2::convert(poseGlobal.pose.orientation, q_goal);
  tf2::convert(egoOdom.pose.pose.orientation, q_ego);
  // --- get roll pitch yaw
  tf2::Matrix3x3 m_goal(q_goal);
  tf2::Matrix3x3 m_odom(q_ego);
  m_goal.getRPY(goalR, goalP, goalY);
  m_odom.getRPY(egoR, egoP, egoY);

  // - calculate new pose
  double newX = cos(-egoY) * delX - sin(-egoY) * delY;
  double newY = sin(-egoY) * delX + cos(-egoY) * delY;
  double newZ = delZ;
  double newYaw = goalY - egoY;

  // - Update pose
  // -- quaternion to RPY 
  // -- RPY to Quaternion
  tf2::Quaternion globalQ, globalQ_new;
  globalQ.setRPY(0.0, 0.0, newYaw);
  globalQ_new = globalQ.normalize();

  poseLocal.pose.position.x = newX;
  poseLocal.pose.position.y = newY;
  poseLocal.pose.position.z = newZ;
  tf2::convert(globalQ_new, poseLocal.pose.orientation);

  return poseLocal;
}

Node MotionPlanner::LocalToPlannerCordinate(Node nodeLocal)
{
  /*
    TODO: Transform from local to occupancy grid map coordinate
    - local coordinate ([m]): x [map min x, map max x], y [map min y, map max y]
    - map coordinate ([cell]): x [0, map width], y [map height]
    - convert [m] to [cell] using map resolution ([m]/[cell])
  */
  // Copy data nodeLocal to nodeMap
  Node nodeMap;
  memcpy(&nodeMap, &nodeLocal, sizeof(struct Node));
  // Transform from local (min x, max x) [m] to map (0, map width) [grid] coordinate
  // nodeMap.x = 0;
  nodeMap.x = round((nodeLocal.x - this->origin_x) / this->mapResol);
  
  // Transform from local (min y, max y) [m] to map (0, map height) [grid] coordinate
  // nodeMap.y = 0;
  nodeMap.y = round((nodeLocal.y - this->origin_y) / this->mapResol);

  // ROS_INFO("LocalToPlannerCordinate : local (%.2f, %.2f) -> map (%.2f, %.2f)", nodeLocal.x, nodeLocal.y, nodeMap.x, nodeMap.y);

  return nodeMap;
}

void MotionPlanner::CallbackGlobalPath(const nav_msgs::Path& msg)
{
    // --- 1. HANDLE EMPTY/NEW PATH ---
    // If the stored path is empty, we must accept the new path immediately.
    if (this->globalPath.poses.empty()) {
        goto new_path_received;
    }

    // --- 2. COMPARE CONTENT ---
    // Safety check: If sizes are different, the content MUST be different.
    if (msg.poses.size() != this->globalPath.poses.size()) {
        goto new_path_received;
    }

    // Compare the content element by element only if sizes match.
    // NOTE: std::equal is robust when comparing two ranges of equal size.
    if (std::equal(msg.poses.begin(), msg.poses.end(), this->globalPath.poses.begin())) {
        // Paths are identical in size and content. DO NOTHING and exit.
        // If the path is identical, we should not assign 'this->globalPath = msg;'
        // as that defeats the purpose of checking for duplicates.
        return; 
    }

    // --- 3. PROCESS NEW PATH ---
    new_path_received:
    this->globalPath = msg;
    this->bGetGoal = true;
    this->bGetGlobalPath = true;
    
    // You would typically reset your waypoint tracker here:
    // this->currentWaypointIndex = 0; 

    ROS_INFO("Received NEW global path with %lu poses. Content has changed.", msg.poses.size());
}


/* ----- Publisher ----- */

void MotionPlanner::PublishData(std::vector<Node> motionMinCost, std::vector<std::vector<Node>> motionPrimitives)
{

  // Check if a valid motion was selected
  // if (!motionMinCost.empty()) {
  //     const Node& finalNode = motionMinCost.back();
  //     const Node& startNode = motionMinCost.front();
  //     double actual_progress = (finalNode.idx + 1) * this->DIST_RESOL; 

  //     // =========================================================================
  //     // DEBUG OUTPUT: Final Selected Motion Cost Breakdown
  //     // =========================================================================
  //     ROS_INFO("=================================================");
  //     ROS_INFO(">>> FINAL SELECTED MOTION <<<");
  //     ROS_INFO("Delta: %.2f deg | Total Cost: %.2f", 
  //               startNode.delta * 180.0 / M_PI, finalNode.cost_total);
  //     ROS_INFO("End Pose: (%.2f, %.2f)m | Length: %.2fm", 
  //               finalNode.x, finalNode.y, actual_progress);
  //     ROS_INFO("Cost Breakdown:");
  //     ROS_INFO("  - Collision Cost:  %.0f (Weighted: %.0f)", 
  //               finalNode.cost_colli, this->W_COST_TRAVERSABILITY * finalNode.cost_colli);
  //     ROS_INFO("  - Goal Dist Cost:  %.2f (Weighted: %.2f)", 
  //               finalNode.minDistGoal, this->W_COST_DIRECTION * finalNode.minDistGoal);
  //     ROS_INFO("=================================================");
  //     // =========================================================================
  // } else {
  //     ROS_WARN("PublishData called with empty motionMinCost. Robot may stop.");
  // }

  // Publisher
  // - visualize selected motion primitive
  PublishSelectedMotion(motionMinCost);
  // - visualize motion primitives
  PublishMotionPrimitives(motionPrimitives);
  // - publish command
  PublishCommand(motionMinCost);
}

/* ----- Main ----- */

int main(int argc, char* argv[])
{ 
  std::cout << "start main process" << std::endl;

  ros::init(argc, argv, "control_space_planner");
  // for subscribe
  ros::NodeHandle nh;
  ros::Rate rate(50.0);
  MotionPlanner MotionPlanner(nh);

  // Planning loop
  while (MotionPlanner.nh_.ok()) {
      // Spin ROS
      ros::spinOnce();
      // check run condition
      if (MotionPlanner.CheckRunCondition()) {
        // Run algorithm
        MotionPlanner.Plan();
      }
      rate.sleep();
  }

  return 0;

}
