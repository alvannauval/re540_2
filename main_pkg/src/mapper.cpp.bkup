#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PointStamped.h>
#include <iostream>
#include <algorithm>

// Macro to convert 2D grid coordinates (x, y) to a 1D array index
// Note: sx is the width of the map (number of cells in the X direction)
#define MAP_IDX(sx, i, j) ((sx) * (j) + (i))

/**
 * @brief Node to accumulate transient local maps into a persistent global map.
 * * This node subscribes to a local occupancy grid (published relative to the 
 * robot's frame, like 'base_link') and fuses it into a single, static global 
 * map (in the 'map' frame) using TF lookups to determine the robot's pose.
 */
class GlobalMapAccumulator
{
public:
    GlobalMapAccumulator();

private:
    ros::NodeHandle nh_;
    ros::Subscriber local_map_sub_;
    ros::Publisher global_map_pub_;
    tf::TransformListener tf_listener_;

    // --- GLOBAL MAP STATE ---
    nav_msgs::OccupancyGrid global_map_;
    bool is_map_initialized_ = false;

    // --- PARAMETERS (MATCH THESE TO YOUR CUSTOM NODE AND ENVIRONMENT) ---
    // The topic published by your HeightmapToCostMap node
    const std::string LOCAL_MAP_TOPIC = "/map/local_map/obstacle";
    
    // The required output topic for AMCL
    const std::string GLOBAL_MAP_TOPIC = "/map"; 
    
    // Global map properties (set large enough to cover the entire environment)
    const float GLOBAL_RESOLUTION = 0.1; // [m/cell] Must match the input map for simplicity
    const int GLOBAL_WIDTH = 400;        // 40m wide total map
    const int GLOBAL_HEIGHT = 400;       // 40m tall total map
    const std::string GLOBAL_FRAME_ID = "map"; // The permanent, static world frame

    /**
     * @brief Initializes the metadata and data buffer for the global_map_ object.
     */
    void initializeGlobalMap();

    /**
     * @brief Callback function to process incoming local maps and fuse them globally.
     * @param local_map_msg The transient occupancy grid published by the custom sensor node.
     */
    void localMapCallback(const nav_msgs::OccupancyGridConstPtr& local_map_msg);

    /**
     * @brief Converts global (x, y) coordinates to a 1D index in the global_map_.
     * @param global_x Global X-coordinate (meters).
     * @param global_y Global Y-coordinate (meters).
     * @return The 1D index, or -1 if outside the global map bounds.
     */
    int getGlobalMapIndex(double global_x, double global_y);
};

GlobalMapAccumulator::GlobalMapAccumulator()
{
    // Initialize the TF Listener
    // The TransformListener object must persist, so we define it here.
    
    // Subscribe to the local map output from your custom node
    local_map_sub_ = nh_.subscribe(LOCAL_MAP_TOPIC, 1, 
                                   &GlobalMapAccumulator::localMapCallback, this);

    // Advertise the global map output on the standard /map topic
    global_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(GLOBAL_MAP_TOPIC, 1, true); 
    // 'true' makes it a latched topic, ideal for static maps

    ROS_INFO("[GlobalMapAccumulator] Node initialized. Waiting for local map data...");
}

void GlobalMapAccumulator::initializeGlobalMap()
{
    global_map_.header.frame_id = GLOBAL_FRAME_ID;
    global_map_.info.resolution = GLOBAL_RESOLUTION;
    global_map_.info.width = GLOBAL_WIDTH;
    global_map_.info.height = GLOBAL_HEIGHT;

    // Calculate origin to center the map around (0,0) of the 'map' frame.
    global_map_.info.origin.position.x = -GLOBAL_WIDTH * GLOBAL_RESOLUTION / 2.0;
    global_map_.info.origin.position.y = -GLOBAL_HEIGHT * GLOBAL_RESOLUTION / 2.0;
    global_map_.info.origin.orientation.w = 1.0; // Identity orientation

    // Fill the map data with 'unknown' (-1)
    global_map_.data.assign(GLOBAL_WIDTH * GLOBAL_HEIGHT, -1);
    is_map_initialized_ = true;
    ROS_INFO("[GlobalMapAccumulator] Global map initialized to %dx%d cells.", GLOBAL_WIDTH, GLOBAL_HEIGHT);
}

int GlobalMapAccumulator::getGlobalMapIndex(double global_x, double global_y)
{
    // Convert global coordinate (meters) to map index (cells)
    double map_x_meters = global_x - global_map_.info.origin.position.x;
    double map_y_meters = global_y - global_map_.info.origin.position.y;

    int map_x_cell = static_cast<int>(map_x_meters / GLOBAL_RESOLUTION);
    int map_y_cell = static_cast<int>(map_y_meters / GLOBAL_RESOLUTION);

    // Check bounds
    if (map_x_cell < 0 || map_x_cell >= GLOBAL_WIDTH || 
        map_y_cell < 0 || map_y_cell >= GLOBAL_HEIGHT)
    {
        return -1; // Out of bounds
    }

    return MAP_IDX(GLOBAL_WIDTH, map_x_cell, map_y_cell);
}

void GlobalMapAccumulator::localMapCallback(const nav_msgs::OccupancyGridConstPtr& local_map_msg)
{
    if (!is_map_initialized_) {
        initializeGlobalMap();
    }

    tf::StampedTransform transform;
    
    // The local map frame is provided by the custom node (e.g., "base_link")
    std::string local_frame = local_map_msg->header.frame_id; 

    // 1. Look up the transformation: from the local map's frame to the global 'map' frame.
    try
    {
        // Wait for and look up the transform at the time of the sensor reading
        tf_listener_.waitForTransform(GLOBAL_FRAME_ID, local_frame, 
                                     ros::Time(0), ros::Duration(0.1)); // <--- Changed ros::Time(0)
        tf_listener_.lookupTransform(GLOBAL_FRAME_ID, local_frame, 
                                     ros::Time(0), transform);         // <--- Changed ros::Time(0)
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR_THROTTLE(1.0, "TF Lookup Failed (%s -> %s): %s", 
                           local_frame.c_str(), GLOBAL_FRAME_ID.c_str(), ex.what());
        return;
    }

    // 2. Fuse the local map data into the persistent global map
    for (int local_y = 0; local_y < local_map_msg->info.height; ++local_y)
    {
        for (int local_x = 0; local_x < local_map_msg->info.width; ++local_x)
        {
            int local_index = MAP_IDX(local_map_msg->info.width, local_x, local_y);
            signed char cell_value = local_map_msg->data[local_index];

            if (cell_value == -1) continue; // Skip unknown cells

            // a. Calculate point coordinates (meters) in the LOCAL frame (local_frame)
            double local_p_x = local_map_msg->info.origin.position.x + (local_x + 0.5) * local_map_msg->info.resolution;
            double local_p_y = local_map_msg->info.origin.position.y + (local_y + 0.5) * local_map_msg->info.resolution;

            // b. Transform the point from the local frame to the GLOBAL 'map' frame
            tf::Point pt_local(local_p_x, local_p_y, 0.0);
            tf::Point pt_global = transform * pt_local;

            // c. Convert global coordinates to GLOBAL map indices
            int global_index = getGlobalMapIndex(pt_global.x(), pt_global.y());

            // d. Update the global map
            if (global_index != -1)
            {
                // Simple fusion rule: if the new value is occupied (100), update it.
                // This ensures obstacles are permanently marked.
                if (cell_value == 100 || global_map_.data[global_index] == -1)
                {
                    global_map_.data[global_index] = cell_value;
                }
            }
        }
    }
    
    // 3. Publish the updated global map
    global_map_.header.stamp = ros::Time::now();
    global_map_pub_.publish(global_map_);
}

int main(int argc, char **argv)
{
    // The temporary TF bridge is essential during mapping!
    ROS_INFO("===================================================================");
    ROS_INFO("NOTE: You must run 'rosrun tf static_transform_publisher 0 0 0 0 0 0 map odom 100' ");
    ROS_INFO("in a separate terminal for this accumulator to work!");
    ROS_INFO("===================================================================");
    
    ros::init(argc, argv, "global_map_accumulator");

    // We instantiate the class, but we don't need the polling loop (rate(50) + generate_costmap())
    // because all the logic is inside the subscription callback.
    GlobalMapAccumulator gma; 
    
    // Use ros::spin() to process all callbacks (the localMapCallback)
    ros::spin(); 
    return 0;
}
