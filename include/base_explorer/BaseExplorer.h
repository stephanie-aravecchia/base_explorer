#ifndef BASE_EXPLORER_H
#define BASE_EXPLORER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <actionlib/client/simple_action_client.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <map>
#include <tf/tf.h>
#include <tf/transform_listener.h>

class BaseExploration {
    /*
    This class implements Random and Closest Exploration policies:
    From an occupancy grid, the node publishes a goal, sampled on the frontier, either random or closest
    Another policy is also implemented: Random Free, where the goal is sampled randomly in free space.
    */
    
	protected:
		ros::NodeHandle nh_;
		ros::Subscriber oGridSub_;
        ros::Publisher goalPub_; 
        tf::TransformListener listener_;

	    double safety_meters_;
        std::string map_frame_; 
        std::string robot_frame_; 
        
        //The actual map derived from the occupancy grid, with known, unknown, empty labels
		cv::Mat_<uint8_t> img_map_;
        double map_resolution_;
        geometry_msgs::Pose map_origin_;

        cv::Point2i robot_;

		//The 3 binary images
        cv::Mat_<bool> occ_map_;
        cv::Mat_<bool> unknown_map_;
        cv::Mat_<bool> free_map_;

        std::multimap<double, cv::Point2i, std::less<double>> candidates_;
        std::vector<cv::Point2i> free_candidates_;
    public:
        enum class PolicyChoice{RANDOM_FRONTIER, CLOSEST_FRONTIER, RANDOM_FREE};
        
        struct BBox {
            double xmin;
            double xmax;
            double ymin;
            double ymax;
            BBox() {}
            BBox(double xmin, double xmax, double ymin, double ymax) :
                xmin(xmin), xmax(xmax), ymin(ymin), ymax(ymax) {}
        };
        struct PixBox {
            int imin;
            int imax;
            int jmin;
            int jmax;
            PixBox() {}
            PixBox(int imin, int imax, int jmin, int jmax) :
                imin(imin), imax(imax), jmin(jmin), jmax(jmax) {}
        };
    protected:
        PolicyChoice policy_; 

        BBox bbox_;
        BBox costmap_bbox_;

        //Utility functions
        //check if a point is in a BBox, limits included    
        bool isInBBox(const cv::Point2i& p, const PixBox& b) const {
            if ((p.x < b.imin) || (p.x) > b.imax) return false;
            if ((p.y < b.jmin) || (p.y) > b.jmax) return false;
            return true;
        };
        //conversions
        cv::Point2i metersToPix(double x, double y) const;
        cv::Point2d pixToMeters(const cv::Point2i&) const;

        //get bbox of size s, centered around i,j, img bounds checked
        PixBox getBBox(int i, int j, int s) const;

        //Compute the intersection of the occupancy grid and the exploration bbox
        PixBox getOccGridInterBBox() const;
        
        //Takes the occupancy grid msg, and stores it into a ternary img (img_map_)
		void convertOGridToImage(const nav_msgs::OccupancyGridConstPtr&);
		
        //       
        void selectCandidates();
	    cv::Point2i getRandomCandidate() const;
	    cv::Point2i getClosestCandidate() const;
	    cv::Point2i getRandomFreeCandidate() const;
		
        void imshowFrontiers();
		void imshowBest(cv::Point2i);
        bool isOneNeighbourFree(int i, int j) const;
        bool isOneNeighbourUnknown(int i, int j) const;
        bool areNeighboursSafe(int i, int j) const;
		void publishGoal(const cv::Point2i&) const;


        // Callback
		void occupancyGridCallback(const nav_msgs::OccupancyGridConstPtr&);
	
    public:
		BaseExploration();
};

std::ostream& operator<<(std::ostream& out, const BaseExploration::PixBox& bbox) {
    out << "imin: " << bbox.imin << " " 
        << "imax: " << bbox.imax << " " 
        << "jmin: " << bbox.jmin << " " 
        << "jmax: " << bbox.jmax; 
    return out;
};
std::ostream& operator<<(std::ostream& out, const BaseExploration::BBox& bbox) {
    out << "xmin: " << bbox.xmin << " " 
        << "xmax: " << bbox.xmax << " " 
        << "ymin: " << bbox.ymin << " " 
        << "ymax: " << bbox.ymax; 
    return out;
};

#endif // BASE_EXPLORER_H
