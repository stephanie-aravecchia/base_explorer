#include <math.h>
#include <iostream>
#include <cstdlib>
#include <iterator>
#include <set>
#include <numeric>
#include <functional>
#include <random>
#include <ctime>
#include "base_explorer/BaseExplorer.h"

//#define DEBUG
//#define SHOW_IMG

#define UNKNOWN 127
#define FREE 0
#define OCCUPIED 255
#define UNKNOWN_OCC_GRID -1
#define FREE_THRES_OCC_GRID 30

using namespace std;

BaseExploration::BaseExploration() : nh_("~") {
	double xmin; double xmax; double ymin; double ymax;
    double border;
    int p;
    nh_.param<double>("safety_area", safety_meters_, 1.);//in meters, a candidate is discarded if at least one occupied voxel within
	nh_.param<double>("border_offset", border, 1.);//border around the image discarded
	nh_.param<int>("policy", p, 0);//Policy can be random frontier 0, closest frontier 1, or random in free space
	nh_.param<double>("xmin", xmin, -15.);
	nh_.param<double>("xmax", xmax, 15.);
	nh_.param<double>("ymin", ymin, -15.);
	nh_.param<double>("ymax", ymax, 15.);
    nh_.param<std::string>("robot_frame",robot_frame_,"/base_link");
    bbox_ = BBox(xmin + border, xmax - border, ymin + border, ymax -border);
    
    switch(p) {
        case 0:
            policy_ = PolicyChoice::RANDOM_FRONTIER;
            break;
        case 1:
            policy_ = PolicyChoice::CLOSEST_FRONTIER;
            break;
        case 2:
            policy_ = PolicyChoice::RANDOM_FREE;
            break;
        default:
            ROS_ERROR("Error, wrong choice in policy, should be either 0 Random Frontier, 1 Closest Frontier or 2 Random Free");
            exit(EXIT_FAILURE);
    }

    //create subscribers and publishers
    oGridSub_ = nh_.subscribe("/map",1,&BaseExploration::occupancyGridCallback,this);
	goalPub_ = nh_.advertise<geometry_msgs::PoseStamped>("/nav_goals",1);
    ROS_INFO_STREAM("Policy is: " << static_cast<int>(policy_));
}

void BaseExploration::occupancyGridCallback(const nav_msgs::OccupancyGridConstPtr& data) {
    convertOGridToImage(data);
#ifdef SHOW_IMG
    try {
        cv::imshow("occ grid", img_map_);
        cv::waitKey(5);
    } catch (...) {
        std::cout << "error in imshow\n";
    }
#endif
    selectCandidates();
    
    cv::Point2i goal;
    switch (policy_) {
        case PolicyChoice::RANDOM_FRONTIER:
            if (candidates_.size() == 0) {
                ROS_INFO_STREAM("No frontier candidate");
                return;
            } else {
                goal = getRandomCandidate();
            }
            break;
        case PolicyChoice::CLOSEST_FRONTIER:
            if (candidates_.size() == 0) {
                ROS_INFO_STREAM("No frontier candidate");
                return;
            } else {
                goal = getClosestCandidate();
            }
            break;
        case PolicyChoice::RANDOM_FREE:
            if (free_candidates_.size() == 0) {
                ROS_INFO_STREAM("No free candidate");
                return;
            } else {
                goal = getRandomFreeCandidate();
            }
            break;
        default:
            ROS_ERROR_STREAM("Error in choice of policy");
            exit(EXIT_FAILURE);
    }
    publishGoal(goal);
#ifdef SHOW_IMG
    imshowBest(goal);
#endif
}
cv::Point2i BaseExploration::getRandomFreeCandidate() const {
    static std::random_device rd;
    static std::default_random_engine rng(rd());
    uniform_int_distribution<int> distri(0,static_cast<int>(free_candidates_.size()-1));
    return free_candidates_[distri(rng)];
}

cv::Point2i BaseExploration::getRandomCandidate() const {
    static std::random_device rd;
    static std::default_random_engine rng(rd());
    uniform_int_distribution<int> distri(0,static_cast<int>(candidates_.size()-1));
    auto c = *std::next(candidates_.cbegin(), distri(rng));
    return c.second;
}
cv::Point2i BaseExploration::getClosestCandidate() const {
    auto c = *candidates_.cbegin();
    return c.second;
}

//Convert the Occupancy Grid message into a cv::Mat_ with 3 labels: free, occupied, unknown
void BaseExploration::convertOGridToImage(const nav_msgs::OccupancyGridConstPtr& data){
    map_resolution_ = data->info.resolution;
    map_frame_ = data ->header.frame_id;
    int width = data->info.width;
    int height = data->info.height;
    assert(width>0);
    assert(height>0);
    img_map_ = cv::Mat_<uint8_t>(height, width);
    occ_map_ = cv::Mat_<uint8_t>::zeros(height, width);
    unknown_map_ = cv::Mat_<uint8_t>::zeros(height, width);
    free_map_ = cv::Mat_<uint8_t>::zeros(height, width);
    map_origin_ = data->info.origin;
    for (int j=0; j < height; ++j){
        for (int i=0; i < width; ++i) {
            assert(static_cast<size_t>(j*width + i) < data->data.size());
            if (data->data[j*width+i] == UNKNOWN_OCC_GRID) {
                img_map_(height-1-j,i) = UNKNOWN;
                unknown_map_(height-1-j,i) = 255;
            } else if (data->data[j*width+i] <= FREE_THRES_OCC_GRID) {
                img_map_(height-1-j,i) = FREE;
                free_map_(height-1-j,i) = 255;
            } else {
                img_map_(height-1-j,i) = OCCUPIED;
                occ_map_(height-1-j,i) = 255;
            }
        }
    }
    //Now, we expand occupied and unknown space (dilate)
    //and we reduce known space (erode)
    int erosion_type = cv::MORPH_ELLIPSE;
    int erosion_size = (safety_meters_/map_resolution_);
    cv::Mat element = cv::getStructuringElement(erosion_type,
            cv::Size(2*erosion_size+1,2*erosion_size+1),
            cv::Point( erosion_size, erosion_size));
    cv::dilate(occ_map_,occ_map_, element);
    cv::dilate(unknown_map_,unknown_map_, element);
    erosion_size= max(erosion_size/2,1);
    element = cv::getStructuringElement(erosion_type,
            cv::Size(2*erosion_size+1,2*erosion_size+1),
            cv::Point( erosion_size, erosion_size));
    cv::erode(free_map_,free_map_, element);
#ifdef SHOW_IMG
    cv::imshow( "Eroded free", free_map_);
    cv::imshow( "Dilated occ", occ_map_);
    cv::imshow( "Dilated unknown", unknown_map_);
    cv::waitKey(5);
#endif

#ifdef DEBUG
    std::cout << "occupancy grid size: " << data->data.size() << "\n"
            << "matrix size: rows x cols " << img_map_.rows << " , " << img_map_.cols << "\n";
    (data->data.size() == (img_map_.rows*img_map_.cols)) ? (std::cout << "size match\n"):(std::cout << "size DON'T match\n");
#endif

}

//Converts cv::Point to goal in world
void BaseExploration::publishGoal(const cv::Point2i& point) const {
    cv::Point2d p = pixToMeters(point); 
    geometry_msgs::PoseStamped goal;
    goal.header.frame_id = "world";
    goal.header.stamp = ros::Time::now();
    
    goal.pose.position.x = p.x;
    goal.pose.position.y = p.y;
    goal.pose.orientation.w = 1.0;

    goalPub_.publish(goal);
}

//Returns a bbox of size s around (i,j) 
BaseExploration::PixBox BaseExploration::getBBox(int i, int j, int s) const {
        PixBox box;
        assert(s>=0);
        box.imin = max(0,i-s);
        box.imax = min(i+s,img_map_.cols);
        box.jmin = max(0,j-s);
        box.jmax = min(j+s,img_map_.rows);
        return box;
}
//Returns True if at least one neighbour is free
bool BaseExploration::isOneNeighbourFree(int i, int j) const {
        PixBox nbr = getBBox(i,j,1); 
        if ((nbr.imin>=nbr.imax) || (nbr.jmin >= nbr.jmax)) {
            ROS_INFO_STREAM("Free: min max in bbox do not match: " << nbr);
            return false;
        }
        for (int x = nbr.imin; x < nbr.imax; ++x) {
            for (int y = nbr.jmin; y < nbr.jmax; ++y) {
                assert(x>=0);
                assert(y>=0);
                assert(x<img_map_.cols);
                assert(y<img_map_.rows);
                if (img_map_(y,x) == FREE) {
                    return true;
                }
            }
        }
        return false;
}
//Returns True if at least one neighbour is Unknown
bool BaseExploration::isOneNeighbourUnknown(int i, int j) const {
        PixBox nbr = getBBox(i,j,1); 
        if ((nbr.imin>=nbr.imax) || (nbr.jmin >= nbr.jmax)) {
            ROS_INFO_STREAM("Free: min max in bbox do not match: " << nbr);
            return false;
        }
        for (int x = nbr.imin; x < nbr.imax; ++x) {
            for (int y = nbr.jmin; y < nbr.jmax; ++y) {
                assert(x>=0);
                assert(y>=0);
                assert(x<img_map_.cols);
                assert(y<img_map_.rows);
                if (img_map_(y,x) == UNKNOWN) {
                    return true;
                }
            }
        }
        return false;
}

//Returns true if there is no occupied voxel in the neighborhood    
bool BaseExploration::areNeighboursSafe(int i, int j) const {
        int safety = ceil(safety_meters_ / map_resolution_);
        PixBox nbr = getBBox(i,j, safety); 
        if ((nbr.imin>=nbr.imax) || (nbr.jmin >= nbr.jmax)) {
            ROS_INFO_STREAM("Safe: min max in bbox do not match: " << nbr);
            return false;
        }
        for (int x = nbr.imin; x < nbr.imax; ++x) {
            for (int y = nbr.jmin; y < nbr.jmax; ++y) {
                assert(x>=0);
                assert(y>=0);
                assert(x<img_map_.cols);
                assert(y<img_map_.rows);
                if (img_map_(y,x) == OCCUPIED) {
                    return false;
                }
            }
        }
        return true;

}
cv::Point2i BaseExploration::metersToPix(double x, double y) const {
    cv::Point2i pix;
    std::cout << img_map_.rows << ", " << img_map_.cols << endl;
    pix.x = std::min(std::max(static_cast<int>((x-costmap_bbox_.xmin)/map_resolution_), 0), img_map_.cols-1);
    pix.y = std::min(std::max(static_cast<int>((costmap_bbox_.ymax-y)/map_resolution_), 0), img_map_.rows-1);
    return pix;

}
cv::Point2d BaseExploration::pixToMeters(const cv::Point2i& pix) const {
    cv::Point2d p;
    p.x = pix.x*map_resolution_ + costmap_bbox_.xmin;
    p.y =  (img_map_.rows - pix.y)*map_resolution_ + costmap_bbox_.ymin;
    return p;
};
//Returns the pixbox in imMap inside the exploration bbox
BaseExploration::PixBox BaseExploration::getOccGridInterBBox() const {
#ifdef DEBUG
    cout << "map size: " << img_map_.rows << ", " << img_map_.cols << endl;
    cout << "map :" << map_origin_.position.x << ", " << map_origin_.position.y << endl;
    cout << "resolutin " << map_resolution_ << endl;
#endif
    BBox inter_bbox;
    inter_bbox.xmin = max(costmap_bbox_.xmin, bbox_.xmin);
    inter_bbox.ymin = max(costmap_bbox_.ymin, bbox_.ymin);
    inter_bbox.xmax = min(costmap_bbox_.xmax, bbox_.xmax);
    inter_bbox.ymax = min(costmap_bbox_.ymax, bbox_.ymax);

    PixBox inter_pixbox;
    //top left corner is:
    cv::Point2i tl = metersToPix(inter_bbox.xmin, inter_bbox.ymax);    
    //bottom right corner is:    
    cv::Point2i br = metersToPix(inter_bbox.xmax, inter_bbox.ymin);    
    inter_pixbox.imin = tl.x;
    inter_pixbox.jmin = tl.y;
    inter_pixbox.imax = br.x;
    inter_pixbox.jmax = br.y;
#ifdef DEBUG
    cout << "explo bbox " << bbox_ << endl; 
    cout << "occ bbox " << costmap_bbox_ << endl; 
    cout << "inter bbox " << inter_bbox << endl; 
    cout << "inter pixbox " << inter_pixbox << endl; 
#endif
    return inter_pixbox;
}

void BaseExploration::selectCandidates(){
    int width = img_map_.cols;
    int height = img_map_.rows;
    candidates_.clear(); 
    free_candidates_.clear(); 
    costmap_bbox_.xmin = map_origin_.position.x;
    costmap_bbox_.ymin = map_origin_.position.y;
    costmap_bbox_.xmax = map_origin_.position.x + (img_map_.cols * map_resolution_);
    costmap_bbox_.ymax = map_origin_.position.y + (img_map_.rows * map_resolution_);
    PixBox area = getOccGridInterBBox();
    assert(area.imin>=0);
    assert(area.jmin>=0);
    assert(area.imax<width);
    assert(area.jmax<height);
    
    //get the robot pose in the map frame
    tf::StampedTransform transform;
    try { 
        listener_.lookupTransform(map_frame_,robot_frame_, ros::Time(0), transform);
    } catch (tf::TransformException &ex){
        ROS_ERROR("%s",ex.what());
        return;
    }
#ifdef DEBUG
    cout << "Robot in world: " << transform.getOrigin().x() << ", " << transform.getOrigin().y() << endl;
#endif
    robot_ = metersToPix(transform.getOrigin().x(), transform.getOrigin().y());    
#ifdef DEBUG
    cout << "Robot: " << robot_ << endl;
#endif
    //Potential candidate are in expanded unknown and previously known
    assert(unknown_map_.rows == height);
    assert(free_map_.rows == height);
    assert(occ_map_.rows == height);
    assert(unknown_map_.cols == width);
    assert(free_map_.cols == width);
    assert(occ_map_.cols == width);
    
    cv::Mat_<uint8_t> potential_map(height, width);
    cv::Mat_<uint8_t> candidates_map(height, width);
    cv::Mat_<uint8_t> not_occ_map(height, width);
    cv::bitwise_and(unknown_map_, free_map_, potential_map);
    cv::bitwise_not(occ_map_, not_occ_map);
    cv::bitwise_and(potential_map, not_occ_map, candidates_map);
    
#ifdef SHOW_IMG
    int w = width*5;
    int h = height*5;
    cv::Mat_<uint8_t> tmp_c = candidates_map;
    cv::Mat_<uint8_t> tmp_p = potential_map;
    cv::Mat_<uint8_t> tmp_f = free_map_;
    for (int j=0; j<height; j++) {
        for (int i=0; i<width; i++) {
            if (tmp_c(j,i)>0) {
                tmp_c(j,i)=255;
            }
            if (tmp_p(j,i)>0) {
                tmp_p(j,i)=255;
            }
            if (tmp_f(j,i)>0) {
                tmp_f(j,i)=255;
            }
        }
    }
    cv::Mat_<uint8_t> resized_c(h,w);
    cv::Mat_<uint8_t> resized_p(h,w);
    cv::Mat_<uint8_t> resized_f(h,w);
    cv::resize(tmp_c, resized_c, cv::Size(h,w), cv::INTER_LINEAR);
    cv::resize(tmp_p, resized_p, cv::Size(h,w), cv::INTER_LINEAR);
    cv::resize(tmp_f, resized_f, cv::Size(h,w), cv::INTER_LINEAR);
    cv::imshow( "Candidates map", resized_c);
    cv::imshow( "Potential candidates", resized_p);
    cv::imshow( "Free candidates", resized_f);
    cv::waitKey(5);
#endif

    for (int i=area.imin; i < area.imax; i++){
        for (int j=area.jmin; j < area.jmax; j++) {
            if (candidates_map(j,i)>0){
                    double dist = hypot(robot_.x-i, robot_.y-j);
                    candidates_.insert(make_pair(dist, cv::Point2i(i,j)));
                    //candidates_vect_.push_back(cv::Point)
            }
            if (free_map_(j,i)>0){
                    free_candidates_.push_back(cv::Point2i(i,j));
            }
        }
    }
#ifdef SHOW_IMG
    //imshowFrontiers();
#endif
}
void BaseExploration::imshowFrontiers(){
#ifdef SHOW_IMG
#ifdef DEBUG
    cout << "number of frontiers points: " << candidates_.size() << endl;
#endif
    int width = img_map_.cols;
    int height = img_map_.rows;
    cv::Mat_<uint8_t> tmpImg = img_map_;
    cv::Mat color;
    cv::cvtColor(tmpImg, color, cv::COLOR_GRAY2BGR);
    for (auto point : candidates_) {
        color.at<cv::Vec3b>(point.second.y, point.second.x) = cv::Vec3b(0,255,0);
    }
    int w = width*5;
    int h = height*5;
    cv::Mat_<cv::Vec3b> resized(h,w);
    cv::resize(color, resized, cv::Size(h,w), cv::INTER_LINEAR);
    cv::imshow("Frontiers points", resized);
    cv::waitKey(5);
#endif
}
void BaseExploration::imshowBest(cv::Point2i bestPoint){
#ifdef SHOW_IMG
#ifdef DEBUG
    cout << "number of frontiers points: " << candidates_.size() << endl;
#endif
    int width = img_map_.cols;
    int height = img_map_.rows;
    cv::Mat_<uint8_t> tmpImg = img_map_;
    cv::Mat color;
    cv::cvtColor(tmpImg, color, cv::COLOR_GRAY2BGR);
    cv::circle(tmpImg, robot_, 5, cv::Scalar(255,0,0));
    for (auto point : candidates_) {
        color.at<cv::Vec3b>(point.second.y, point.second.x) = cv::Vec3b(0,255,0);
    }
    int j = bestPoint.y;  
    int i = bestPoint.x;
#ifdef DEBUG
    cout << "Best : " << i << ", " << j << ", width " << width << ", height " << height<< endl;  
#endif
    for (int y=max(0,j-2); y<min(j+2,height); y++) {
        for (int x=max(0,i-2); x<min(i+2,width); x++) {
            assert(x>=0);
            assert(y>=0);
            assert(x<width);
            assert(y<height);
            color.at<cv::Vec3b>(y, x) = cv::Vec3b(0,0,255);
        }
    }
    int w = width*5;
    int h = height*5;
    cv::Mat_<cv::Vec3b> resized(h,w);
    cv::resize(color, resized, cv::Size(h,w), cv::INTER_LINEAR);
    cv::imshow("Best", resized);
    cv::waitKey(5);
#endif
}


int main(int argc, char * argv[]) {
	ros::init(argc,argv,"base_explorer");
	BaseExploration exploration;
	ros::spin();
}

