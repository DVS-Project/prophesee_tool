// Convert xxx.bag to a .txt file

#include <iostream>
#include <algorithm>
#include <vector>
#include <unistd.h>
#include <boost/filesystem.hpp>
#include <opencv2/opencv.hpp>

#include <gflags/gflags.h>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>        // needed by BOOST_FOREACH
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/String.h>

#include <prophesee_event_msgs/Event.h>
#include <prophesee_event_msgs/EventArray.h>

typedef prophesee_event_msgs::Event EventT;
typedef prophesee_event_msgs::EventArray EventArrayT;

using namespace std;
using namespace cv;


DEFINE_string(input_bag, "default.bag", "dataset path");
DEFINE_string(output_dir, ".", "dataset path");
DEFINE_bool(with_images, false, "extract images from rosbag?");

DEFINE_string(event_topic, "/sync/events", "default event topic");
DEFINE_string(image_topic, "/sync/image", "default event topic");


int main(int argc, char** argv){

    google::ParseCommandLineFlags(&argc, &argv, true);
	ros::init(argc, argv, "save_data_node");
	ROS_INFO("data clean begin...");
	ros::NodeHandle nh;
    ROS_INFO_STREAM("Read rosbag: <" + FLAGS_input_bag + ">");

    // Load data from bag.
    std::vector<std::string> topics;        // get topics.
    topics.push_back(FLAGS_event_topic);
    if(FLAGS_with_images)
        topics.push_back(FLAGS_image_topic);
    
    rosbag::Bag bag(FLAGS_input_bag, rosbag::bagmode::Read);
    rosbag::View view(bag, rosbag::TopicQuery(topics));
    if(!bag.isOpen()){
        ROS_ERROR("Bag not open...");
        return -1;
    }
    ROS_INFO("Extract the following topics...");
    for(auto t: topics){
        cout << t << endl;
    }
    
    // read data    
    vector<sensor_msgs::Image> frames;
    vector<EventT> events;
    BOOST_FOREACH(rosbag::MessageInstance const m, view){       // get topics
        const std::string& topic_name = m.getTopic();
        if (topic_name == FLAGS_image_topic){
            sensor_msgs::Image image_msg = *(m.instantiate<sensor_msgs::Image>());
            frames.push_back(image_msg);
            // cout << "<image time>: " << image_msg.header.stamp << endl;
        }
        if (topic_name == FLAGS_event_topic){
            EventArrayT ev = *(m.instantiate<EventArrayT>());
            for (auto e : ev.events){
                events.push_back(e);
            }
        }
    }
    bag.close();

    ros::Time first_image_ts = frames[0].header.stamp;
    printf("First image ts: %f \n", first_image_ts.toSec());

    // extract images;
    if(FLAGS_with_images){
        ROS_INFO_STREAM("Save images ts file: "<< FLAGS_output_dir <<"/images.txt");
        ROS_INFO_STREAM("Save images : "<< FLAGS_output_dir <<"/xxx.bmp");
        
        // save images;
        ofstream of_img(FLAGS_output_dir + "/images.txt");
        int counter = 0;
        if(!of_img.is_open()){
            ROS_ERROR("Cannot save images...");
            return -1;
        }
        for(int i=0; i<frames.size(); ++i){
            cv_bridge::CvImagePtr cv_ptr;
            try{
                cv_ptr = cv_bridge::toCvCopy(frames[i], "bgr8");
                imwrite(FLAGS_output_dir + "/images/" + to_string(counter++) + ".bmp", cv_ptr->image);
                // of_img << counter << " " << setprecision(16) << cv_ptr->header.stamp.toSec() <<endl;
                of_img << counter << " " << (int)((cv_ptr->header.stamp - first_image_ts).toSec()*1e6) <<endl;
            }
            catch (cv_bridge::Exception& e){
                ROS_ERROR("cv_bridge exception: %s", e.what());
                return -1;
            }
        }
    }
    else{
        ROS_INFO("Do not extract frames...");
    }


    // save data
    ROS_INFO_STREAM("Save events file: " << FLAGS_output_dir << "/events.txt");
    // save events to xxx/events.txt
    ofstream of(FLAGS_output_dir + "/events.txt");
    if(!of.is_open()){
        ROS_ERROR("Cannot save events...");
        return -1;
    }
    int skip_event_counter = 0;
    for(int i=0; i<events.size(); ++i){
        auto e = events[i];
        if(e.ts < first_image_ts){       // skip first events;
            skip_event_counter++;
            continue;
        }
        int p = e.polarity == true ? 1 : 0;
        of << (int)((e.ts - first_image_ts).toSec() * 1e6) << " " << e.x << " " << e.y << " " << p << endl;
        if (i % (events.size() / 10) == 0)
            ROS_INFO_STREAM("Processing: " << i << "/" << events.size());
    }
    cout << endl;
    ROS_INFO_STREAM("Skip events number: " << skip_event_counter);
    
    ROS_INFO("--- Extraction Done ---");
    return 0;
}