#include <fsmore/map_fsmore_ros.h>
#include <octomap_msgs/OctomapWithPose.h>

// CONSTRUCTOR
MapFsmoreROS::MapFsmoreROS(){
    n=new ros::NodeHandle("~");
    force_sub = n->subscribe("/contact_force",10,&MapFsmoreROS::cb_contforce, this);
    line_pub =n->advertise<visualization_msgs::Marker>("lines",2);

    Initialize();
}

bool MapFsmoreROS::Initialize(){
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    return(true);
}

inline Eigen::Vector3f MapFsmoreROS::toEigen(geometry_msgs::Vector3 in){
    return(Eigen::Vector3f(static_cast<float>(in.x),static_cast<float>(in.y),static_cast<float>(in.z)));
}

inline Eigen::Affine3f MapFsmoreROS::toEigen(geometry_msgs::Transform m){
    return(Eigen::Translation3f(static_cast<float>(m.translation.x),
                                static_cast<float>(m.translation.y),
                                static_cast<float>(m.translation.z)) *
           Eigen::Quaternionf(static_cast<float>(m.rotation.w),
                              static_cast<float>(m.rotation.x),
                              static_cast<float>(m.rotation.y),
                              static_cast<float>(m.rotation.z))
           );
}


// ROS CALLBACKS
void MapFsmoreROS::cb_contforce(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    if(first_ft_cb){
        bias_ft=msg->wrench;
        first_ft_cb=false;
    }
    geometry_msgs::WrenchStamped w=*msg;
    w.wrench.force=MapTools::v_minus(w.wrench.force,bias_ft.force);
    w.wrench.torque=MapTools::v_minus(w.wrench.torque,bias_ft.torque);
    geometry_msgs::TransformStamped gTw;
    try {
        gTw=tfBuffer.lookupTransform("world", "graspedobject",ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(0.1).sleep();
        return;
    }
    if(MapTools::norm(w.wrench.force)>5.0){
        Eigen::Affine3f T;
        mapper.AddLine(toEigen(w.wrench.force),toEigen(w.wrench.torque),toEigen(gTw.transform));
    }

    pcl::toROSMsg(mapper.getMapPointCloud());
    //if(norm(w.wrench.force)<0.1){
    //    AddEmpty(gTw);
    // }
}

visualization_msgs::Marker MapFsmoreROS::setupLines(std::string frame_id){
    visualization_msgs::Marker lines;
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.scale.x = 0.01;
    lines.pose.orientation.w=1.0;
    lines.color.a = 1.0;
    lines.points.resize(40);

    lines.header.frame_id=frame_id;
    lines.ns=frame_id;
    lines.color.g = 1.0;
    return(lines);
}

void MapFsmoreROS::AddToMarkerLines(Line l,visualization_msgs::Marker &m){
    geometry_msgs::Point p1,p2;
    p1.x=static_cast<double>(l.p1.x());
    p1.y=static_cast<double>(l.p1.y());
    p1.z=static_cast<double>(l.p1.z());
    p2.x=static_cast<double>(l.p2.x());
    p2.y=static_cast<double>(l.p2.y());
    p2.z=static_cast<double>(l.p2.z());
    m.points.push_back(p1);
    m.points.push_back(p2);
}


