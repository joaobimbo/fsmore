#include <fsmore/map_fsmore_ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

// CONSTRUCTOR
MapFsmoreROS::MapFsmoreROS(){
    n=new ros::NodeHandle();
    force_sub = n->subscribe("/contact_force",1,&MapFsmoreROS::cb_contforce, this);
    pose_sub = n->subscribe("/robot_ee",1,&MapFsmoreROS::cb_eepose, this);
    pub_line =n->advertise<visualization_msgs::Marker>("lines",2);
    pub_map_pc =n->advertise<sensor_msgs::PointCloud2>("map_pc",2);
    pub_obj_pc =n->advertise<sensor_msgs::PointCloud2>("obj_pc",2);
    pub_oct_map = n->advertise<octomap_msgs::Octomap>("oct_map",2);
    pub_oct_obj = n->advertise<octomap_msgs::Octomap>("oct_obj",2);

    Initialize();
}

bool MapFsmoreROS::Initialize(){
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    n->param<std::string>("/object_file", mesh_filename, "mesh.stl");
    n->param<double>("/decay_time", mapper.decay_time, 30.0);
    n->param<float>("/line_length", mapper.line_half_length, 1.0);

    n->param<std::string>("world_frame",world_frame,"world");
    n->param<std::string>("object_frame",object_frame,"object");
    double map_resolution;
    n->param<double>("map_resolution", map_resolution, 0.01);
    mapper.resetMap(map_resolution);

    boost::function<bool (octomap_msgs::GetOctomap::Request&,octomap_msgs::GetOctomap::Response&)> getmap_handle( boost::bind(&MapFsmoreROS::getOctomap,this, _1,_2,mapper.oct_map,world_frame) );
    boost::function<bool (octomap_msgs::GetOctomap::Request&,octomap_msgs::GetOctomap::Response&)> getobj_handle( boost::bind(&MapFsmoreROS::getOctomap,this, _1,_2,mapper.oct_obj,object_frame) );
    get_map_octree = n->advertiseService("/get_oct_map", getmap_handle);
    get_obj_octree = n->advertiseService("/get_oct_obj", getobj_handle);

    PCTypePtr stl_pc(new PCType);

    //LoadSTL(mesh_filename, stl_pc);
    //AddPointsFromPC(stl_pc,mapper.oct_obj,mapper.pc_obj,mapper.map_obj);
    return(true);
}

bool MapFsmoreROS::getOctomap(octomap_msgs::GetOctomap::Request  &req,octomap_msgs::GetOctomap::Response &res, OctTypePtr octo,std::string frame){

    ros::Time ts=ros::Time::now();


    ros::Duration d1=ros::Time::now()-ts;
    printf("to0 -- A -- %f\n",d1.toSec());

    octomap_msgs::Octomap oct_map_msg;
    octo->setOccupancyThres(0.5);
    OctType octo_out(octo->getResolution());
    for (OctType::leaf_iterator it=octo->begin_leafs();it!=octo->end_leafs();it++){
        if(it->getOccupancy()>octo->getOccupancyThres()){
            octo_out.setNodeValue(it.getX(),it.getY(),it.getZ(),it->getLogOdds());
        }
    }
    ros::Duration d2=ros::Time::now()-ts;
    printf("to1 -- B -- %f\n",d2.toSec());

    octomap_msgs::fullMapToMsg(octo_out,oct_map_msg);
    //octomap_msgs::binaryMapToMsg(*(octo),oct_map_msg);
    oct_map_msg.header.stamp=ros::Time::now();
    oct_map_msg.header.frame_id=frame;
    res.map=oct_map_msg;

    //    octomap::AbstractOcTree* oct_test = octomap_msgs::msgToMap(oct_map_msg);
    //    OctType *tree = dynamic_cast<octomap::OcTree*>(oct_test);
    //    if(frame==world_frame){
    //        pub_oct_map.publish(oct_map_msg);
    //    }
    //    else{
    //        pub_oct_obj.publish(oct_map_msg);
    //    }
    ros::Duration d3=ros::Time::now()-ts;
    printf("to1 -- C -- %f\n",d3.toSec());

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
void MapFsmoreROS::cb_eepose(const geometry_msgs::PoseStamped::ConstPtr& msg){
    last_pose=*msg;
}


void MapFsmoreROS::cb_contforce(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    ros::Time ts=ros::Time::now();

    if(first_ft_cb){
        bias_ft=msg->wrench;
        first_ft_cb=false;
    }

//    if((last_pose.header.stamp - msg->header.stamp).toSec()<0.005){
//        ROS_WARN("Force message is too old");
//        return;
//    }


    geometry_msgs::WrenchStamped w=*msg;
    w.wrench.force=MapTools::v_minus(w.wrench.force,bias_ft.force);
    w.wrench.torque=MapTools::v_minus(w.wrench.torque,bias_ft.torque);
    geometry_msgs::TransformStamped gTw;

    gTw.header=last_pose.header;
    gTw.transform.rotation=last_pose.pose.orientation;
    gTw.transform.translation.x=last_pose.pose.position.x;
    gTw.transform.translation.y=last_pose.pose.position.y;
    gTw.transform.translation.z=last_pose.pose.position.z;

    /* //Doing ee_pose instead
    try {
        gTw=tfBuffer.lookupTransform(world_frame, object_frame,ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(0.1).sleep();
        return;
    }
    */
    if(MapTools::norm(w.wrench.force)>5.0){
        last_hit=w.header.stamp;
        Line l_map,l_obj;
        visualization_msgs::Marker lm_m=setupLines(world_frame);
        if(mapper.AddLine(toEigen(w.wrench.force),toEigen(w.wrench.torque),toEigen(gTw.transform),l_map,l_obj)){
           lm_m.color.b=0.5;
        }
        AddToMarkerLines(l_map,lm_m);
        pub_line.publish(lm_m);
    }
    if(MapTools::norm(w.wrench.force)<0.5 && (w.header.stamp-last_hit).toSec()>0.5){
        mapper.AddEmpty(toEigen(gTw.transform));
    }

    mapper.CleanupLines();
    double min_intensity;
    n->param<double>("/min_intensity", min_intensity, 0.50);

    PublishPointCloud(mapper.getMapPointCloud(min_intensity),pub_map_pc,world_frame);
    PublishPointCloud(mapper.getObjectPointCloud(min_intensity),pub_obj_pc,object_frame);

    //PublishOctrees(mapper.oct_map,pub_oct_map,world_frame);
    //PublishOctrees(mapper.oct_obj,pub_oct_obj,object_frame);
    ros::Duration d=ros::Time::now()-ts;
    printf("fcallback-- %f\n",d.toSec());

}

void MapFsmoreROS::PublishPointCloud(pcl::PointCloud<pcl::PointXYZI> pc,ros::Publisher pub,std::string frame){
    sensor_msgs::PointCloud2 cloud;
    pcl::toROSMsg(pc,cloud);
    cloud.header.stamp=ros::Time::now();
    cloud.header.frame_id=frame;
    pub.publish(cloud);
}


void MapFsmoreROS::PublishOctrees(OctTypePtr octo,ros::Publisher pub,std::string frame){
    octomap_msgs::Octomap oct_map_msg;
    mapper.oct_map->setOccupancyThres(0.5);
    octomap_msgs::fullMapToMsg(*octo,oct_map_msg);

    oct_map_msg.header.stamp=ros::Time::now();
    oct_map_msg.header.frame_id=frame;
    pub.publish(oct_map_msg);
}


visualization_msgs::Marker MapFsmoreROS::setupLines(std::string frame_id){
    visualization_msgs::Marker lines;
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.scale.x = 0.005;
    lines.pose.orientation.w=1.0;
    lines.color.a = 0.2f;
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


void MapFsmoreROS::AddPointsFromPC(PCTypePtr in,OctTypePtr tree_out, PCTypePtr cloud_out,std::map<size_t,Voxel> &map_out){

    for(PCType::iterator it = in->begin();it!=in->end();it++){
        size_t key = mapper.KeyHasher(tree_out->coordToKey(*it));
        if(map_out.find(key)==map_out.end()){
            cloud_out->push_back(*it);
            Voxel v(tree_out,*it,key);
            v.setLikelihood(0.99f);
            map_out.insert(std::pair<size_t,Voxel>(key,v));
        }
    }
}



void MapFsmoreROS::LoadSTL(std::string filename, PCTypePtr tree){
    printf("LOADING FILE %s\n",filename.c_str());
    std::ifstream infile(filename);
    std::string line;
    int ii=0;
    std::vector<float> n(3,0);
    int j=0;
    PType p2;

    while (std::getline(infile, line))
    {
        line=line.substr(line.find_first_not_of(" ")); //remove trailing spaces
        if(!line.compare(0,8,"    outer loop")) ii=1;
        if(strncmp(line.c_str(),"facet normal",12)==0){
            std::istringstream iss(line.substr(line.find_first_of(" ")+8));
            n.clear();
            std::copy(std::istream_iterator<float>(iss),
                      std::istream_iterator<float>(),
                      std::back_inserter(n));
            j=0;
            p2=PType();
        }

        if(strncmp(line.c_str(),"vertex",6)==0){
            std::istringstream iss(line.substr(line.find_first_of(" ")));
            std::vector<float> v;
            std::copy(std::istream_iterator<float>(iss),
                      std::istream_iterator<float>(),
                      std::back_inserter(v));

            p2.x()+=v.at(0)/1000;
            p2.y()+=v.at(1)/1000;
            p2.z()+=v.at(2)/1000;
            j++;
            if(j==3){
                p2.x()/=3;
                p2.y()/=3;
                p2.z()/=3;
                //p2.intensity=1.0;
                tree->push_back(p2);
            }
        }

    }
}

