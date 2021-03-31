#include <fsmore/map_fsmore_ros.h>


// CONSTRUCTOR
MapFsmoreROS::MapFsmoreROS(){
    n=new ros::NodeHandle();
    force_sub = n->subscribe("/contact_force",10,&MapFsmoreROS::cb_contforce, this);
    pub_line =n->advertise<visualization_msgs::Marker>("lines",2);
    pub_map_pc =n->advertise<sensor_msgs::PointCloud2>("map_pc",2);
    pub_obj_pc =n->advertise<sensor_msgs::PointCloud2>("obj_pc",2);

    Initialize();
}

bool MapFsmoreROS::Initialize(){
    tfListener = new tf2_ros::TransformListener(tfBuffer);
    n->param<std::string>("/object_file", mesh_filename, "mesh.stl");    

    PCTypePtr stl_pc(new PCType);

    LoadSTL(mesh_filename, stl_pc);
    AddPointsFromPC(stl_pc,mapper.oct_obj,mapper.pc_obj,mapper.map_obj);
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

    sensor_msgs::PointCloud2 cloud;

    pcl::toROSMsg(mapper.getMapPointCloud(),cloud);
    cloud.header.stamp=ros::Time::now();
    cloud.header.frame_id="world";
    pub_map_pc.publish(cloud);

    pcl::toROSMsg(mapper.getObjectPointCloud(),cloud);
    cloud.header.stamp=ros::Time::now();
    cloud.header.frame_id="graspedobject";
    pub_obj_pc.publish(cloud);



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


void MapFsmoreROS::AddPointsFromPC(PCTypePtr in,OctTypePtr tree_out, PCTypePtr cloud_out,std::map<size_t,Voxel> &map_out){

    for(PCType::iterator it = in->begin();it!=in->end();it++){
        size_t key = mapper.KeyHasher(tree_out->coordToKey(*it));
        if(map_out.find(key)==map_out.end()){
            cloud_out->push_back(*it);
            Voxel v(tree_out,*it,key);
            v.setLikelihood(100.0f);
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

