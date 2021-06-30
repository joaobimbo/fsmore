#include <fsmore/plan_fsmore_ros.h>

PlanFsmoreROS::PlanFsmoreROS()
{
    n=new ros::NodeHandle();
    //PlanFsmore_2D planner2D;
    //planner = &planner2D;
#ifdef PLANNER_2D
    planner = new PlanFsmore_2D();
#endif

#ifdef PLANNER_CSPACE
    planner = new Plan_CSpace(n);
#endif


    boost::function<bool (nav_msgs::GetPlan::Request&,nav_msgs::GetPlan::Response&)> planPath_handle( boost::bind(&PlanFsmoreROS::planPath,this, _1,_2,planner) );
    boost::function<bool (fsmore::CheckPoseValid::Request&,fsmore::CheckPoseValid::Response&)> checkValid_handle( boost::bind(&PlanFsmoreROS::checkValid,this, _1,_2,planner) );
    plan_service = n->advertiseService("get_plan", planPath_handle);
    plan_service_check = n->advertiseService("check_pose", checkValid_handle);
    pub_plan_markers = n->advertise<visualization_msgs::MarkerArray>("plan_steps",1);
    //sub_oct_map = n->subscribe("/oct_map",2,&PlanFsmoreROS::cb_oct_map,this);
    //sub_oct_obj = n->subscribe("/oct_obj",2,&PlanFsmoreROS::cb_oct_obj,this);

    srv_oct_map = n->serviceClient<octomap_msgs::GetOctomap>("/get_oct_map",false);
    srv_oct_obj = n->serviceClient<octomap_msgs::GetOctomap>("/get_oct_obj",false);

    n->param<std::string>("object_file", mesh_filename, "mesh.stl");

    pub_map = n->advertise<octomap_msgs::Octomap>("/oct_map_rec",1);
    pub_obj = n->advertise<octomap_msgs::Octomap>("/oct_obj_rec",1);

    pub_plan=n->advertise<geometry_msgs::PoseArray>("plan_tree",1);

}


//void PlanFsmoreROS::cb_oct_map(const octomap_msgs::Octomap::ConstPtr& msg){
//    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
//    planner->oct_map = dynamic_cast<octomap::OcTree*>(tree);
//}

//void PlanFsmoreROS::cb_oct_obj(const octomap_msgs::Octomap::ConstPtr& msg){
//    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
//    planner->oct_obj = dynamic_cast<octomap::OcTree*>(tree);
//}

void PlanFsmoreROS::initializePlanner(){
    Eigen::Vector3d min,max;
    double step_size,plan_timeout;
    n->param<double>("min_x", min.x(),-1);
    n->param<double>("max_x", max.x(),1);
    n->param<double>("min_y", min.y(),-1);
    n->param<double>("max_y", max.y(),1);
    n->param<double>("min_z", min.z(),-1);
    n->param<double>("max_z", max.z(),1);
    n->param<double>("planner_step_size", step_size,0.05);
    n->param<double>("plan_timeout", plan_timeout,10.0);
    planner->setPlannerOptions(step_size,plan_timeout,min,max);
    planner->setupPlanner();

    //nh.param<float>("sq_clearance", clearance,0.0001);
    //nh.param<float>("min_intensity", min_intensity,5);

}
bool PlanFsmoreROS::checkValid(fsmore::CheckPoseValid::Request  &req,
                               fsmore::CheckPoseValid::Response &res, PlanFsmore* planner){

    if(!setupOctrees(planner)) return(false);

    initializePlanner();
    res.valid=planner->isValid(req.pose);
    return(true);

}
bool PlanFsmoreROS::setupOctrees(PlanFsmore* planner){
    if(srv_oct_map.exists() && srv_oct_obj.exists()){
        octomap_msgs::GetOctomap srv1,srv2;
        srv_oct_map.call(srv1.request,srv1.response);
        pub_map.publish(srv1.response.map);
        srv_oct_obj.call(srv2.request,srv2.response);
        srv2.response.map.header.stamp=ros::Time::now();
        pub_obj.publish(srv2.response.map);

        ROS_WARN("Size: %zu %zu",srv1.response.map.data.size(),srv2.response.map.data.size());
        octomap::AbstractOcTree *tree_map,*tree_obj;
        tree_map = octomap_msgs::msgToMap(srv1.response.map);
        planner->setMapOctree(*(dynamic_cast<octomap::OcTree*>(tree_map)));
        tree_obj = octomap_msgs::msgToMap(srv2.response.map);
        planner->setObjOctree(*(dynamic_cast<octomap::OcTree*>(tree_obj)));
        return(true);
    }
    else{
        return(false);
    }
}

bool PlanFsmoreROS::planPath(nav_msgs::GetPlan::Request  &req,
                             nav_msgs::GetPlan::Response &res, PlanFsmore* planner)
{

    printf("STARTING THE SERVICE THING\n");

    if(!setupOctrees(planner)) return(false);

    initializePlanner();
    geometry_msgs::PoseArray pose_solutions,pose_vertexes;

    ompl::base::PlannerStatus success = planner->getPlan(req.start.pose,req.goal.pose,pose_solutions,pose_vertexes);
    if(!success){
        return(false);
    }

    publishMarkerArray(pose_solutions.poses);

    pose_vertexes.header.frame_id=world_frame;
    pose_vertexes.header.stamp=ros::Time::now();
    pub_plan.publish(pose_vertexes);

    for (size_t i=0;i<pose_solutions.poses.size();i++){
        geometry_msgs::PoseStamped poseS;
        poseS.pose=pose_solutions.poses.at(i);
        res.plan.poses.push_back(poseS);
    }

    res.plan.header.frame_id=req.start.header.frame_id;
    res.plan.header.stamp=ros::Time::now();
    return true;
}


void PlanFsmoreROS::publishMarkerArray(std::vector<geometry_msgs::Pose> poses) {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker m;
    m.type=visualization_msgs::Marker::MESH_RESOURCE;
    m.action=visualization_msgs::Marker::ADD;
    m.scale.x=0.001;
    m.scale.y=0.001;
    m.scale.z=0.001;
    m.color.a=0.1f;
    m.color.b=1.0;
    n->param<std::string>("world_frame",world_frame,"world");
    m.header.frame_id=world_frame;
    m.mesh_resource="file://"+mesh_filename;

    for(size_t i=0;i<poses.size();i++){
        m.pose=poses.at(i);
        m.header.stamp=ros::Time::now();
        //m.lifetime=ros::Duration(100.0);
        m.id=static_cast<int>(i);
        m.ns=std::to_string(i);
        float color = static_cast<float>(i)/static_cast<float>(poses.size());
        m.color.g = 1-color;
        m.color.r = color;
        m.color.a = 0.1f;
        markers.markers.push_back(m);
    }
    pub_plan_markers.publish(markers);
}
