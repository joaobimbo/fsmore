#include <fsmore/plan_fsmore_ros.h>

PlanFsmoreROS::PlanFsmoreROS()
{
    n=new ros::NodeHandle();
    PlanFsmore_2D planner2D;
    planner = &planner2D;

    boost::function<bool (nav_msgs::GetPlan::Request&,nav_msgs::GetPlan::Response&)> planPath_handle( boost::bind(&PlanFsmoreROS::planPath,this, _1,_2,&planner2D) );
    plan_service = n->advertiseService("get_plan", planPath_handle);
    pub_plan_markers = n->advertise<visualization_msgs::MarkerArray>("plan_steps",1);
    //sub_oct_map = n->subscribe("/oct_map",2,&PlanFsmoreROS::cb_oct_map,this);
    //sub_oct_obj = n->subscribe("/oct_obj",2,&PlanFsmoreROS::cb_oct_obj,this);

    srv_oct_map = n->serviceClient<octomap_msgs::GetOctomap>("/get_oct_map",false);
    srv_oct_obj = n->serviceClient<octomap_msgs::GetOctomap>("/get_oct_obj",false);

}

//void PlanFsmoreROS::cb_oct_map(const octomap_msgs::Octomap::ConstPtr& msg){
//    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
//    planner->oct_map = dynamic_cast<octomap::OcTree*>(tree);
//}

//void PlanFsmoreROS::cb_oct_obj(const octomap_msgs::Octomap::ConstPtr& msg){
//    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
//    planner->oct_obj = dynamic_cast<octomap::OcTree*>(tree);
//}


bool PlanFsmoreROS::planPath(nav_msgs::GetPlan::Request  &req,
                             nav_msgs::GetPlan::Response &res, PlanFsmore* plan)
{

    if(srv_oct_map.exists() && srv_oct_obj.exists()){
        octomap_msgs::GetOctomap srv1,srv2;
        srv_oct_map.call(srv1.request,srv1.response);
        srv_oct_obj.call(srv2.request,srv2.response);
        ROS_WARN("Size: %d %d",srv1.response.map.data.size(),srv2.response.map.data.size());
        octomap::AbstractOcTree* tree;
        tree = octomap_msgs::msgToMap(srv1.response.map);
        plan->setMapOctree(*(dynamic_cast<octomap::OcTree*>(tree)));

        tree = octomap_msgs::msgToMap(srv2.response.map);
        plan->setObjOctree(*(dynamic_cast<octomap::OcTree*>(tree)));
    }

    std::vector<geometry_msgs::Pose> poses=plan->getPlan(req.start.pose,req.goal.pose);

    if(poses.size()<2) {
        return(false);
    }

    publishMarkerArray(poses);

    for (size_t i=0;i<poses.size();i++){
        geometry_msgs::PoseStamped poseS;
        poseS.pose=poses.at(i);
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
    m.color.a=0.5;
    m.color.b=1.0;
    n->param<std::string>("world_frame",world_frame,"world");
    m.header.frame_id=world_frame;
    m.mesh_resource="file://"+mesh_filename;

    for(size_t i=0;i<poses.size();i++){
        m.pose=poses.at(i);
        m.header.stamp=ros::Time::now();
        m.lifetime=ros::Duration(10.0);
        m.id=static_cast<int>(i);
        m.ns=std::to_string(i);
        float color = static_cast<float>(i)/static_cast<float>(poses.size());
        m.color.g = 1-color;
        m.color.r = color;
        m.color.a = 0.2f;
        markers.markers.push_back(m);
    }
    pub_plan_markers.publish(markers);
}