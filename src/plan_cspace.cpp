#include <fsmore/plan_cspace.h>

Plan_CSpace::Plan_CSpace() :
bounds(2)
{

}

Plan_CSpace::Plan_CSpace(ros::NodeHandle* n) :
bounds(2)
{
    this->nh=n;
    sub1=nh->subscribe("/contact_force",1,&Plan_CSpace::contact_cb,this);
    sub2=nh->subscribe("/robot_ee",1,&Plan_CSpace::pose_cb,this);
}

void Plan_CSpace::contact_cb(const geometry_msgs::WrenchStamped &msg){
    double force_mag=sqrt(msg.wrench.force.x*msg.wrench.force.x+
                          msg.wrench.force.y*msg.wrench.force.y+
                          msg.wrench.force.z*msg.wrench.force.z);
    if(force_mag>5.0){

        occupied_poses.push_back(pose3Dto2D(current_pose.pose));
        occupied_poses3D.push_back(current_pose.pose);
        ROS_INFO("BUMP! %f",force_mag);
    }

}

void Plan_CSpace::pose_cb(const geometry_msgs::PoseStamped &msg){
    current_pose=msg;
}

double Plan_CSpace::diff_pose(geometry_msgs::Pose2D p1, const ompl::base::SE2StateSpace::StateType *p2){
    double pdiff=sqrt((p1.x-p2->getX())*(p1.x-p2->getX())+(p1.y-p2->getY())*(p1.y-p2->getY()));
    double adiff=fabs(p1.theta-p2->getYaw());
    return(pdiff+adiff);
}


geometry_msgs::Pose2D Plan_CSpace::pose3Dto2D(geometry_msgs::Pose in){
    geometry_msgs::Pose2D pose;
    pose.x=in.position.x;
    pose.y=in.position.y;
    pose.theta=asin(-2*in.orientation.x*in.orientation.y);
    return(pose);
}



void Plan_CSpace::setupPlanner(){
    space=std::make_shared<ompl::base::SE2StateSpace>();
    space->setBounds(bounds);
    si=std::make_shared<ompl::base::SpaceInformation>(space);
    pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
    boost::function<bool (const ompl::base::State*)> isStateValid_handle( boost::bind( &Plan_CSpace::isStateValid, this, _1 ) );
    si->setStateValidityChecker(isStateValid_handle);
    si->setup();

}

void Plan_CSpace::setStartAndGoal(geometry_msgs::Pose start_pose,geometry_msgs::Pose goal_pose){

    //    std::shared_ptr<ompl::base::ProblemDefinition> pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(space);
    start->setXY(start_pose.position.x,start_pose.position.y);
    start->setYaw(asin(-2*start_pose.orientation.x*start_pose.orientation.y));

    ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(space);
    goal->setXY(goal_pose.position.x,goal_pose.position.y);
    start->setYaw(asin(-2*goal_pose.orientation.x*goal_pose.orientation.y));

    pdef->setStartAndGoalStates(start,goal);
}

void Plan_CSpace::setPlannerOptions(double step_siz,double timeout,Eigen::Vector3d min_bound, Eigen::Vector3d max_bound){
    step_size=step_siz;
    plan_timeout=timeout;
    setBounds(min_bound,max_bound);
}

void Plan_CSpace::setBounds(Eigen::Vector3d min, Eigen::Vector3d max){
    bounds.setLow(0,min.x());
    bounds.setHigh(0,max.x());
    bounds.setLow(1,min.y());
    bounds.setHigh(1,max.y());

}

bool Plan_CSpace::isValid(geometry_msgs::Pose pose){
    ompl::base::SE2StateSpace::StateType state_checked;
    state_checked.setXY(pose.position.x,pose.position.y);
    state_checked.setYaw(asin(-2*pose.orientation.x*pose.orientation.y));
    return(isStateValid(&state_checked));
}


bool Plan_CSpace::isStateValid(const ompl::base::State *state){
    const ompl::base::SE2StateSpace::StateType *se2state = state->as<ompl::base::SE2StateSpace::StateType>();
    for(auto it=occupied_poses.begin();it!=occupied_poses.end();it++){
        if(diff_pose(*it,se2state)<0.05){
            ROS_INFO("BIMP!");
            return(false);
        }
    }
    return(true);
}


ompl::base::PlannerStatus Plan_CSpace::getPlan(geometry_msgs::Pose start, geometry_msgs::Pose goal,geometry_msgs::PoseArray &solution,geometry_msgs::PoseArray &vertexes){
    ompl::base::PlannerData data(si);
    std::vector<geometry_msgs::Pose> plan;
    setStartAndGoal(start,goal);
    std::shared_ptr<ompl::geometric::RRT> planner(std::make_shared<ompl::geometric::RRT>(si));
    //std::shared_ptr<ompl::geometric::KPIECE1> planner(std::make_shared<ompl::geometric::KPIECE1>(si));
    //plan_ptr=std::make_shared<ompl::base::Planner>(planner);

    planner->setProblemDefinition(pdef);


    try{
        planner->checkValidity();
    }
    catch(std::exception e){
        return ompl::base::PlannerStatus::INVALID_START;
    }

    printf("Planner2D: Starting to solve...\n");


    planner->setRange(step_size);
    //planner->setGoalBias(0.001);
    planner->setup();

    ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(static_cast<double>(plan_timeout));
    printf("Solution: %s\n",solved.asString().c_str());
    if (solved)
    {
        ompl::base::PathPtr path = pdef->getSolutionPath();
        ompl::geometric::PathGeometric* path_geo = path->as<ompl::geometric::PathGeometric>();
        for (size_t i=0;i<path_geo->getStateCount();i++){
            ompl::base::SE2StateSpace::StateType* state = path_geo->getState(i)->as<ompl::base::SE2StateSpace::StateType>();
            geometry_msgs::Pose pose = pose2Dto3D(state->getX(),state->getY(),start.position.z,state->getYaw());
            solution.poses.push_back(pose);
        }
    }

    planner->getPlannerData(data);
/*
    for (size_t i=0;i<data.numVertices();i++) {
        const ompl::base::SE2StateSpace::StateType *state = data.getVertex(i).getState()->as<ompl::base::SE2StateSpace::StateType>();
        vertexes.poses.push_back(pose2Dto3D(state->getX(),state->getY(),start.position.z,state->getYaw()));
    }
*/
    for (size_t i=0;i<occupied_poses3D.size();i++) {
        vertexes.poses.push_back(occupied_poses3D.at(i));
    }

    return(solved);

}


geometry_msgs::Pose Plan_CSpace::pose2Dto3D(double x,double y,double z,double ang){
    geometry_msgs::Pose pose;
    pose.position.x=x;
    pose.position.y=y;
    pose.position.z=z;

    pose.orientation.w=0;
    pose.orientation.x=sin(-ang/2);
    pose.orientation.y=cos(-ang/2);
    pose.orientation.z=0;
    return(pose);
}
