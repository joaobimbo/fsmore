#include <fsmore/plan_fsmore_2d.h>

PlanFsmore_2D::PlanFsmore_2D() :
    bounds(2)
{

}

void PlanFsmore_2D::setupPlanner(){
    space=std::make_shared<ompl::base::SE2StateSpace>();
    space->setBounds(bounds);
    si=std::make_shared<ompl::base::SpaceInformation>(space);
    pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
    boost::function<bool (const ompl::base::State*)> isStateValid_handle( boost::bind( &PlanFsmore_2D::isStateValid, this, _1 ) );
    si->setStateValidityChecker(isStateValid_handle);

}

void PlanFsmore_2D::setStartAndGoal(geometry_msgs::Pose start_pose,geometry_msgs::Pose goal_pose){

    //    std::shared_ptr<ompl::base::ProblemDefinition> pdef(std::make_shared<ompl::base::ProblemDefinition>(si));
    ompl::base::ScopedState<ompl::base::SE2StateSpace> start(space);
    start->setXY(start_pose.position.x,start_pose.position.y);
    start->setYaw(asin(-2*start_pose.orientation.x*start_pose.orientation.y));

    ompl::base::ScopedState<ompl::base::SE2StateSpace> goal(space);
    goal->setXY(goal_pose.position.x,goal_pose.position.y);
    start->setYaw(asin(-2*goal_pose.orientation.x*goal_pose.orientation.y));

    pdef->setStartAndGoalStates(start,goal);
}

void PlanFsmore_2D::setPlannerOptions(double step_siz,double timeout,Eigen::Vector3d min_bound, Eigen::Vector3d max_bound){
    step_size=step_siz;
    plan_timeout=timeout;
    setBounds(min_bound,max_bound);
}

void PlanFsmore_2D::setBounds(Eigen::Vector3d min, Eigen::Vector3d max){
    bounds.setLow(0,min.x());
    bounds.setHigh(0,max.x());
    bounds.setLow(1,min.y());
    bounds.setHigh(1,max.y());

}

bool PlanFsmore_2D::isStateValid(const ompl::base::State *state){
    const ompl::base::SE2StateSpace::StateType *se2state = state->as<ompl::base::SE2StateSpace::StateType>();
    //printf("checking state: %f %f %f\n",se2state->getX(),se2state->getY(),se2state->getYaw());

    ros::Time ts=ros::Time::now();
    fcl::CollisionRequest<float> req;    
    req.enable_contact=true;
    req.num_max_contacts=10;
    fcl::CollisionResult<float> res;
    fcl::Transform3f T1,T2;
    T1.setIdentity();
    float ang = static_cast<float>(se2state->getYaw());
    T2 =  Eigen::Translation3f(se2state->getX(),se2state->getY(),height_z) * Eigen::Quaternionf(0,sin(-ang/2),cos(-ang/2),0);
    bool ret=fcl::collide(col_map,T1,col_obj,T2,req,res);

    ros::Duration d=ros::Time::now()-ts;
    printf("NUM: %d %d - %fs\n",col_map->tree->getNumLeafNodes(),col_obj->tree->getNumLeafNodes(),d.toSec());
    size_t n_contacts=res.numContacts();
    //    printf("NC: %d\n",n_contacts);
    //    for (int i=0;i<n_contacts;i++){
    //        fcl::Contactf c=res.getContact(i);
    //        std::cout << "Contact " << i << ":\n" << c.pos << "\n";
    //    }
    if(n_contacts>2) return(false);
    else return(true);

    bool whatever=false;
    if(n_contacts>0){
        for (int i=0;i<n_contacts;i++){
            fcl::Contactf c=res.getContact(i);
            std::cout << "Contact " << i << "\n";
            if(col_map->tree->search(c.pos.x(),c.pos.y(),c.pos.z())!=NULL){
                std::cout << c.pos << "\n" << "Occ: " << col_map->tree->search(c.pos.x(),c.pos.y(),c.pos.z())->getOccupancy();
                whatever=true;
            }
            else std::cout << "NULL\n";
        }
        if(whatever) return(false);
    }
    return(true);
}


ompl::base::PlannerStatus PlanFsmore_2D::getPlan(geometry_msgs::Pose start, geometry_msgs::Pose goal,geometry_msgs::PoseArray &solution,geometry_msgs::PoseArray &vertexes){
    ompl::base::PlannerData data(si);
    std::vector<geometry_msgs::Pose> plan;
    setStartAndGoal(start,goal);
    std::shared_ptr<ompl::geometric::RRT> planner(std::make_shared<ompl::geometric::RRT>(si));
    //plan_ptr=std::make_shared<ompl::base::Planner>(planner);

    planner->setProblemDefinition(pdef);

    try{
        planner->checkValidity();
    }
    catch(std::exception e){
        return ompl::base::PlannerStatus::INVALID_START;
    }


    std::vector<std::array<float,6> > boxes =  col_map->toBoxes();
    printf("box1\n");
    for (int i=0;i<boxes.size();i+=1000){


        printf("%d - %f %f %f - %f %f %f - %f\n",
               i,boxes.at(i).at(0),boxes.at(i).at(1),
               boxes.at(i).at(2),boxes.at(i).at(3),
               boxes.at(i).at(4),boxes.at(i).at(5),
               col_map->tree->search(boxes.at(i).at(0),boxes.at(i).at(1),boxes.at(i).at(2))->getValue());
    }
    std::vector<std::array<float,6> > boxes2 =  col_obj->toBoxes();
    printf("box2\n");
    for (int i=0;i<boxes2.size();i+=1000){
        printf("|%d - %f %f %f - %f %f %f - %f\n",
               i,boxes2.at(i).at(0),boxes2.at(i).at(1),
               boxes2.at(i).at(2),boxes2.at(i).at(3),
               boxes2.at(i).at(4),boxes2.at(i).at(5),
               col_obj->tree->search(boxes2.at(i).at(0),boxes2.at(i).at(1),boxes2.at(i).at(2))->getValue());
    }


    printf("Planner2D: Starting to solve...\n");


    planner->setRange(step_size);
    planner->setGoalBias(0.01);
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
    for (int i=0;i<data.numVertices();i++) {
        const ompl::base::SE2StateSpace::StateType *state = data.getVertex(i).getState()->as<ompl::base::SE2StateSpace::StateType>();
        vertexes.poses.push_back(pose2Dto3D(state->getX(),state->getY(),start.position.z,state->getYaw()));
    }

    return(solved);

}

geometry_msgs::Pose PlanFsmore_2D::pose2Dto3D(double x,double y,double z,double ang){
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
