#include "Parse_OpenMesh>FCL/get_OpenMesh_data.h"

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include "isValid_class.h"
#include "Sphere_handle/WorkspaceSphereExplorer.h"
//Here we need to define everything from the OMPL part
namespace ob = ompl::base;
namespace og = ompl::geometric;

int main(){
    //std::shared_ptr<fcl::BVHModel<fcl::OBBRSS>>
    typedef fcl::BVHModel<fcl::OBBRSS> Model;
    std::shared_ptr<Model> model;
    //Loading model from file
    try {
        model = get_OpenMesh_data::model(
                "/home/vasko/CLionProjects/Open_mesh_getData/cmake-build-debug/BugTrap_planar_env.ply", true);
    }catch (std::exception& e ) {
        std::cout << "Error " << e.what() << std::endl;
    }
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE3StateSpace>());
    /*      Setting Upper bounds
     * X=55,Y=55.01,Z=8.17
     *      Setting Lower bounds
     * X=-55,Y=-55.01,Z=0 */
    // set the bounds for the R^3 part of SE(3)
    ob::RealVectorBounds bounds(3);

    bounds.setLow(0,-55);       //For X
    bounds.setHigh(0,55);
    bounds.setLow(1,-8.17);         //For Y
    bounds.setHigh(1,0);
    bounds.setLow(2,-55.01);    //For Z
    bounds.setHigh(2,55.01);

    space->setBounds(bounds);

    // construct an instance of  space information from this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));
//    ValidityChecker FCL_State(model,si);
//    auto FCL_shared_state(std::make_shared(FCL_State));

    auto FCL_State=std::make_shared<ValidityChecker> (model,si);



    // set state validity checking for this space
    si->setStateValidityChecker(FCL_State);
    ob::ScopedState<> start(space);
    ob::ScopedState<> goal(space);
    while(true) {
        // create a random start and goal state
        start.random();
        goal.random();
        if (FCL_State->isValid(start.get()) ) break;
        std::cout<<"Print the wrong start space"<<std::endl;
        start.print(std::cout);

    }
    std::cout<<"Print the start space"<<std::endl;
    start.print(std::cout);

    // create a random goal state

    std::cout<<"Print the goal space"<<std::endl;
    goal.print(std::cout);

    //FOR THE SPHERE TREE
    rl::plan::WorkspaceSphereExplorer SphereTree; //Making the object for the tree exploration
    SphereTree.model=model; //Adding a FCL model to the SphereTree
    SphereTree.radius=2; //According to enviroment details

    typedef ::Eigen::Matrix<double, 3, 1> Vec3; //For the seeding start and goal to SphereTree

    //Cast the start state
//    std::vector<double> start_state(start.reals()); //Contains in position 0,1,2 a Vector and in 3,4,5,6 SO3 orientation in quaternion
//    auto start_vector=Vec3(start_state[0],start_state[1],start_state[2]); //For using casted start_state
    auto start_vector=Vec3(-20,-3,-30); //For seeding manual start_state
    SphereTree.start=&start_vector;

    //Cast the goal state
//    std::vector<double> goal_state(goal.reals()); //Contains in position 0,1,2 a Vector and in 3,4,5,6 SO3 quaternion
//    auto goal_vector=Vec3(goal_state[0],goal_state[1],goal_state[2]); //For using casted goal_state
    auto goal_vector=Vec3(0,-3,0); //For seeding manual goal_state
    SphereTree.goal=&goal_vector;
    std::cout<<"Now start exploring"<<std::endl;
    //Explore the space
    //********************************************************************
    /*      Possible types of exploration default:GREEDY_DISTANCE
     *      GREEDY_DISTANCE,
            GREEDY_SOURCE_DISTANCE,
            GREEDY_SPACE*/

//    SphereTree.greedy=rl::plan::WorkspaceSphereExplorer::GREEDY_SPACE;
    if(SphereTree.explore()){
        std::cout<<"Tree explorations SUCESSFULL"<<std::endl;
    }
    OpenMesh::TriMesh_ArrayKernelT<> mesh=SphereTree.meshOpenMesh;

    if ( !OpenMesh::IO::write_mesh(mesh, "GREEDY_SPACE.ply") )
    {
        std::cerr << "Cannot write mesh to file 'GREEDY_SPACE.ply'" << std::endl;
        return 1;
    }


    //END FOR THE SPHERE TREE

    //FOR OMPL PROBLEM DEFINITION AND EXAMPLE OF ADDING ALGORITHM TO THE  PLANNER
    //FOR FUTURE DEVELOPMENT, NOT IN USE FOR NOW

    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // create a planner for the defined space
    auto planner(std::make_shared<ompl::geometric::RRTConnect>(si));

    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();


    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(10.0);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ob::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        path->print(std::cout);
    }
    else
        std::cout << "No solution found" << std::endl;
    //END FOR OMPL PROBLEM DEFINITION

}