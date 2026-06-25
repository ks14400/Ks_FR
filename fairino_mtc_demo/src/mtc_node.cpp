#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/task_constructor/task.h>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>


static const rclcpp::Logger LOGGER = rclcpp::get_logger("fairino_mtc_demo");

namespace mtc = moveit::task_constructor;

class MTCTaskNode{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);
  ~MTCTaskNode();
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface();
    
  void setupPlanningScene();

  void doTask();
  void doReverseTask();

private:
  mtc::Task _createTask();
  mtc::Task _createReverseTask();
  mtc::Task _task;
  mtc::Task _reverse_task;
  rclcpp::Node::SharedPtr _node;
};

MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  :_node{std::make_shared<rclcpp::Node>("fairino_mtc_demo_node",options)}
{
  

}

MTCTaskNode::~MTCTaskNode(){

  
}

rclcpp::node_interfaces::NodeBaseInterface::SharedPtr MTCTaskNode::get_node_base_interface(){
  return _node->get_node_base_interface();
}


void MTCTaskNode::setupPlanningScene(){
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";
  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = {0.1,0.02};

  geometry_msgs::msg::Pose pose;
  pose.position.x = 0.1;
  pose.position.y = -0.55;
  pose.position.z = 0.1;
  pose.orientation.w = 1.0;
  object.pose = pose;

  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);

}


void MTCTaskNode::doTask(){
  _task = _createTask();
  RCLCPP_INFO(LOGGER,"start to init task!");
  try{
    _task.init();
  }
  catch(mtc::InitStageException& e){
    RCLCPP_ERROR_STREAM(LOGGER,e);
    return;
  }
  try{
    if(!_task.plan(10)){
      RCLCPP_ERROR_STREAM(LOGGER,"Task planning failed!");
    }
  }catch(moveit::task_constructor::InitStageException& e){
    RCLCPP_ERROR_STREAM(LOGGER,e);
  }

  for(auto item : _task.solutions()){
      _task.introspection().publishSolution(*item);
      auto result = _task.execute(*item);
      if(result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
        RCLCPP_ERROR_STREAM(LOGGER,"Task execution failed!");
      return;
    }
  }
  RCLCPP_INFO(LOGGER,"Task execution SUCCESS!");
}



mtc::Task MTCTaskNode::_createTask(){
  mtc::Task task;
  task.stages()->setName("fairino_mtc_task");
  task.loadRobotModel(_node);
  const auto& arm_group_name = "fairino5_v6_group";
  const auto& tip_frame = "wrist3_link";
  
  task.setProperty("group",arm_group_name);
  task.setProperty("ik_frame",tip_frame);

  mtc::Stage* current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));
  
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(_node);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();

  //第一个动作，移动到取物体的上方
  // auto stage_move_robot_to_ready = std::make_unique<mtc::stages::MoveTo>("moveready",sampling_planner);
  // stage_move_robot_to_ready->setGroup(arm_group_name);
  // stage_move_robot_to_ready->setTimeout(10);
  // stage_move_robot_to_ready->setGoal("pos1");
  // task.add(std::move(stage_move_robot_to_ready));




  auto stage_move_robot = std::make_unique<mtc::stages::MoveRelative>("movepos1",cartesian_planner);
  stage_move_robot->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_move_robot->setMinMaxDistance(0.1, 0.2);
  stage_move_robot->setIKFrame(tip_frame);
  stage_move_robot->properties().set("marker_ns", "movepos1");
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.x = 0;
  vec.vector.y = -1;
  vec.vector.z = 0;
  stage_move_robot->setDirection(vec);
  task.add(std::move(stage_move_robot));


  //第二步，向下移动靠近物体
  auto stage = std::make_unique<mtc::stages::MoveRelative>("approch object", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.2);
  stage->setIKFrame(tip_frame);
  stage->properties().set("marker_ns", "approch object");

  //geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.x = 0;
  vec.vector.y = 0;
  vec.vector.z = -1;
  stage->setDirection(vec);
  task.add(std::move(stage));
 

  //抓取物体
  auto stage_grap = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  stage_grap->attachObject("object", tip_frame);
  current_state_ptr = stage_grap.get();
  task.add(std::move(stage_grap));


  //提升物体
  auto stage_lift = std::make_unique<mtc::stages::MoveRelative>("liftobj", cartesian_planner);
  stage_lift->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_lift->setMinMaxDistance(0.1, 0.2);
  stage_lift->setIKFrame(tip_frame);
  stage_lift->properties().set("marker_ns", "liftobj");
  vec.vector.x = 0;
  vec.vector.y = 0;
  vec.vector.z = 1;//设置相对移动距离
  stage_lift->setDirection(vec);
  task.add(std::move(stage_lift));


  //移动到放置位置
  // auto stage_move_robot2 = std::make_unique<mtc::stages::MoveTo>("movepos2",sampling_planner);
  // stage_move_robot2->setGroup(arm_group_name);
  // stage_move_robot2->setGoal("pos2");
  // stage_move_robot2->setTimeout(10);
  auto stage_move_robot2 = std::make_unique<mtc::stages::MoveRelative>("movepos2",cartesian_planner);
  stage_move_robot2->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_move_robot2->setMinMaxDistance(0.1, 0.2);
  stage_move_robot2->setIKFrame(tip_frame);
  stage_move_robot2->properties().set("marker_ns", "movepos2");
  vec.header.frame_id = "world";
  vec.vector.x = -1;
  vec.vector.y = 0;
  vec.vector.z = 0;
  stage_move_robot2->setDirection(vec);
  task.add(std::move(stage_move_robot2));

  //向下移动放置物体
  auto stage_put = std::make_unique<mtc::stages::MoveRelative>("putobj", cartesian_planner);
  stage_put->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_put->setMinMaxDistance(0.1, 0.2);
  stage_put->setIKFrame(tip_frame);
  stage_put->properties().set("marker_ns", "putobj");
  vec.vector.x = 0;
  vec.vector.y = 0;
  vec.vector.z = -1;//设置相对移动距离
  stage_put->setDirection(vec);
  task.add(std::move(stage_put));

  //放下物体
  auto stage_detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  stage_detach->detachObject("object", tip_frame);
  task.add(std::move(stage_detach));

  //抬起机械臂
  auto stage_end = std::make_unique<mtc::stages::MoveRelative>("returnhome", cartesian_planner);
  stage_end->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_end->setMinMaxDistance(0.1,0.2);
  stage_end->setIKFrame(tip_frame);
  stage_end->properties().set("marker_ns", "returnhome");
  vec.vector.x = 0;
  vec.vector.y = 0;
  vec.vector.z = 1;//设置相对移动距离
  stage_end->setDirection(vec);
  
  task.add(std::move(stage_end));

  return task;
}


void MTCTaskNode::doReverseTask(){
  _task.clear();
  _task = _createReverseTask();
  RCLCPP_INFO(LOGGER,"start to init reverse task!");
  try{
    _task.init();
  }
  catch(mtc::InitStageException& e){
    RCLCPP_ERROR_STREAM(LOGGER,e);
    return;
  }
  try{
    if(!_task.plan(10)){
      RCLCPP_ERROR_STREAM(LOGGER,"Reverse Task planning failed!");
    }
  }catch(moveit::task_constructor::InitStageException& e){
    RCLCPP_ERROR_STREAM(LOGGER,e);
  }

  for(auto item : _task.solutions()){
      _task.introspection().publishSolution(*item);
      auto result = _task.execute(*item);
      if(result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS){
        RCLCPP_ERROR_STREAM(LOGGER,"Reverse Task execution failed!");
      return;
    }
  }
  RCLCPP_INFO(LOGGER,"Reverse Task execution SUCCESS!");
  _task.clear();
}

mtc::Task MTCTaskNode::_createReverseTask(){
  mtc::Task task;
  task.stages()->setName("fairino_mtc_reverse_task");
  task.loadRobotModel(_node);
  const auto& arm_group_name = "fairino5_v6_group";
  const auto& tip_frame = "wrist3_link";
  
  task.setProperty("group",arm_group_name);
  task.setProperty("ik_frame",tip_frame);

  mtc::Stage* current_state_ptr = nullptr;
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));
  
  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(_node);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();


  //第1步，向下移动靠近物体
  auto stage = std::make_unique<mtc::stages::MoveRelative>("approch object", cartesian_planner);
  stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage->setMinMaxDistance(0.1, 0.2);
  stage->setIKFrame(tip_frame);
  stage->properties().set("marker_ns", "approch object");
  geometry_msgs::msg::Vector3Stamped vec;
  vec.header.frame_id = "world";
  vec.vector.x = 0;
  vec.vector.y = 0;
  vec.vector.z = -1;
  stage->setDirection(vec);
  task.add(std::move(stage));
 

  //抓取物体
  auto stage_grap = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
  stage_grap->attachObject("object", tip_frame);
  current_state_ptr = stage_grap.get();
  task.add(std::move(stage_grap));


  //提升物体
  auto stage_lift = std::make_unique<mtc::stages::MoveRelative>("liftobj", cartesian_planner);
  stage_lift->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_lift->setMinMaxDistance(0.1, 0.2);
  stage_lift->setIKFrame(tip_frame);
  stage_lift->properties().set("marker_ns", "liftobj");
  vec.vector.x = 0;
  vec.vector.y = 0;
  vec.vector.z = 1;//设置相对移动距离
  stage_lift->setDirection(vec);
  task.add(std::move(stage_lift));


  //移动到放置位置
  // auto stage_move_robot2 = std::make_unique<mtc::stages::MoveTo>("movepos2",sampling_planner);
  // stage_move_robot2->setGroup(arm_group_name);
  // stage_move_robot2->setGoal("pos2");
  // stage_move_robot2->setTimeout(10);
  auto stage_move_robot = std::make_unique<mtc::stages::MoveRelative>("moveback",cartesian_planner);
  stage_move_robot->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_move_robot->setMinMaxDistance(0.1, 0.2);
  stage_move_robot->setIKFrame(tip_frame);
  stage_move_robot->properties().set("marker_ns", "moveback");
  vec.header.frame_id = "world";
  vec.vector.x = 1;
  vec.vector.y = 0;
  vec.vector.z = 0;
  stage_move_robot->setDirection(vec);
  task.add(std::move(stage_move_robot));


  //向下移动防置物体
  auto stage_put = std::make_unique<mtc::stages::MoveRelative>("putobj", cartesian_planner);
  stage_put->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_put->setMinMaxDistance(0.1, 0.2);
  stage_put->setIKFrame(tip_frame);
  stage_put->properties().set("marker_ns", "putobj");
  vec.vector.x = 0;
  vec.vector.y = 0;
  vec.vector.z = -1;//设置相对移动距离
  stage_put->setDirection(vec);
  task.add(std::move(stage_put));

  //放下物体
  auto stage_detach = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
  stage_detach->detachObject("object", tip_frame);
  task.add(std::move(stage_detach));

  //抬起机械臂
  auto stage_end = std::make_unique<mtc::stages::MoveRelative>("liftarm", cartesian_planner);
  stage_end->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_end->setMinMaxDistance(0.1,0.2);
  stage_end->setIKFrame(tip_frame);
  stage_end->properties().set("marker_ns", "liftarm");
  vec.vector.x = 0;
  vec.vector.y = 0;
  vec.vector.z = 1;//设置相对移动距离
  stage_end->setDirection(vec);
  task.add(std::move(stage_end));

  auto stage_move_robot2 = std::make_unique<mtc::stages::MoveRelative>("returnstart",cartesian_planner);
  stage_move_robot2->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
  stage_move_robot2->setMinMaxDistance(0.1, 0.2);
  stage_move_robot2->setIKFrame(tip_frame);
  stage_move_robot2->properties().set("marker_ns", "returnstart");
  vec.header.frame_id = "world";
  vec.vector.x = 0;
  vec.vector.y = 1;
  vec.vector.z = 0;
  stage_move_robot2->setDirection(vec);
  task.add(std::move(stage_move_robot2));


  return task;
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);
  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>([&executor, &mtc_task_node]() {
    executor.add_node(mtc_task_node->get_node_base_interface());
    executor.spin();
    executor.remove_node(mtc_task_node->get_node_base_interface());
  });

  mtc_task_node->setupPlanningScene(); 

  for(int i=0;i<1000;i++){
    mtc_task_node->doTask();
    mtc_task_node->doReverseTask();
  }

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}
