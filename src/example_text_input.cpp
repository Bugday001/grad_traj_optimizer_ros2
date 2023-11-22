#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <fstream>
#include <time.h>
#include <stdlib.h>
#include "display.h"
#include "grad_traj_optimizer.h"

using namespace std;
using ::std::placeholders::_1;

class MinimalPublisher : public rclcpp::Node
{
  public:
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr setpoint_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_point_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr traj_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr uav_pose;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr target_sub_;

    GradTrajOptimizer* grad_traj_opt;
    Configs configs;
    rclcpp::TimerBase::SharedPtr cmdTimer, optTimer;
    int point_num;
    int seg; //当前段数
    double start_time, history_time;
    Eigen::MatrixXd coeff;  //系数  
    enum State{
      WAIT_TARGET, WAIT_OPTIM, WAIT_FLYING
    } planner_state;
    vector<string> state_str = {"WAIT_TARGET", "WAIT_OPTIM", "WAIT_FLYING"};
    
    /* 公共构造函数将节点命名为minimal_publisher，并将count_初始化为0 */
    MinimalPublisher()
    : Node("minimal_publisher")
    {
      initSys();
      this->setpoint_pub = this->create_publisher<visualization_msgs::msg::Marker>("trajopt/setpoint", 10);
      traj_point_pub = this->create_publisher<visualization_msgs::msg::Marker>("trajopt/traj_point", 10);
      traj_pub = this->create_publisher<nav_msgs::msg::Path>("trajopt/init_traj", 5);
      uav_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("trajopt/uavPose", 5);
      target_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "target_path", 10, std::bind(&MinimalPublisher::targetCallback, this, _1));
      cmdTimer = this->create_wall_timer(10ms, std::bind(&MinimalPublisher::cmdCallback, this));
      optTimer = this->create_wall_timer(1s, std::bind(&MinimalPublisher::runOptimize, this));
    }
    void initSys()
    {
      initParam();
      this->seg = 0;
      this->start_time = 0;
      this->history_time = 0;
    }

    /**
     * 获取规划器参数
    */
    void initParam() {
      configs.algorithm = declare_parameter<int>("/traj_opti_node1/alg", configs.algorithm);
      configs.time_limit_1 = declare_parameter<double>("/traj_opti_node1/time_limit_1", configs.time_limit_1);
      configs.time_limit_2 = declare_parameter<double>("/traj_opti_node1/time_limit_2", configs.time_limit_2);
      configs.try_limit = declare_parameter<double>("/traj_opti_node1/try_limit", configs.try_limit);
      configs.offset = declare_parameter<double>("/traj_opti_node1/offset", configs.offset);
      configs.deltat = declare_parameter<double>("/traj_opti_node1/dt", configs.deltat);
      configs.bos = declare_parameter<double>("/traj_opti_node1/bos", configs.bos);
      configs.vos = declare_parameter<double>("/traj_opti_node1/vos", configs.vos);
      configs.aos = declare_parameter<double>("/traj_opti_node1/aos", configs.aos);
      configs.gd_value = declare_parameter<double>("/traj_opti_node1/gd_value", configs.gd_value);
      configs.gd_type = declare_parameter<int>("/traj_opti_node1/gd_type", configs.gd_type);
      configs.retry_offset = declare_parameter<double>("/traj_opti_node1/retry_offset", configs.retry_offset);
      configs.w_smooth = declare_parameter<double>("/traj_opti_node1/ws", configs.w_smooth);
      configs.w_collision = declare_parameter<double>("/traj_opti_node1/wc", configs.w_collision);
      configs.w_collision_temp = configs.w_collision;
      configs.d0 = declare_parameter<double>("/traj_opti_node1/d0", configs.d0);
      configs.r = declare_parameter<double>("/traj_opti_node1/r", configs.r);
      configs.alpha = declare_parameter<double>("/traj_opti_node1/alpha", configs.alpha);

      configs.v0 = declare_parameter<double>("/traj_opti_node1/v0", configs.v0);
      configs.rv = declare_parameter<double>("/traj_opti_node1/rv", configs.rv);
      configs.alphav = declare_parameter<double>("/traj_opti_node1/alphav", configs.alphav);
      configs.a0 = declare_parameter<double>("/traj_opti_node1/a0", configs.a0);
      configs.ra = declare_parameter<double>("/traj_opti_node1/ra", configs.ra);
      configs.alphaa = declare_parameter<double>("/traj_opti_node1/alphaa", configs.alphaa);

      configs.sgm_time = declare_parameter<double>("/traj_opti_node1/segment_time", configs.sgm_time);
      configs.init_time = declare_parameter<double>("/traj_opti_node1/init_time", configs.init_time);
      configs.mean_v = declare_parameter<double>("/traj_opti_node1/mean_v", configs.mean_v);
    }
    
    void targetCallback(const nav_msgs::msg::Path::SharedPtr msg)
    {
      //取前两个
      geometry_msgs::msg::PoseStamped start, target;
      start = msg->poses[0];
      target = msg->poses[1];
      std::vector<Eigen::Vector3d> way_points(2);
      way_points[0] = {start.pose.position.x, start.pose.position.y, start.pose.position.z};
      way_points[1] = {target.pose.position.x, target.pose.position.y, target.pose.position.z};
      std::vector<Eigen::Vector3d> final_targets;
      Eigen::Vector3d delta_xyz;
      delta_xyz = way_points[1] - way_points[0];
      point_num = delta_xyz.norm()/10+1>3?delta_xyz.norm()/10+1:3;  //10m一个点
      std::cout<<way_points[1]<<std::endl<<way_points[0]<<std::endl<<delta_xyz<<std::endl<<point_num<<std::endl;;
      for(int i=0;i<point_num;i++) {
        Eigen::Vector3d wp;
        wp = way_points[0] + delta_xyz*i/(point_num-1);
        final_targets.push_back(wp);
      }
      this->grad_traj_opt = new GradTrajOptimizer(configs, final_targets);
      RCLCPP_INFO(this->get_logger(), "Get targets!");
      planner_state = WAIT_OPTIM;
    }

    void cmdCallback()
    {
      if(planner_state!=WAIT_FLYING || (this->seg >= point_num-1)) return;
      double curr_time = rclcpp::Clock().now().seconds();
      double whole_dt = curr_time - this->start_time;
      double seg_dt = curr_time - this->start_time - this->history_time;
      if(seg_dt > my_time(this->seg)) {
        this->history_time += my_time(this->seg);
        this->seg++;
      }
      if(this->seg >= point_num-1) {
        planner_state = WAIT_TARGET;
        std::cout<<"Finish!"<<std::endl;
        return;
      }

      vector<Eigen::Vector3d> pva(3, Eigen::Vector3d());
      grad_traj_opt->getPVAFromCoeff(pva, coeff, this->seg, seg_dt);
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = "world";
      pose.pose.position.x = pva[0](0);
      pose.pose.position.y = pva[0](1);
      pose.pose.position.z = pva[0](2);

      pose.pose.orientation.w = 1;
      pose.pose.orientation.x = 0;
      pose.pose.orientation.y = 0;
      pose.pose.orientation.z = 0;
      uav_pose->publish(pose);
    }

    void runOptimize()
    {
      RCLCPP_INFO(this->get_logger(), "[CURRENT STATE:%s]", state_str[planner_state].c_str());
      if(planner_state!=WAIT_OPTIM) return;
      planner_state = WAIT_FLYING;
      this->seg = 0;
      this->history_time = 0;

      double resolution = 0.1;
      grad_traj_opt->setSignedDistanceField(/*&sdf,*/ resolution);

      grad_traj_opt->getCoefficient(coeff);
      grad_traj_opt->getSegmentTime(my_time);
      displayTrajectory(coeff, false);

      // first step optimization
      grad_traj_opt->optimizeTrajectory(OPT_FIRST_STEP);
      grad_traj_opt->getCoefficient(coeff);
      displayTrajectory(coeff, false);

      //  second step optimization
      grad_traj_opt->optimizeTrajectory(OPT_SECOND_STEP);
      grad_traj_opt->getCoefficient(coeff);
      displayTrajectory(coeff, true);
      
      //pub pose
      this->start_time = rclcpp::Clock().now().seconds();
    }
   
    // visualize initial waypoint
    void visualizeSetPoints(vector<Eigen::Vector3d> points)
    {
      // send them to rviz
      for(int i = 0; i < points.size(); ++i)
      {
        
        visualization_msgs::msg::Marker p;
        p.header.frame_id = "world";
        p.header.stamp = rclcpp::Clock().now();
        p.id = i;

        p.type = visualization_msgs::msg::Marker::SPHERE;
        p.action = visualization_msgs::msg::Marker::ADD;

        p.pose.position.x = points[i][0];
        p.pose.position.y = points[i][1];
        p.pose.position.z = points[i][2];
        p.pose.orientation.w = 1;
        p.pose.orientation.x = 0;
        p.pose.orientation.y = 0;
        p.pose.orientation.z = 0;

        p.scale.x = p.scale.y = p.scale.z = 0.2;

        p.color.a = p.color.r = 1.0;
        p.color.g = p.color.b = 0.0;

        p.lifetime = rclcpp::Duration::from_seconds(20);

        this->setpoint_pub->publish(p);
        rclcpp::sleep_for(std::chrono::milliseconds(1));

        // std::cout<<"publish set point"<<std::endl;
      }
    }

    // use coefficient of polynomials to draw the trajectory
    void displayTrajectory(Eigen::MatrixXd coeff, bool animate)
    {
      nav_msgs::msg::Path path;
      path.header.frame_id = "world";
      path.header.stamp = rclcpp::Clock().now();

      // publish the whole trajectory
      double _t;
      vector<Eigen::Vector3d> poses;
      for(int s = 0; s < (point_num - 1); s++)
      {
        // show optimized set point
        visualization_msgs::msg::Marker p;
        p.header.frame_id = "world";
        p.header.stamp = rclcpp::Clock().now();
        p.id = s + point_num;

        p.type = visualization_msgs::msg::Marker::SPHERE;
        p.action = visualization_msgs::msg::Marker::ADD;

        Eigen::Vector3d pos(3);
        getPositionFromCoeff(pos, coeff, s, 0);

        p.pose.position.x = pos(0);
        p.pose.position.y = pos(1);
        p.pose.position.z = pos(2);
        p.pose.orientation.w = 1;
        p.pose.orientation.x = 0;
        p.pose.orientation.y = 0;
        p.pose.orientation.z = 0;

        p.scale.x = p.scale.y = p.scale.z = 0.2;

        p.color.a = p.color.r = p.color.g = 1.0;
        p.color.b = 0.0;

        p.lifetime = rclcpp::Duration::from_seconds(7);

        setpoint_pub->publish(p);

        // show path
        _t = my_time(s);
        for(float t = 0; t < _t; t += 0.2)
        {
          Eigen::Vector3d pos(3);
          getPositionFromCoeff(pos, coeff, s, t);

          // publish these point
          geometry_msgs::msg::PoseStamped pose;
          pose.header.frame_id = "world";

          pose.pose.position.x = pos(0);
          pose.pose.position.y = pos(1);
          pose.pose.position.z = pos(2);

          pose.pose.orientation.w = 1;
          pose.pose.orientation.x = 0;
          pose.pose.orientation.y = 0;
          pose.pose.orientation.z = 0;
          poses.push_back(pos);
          path.poses.push_back(pose);
        }
      }
      traj_pub->publish(path);
      if(!animate)
        return;
      // then publish point on trajectory one by one
      for(int s = 0; s < (point_num - 1); s++)
      {
        _t = my_time(s);
        for(float t = 0; t < _t; t += 0.05)
        {
          visualization_msgs::msg::Marker p;
          p.header.frame_id = "world";
          p.header.stamp = rclcpp::Clock().now();
          p.id = 1;

          p.type = visualization_msgs::msg::Marker::SPHERE;
          p.action = visualization_msgs::msg::Marker::ADD;

          Eigen::Vector3d pos(3);
          getPositionFromCoeff(pos, coeff, s, t);

          p.pose.position.x = pos(0);
          p.pose.position.y = pos(1);
          p.pose.position.z = pos(2);
          p.pose.orientation.w = 1;
          p.pose.orientation.x = 0;
          p.pose.orientation.y = 0;
          p.pose.orientation.z = 0;

          p.scale.x = p.scale.y = p.scale.z = 0.3;

          p.color.a = p.color.r = p.color.g = 1.0;
          p.color.b = 0.0;

          p.lifetime = rclcpp::Duration::from_seconds(0.05);
          traj_point_pub->publish(p);
          rclcpp::sleep_for(std::chrono::milliseconds(1));
          // ros::Duration(0.01).sleep();
        }
      }
    }
};

int main(int argc, char **argv)
{
  /* 初始化ROS2 */
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  /* 退出ROS2 */
  rclcpp::shutdown();
  return 0;
}
