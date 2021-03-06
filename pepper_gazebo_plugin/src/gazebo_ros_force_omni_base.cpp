/*
 * Based on the force base plugin by Stefan Kohlbrecher
 */


#include <pepper_gazebo_plugin/gazebo_ros_force_omni_base.h>

namespace gazebo 
{

    GazeboRosForceOmniBase::GazeboRosForceOmniBase() {}

    GazeboRosForceOmniBase::~GazeboRosForceOmniBase() {}

  // Load the controller
  void GazeboRosForceOmniBase::Load(physics::ModelPtr parent,
      sdf::ElementPtr sdf) 
  {

    parent_ = parent;

    /* Parse parameters */

    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace")) 
    {
      ROS_INFO("PlanarMovePlugin missing <robotNamespace>, "
          "defaults to \"%s\"", robot_namespace_.c_str());
    }
    else 
    {
      robot_namespace_ = 
        sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    command_topic_ = "cmd_vel";
    if (!sdf->HasElement("commandTopic")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <commandTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), command_topic_.c_str());
    } 
    else 
    {
      command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
    }

    odometry_topic_ = "odom";
    if (!sdf->HasElement("odometryTopic")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryTopic>, "
          "defaults to \"%s\"", 
          robot_namespace_.c_str(), odometry_topic_.c_str());
    } 
    else 
    {
      odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    odometry_frame_ = "odom";
    if (!sdf->HasElement("odometryFrame")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), odometry_frame_.c_str());
    }
    else 
    {
      odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
    }
    
    /////////////// HARDCODEADO
    robot_base_link_frame_ = "base_link";
    robot_base_footprint_frame_ = "base_footprint";
    ////////////////////////////////

    torque_yaw_velocity_p_gain_ = 1000.0;
    torque_yaw_velocity_i_gain_ = 300.0;
    
    force_x_velocity_p_gain_ = 1000.0;
    force_x_velocity_i_gain_ = 300.0;

    force_y_velocity_p_gain_ = 1000.0;
    force_y_velocity_i_gain_ = 300.0;
    
    // Torque yaw contrller
    if (sdf->HasElement("yaw_velocity_p_gain"))
      (sdf->GetElement("yaw_velocity_p_gain")->GetValue()->Get(torque_yaw_velocity_p_gain_));
    if (sdf->HasElement("yaw_velocity_i_gain"))
      (sdf->GetElement("yaw_velocity_i_gain")->GetValue()->Get(torque_yaw_velocity_i_gain_));
    // Force x controller
    if (sdf->HasElement("x_velocity_p_gain"))
      (sdf->GetElement("x_velocity_p_gain")->GetValue()->Get(force_x_velocity_p_gain_));
    if (sdf->HasElement("x_velocity_i_gain"))
      (sdf->GetElement("x_velocity_i_gain")->GetValue()->Get(force_x_velocity_i_gain_));
    // Force y controller
    if (sdf->HasElement("y_velocity_p_gain"))
      (sdf->GetElement("y_velocity_p_gain")->GetValue()->Get(force_y_velocity_i_gain_));
    if (sdf->HasElement("y_velocity_i_gain"))
      (sdf->GetElement("y_velocity_i_gain")->GetValue()->Get(force_y_velocity_i_gain_));
    // Max values for torque and force
    max_torque_ = 8.0;
    max_force_ = 50.0;
    if (sdf->HasElement("max_torque"))
      (sdf->GetElement("max_torque")->GetValue()->Get(max_torque_));
    if (sdf->HasElement("max_force"))
      (sdf->GetElement("max_force")->GetValue()->Get(max_force_));
      
    ROS_INFO_STREAM("ForceBasedMove using gains: yaw: " << torque_yaw_velocity_p_gain_ <<
                                                 " x: " << force_x_velocity_p_gain_ <<
                                                 " y: " << force_y_velocity_p_gain_ << "\n");

    robot_base_frame_ = "base_footprint";
    if (!sdf->HasElement("robotBaseFrame")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <robotBaseFrame>, "
          "defaults to \"%s\"",
          robot_namespace_.c_str(), robot_base_frame_.c_str());
    } 
    else 
    {
      robot_base_frame_ = sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    ROS_INFO_STREAM("robotBaseFrame for force based move plugin: " << robot_base_frame_  << "\n");

    this->link_ = parent->GetLink(robot_base_frame_);

    odometry_rate_ = 20.0;
    if (!sdf->HasElement("odometryRate")) 
    {
      ROS_WARN("PlanarMovePlugin (ns = %s) missing <odometryRate>, "
          "defaults to %f",
          robot_namespace_.c_str(), odometry_rate_);
    } 
    else 
    {
      odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
    } 

    this->publish_odometry_tf_ = true;
    //if (!sdf->HasElement("publishOdometryTf")) {
    //  ROS_WARN("PlanarMovePlugin Plugin (ns = %s) missing <publishOdometryTf>, defaults to %s",
    //           this->robot_namespace_.c_str(), this->publish_odometry_tf_ ? "true" : "false");
    //} else {
    //  this->publish_odometry_tf_ = sdf->GetElement("publishOdometryTf")->Get<bool>();
    //}
 
    last_odom_publish_time_ = parent_->GetWorld()->GetSimTime();
    last_odom_pose_ = parent_->GetWorldPose();
    x_ = 0;
    y_ = 0;
    rot_ = 0;
    alive_ = true;

    odom_transform_.setIdentity();

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized()) 
    {
      ROS_FATAL_STREAM("PlanarMovePlugin (ns = " << robot_namespace_
        << "). A ROS node for Gazebo has not been initialized, "
        << "unable to load plugin. Load the Gazebo system plugin "
        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
      return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    ROS_INFO("PlanarMovePlugin (%s) has started!", 
        robot_namespace_.c_str());

    // Init PID controllers
    torque_yaw_pid_.initPid(torque_yaw_velocity_p_gain_, torque_yaw_velocity_i_gain_, 0.0, 10.0, -10.0);
    ros::NodeHandle torque_yaw_nh(*rosnode_, "base_controller/torque_yaw_pid");
    torque_yaw_pid_.initDynamicReconfig(torque_yaw_nh);
    
    force_x_pid_.initPid(force_x_velocity_p_gain_, force_x_velocity_i_gain_, 0.0, 20.0, -20.0);
    ros::NodeHandle force_x_nh(*rosnode_, "base_controller/force_x_pid");
    force_x_pid_.initDynamicReconfig(force_x_nh);

    force_y_pid_.initPid(force_y_velocity_p_gain_, force_y_velocity_i_gain_, 0.0, 20.0, -20.0);
    ros::NodeHandle force_y_nh(*rosnode_, "base_controller/force_y_pid");
    force_y_pid_.initDynamicReconfig(force_y_nh);

    last_ctrl_loop_ = ros::Time::now();


    tf_prefix_ = tf::getPrefixParam(*rosnode_);

    if (publish_odometry_tf_){
      transform_broadcaster_.reset(new tf::TransformBroadcaster());
      transform_listener_.reset(new tf::TransformListener());
  }
    // subscribe to the odometry topic
    ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
          boost::bind(&GazeboRosForceOmniBase::cmdVelCallback, this, _1),
          ros::VoidPtr(), &queue_);

    vel_sub_ = rosnode_->subscribe(so);
    odometry_pub_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    // start custom queue for diff drive
    callback_queue_thread_ = 
      boost::thread(boost::bind(&GazeboRosForceOmniBase::QueueThread, this));

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = 
      event::Events::ConnectWorldUpdateBegin(
          boost::bind(&GazeboRosForceOmniBase::UpdateChild, this));

  }

  // Update the controller
  void GazeboRosForceOmniBase::UpdateChild()
  {
    boost::mutex::scoped_lock scoped_lock(lock);

    // Get current speed
    math::Vector3 angular_vel = parent_->GetWorldAngularVel();
    math::Vector3 linear_vel = parent_->GetRelativeLinearVel();

    ros::Time current_time = ros::Time::now();
    ros::Duration dt = current_time - last_ctrl_loop_;
    last_ctrl_loop_ = current_time;

    double yaw_error = rot_ - angular_vel.z;
    double x_error = x_ - linear_vel.x;
    double y_error = y_ - linear_vel.y;

    // Apply limits
    double torque_cmd = std::max( -max_torque_, std::min( torque_yaw_pid_.computeCommand(yaw_error, dt), max_torque_) );
    double force_x_cmd = std::max( -max_force_, std::min( force_x_pid_.computeCommand(x_error, dt), max_force_) );
    double force_y_cmd = std::max( -max_force_, std::min( force_y_pid_.computeCommand(y_error, dt), max_force_) );
    // Apply commands
    link_->AddTorque(math::Vector3(0.0, 0.0, torque_cmd));
    link_->AddRelativeForce(math::Vector3(force_x_cmd, force_y_cmd, 0.0));
    //std::cout << force_x_cmd << " , " << link_->GetRelativeForce().x << std::endl;
    //parent_->PlaceOnNearestEntityBelow();
    //parent_->SetLinearVel(math::Vector3(
    //      x_ * cosf(yaw) - y_ * sinf(yaw),
    //      y_ * cosf(yaw) + x_ * sinf(yaw),
    //      0));
    //parent_->SetAngularVel(math::Vector3(0, 0, rot_));

    if (odometry_rate_ > 0.0) {
      common::Time current_time = parent_->GetWorld()->GetSimTime();
      double seconds_since_last_update = 
        (current_time - last_odom_publish_time_).Double();
      if (seconds_since_last_update > (1.0 / odometry_rate_)) {
        publishOdometry(seconds_since_last_update);
        last_odom_publish_time_ = current_time;
      }
    }
  }

  // Finalize the controller
  void GazeboRosForceOmniBase::FiniChild() {
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
  }

  void GazeboRosForceOmniBase::cmdVelCallback(
      const geometry_msgs::Twist::ConstPtr& cmd_msg) 
  {
    boost::mutex::scoped_lock scoped_lock(lock);
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    rot_ = cmd_msg->angular.z;
  }

  void GazeboRosForceOmniBase::QueueThread()
  {
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok()) 
    {
      queue_.callAvailable(ros::WallDuration(timeout));
    }
  }

  void GazeboRosForceOmniBase::publishOdometry(double step_time)
  {

    ros::Time current_time = ros::Time::now();
    std::string odom_frame = tf::resolve(tf_prefix_, odometry_frame_);
    std::string base_footprint_frame = tf::resolve(tf_prefix_, robot_base_footprint_frame_);
    std::string base_link_frame = tf::resolve(tf_prefix_, robot_base_link_frame_);
    
    tf::StampedTransform base_footprint_to_base_link;
    tf::Transform odom_to_base_link;
    tf::StampedTransform odom_to_base_footprint;

    math::Vector3 angular_vel = parent_->GetRelativeAngularVel();
    math::Vector3 linear_vel = parent_->GetRelativeLinearVel();

    odom_transform_= odom_transform_ * this->getTransformForMotion(linear_vel.x, angular_vel.z, step_time);
    odom_to_base_footprint = tf::StampedTransform(odom_transform_,current_time,odom_frame,base_footprint_frame);

    try
    { 
        if(transform_listener_.get()){
          transform_listener_->lookupTransform(base_footprint_frame,base_link_frame,ros::Time(0),base_footprint_to_base_link);
          ROS_DEBUG("OK Transform to %s from %s \n", base_footprint_frame.c_str(), base_link_frame.c_str());
        }
   
    }
    catch(tf::TransformException &e)
    {
        ROS_ERROR("Failed to transform to %s from %s: %s\n", base_footprint_frame.c_str(), base_link_frame.c_str(), e.what());
        return;
    }



    tf::poseTFToMsg(odom_transform_, odom_.pose.pose);
    odom_.twist.twist.angular.z = angular_vel.z;
    odom_.twist.twist.linear.x  = linear_vel.x;

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odom_frame;
    //odom_.child_frame_id = base_footprint_frame;
    odom_.child_frame_id = base_link_frame;

    odom_to_base_link = odom_to_base_footprint * base_footprint_to_base_link;

    if (transform_broadcaster_.get()){
      transform_broadcaster_->sendTransform(
          tf::StampedTransform(odom_to_base_link, current_time, odom_frame,
              base_link_frame));
      ROS_DEBUG("Sending Transform from %s to %s \n",odom_frame.c_str(),base_link_frame.c_str());
    }
    
    odom_.pose.covariance[0] = 0.001;
    odom_.pose.covariance[7] = 0.001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    
    if (std::abs(angular_vel.z) < 0.0001) {
      odom_.pose.covariance[35] = 0.01;
    }else{
      odom_.pose.covariance[35] = 100.0;
    }

    odom_.twist.covariance[0] = 0.001;
    odom_.twist.covariance[7] = 0.001;
    odom_.twist.covariance[14] = 0.001;
    odom_.twist.covariance[21] = 1000000000000.0;
    odom_.twist.covariance[28] = 1000000000000.0;

    if (std::abs(angular_vel.z) < 0.0001) {
      odom_.twist.covariance[35] = 0.01;
    }else{
      odom_.twist.covariance[35] = 100.0;
    }

    odometry_pub_.publish(odom_);
  }


  tf::Transform GazeboRosForceOmniBase::getTransformForMotion(double linear_vel_x, double angular_vel, double timeSeconds) const
  {
    tf::Transform tmp;
    tmp.setIdentity();


    if (std::abs(angular_vel) < 0.0001) {
      //Drive straight
      tmp.setOrigin(tf::Vector3(static_cast<double>(linear_vel_x*timeSeconds), 0.0, 0.0));
    } else {
      //Follow circular arc
      double distChange = linear_vel_x * timeSeconds;
      double angleChange = angular_vel * timeSeconds;

      double arcRadius = distChange / angleChange;

      tmp.setOrigin(tf::Vector3(std::sin(angleChange) * arcRadius,
                                arcRadius - std::cos(angleChange) * arcRadius,
                                0.0));
      tmp.setRotation(tf::createQuaternionFromYaw(angleChange));
    }

    return tmp;
  }

  GZ_REGISTER_MODEL_PLUGIN(GazeboRosForceOmniBase)
}

