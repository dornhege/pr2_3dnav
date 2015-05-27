/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Philipp Jankov*/

#include <pr2_3dnav_slice_rviz_plugin/pr2_3dnav_slice_display.h>
#include <moveit/robot_interaction/interactive_marker_helpers.h>
#include <moveit/robot_state/conversions.h>

#include <rviz/visualization_manager.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>

#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/ros_topic_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <eigen_conversions/eigen_msg.h>

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

namespace pr2_3dnav_rviz_plugin
{
  // ******************************************************************************************
  // Base class contructor
  // ******************************************************************************************
  RobotPathDisplay::RobotPathDisplay() :
    Display(),
    update_state_(false)
  {
    robot_description_property_ =
      new rviz::StringProperty( "Robot Description", "robot_description", "The name of the ROS parameter where the URDF for the robot is loaded", this,
                                SLOT( changedRobotDescription() ), this );

    // Planning scene category -------------------------------------------------------------------------------------------
    robot_alpha_property_ =
      new rviz::FloatProperty( "Robot Alpha", 0.1, "Specifies the alpha for the marker.", this,
                              SLOT( changedRobotAlpha() ), this );
    robot_alpha_property_->setMin( 0.0 );
    robot_alpha_property_->setMax( 1.0 );
  }

  // ******************************************************************************************
  // Deconstructor
  // ******************************************************************************************
  RobotPathDisplay::~RobotPathDisplay()
  {
  }

  void RobotPathDisplay::onInitialize()
  {
    Display::onInitialize();
    robot_.reset(new moveit_rviz_plugin::RobotStateVisualization(scene_node_, context_, "Robot Slice", this));
    robot_->setVisible(false);
    
  }

  void RobotPathDisplay::reset()
  {
    robot_->clear();
    rdf_loader_.reset();
    kpsm_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_description_property_->getStdString()));
    loadRobotModel();
    Display::reset();
    robot_->setVisible(true);
  }

  void RobotPathDisplay::newCallback()
  {
    if(!kmodel_)
        return;

    if(!kpsm_ || !kpsm_->requestPlanningSceneState()) return;
    planning_scene_monitor::LockedPlanningSceneRO psm(kpsm_);
    robot_state::RobotState current_state = psm->getCurrentState();

    update_state_ = true;
  }

  void RobotPathDisplay::changedRobotDescription()
  {
    if (isEnabled())
      reset();
  }

  void RobotPathDisplay::changedRobotAlpha()
  {
    if (robot_)
    {
      robot_->setAlpha(robot_alpha_property_->getFloat());
      update_state_ = true;
    }
  }

  // ******************************************************************************************
  // Load
  // ******************************************************************************************
  void RobotPathDisplay::loadRobotModel()
  {
    if (!rdf_loader_)
      rdf_loader_.reset(new rdf_loader::RDFLoader(robot_description_property_->getStdString()));

    if (rdf_loader_->getURDF())
    {
      const boost::shared_ptr<srdf::Model> &srdf = rdf_loader_->getSRDF() ? rdf_loader_->getSRDF() : boost::shared_ptr<srdf::Model>(new srdf::Model());
      kmodel_.reset(new robot_model::RobotModel(rdf_loader_->getURDF(), srdf));
      robot_->load(*kmodel_->getURDF());
      robot_->setAlpha(robot_alpha_property_->getFloat());
      if(kpsm_ && kpsm_->requestPlanningSceneState()){
        planning_scene_monitor::LockedPlanningSceneRO psm(kpsm_);
        robot_state::RobotState current_state = psm->getCurrentState();
        kstate_.reset(new robot_state::RobotState(current_state));
      } else {
        kstate_.reset(new robot_state::RobotState(kmodel_));
        kstate_->setToDefaultValues();
      }

      update_state_ = true;
      setStatus( rviz::StatusProperty::Ok, "RobotModel", "Model Loaded Successfully" );
    }
    else
      setStatus( rviz::StatusProperty::Error, "RobotModel", "No  Model Loaded" );
  }

  void RobotPathDisplay::onEnable()
  {
    Display::onEnable();
    kpsm_.reset(new planning_scene_monitor::PlanningSceneMonitor(robot_description_property_->getStdString()));
    loadRobotModel();
    if (robot_)
      robot_->setVisible(true);
    calculateOffsetPosition();

    server_.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false));
    menu_handler_.insert("First Entry", boost::bind(&RobotPathDisplay::processFeedback, this, _1));
    tf::Vector3 position = tf::Vector3(0, 0, 0);
    makeChessPieceMarker(position);
    server_->applyChanges();
  }

  // ******************************************************************************************
  // Disable
  // ******************************************************************************************
  void RobotPathDisplay::onDisable()
  {
    if (robot_)
      robot_->setVisible(false);
    Display::onDisable();
  }

  void RobotPathDisplay::update(float wall_dt, float ros_dt)
  {
    Display::update(wall_dt, ros_dt);
    if (robot_ && update_state_)
    {
      update_state_ = false;
      kstate_->update();
      robot_->update(kstate_);
    }
  }

  // ******************************************************************************************
  // Calculate Offset Position
  // ******************************************************************************************
  void RobotPathDisplay::calculateOffsetPosition()
  {
    if (!kmodel_)
      return;

    ros::Time stamp;
    std::string err_string;
    if (context_->getTFClient()->getLatestCommonTime(fixed_frame_.toStdString(), kmodel_->getModelFrame(), stamp, &err_string) != tf::NO_ERROR)
      return;

    tf::Stamped<tf::Pose> pose(tf::Pose::getIdentity(), stamp, kmodel_->getModelFrame());

    if (context_->getTFClient()->canTransform(fixed_frame_.toStdString(), kmodel_->getModelFrame(), stamp))
    {
      try
      {
        context_->getTFClient()->transformPose(fixed_frame_.toStdString(), pose, pose);
      }
      catch (tf::TransformException& e)
      {
        ROS_ERROR( "Error transforming from frame '%s' to frame '%s'", pose.frame_id_.c_str(), fixed_frame_.toStdString().c_str() );
      }
    }

    Ogre::Vector3 position(pose.getOrigin().x(), pose.getOrigin().y(), pose.getOrigin().z());
    const tf::Quaternion &q = pose.getRotation();
    Ogre::Quaternion orientation( q.getW(), q.getX(), q.getY(), q.getZ() );
    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);

    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.stamp = stamp;
    poseStamped.header.frame_id = kmodel_->getModelFrame();

    poseStamped.pose.position.x = pose.getOrigin().x();
    poseStamped.pose.position.y = pose.getOrigin().y();
    poseStamped.pose.position.z = pose.getOrigin().z();
    poseStamped.pose.orientation.w = q.getW();
    poseStamped.pose.orientation.x = q.getX(); 
    poseStamped.pose.orientation.y = q.getY();
    poseStamped.pose.orientation.z = q.getZ();

//     // we want to use our special callback function
//     server_.reset( new interactive_markers::InteractiveMarkerServer("basic_controls","",false) );    
//     int_marker_ = robot_interaction::makePlanarXYMarker("islice_marker", poseStamped, 1.0, false);
//     server_->insert(int_marker_);
//     server_->setCallback(int_marker_.name, boost::bind(&RobotPathDisplay::processFeedback, this, _1));
// 
//     // set different callback for POSE_UPDATE feedback
//     server_->setCallback(int_marker_.name, boost::bind(&RobotPathDisplay::alignMarker, this, _1), visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE );
//     menu_handler_.apply(*server_, int_marker_.name);
//     server_->applyChanges();
  }

  void RobotPathDisplay::fixedFrameChanged()
  {
    Display::fixedFrameChanged();
    calculateOffsetPosition();
  }

  // ******************************************************************************************
  // Interactive Marker Feedback
  // ******************************************************************************************
  void RobotPathDisplay::makeChessPieceMarker(const tf::Vector3& position)
  {
    int_marker_.header.frame_id = "base_link";
    tf::pointTFToMsg(position, int_marker_.pose.position);
    int_marker_.scale = 1;

    int_marker_.name = "chess_piece";
    int_marker_.description = "Chess Piece\n(2D Move + Alignment)";

    visualization_msgs::InteractiveMarkerControl control;

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_PLANE;
    int_marker_.controls.push_back(control);

    // make a box which also moves in the plane
    control.markers.push_back(makeBox(int_marker_));
    control.always_visible = true;
    int_marker_.controls.push_back(control);

    // we want to use our special callback function
    server_->insert(int_marker_);
    server_->setCallback(int_marker_.name, boost::bind(&RobotPathDisplay::processFeedback, this, _1));

    // set different callback for POSE_UPDATE feedback
    server_->setCallback(int_marker_.name, boost::bind(&RobotPathDisplay::alignMarker, this, _1), visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE);
  }

  void RobotPathDisplay::processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    std::ostringstream s;
    s << "Feedback from marker '" << feedback->marker_name << "' "
        << " / control '" << feedback->control_name << "'";

    std::ostringstream mouse_point_ss;
    if( feedback->mouse_point_valid )
    {
      mouse_point_ss << " at " << feedback->mouse_point.x
                    << ", " << feedback->mouse_point.y
                    << ", " << feedback->mouse_point.z
                    << " in frame " << feedback->header.frame_id;
    }

    switch ( feedback->event_type )
    {
      case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
        ROS_INFO_STREAM( s.str() << ": menu item " << feedback->menu_entry_id << " clicked" << mouse_point_ss.str() << "." );
        break;

      case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        ROS_INFO_STREAM( s.str() << ": pose changed"
            << "\nposition = "
            << feedback->pose.position.x
            << ", " << feedback->pose.position.y
            << ", " << feedback->pose.position.z
            << "\norientation = "
            << feedback->pose.orientation.w
            << ", " << feedback->pose.orientation.x
            << ", " << feedback->pose.orientation.y
            << ", " << feedback->pose.orientation.z
            << "\nframe: " << feedback->header.frame_id
            << " time: " << feedback->header.stamp.sec << "sec, "
            << feedback->header.stamp.nsec << " nsec" );
        break;

      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_DOWN:
        ROS_INFO_STREAM( s.str() << ": mouse down" << mouse_point_ss.str() << "." );
        break;

      case visualization_msgs::InteractiveMarkerFeedback::MOUSE_UP:
        ROS_INFO_STREAM( s.str() << ": mouse up" << mouse_point_ss.str() << "." );
        break;
    }
    server_->applyChanges();

    Eigen::Affine3d epose;
    tf::poseMsgToEigen(feedback->pose, epose);
    kstate_->setJointPositions(kmodel_->getRootJoint(), epose);
  }

  void RobotPathDisplay::alignMarker( const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback )
  {
    geometry_msgs::Pose pose = feedback->pose;

    pose.position.x = round(pose.position.x-0.5)+0.5;
    pose.position.y = round(pose.position.y-0.5)+0.5;

    ROS_INFO_STREAM( feedback->marker_name << ":"
        << " aligning position = "
        << feedback->pose.position.x
        << ", " << feedback->pose.position.y
        << ", " << feedback->pose.position.z
        << " to "
        << pose.position.x
        << ", " << pose.position.y
        << ", " << pose.position.z );

    server_->setPose( feedback->marker_name, pose );
    server_->applyChanges();
  }

  visualization_msgs::Marker RobotPathDisplay::makeBox(visualization_msgs::InteractiveMarker &msg)
  {
    visualization_msgs::Marker marker;

    marker.type = visualization_msgs::Marker::CUBE;
    marker.scale.x = msg.scale * 0.45;
    marker.scale.y = msg.scale * 0.45;
    marker.scale.z = msg.scale * 0.45;
    marker.color.r = 0.5;
    marker.color.g = 0.5;
    marker.color.b = 0.5;
    marker.color.a = 1.0;

    return marker;
  }
} // namespace pr2_3dnav_rviz_plugin
