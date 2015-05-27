/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef PR2_3DNAV_VISUALIZATION_SLICE_DISPLAY_RVIZ_SLICE_DISPLAY_
#define PR2_3DNAV_VISUALIZATION_SLICE_DISPLAY_RVIZ_SLICE_DISPLAY_

#include <rviz/display.h>

#ifndef Q_MOC_RUN
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/rviz_plugin_render_tools/robot_state_visualization.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/ros.h>
#endif

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class Robot;
class StringProperty;
class BoolProperty;
class FloatProperty;
class RosTopicProperty;
class ColorProperty;
}

namespace pr2_3dnav_rviz_plugin
{

class RobotPathVisualization;

class RobotPathDisplay : public rviz::Display
{
  Q_OBJECT

public:

  RobotPathDisplay();
  virtual ~RobotPathDisplay();

  virtual void update(float wall_dt, float ros_dt);
  virtual void reset();

private Q_SLOTS:

  // ******************************************************************************************
  // Slot Event Functions
  // ******************************************************************************************
  void changedRobotDescription();
  void changedRobotAlpha();

protected:

  void loadRobotModel();

  /**
   * \brief Set the scene node's position, given the target frame and the planning frame
   */
  void calculateOffsetPosition();

  void newCallback();

  // overrides from Display
  virtual void onInitialize();
  virtual void onEnable();
  virtual void onDisable();
  virtual void fixedFrameChanged();

  // render the robots
  ros::NodeHandle root_nh_;
  rdf_loader::RDFLoaderPtr rdf_loader_;

  planning_scene_monitor::PlanningSceneMonitorPtr kpsm_;

  robot_model::RobotModelConstPtr kmodel_;
  robot_state::RobotStatePtr kstate_;
  moveit_rviz_plugin::RobotStateVisualizationPtr robot_;

  rviz::StringProperty* robot_description_property_;
  rviz::FloatProperty* robot_alpha_property_;
  bool update_state_;

  interactive_markers::MenuHandler menu_handler_;
  visualization_msgs::InteractiveMarker int_marker_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  void makeChessPieceMarker(const tf::Vector3& position);
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  void alignMarker(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
  visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg);
};

} // namespace pr2_3dnav_rviz_plugin

#endif
