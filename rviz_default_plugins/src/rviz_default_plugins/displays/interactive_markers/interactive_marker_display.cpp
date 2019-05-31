/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <tf2_ros/transform_listener.h>

#include "rviz_common/frame_manager_iface.hpp"
#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/validate_floats.hpp"
#include "rviz_common/display_context.hpp"

#include "rviz_default_plugins/transformation/tf_wrapper.hpp"

#include "rviz_default_plugins/displays/interactive_markers/interactive_marker_display.hpp"

namespace rviz_default_plugins
{
namespace displays
{
//////////////
bool validateFloats(const visualization_msgs::msg::InteractiveMarker& msg)
{
  bool valid = true;
  valid = valid && rviz_common::validateFloats(msg.pose);
  valid = valid && rviz_common::validateFloats(msg.scale);
  for ( unsigned c=0; c<msg.controls.size(); c++)
  {
    valid = valid && rviz_common::validateFloats( msg.controls[c].orientation );
    for ( unsigned m=0; m<msg.controls[c].markers.size(); m++ )
    {
      valid = valid && rviz_common::validateFloats(msg.controls[c].markers[m].pose);
      valid = valid && rviz_common::validateFloats(msg.controls[c].markers[m].scale);
      valid = valid && rviz_common::validateFloats(msg.controls[c].markers[m].color);
      valid = valid && rviz_common::validateFloats(msg.controls[c].markers[m].points);
    }
  }
  return valid;
}
/////////////



InteractiveMarkerDisplay::InteractiveMarkerDisplay()
  : RTDClass()
{
  // marker_update_topic_property_ = new RosTopicProperty( "Update Topic", "",
  //                                                       ros::message_traits::datatype<visualization_msgs::msg::InteractiveMarkerUpdate>(),
  //                                                       "visualization_msgs::msg::InteractiveMarkerUpdate topic to subscribe to.",
  //                                                       this, SLOT( updateTopic() ));

  show_descriptions_property_ = new BoolProperty( "Show Descriptions", true,
                                                  "Whether or not to show the descriptions of each Interactive Marker.",
                                                  this, SLOT( updateShowDescriptions() ));

  show_axes_property_ = new BoolProperty( "Show Axes", false,
                                          "Whether or not to show the axes of each Interactive Marker.",
                                          this, SLOT( updateShowAxes() ));

  show_visual_aids_property_ = new BoolProperty( "Show Visual Aids", false,
                                                 "Whether or not to show visual helpers while moving/rotating Interactive Markers.",
                                                 this, SLOT( updateShowVisualAids() ));
  enable_transparency_property_ = new BoolProperty( "Enable Transparency", true,
                                                 "Whether or not to allow transparency for auto-completed markers (e.g. rings and arrows).",
                                                 this, SLOT( updateEnableTransparency() ));
}

void InteractiveMarkerDisplay::onInitialize()
{
  topic_property_->setDescription("visualization_msgs::msg::InteractiveMarkerUpdate topic to subscribe to.");
  
  auto tf_wrapper = std::dynamic_pointer_cast<transformation::TFWrapper>(
    context_->getFrameManager()->getConnector().lock());

  if(tf_wrapper) {
    auto tf = tf_wrapper->getBuffer().get();
    auto node = rviz_ros_node_.lock()->get_raw_node();

    im_client_.reset( new ::interactive_markers::InteractiveMarkerClient( node, *tf, fixed_frame_.toStdString() ) );

    im_client_->setInitCb( std::bind( &InteractiveMarkerDisplay::initCb, this, std::placeholders::_1 ) );
    im_client_->setUpdateCb( std::bind( &InteractiveMarkerDisplay::updateCb, this, std::placeholders::_1 ) );
    im_client_->setResetCb( std::bind( &InteractiveMarkerDisplay::resetCb, this, std::placeholders::_1 ) );
    im_client_->setStatusCb( std::bind( &InteractiveMarkerDisplay::statusCb, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3 ) );
    client_id_ = rviz_ros_node_.lock()->get_node_name() + "/" + getNameStd();

    onEnable();
  }
}

void InteractiveMarkerDisplay::updateTopic()
{
  unsubscribe();

  std::string update_topic = topic_property_->getTopicStd();

  size_t idx = update_topic.find( "/update" );
  if ( idx != std::string::npos )
  {
    topic_ns_ = update_topic.substr( 0, idx );
    subscribe();
  }
  else
  {
    setStatusStd( rviz_common::properties::StatusProperty::Error, "Topic", "Invalid topic name: " + update_topic );
  }

}

void InteractiveMarkerDisplay::subscribe()
{
  if ( isEnabled() )
  {
    im_client_->subscribe(topic_ns_);

    auto node = rviz_ros_node_.lock()->get_raw_node();
    std::string feedback_topic = topic_ns_ + "/feedback";
    feedback_pub_ = node->create_publisher<visualization_msgs::msg::InteractiveMarkerFeedback>( feedback_topic, 100 );
  }
}

void InteractiveMarkerDisplay::publishFeedback(visualization_msgs::msg::InteractiveMarkerFeedback &feedback)
{
  feedback.client_id = client_id_;
  feedback_pub_->publish( feedback );
}

void InteractiveMarkerDisplay::onStatusUpdate( 
  rviz_common::properties::StatusProperty::Level level, const std::string& name, const std::string& text )
{
  setStatusStd(level,name,text);
}

void InteractiveMarkerDisplay::unsubscribe()
{
  if (im_client_)
  {
    im_client_->shutdown();
  }
  feedback_pub_.reset();
}

void InteractiveMarkerDisplay::update(float wall_dt, float ros_dt)
{
  (void) ros_dt;
  im_client_->update();

  M_StringToStringToIMPtr::iterator server_it;
  for ( server_it = interactive_markers_.begin(); server_it != interactive_markers_.end(); server_it++ )
  {
    M_StringToIMPtr::iterator im_it;
    for ( im_it = server_it->second.begin(); im_it != server_it->second.end(); im_it++ )
    {
      im_it->second->update( wall_dt );
    }
  }
}

InteractiveMarkerDisplay::M_StringToIMPtr& InteractiveMarkerDisplay::getImMap( std::string server_id )
{
  M_StringToStringToIMPtr::iterator im_map_it = interactive_markers_.find( server_id );

  if ( im_map_it == interactive_markers_.end() )
  {
    im_map_it = interactive_markers_.insert( std::make_pair( server_id, M_StringToIMPtr() ) ).first;
  }

  return im_map_it->second;
}

void InteractiveMarkerDisplay::updateMarkers(
    const std::string& server_id,
    const std::vector<visualization_msgs::msg::InteractiveMarker>& markers
    )
{
  M_StringToIMPtr& im_map = getImMap( server_id );

  for ( size_t i=0; i<markers.size(); i++ )
  {
    const visualization_msgs::msg::InteractiveMarker& marker = markers[i];

    if ( !validateFloats( marker ) )
    {
      setStatusStd( rviz_common::properties::StatusProperty::Error, marker.name, "Marker contains invalid floats!" );
      //setStatusStd( StatusProperty::Error, "General", "Marker " + marker.name + " contains invalid floats!" );
      continue;
    }
    RVIZ_COMMON_LOG_DEBUG_STREAM("Processing interactive marker '"
      << marker.name << "'. "
      << (int)marker.controls.size());

    std::map< std::string, IMPtr >::iterator int_marker_entry = im_map.find( marker.name );

    if ( int_marker_entry == im_map.end() )
    {
      int_marker_entry = im_map.insert( std::make_pair(marker.name, IMPtr ( new interactive_markers::InteractiveMarker(getSceneNode(), context_) ) ) ).first;
      connect( int_marker_entry->second.get(),
               SIGNAL( userFeedback(visualization_msgs::msg::InteractiveMarkerFeedback&) ),
               this,
               SLOT( publishFeedback(visualization_msgs::msg::InteractiveMarkerFeedback&) ));
      connect( int_marker_entry->second.get(),
               SIGNAL( statusUpdate(rviz_common::properties::StatusProperty::Level, const std::string&, const std::string&) ),
               this,
               SLOT( onStatusUpdate(rviz_common::properties::StatusProperty::Level, const std::string&, const std::string&) ) );
    }

    if ( int_marker_entry->second->processMessage( marker ) )
    {
      int_marker_entry->second->setShowAxes( show_axes_property_->getBool() );
      int_marker_entry->second->setShowVisualAids( show_visual_aids_property_->getBool() );
      int_marker_entry->second->setShowDescription( show_descriptions_property_->getBool() );
    }
    else
    {
      unsubscribe();
      return;
    }
  }
}

void InteractiveMarkerDisplay::eraseMarkers(
    const std::string& server_id,
    const std::vector<std::string>& erases )
{
  M_StringToIMPtr& im_map = getImMap( server_id );

  for ( size_t i=0; i<erases.size(); i++ )
  {
    im_map.erase( erases[i] );
    deleteStatusStd( erases[i] );
  }
}

void InteractiveMarkerDisplay::updatePoses(
    const std::string& server_id,
    const std::vector<visualization_msgs::msg::InteractiveMarkerPose>& marker_poses )
{
  M_StringToIMPtr& im_map = getImMap( server_id );

  for ( size_t i=0; i<marker_poses.size(); i++ )
  {
    const visualization_msgs::msg::InteractiveMarkerPose& marker_pose = marker_poses[i];

    if ( !rviz_common::validateFloats( marker_pose.pose ) )
    {
      setStatusStd( rviz_common::properties::StatusProperty::Error, marker_pose.name, "Pose message contains invalid floats!" );
      return;
    }

    std::map< std::string, IMPtr >::iterator int_marker_entry = im_map.find( marker_pose.name );

    if ( int_marker_entry != im_map.end() )
    {
      int_marker_entry->second->processMessage( marker_pose );
    }
    else
    {
      setStatusStd( rviz_common::properties::StatusProperty::Error, marker_pose.name, "Pose received for non-existing marker '" + marker_pose.name );
      unsubscribe();
      return;
    }
  }
}

void InteractiveMarkerDisplay::initCb( visualization_msgs::msg::InteractiveMarkerInit::ConstSharedPtr msg )
{
  resetCb( msg->server_id );
  updateMarkers( msg->server_id, msg->markers );
}

void InteractiveMarkerDisplay::updateCb( visualization_msgs::msg::InteractiveMarkerUpdate::ConstSharedPtr msg )
{
  updateMarkers( msg->server_id, msg->markers );
  updatePoses( msg->server_id, msg->poses );
  eraseMarkers( msg->server_id, msg->erases );
}

void InteractiveMarkerDisplay::resetCb( std::string server_id )
{
  interactive_markers_.erase( server_id );
  deleteStatusStd(server_id);
}

void InteractiveMarkerDisplay::statusCb(
    ::interactive_markers::InteractiveMarkerClient::StatusT status,
    const std::string& server_id,
    const std::string& msg )
{
  setStatusStd( static_cast<rviz_common::properties::StatusProperty::Level>(status), server_id, msg );
}

void InteractiveMarkerDisplay::fixedFrameChanged()
{  
  if (im_client_)
    im_client_->setTargetFrame( fixed_frame_.toStdString() );
  reset();
}

void InteractiveMarkerDisplay::reset()
{
  RTDClass::reset();
  unsubscribe();
  subscribe();
}

void InteractiveMarkerDisplay::updateShowDescriptions()
{
  bool show = show_descriptions_property_->getBool();

  M_StringToStringToIMPtr::iterator server_it;
  for ( server_it = interactive_markers_.begin(); server_it != interactive_markers_.end(); server_it++ )
  {
    M_StringToIMPtr::iterator im_it;
    for ( im_it = server_it->second.begin(); im_it != server_it->second.end(); im_it++ )
    {
      im_it->second->setShowDescription( show );
    }
  }
}

void InteractiveMarkerDisplay::updateShowAxes()
{
  bool show = show_axes_property_->getBool();

  M_StringToStringToIMPtr::iterator server_it;
  for ( server_it = interactive_markers_.begin(); server_it != interactive_markers_.end(); server_it++ )
  {
    M_StringToIMPtr::iterator im_it;
    for ( im_it = server_it->second.begin(); im_it != server_it->second.end(); im_it++ )
    {
      im_it->second->setShowAxes( show );
    }
  }
}

void InteractiveMarkerDisplay::updateShowVisualAids()
{
  bool show = show_visual_aids_property_->getBool();

  M_StringToStringToIMPtr::iterator server_it;
  for ( server_it = interactive_markers_.begin(); server_it != interactive_markers_.end(); server_it++ )
  {
    M_StringToIMPtr::iterator im_it;
    for ( im_it = server_it->second.begin(); im_it != server_it->second.end(); im_it++ )
    {
      im_it->second->setShowVisualAids( show );
    }
  }
}

void InteractiveMarkerDisplay::updateEnableTransparency()
{
  // This is not very efficient... but it should do the trick.
  unsubscribe();
  im_client_->setEnableAutocompleteTransparency( enable_transparency_property_->getBool() );
  subscribe();
}

void InteractiveMarkerDisplay::processMessage(visualization_msgs::msg::InteractiveMarkerUpdate::ConstSharedPtr msg ) {
  (void) msg;
}
}
} // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz_default_plugins::displays::InteractiveMarkerDisplay, rviz_common::Display )
