/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <sstream>
#include <gazebo/msgs/msgs.hh>
#include "GUIExampleSpawnWidget.hh"
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>
#include "ros/ros.h"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(GUIExampleSpawnWidget)

bool object_clicked = FALSE;



void callback(Sel &_msg)
{
  
  object_clicked = TRUE;
  std::cout << "I'm here4" << std::endl;
  //std::cout << "I'm here" << std::endl;
  //std::cout << msg << std::endl;
  //transport::NodePtr selection_publisher = transport::NodePtr(new transport::Node());
  //selection_publisher->Init();
  //transport::PublisherPtr pub = selection_publisher->Advertise<msgs::PosesStamped>("~/pose/info");
  //pub->Publish(msg);

  //need to figure out how to make this publish the correct type

}

/////////////////////////////////////////////////
GUIExampleSpawnWidget::GUIExampleSpawnWidget()
  : GUIPlugin()
{
  this->counter = 0;
std::cout << "I'm here1" << std::endl;
  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QVBoxLayout *frameLayout = new QVBoxLayout();

  // Create a push button, and connect it to the OnButton function
  QPushButton *button = new QPushButton(tr("Move Selected Object"));
  connect(button, SIGNAL(clicked()), this, SLOT(OnButton()));

  // Add the button to the frame's layout
  frameLayout->addWidget(button);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(10, 10);
  this->resize(300, 30);

  transport::NodePtr selection_subscriber = transport::NodePtr(new transport::Node());
  selection_subscriber->Init();

  gazebo::transport::SubscriberPtr sub = selection_subscriber->Subscribe("~/selection",callback);
  ros::spin();
  // Create a node for transportation
  // this->node = transport::NodePtr(new transport::Node());
  // this->node->Init();
  // this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");

  //this->node = transport::NodePtr(new transport::Node());
  //this->node->Init();
  //this->factoryPub = this->node->Advertise<msgs::PosesStamped>("~/pose/info"); // need to define this topic


}

/////////////////////////////////////////////////
GUIExampleSpawnWidget::~GUIExampleSpawnWidget()
{
}



/////////////////////////////////////////////////
void GUIExampleSpawnWidget::OnButton()
{


  //gazebo::transport::SubscriberPtr sub = selection_subscriber->Subscribe("~/world_stats",callback);

//callback is not being called, need to figure out why
     //physics::ModelState object;
     //object.GetLinkState("")

     //physics::WorldPtr world = physics::World::World("world");
     //objectPtr = _world.GetSelectedEntity();

     //physics::Link object;
     //object.GetLinkState(objectPtr _link)
     //objectPose = object.GetPose();

     //publish the selected object's pose to a topic yet to be defined

     //this->factoryPub->Publish(objectPose);

  // Send the model to the gazebo server

  // msgs::Factory msg;
  // msg.set_sdf(newModelStr.str());
  // this->factoryPub->Publish(msg);
}
