/*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Willow Garage, Inc.
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
*   * Neither the name of Willow Garage, Inc. nor the names of its
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
*
* $Id$
*
*/

#define MEASURE_FUNCTION_TIME
#include <pcl/common/time.h> //fps calculations
#include <pcl/common/angles.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/boost.h>
#include <pcl/visualization/image_viewer.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <boost/chrono.hpp>

#include "pcl/io/openni2/openni.h"

#include <time.h>
typedef boost::chrono::high_resolution_clock HRClock;

#define SHOW_FPS 1
#if SHOW_FPS
#define FPS_CALC(_WHAT_) \
  do \
{ \
  static unsigned count = 0;\
  static double last = pcl::getTime ();\
  double now = pcl::getTime (); \
  ++count; \
  if (now - last >= 1.0) \
{ \
  std::cout << "Average framerate ("<< _WHAT_ << "): " << double (count)/double (now - last) << " Hz" <<  std::endl; \
  count = 0; \
  last = now; \
} \
}while (false)
#else
#define FPS_CALC (_WHAT_) \
  do \
{ \
}while (false)
#endif

template <typename PointType>
class OpenNI2Viewer
{
public:
  typedef pcl::PointCloud<PointType> Cloud;
  typedef typename Cloud::ConstPtr CloudConstPtr;

  OpenNI2Viewer (pcl::io::OpenNI2Grabber& grabber)
    : cloud_viewer_ (new pcl::visualization::PCLVisualizer ("PCL OpenNI2 cloud"))
    , grabber_ (grabber)
  {
  }

  void
  cloud_callback (const CloudConstPtr& cloud)
  {
    FPS_CALC ("cloud callback");
    boost::mutex::scoped_lock lock (cloud_mutex_);
    cloud_ = cloud;
  }

  void
  keyboard_callback (const pcl::visualization::KeyboardEvent& event, void*)
  {
    if (event.getKeyCode ())
      cout << "the key \'" << event.getKeyCode () << "\' (" << event.getKeyCode () << ") was";
    else
      cout << "the special key \'" << event.getKeySym () << "\' was";
    if (event.keyDown ())
      cout << " pressed" << endl;
    else
      cout << " released" << endl;
  }

  void
  mouse_callback (const pcl::visualization::MouseEvent& mouse_event, void*)
  {
    if (mouse_event.getType () == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton () == pcl::visualization::MouseEvent::LeftButton)
    {
      cout << "left button pressed @ " << mouse_event.getX () << " , " << mouse_event.getY () << endl;
    }
  }

  /**
  * @brief starts the main loop
  */
  void
	  run()
  {
	  cloud_viewer_->registerMouseCallback(&OpenNI2Viewer::mouse_callback, *this);
	  cloud_viewer_->registerKeyboardCallback(&OpenNI2Viewer::keyboard_callback, *this);
	  cloud_viewer_->setCameraFieldOfView(1.02259994f);
	  boost::function<void(const CloudConstPtr&) > cloud_cb = boost::bind(&OpenNI2Viewer::cloud_callback, this, _1);
	  boost::signals2::connection cloud_connection = grabber_.registerCallback(cloud_cb);

	  bool cloud_init = false;

	  grabber_.start();

	  while (!cloud_viewer_->wasStopped())
		  //while(true)
	  {
		  CloudConstPtr cloud;
		  cloud_viewer_->spinOnce();

		  // See if we can get a cloud
		  if (cloud_mutex_.try_lock())
		  {
			  cloud_.swap(cloud);
			  cloud_mutex_.unlock();
		  }

		  if (cloud)
		  {
			  FPS_CALC("drawing cloud");

			  if (!cloud_init)
			  {
				  cloud_viewer_->setPosition(0, 0);
				  cloud_viewer_->setSize(cloud->width, cloud->height);
				  cloud_init = !cloud_init;
			  }

			  if (!cloud_viewer_->updatePointCloud(cloud, "OpenNICloud"))
			  {
				  cloud_viewer_->addPointCloud(cloud, "OpenNICloud");
				  cloud_viewer_->resetCameraViewpoint("OpenNICloud");
				  cloud_viewer_->setCameraPosition(
					  0, 0, 0,		// Position
					  0, 0, 1,		// Viewpoint
					  0, -1, 0);	// Up
			  }
		  }

	  }
		  grabber_.stop();

		  cloud_connection.disconnect();
  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> cloud_viewer_;

  pcl::io::OpenNI2Grabber& grabber_;
  boost::mutex cloud_mutex_;

  CloudConstPtr cloud_;
};

// Create the PCLVisualizer object
//boost::shared_ptr<pcl::visualization::PCLVisualizer> cld;

/* ---[ */
int
main (int argc, char** argv)
{
  pcl::io::OpenNI2Grabber grabber;
  OpenNI2Viewer<pcl::PointXYZ> openni_viewer(grabber);
  openni_viewer.run();
  return (0);
}
/* ]--- */
