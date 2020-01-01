// The following is a program  written to  estimate the pose of the DeepRacer
// relative to a given target.

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/TransformStamped.h>
#include <parallel_park/ServoCtrlMsg.h> // To replace the ServoCtrlMsg from DeepRacer's ctrl_pkg
//#include <cmath.h>

bool canPark;

//************************************************************
// Function to Compute the Distance Between Two Frames (Essentially 2 Tags)
float computeDistance(float xFirst, float yFirst, float zFirst, float xSecond, float ySecond, float zSecond){
	float xDistance = 0;
	float yDistance = 0;
	float zDistance = 0;

	xDistance = (xFirst - xSecond)*100;
	ROS_INFO_STREAM("\n\rxDistance: " << xDistance);

	return xDistance;
}

//************************************************************
// Callback function to register with tf2_ros::MessageFilter to be called when frame 3 is available
// void frame3_Callback(const geometry_msgs::TransformStampedConstPtr& framePtr)
// {
//   int f;
//   geometry_msgs::TransformStamped transform3;
//   tf2_ros::Buffer tfBuffer3;
//   tf2_ros::TransformListener tfListener3(tfBuffer3);
//
//   try{
//     transform3 = tfBuffer3.lookupTransform("tag_3", "cv_camera", ros::Time(0));
//   }
//   catch(tf2::TransformException &ex){
//     ROS_WARN("%s", ex.what());
//     ros::Duration(1.0).sleep();
//   }
// }

//************************************************************
// Main function
int main(int argc, char **argv){
	// Initialize the ROS system.
	ros::init(argc, argv, "parallel_park");

	// Establish this program as a ROS node
	ros::NodeHandle nh;

	// Send a log message to indicate the node is up and running
	ROS_INFO_STREAM("Parallel Park is up!");

	// Create a publisher object to publish to the /manual_drive topic to
	// control the servo
	ros::Publisher pub = nh.advertise<parallel_park::ServoCtrlMsg>("/manual_drive", 100);

	// Create buffer objects and transformListener objects to deal with
	// getting the transforms
	// tf2_ros::Buffer tfBuffer1;
	// tf2_ros::Buffer tfBuffer2;
  tf2_ros::Buffer tfBuffer3;
	// tf2_ros::TransformListener tfListener1(tfBuffer1);
	// tf2_ros::TransformListener tfListener2(tfBuffer2);
  tf2_ros::TransformListener tfListener3(tfBuffer3);

	// Set the sleep rate to 2s
	ros::Rate rate(2.0);
	long prevSecs3 = 0; // This is how I make sure it's a new transform each time
	float xd = 34;
	float zd = 130;

	// Perform the following as long as the node is running
	while(ros::ok()){
		// Objects to hold the Transforms
		// geometry_msgs::TransformStamped transformStamped1;
		// geometry_msgs::TransformStamped transformStamped2;
		geometry_msgs::TransformStamped transformStamped3;

		// Try to get each transform, and warn if doesn't work each time
		// try{
		// 	transformStamped1 = tfBuffer1.lookupTransform("cv_camera", "tag_1", ros::Time(0));
		// 	transformStamped2 = tfBuffer2.lookupTransform("cv_camera", "tag_2", ros::Time(0));
		// 	//transformStamped3 = tfBuffer3.lookupTransform("cv_camera", "tag_3", ros::Time(0));
		// }
		// catch(tf2::TransformException &ex){
		// 	ROS_WARN("%s", ex.what());
		// 	ros::Duration(1.0).sleep();
		// 	continue;
		// }
		int tag3Ready = 1;
		try{
			transformStamped3 = tfBuffer3.lookupTransform("tag_3", "cv_camera", ros::Time(0));
			tag3Ready = 1;
		}
		catch(tf2::TransformException &ex){
			ROS_INFO_STREAM("tag_3 not ready");
			ros::Duration(1.0).sleep();
			tag3Ready = 0;
			continue;
		}
		// ROS_INFO_STREAM("tag3Ready:" << tag3Ready);

		// Holders for each of the tags' frame translations
		// float x1trans= (transformStamped1.transform.translation.x)*1000;
		// float y1trans = (transformStamped1.transform.translation.y)*1000;
		// float z1trans = (transformStamped1.transform.translation.z)*1000;
		// float x2trans = (transformStamped2.transform.translation.x)*1000;
		// float y2trans = (transformStamped2.transform.translation.y)*1000;
		// float z2trans = (transformStamped2.transform.translation.z)*1000;
		float x3trans = (float)(transformStamped3.transform.translation.x)*(float)1000;
		float y3trans = (float)(transformStamped3.transform.translation.y)*(float)1000;
		float z3trans = (float)(transformStamped3.transform.translation.z)*(float)1000;
		long tfSecs3 = transformStamped3.header.stamp.sec;

		ROS_INFO_STREAM("xTrans:" << x3trans);
		ROS_INFO_STREAM("yTrans:" << y3trans);
		ROS_INFO_STREAM("zTrans:" << z3trans);

		// Create the single Servo Control Message that will be published
		parallel_park::ServoCtrlMsg control;

		// float xDistance = computeDistance(x1trans,y1trans,z1trans,x2trans,y2trans,z2trans);
		// float prevDistance = 0;
		//
		// // ROS_INFO_STREAM("Distance between tag 1 and tag 2: " << xDistance);
		// // If xDistance > 60cm, continue on to the next part
		// if(xDistance > 40){
			// control.angle = (float) (0.9);
			// control.throttle = (float) (0.65);
			// pub.publish(control);
			ros::Duration(2.0).sleep();
			float steering;
			float throttle;

			while(xtrans != xd && z3trans != zd){
				steering = -1*(xd - x3trans);
				throttle = -1*(zd - z3trans);

				ROS_INFO_STREAM("Steering: " << steering);
				ROS_INFO_STREAM("Throttle: " << throttle);
			}

			// if(tag3Ready){
			// 	ROS_INFO_STREAM(x3trans);
			// 	ROS_INFO_STREAM(y3trans);
			// 	ROS_INFO_STREAM(z3trans);
			// }
			// // Find out if tag 3 ready
			// if(tag3Ready){
			// 	// Test if tag 3 is within an acceptable range indicating we can park
			// 	if((x3trans<46 && x3trans>33) && (z3trans<70 && z3trans>56) && (prevSecs3 != tfSecs3)){
			// 		ROS_INFO_STREAM("Starting to park");
			//
			// 		control.angle = (float) (-0.9);
			// 		control.throttle = (float) (-0.65);
			// 		pub.publish(control);
			// 		ROS_INFO_STREAM("Right turn");
			// 		ros::Duration(0.95).sleep();
			//
			// 		control.angle = (float) (0.9);
			// 		control.throttle = (float) (-0.65);
			// 		pub.publish(control);
			// 		ROS_INFO_STREAM("Left turn");
			// 		ros::Duration(0.5).sleep();
			//
			// 		control.angle = (float) (-0.9);
			// 		control.throttle = (float) (0.65);
			// 		pub.publish(control);
			// 		ROS_INFO_STREAM("Correct");
			// 		ros::Duration(0.45).sleep();
			//
			// 		control.angle = (float) (0);
			// 		control.throttle = (float) (0);
			// 		ROS_INFO_STREAM("Rest");
			// 		pub.publish(control);
			//
			// 		prevSecs3 = tfSecs3;
			// 	}
			// 	else{
			// 		control.angle = (float) (0);
			// 		control.throttle = (float) (0);
			// 		ROS_INFO_STREAM("Rest");
			// 		pub.publish(control);
			//
			// 		prevSecs3 = tfSecs3;
			// 	}
			// }

 		} // end while
}
