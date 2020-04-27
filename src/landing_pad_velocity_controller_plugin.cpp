#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/rendering/rendering.hh>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <ignition/math.hh>
#include <sdf/sdf.hh>

#include <thread>

using namespace gazebo;

std::string topic = "/landing_pad/cmd_vel";

class LandingPadVelocityControllerPlugin : public ModelPlugin
{
	public:
 		LandingPadVelocityControllerPlugin() {}
		virtual ~LandingPadVelocityControllerPlugin() {}

	protected:
		// Called when the plugin is loaded into the simulator
		void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			std::cerr << "LandingPadVelocityControllerPlugin is attached to model [" << _model->GetName() << "]" << std::endl;

			this->model = _model;
			this->link = _model->GetLink("link");
			std::cerr << "Using link " << this->link->GetName() << std::endl;
			
			if( ! ros::isInitialized() )
			{
				int argc = 0;
				char **argv = NULL;

				ros::init(argc, argv, "gazebo_landing_pad_velocity_controller_plugin_client");
			}

			if( _sdf->HasElement("topic") )
			{
				topic = _sdf->GetElement("topic")->Get<std::string>();
				std::cerr << "Subscribing to topic [" << topic << "]." << std::endl;
			}
			else
			{
				std::cerr << "Missing parameter 'topic' in Landing Pad Velocity Controller Plugin. Defaulting to [" << topic << "]." << std::endl;
			}

			this->ros_node.reset(new ros::NodeHandle("gazebo_landing_pad_velocity_controller_plugin_client"));

			ros::SubscribeOptions so_landing_pad_velocity = ros::SubscribeOptions::create<geometry_msgs::Twist>(
					topic,
					1,
					boost::bind(&LandingPadVelocityControllerPlugin::set_landing_pad_velocity, this, _1),
					ros::VoidPtr(),
					& this->ros_queue
					);
			this->landing_pad_velocity_subscriber = this->ros_node->subscribe(so_landing_pad_velocity);
			this->ros_queue_thread = std::thread(std::bind(&LandingPadVelocityControllerPlugin::queue_thread, this));
		}

	private:
		physics::ModelPtr model;
		physics::LinkPtr link;
		rendering::VisualPtr visual;
		std::unique_ptr<ros::NodeHandle> ros_node;
		ros::CallbackQueue ros_queue;
		std::thread ros_queue_thread;
		ros::Subscriber landing_pad_velocity_subscriber;

		void set_landing_pad_velocity(const geometry_msgs::TwistConstPtr & msg)
		{
//			std::cerr << "In velocity callback" << std::endl;
//			std::cerr << "< " << msg->linear.x << ", " << msg->linear.y << ", " << msg->linear.z << ">" << std::endl;
			this->model->SetLinearVel({msg->linear.x, msg->linear.y, msg->linear.z});
/*
			ignition::math::Vector3<double>    position = ignition::math::Vector3<double>( _msg->pose.position.x, _msg->pose.position.y, _msg->pose.position.z );
			//										(w, x, y, z) NOT (x, y, z, w)
//			ignition::math::Quaternion<double> rotation = ignition::math::Quaternion<double>(1, 0, 0, 0);
			ignition::math::Quaternion<double> rotation = ignition::math::Quaternion<double>(_msg->pose.orientation.w, _msg->pose.orientation.x, _msg->pose.orientation.y, _msg->pose.orientation.z);
			ignition::math::Pose3<double>          pose = ignition::math::Pose3<double>( position, rotation );

			try
			{
				std::cerr << "( " << pose.Pos().X() << ", " << pose.Pos().Y() << ", " << pose.Pos().Z() << ")" << std::endl;
				this->model->SetWorldPose(pose);
			}
			catch( ... )
			{
				std::cerr << "Caught an exception." << std::endl;
			}
*/
		}

		void queue_thread()
		{
			static const double timeout = 0.05;
			while(this->ros_node->ok())
			{
				this->ros_queue.callAvailable(ros::WallDuration(timeout));
			}
		}
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(LandingPadVelocityControllerPlugin)
