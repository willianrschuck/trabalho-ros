#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <std_msgs/Empty.h>
#include <cstdlib>

#include <robot_actions/MovementAction.h>
#include <robot_actions/MovementGoal.h>
#include <robot_actions/MovementResult.h>
#include <robot_actions/MovementFeedback.h>

class MovementClient {
	
	protected:
	ros::NodeHandle _nh;
	actionlib::SimpleActionClient<robot_actions::MovementAction> _ac;
	ros::Subscriber _cancel_subscriber;

	public:
	MovementClient():
		_ac("/moveto", true)
	{
		ROS_INFO("Esperando pelo servidor...");
		_ac.waitForServer();
		ROS_INFO("Servidor ativo");

		_cancel_subscriber = _nh.subscribe("/cancel_move", 10,
			&MovementClient::cancelMoveCallback, this);
	}

	void cancelMoveCallback(const std_msgs::Empty &msg)
	{
		ROS_INFO("Recebida mensagem para cancelar a ação");
		_ac.cancelGoal();
	}

	void sendGoal(int i) {
		robot_actions::MovementGoal goal;
		goal.iterations = i;
		_ac.sendGoal(goal,
			boost::bind(&MovementClient::doneCb, this, _1, _2),
			boost::bind(&MovementClient::activeCb, this),
			boost::bind(&MovementClient::feedbackCb, this, _1));
		ROS_INFO("Objetivo enviado");
	}

	void doneCb(const actionlib::SimpleClientGoalState& state,
		const robot_actions::MovementResultConstPtr &result)
	{
		ROS_INFO("Estado final: %s",
			state.toString().c_str());
        ROS_INFO("Resultado: Pos(x: %d, y: %d), Battery(power: %d, status: %s)", 
			(int)result->x,
            (int)result->y,
            (int)result->battery,
            result->message.c_str());
	}

	void activeCb() 
	{
		ROS_INFO("Objetivo ativado");
	}	

	void feedbackCb(const robot_actions::MovementFeedbackConstPtr &feedback)
	{
		ROS_INFO("Feedback: Pos(x: %d, y: %d), Battery(power: %d, status: %s)", 
			(int)feedback->x,
            (int)feedback->y,
            (int)feedback->battery,
            feedback->status.c_str());
	}
};

int main (int argc, char **argv) 
{
	ros::init(argc, argv, "move_robot_client");
	MovementClient client;
	client.sendGoal(atoi(argv[1]));
	ros::spin();
}
