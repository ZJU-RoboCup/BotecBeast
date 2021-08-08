#include "main.h"

int stepCount = 6;
bool start = true;
const Eigen::Vector3d S_command = Eigen::Vector3d(0.00, 0.00, 0.0);
const Eigen::Vector3d W_command = Eigen::Vector3d(0.07, 0.00, 0.0);
const Eigen::Vector3d X_command = Eigen::Vector3d(-0.05, 0.00, 0.0);
const Eigen::Vector3d A_command = Eigen::Vector3d(0.00, 0.04, 0.0);
const Eigen::Vector3d D_command = Eigen::Vector3d(0.00, -0.04, 0.0);
const Eigen::Vector3d Z_command = Eigen::Vector3d(0.00, 0.00, 10.0);
const Eigen::Vector3d C_command = Eigen::Vector3d(0.00, 0.00, -10.0);

Eigen::Vector3d gaitCommandStart = W_command;
Eigen::Vector3d gaitCommand = W_command;
int flag_display = 0;

std::vector<Eigen::Vector3d> display_plan;

void init_display()
{
	Eigen::Vector3d temp_step;
	temp_step << 0.03, 0.00, 0;
	display_plan.push_back(temp_step);
	temp_step << 0.03, 0.00, 10;
	display_plan.push_back(temp_step);
	temp_step << 0.03, 0.00, 10;
	display_plan.push_back(temp_step);
	temp_step << 0.03, 0.00, 10;
	display_plan.push_back(temp_step);
	temp_step << 0.03, 0.00, 0;
	display_plan.push_back(temp_step);
	temp_step << 0.03, 0.00, 0;
	display_plan.push_back(temp_step);
	temp_step << 0.03, 0.00, -10;
	display_plan.push_back(temp_step);
	temp_step << 0.03, 0.00, -10;
	display_plan.push_back(temp_step);
	temp_step << 0.03, 0.00, -10;
	display_plan.push_back(temp_step);
}

void reqGaitCommandCallback(const std_msgs::Bool::ConstPtr &req)
{

	std_msgs::Float64MultiArray gaitComm;
	gaitComm.data.resize(3);

	if (start == false)
	{
		if (req->data == 1) //footPrint(Struct queue has less 2 element)
		{
			for (int i = 0; i < 3; ++i)
				gaitComm.data[i] = gaitCommandStart[i];
			std::cout << "gaitCommandStart:" << std::endl
					  << gaitCommandStart.transpose() << std::endl;

			gaitCommand_pub.publish(gaitComm);
		}
		start = true;
		countCommand = 0;
	}
	else if (countCommand < stepCount)
	{
		if (req->data == 1) //footPrint(Struct queue has less 2 element)
		{
			std::cout << "flag_display: " << flag_display << " " << countCommand << std::endl;
			for (int i = 0; i < 3; ++i)
			{
				if (flag_display == 0)
					gaitComm.data[i] = gaitCommand[i];
				else
				{
					std::cout << "dis" << std::endl;
					gaitComm.data[i] = display_plan[countCommand][i];
				}
			}
			Eigen::Vector3d gait_dis;
			gait_dis << gaitComm.data[0], gaitComm.data[1], gaitComm.data[2];
			std::cout << "gaitCommand:" << std::endl
					  << gait_dis.transpose() << std::endl;

			gaitCommand_pub.publish(gaitComm);
		}
		countCommand++;
	}
}

void *command_thread(void *ptr)
{
	ros::Rate loop_rate(10);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gaitCommandNode");

	ros::NodeHandle nh;
	reqGaitCommand_sub = nh.subscribe<std_msgs::Bool>("requestGaitCommand", 1, &reqGaitCommandCallback);
	gaitCommand_pub = nh.advertise<std_msgs::Float64MultiArray>("gaitCommand", 1);

	countCommand = stepCount + 1;
	init_display();

	pthread_t thread_command;
	pthread_create(&thread_command, NULL, command_thread, NULL);

	while (ros::ok())
	{

		usleep(100 * 1000);
		char cmd = getchar();
		double *angle;
		switch (cmd)
		{
		case 'q':
			exit(0);
			break;
		case 's':
			start = false;
			gaitCommandStart = S_command;
			gaitCommand = S_command;
			countCommand = 0;
			break;
		case 'w':
			start = false;
			gaitCommandStart = W_command;
			gaitCommand = W_command;
			countCommand = 0;
			break;
		case 'x':
			start = false;
			gaitCommandStart = X_command;
			gaitCommand = X_command;
			countCommand = 0;
			break;
		case 'a':
			start = false;
			gaitCommandStart = A_command;
			gaitCommand = A_command;
			countCommand = 0;
			break;
		case 'd':
			start = false;
			gaitCommandStart = D_command;
			gaitCommand = D_command;
			countCommand = 0;
			break;
		case 'z':
			start = false;
			gaitCommandStart = Z_command;
			gaitCommand = Z_command;
			countCommand = 0;
			break;
		case 'c':
			start = false;
			gaitCommandStart = C_command;
			gaitCommand = C_command;
			countCommand = 0;
			break;
		case 'k':
			start = false;
			gaitCommandStart = W_command;
			gaitCommand = C_command;
			stepCount = display_plan.size();
			std::cout << "stepCount: " << stepCount << std::endl;
			flag_display = 1;
			countCommand = 0;
			break;
		default:
			break;
		}
	}
}
