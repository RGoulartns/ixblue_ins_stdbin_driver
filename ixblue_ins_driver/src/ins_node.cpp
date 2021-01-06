#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Int8.h>
#include <std_msgs/String.h>

#include <sys/socket.h>
#include <arpa/inet.h>

#define PORT 8110
#define INS_IP "10.0.1.10"

double latitude, longitude, altitude;
bool coordinateReceived = false;
bool responseRequired_ = false;
char response[256];
int command = 0;
int retries = 0;
std::vector<int> commandQueue;


void gpsManualCB(const sensor_msgs::NavSatFixConstPtr& msg)
{
	std::cout << "INS node received the GPS fix" << std::endl;
	coordinateReceived = true;
	latitude = msg->latitude;
	longitude = msg->longitude;
	altitude = msg->altitude;
	commandQueue.push_back(7);
}

void insCommandCB(const std_msgs::Int8ConstPtr& msg)
{
	commandQueue.insert(commandQueue.begin(), msg->data);
}


std::string createMessage(int option)
{

	std::ostringstream oss;
	oss << std::fixed << std::setprecision(6);
	oss << "$PIXSE,CONFIG,";

	switch(option)
	{
		case 1: // Reset INS
			oss << "RESET_";
			break;
		case 2: // Set INS start mode to "wait for position"
			oss << "START_,1";
			break;
		case 3: // Set INS start mode to "Restore Position"
			oss << "START_,2";
			break;
		case 4: // Set INS start mode to "Restore Attitude"
			oss << "START_,3";
			break;
		case 5: // Ignore GPS data
			oss << "GPSKFM,1";
			break;
		case 6: // Use GPS data
			oss << "GPSKFM,2";
			break;
		case 7: // Set a GPS position fix manually
			if(coordinateReceived)
				oss << "MANGPS," << latitude << "," << longitude << "," << altitude << ",0.5,0.5,5.0";
			break;
		case 8: // check gps status
			oss << "GPSKFM,,";
			responseRequired_ = true;
			break;
		case 9: // check starting mode
			oss << "START_,,";
			responseRequired_ = true;
			break;
		default:
			return NULL;
			break;
	}

	int checksum = 0;
	for(int i = 1; i < oss.str().length(); i++)
		checksum = checksum ^ oss.str().at(i);

	oss << "*" << std::hex << checksum;
	oss << "\r\n";
	return oss.str();
}

bool sendCommand(std::string msg, char* resp)
{
	struct sockaddr_in serv_addr;
	int sockfd;
	sockfd = socket(AF_INET, SOCK_STREAM, 0);
	
	if (sockfd < 0)
		ROS_ERROR("ERROR opening socket");
	
	serv_addr.sin_family = AF_INET;
	serv_addr.sin_port = htons(PORT);
	
	if(inet_pton(AF_INET, INS_IP, &serv_addr.sin_addr) <= 0)
		ROS_ERROR("Invalid INS address or address not supported");
	if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
		ROS_ERROR("ERROR connecting");

	int nt = write(sockfd, msg.c_str(), strlen(msg.c_str()));
	if (nt < 0)
	{
		ROS_ERROR("Couldn't send command to INS. Reason: ERROR writing to socket");
		return 0;
	}

	if(responseRequired_)
	{
		nt = read(sockfd, resp, 255);

		char* pch;
		pch = strrchr(resp,',');
		memset(resp, 0, sizeof(resp));

		int i = 0;
		while(*++pch !='*' || pch==NULL)
			resp[i++] = *pch;
	}

	close(sockfd);
	return 1;
}


int main(int argc, char **argv)
{

	ros::init(argc, argv, "ins_node");
	ros::NodeHandle n;
	ros::Rate loopRate(1);

	ros::Subscriber insCommand_sub = n.subscribe<std_msgs::Int8>("/ins/command", 5, &insCommandCB);
	ros::Subscriber gpsManual_sub = n.subscribe<sensor_msgs::NavSatFix>("/gps_coordinates", 5, &gpsManualCB);
	ros::Publisher insCmdResponse_pub = n.advertise<std_msgs::String>("/ins/command/response", 5);

	while( ros::ok() )
	{
		if(!commandQueue.empty())
		{
			std::string msg = createMessage(commandQueue.back());
			bool result = sendCommand(msg, response);

			if(responseRequired_)
			{
				responseRequired_ = false;
				std_msgs::String resp_msg;
				resp_msg.data = response;
				resp_msg.data.append(std::to_string(commandQueue.back()));
				insCmdResponse_pub.publish(resp_msg);
			}

			if(result)
				commandQueue.pop_back();
			else if(++retries > 7)
			{
				commandQueue.pop_back();
				retries = 0;
			}
		}

		ros::spinOnce();
		loopRate.sleep();
	}
	return 0;
}
