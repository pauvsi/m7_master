#include<limits.h>
#include<float.h>
#include<string>
#include <array>
#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "MasterTypes.h"

#define NO_OBS 4
#define NO_TARGETS 10

geometry_msgs::PoseWithCovarianceStamped roombaPose[10], obsPose[4];

/*
 * TODO: Change the java class to make sure that you ignore NN points where it goes out of bounds
 */

double euclideanDistance(double x1, double x2, double y1, double y2)
{
	return (sqrt((x1 - x2)*(x1-x2) + (y1 - y2)*(y1-y2)));
}

std::string systemCall(std::string command)
{
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(command.c_str(), "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != NULL)
            result += buffer.data();
    }
    return result;
}


void execNN(State state)
{
	double input[100];
//	std::string command = "java -jar NN.jar ";
	std::ostringstream command;
	command << " java -jar NN.jar";
	for(int i=0; i<NO_TARGETS; ++i)
	{
		input[i*10 + 0] = euclideanDistance(roombaPose[i].pose.pose.position.x, state.pos.x(), roombaPose[i].pose.pose.position.y, state.pos.y());
		//Time. Maybe used. make sure.
		input[i*10 + 1] = 0.0;
		input[i*10 + 2] = roombaPose[i].pose.pose.position.x;
		input[i*10 + 3] = roombaPose[i].pose.pose.position.y;
		//NOTE: orientation x and y are actually dx and dy
		input[i*10 + 4] = roombaPose[i].pose.pose.orientation.x;
		input[i*10 + 5] = roombaPose[i].pose.pose.orientation.y;

		int index = 0;
		double dist = DBL_MAX;
		for(int k=0; k<NO_OBS; ++k)
		{
			if(euclideanDistance(roombaPose[i].pose.pose.position.x, obsPose[k].pose.pose.position.x,
					roombaPose[i].pose.pose.position.y, obsPose[k].pose.pose.position.y) < dist)
			{
				dist = euclideanDistance(roombaPose[i].pose.pose.position.x, obsPose[k].pose.pose.position.x,
						roombaPose[i].pose.pose.position.y, obsPose[k].pose.pose.position.y);
				index = k;
			}
		}

		input[i*10 + 6] = obsPose[index].pose.pose.position.x;
		input[i*10 + 7] = obsPose[index].pose.pose.position.y;
		//NOTE: orientation x and y are actually dx and dy
		input[i*10 + 8] = obsPose[index].pose.pose.orientation.x;
		input[i*10 + 9] = obsPose[index].pose.pose.orientation.y;

		for(int j=0; j<10; ++j)
		{
			command << input[i*10+j]<<" ";

		}
	}

	systemCall(command.str());
}

