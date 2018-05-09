#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <stdlib.h>
#include <string>
#include <array>
#include <sstream>


std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::shared_ptr<FILE> pipe(popen(cmd, "r"), pclose);
    if (!pipe) throw std::runtime_error("popen() failed!");
    while (!feof(pipe.get())) {
        if (fgets(buffer.data(), 128, pipe.get()) != NULL)
            result += buffer.data();
    }
    std::cout<<result<<std::endl;
    return result;
}

int main()
{
	int input[100];
	for(int i=0; i<100; ++i)
		input[i] = 1;

	std::ostringstream command;
	command << "java -jar NN.jar ";
	for(int i=0; i<100; ++i)
	{
		command<<input[i]<<" ";
	}
	std::string op = exec(command.str().c_str());
	int r = std::stoi(op.substr(0, 1));
	double hc = std::stod(op.substr(2));
	std::cout<<"\n\n"<<r<<" "<<hc;
	
}
