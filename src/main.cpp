
#include "OpenStaServerHandler.hpp"

#include "channel/YasMessageSerdes.hpp"
#include "channel/ShmemSerdesIpcChannel.hpp"
#include "server/StaServerIpcProtocol.hpp"

#include <iostream>
#include <exception>


int main(int argc, char** argv) {

	//some simple command line parsing
	if(argc != 2) {
		std::cout << "Channel name is required as single argument.\n"
				<< "Usage : executable <channel_name>" << std::endl;
		return 1;
	}

	std::cout << "Opening STA channel '" << argv[1] << "'" << std::endl;
	stamask::YasMessageSerdes* serDesPtr =
			new stamask::YasMessageSerdes();
	auto channelPtr = new stamask::ShmemSerdesIpcChannel(
			serDesPtr, argv[1], true);

	if(!channelPtr->connect()) {
		std::cout << "Failed to open the channel '" << argv[1] << "'" << std::endl;
		delete channelPtr;
		delete serDesPtr;
		return 1;
	}

	std::cout << "Preparing command handler" << std::endl;
	stamask::IMessageExecutor* handlerPtr = nullptr;
	try {
		handlerPtr = new stamask::OpenStaServerHandler();
	} catch (const std::exception& ex) {
		std::cout << "handler std ex: " << ex.what() << std::endl;
		return 1;
	} catch(...) {
		std::cout << "handler unknown ex: " << std::endl;
		return 1;
	}

	std::cout << "Running execution loop" << std::endl;
	stamask::StaServerIpcProtocol protocol(channelPtr, handlerPtr);
	try {
		if(!protocol.runCycle()) {
			std::cout << "Exiting loop with error" << std::endl;
			return 1;
		}
	} catch (const std::exception& ex) {
		std::cout << "loop std ex: " << ex.what() << std::endl;
	} catch(...) {
		std::cout << "loop unknown ex: " << std::endl;
	}

	std::cout << "Exiting..." << std::endl;
	return 0;
}


