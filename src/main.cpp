/*
Copyright (c) 2020
TU Berlin, Institut für Werkzeugmaschinen und Fabrikbetrieb
Fachgebiet Industrielle Automatisierungstechnik
Authors: Justin Heinz
All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and /or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
DISCLAIMER: THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include "Menu.h"
#include <iostream> //brauchen wir das?
#include <pcl/console/parse.h>


int main()
{
	//This code allows to generate a charuco board and save it as an image. Just uncomment it and recompile.
	/*cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(14, 14, 3,2, dictionary);
	cv::Mat boardImage;
	board->draw(cv::Size(10000, 10000), boardImage, 10, 1);
	cv::imwrite("charucoBild.jpg",boardImage);*/
	Menu menu;
	std::map<std::string, std::function<void()>>  funcMap = {
		{ "extrinsics", std::function<void(void)>(std::bind(&Menu::extrinsics, &menu))},
		{ "intrinsics", std::function<void(void)>(std::bind(&Menu::intrinsics, &menu))},
		{ "record", std::function<void(void)>(std::bind(&Menu::record, &menu))},
		{"registration",std::function<void(void)>(std::bind(&Menu::registration, &menu))},
		{"merge",std::function<void(void)>(std::bind(&Menu::merge, &menu))},
		{ "hausdorff",  std::function<void(void)>(std::bind(&Menu::hausdorff, &menu))},
		{"tolerances",std::function<void(void)>(std::bind(&Menu::tolerances, &menu))},
		{"view", std::function<void(void)>(std::bind(&Menu::view, &menu))},
		{"snap", std::function<void(void)>(std::bind(&Menu::snap, &menu))},
		{"reconnect",std::function<void(void)>(std::bind(&Menu::reconnect, &menu))},
		{"settings",std::function<void(void)>(std::bind(&Menu::settings, &menu))}
	};
	std::string function;
	while (true){
		std::cout << "\n>> ";
		std::cin >> function;
		if (function == "q" || function == "quit")
			break;
		else if (function == "help" || function == "h")
			menu.displayIntro();
		else if(funcMap.find(function) != funcMap.end()) {
			try {
				std::invoke(funcMap[function]);
			}
			catch (const rs2::error & e)
			{
				PCL_ERROR("RealSense error:\n");
				std::cerr << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
			}
			catch (const std::exception & e)
			{
				PCL_ERROR(e.what()); PCL_ERROR("\n");
			}
		}
		else {
			std::cout << "The given function is not available. Type \"h\" for help.\n";
		}
	}
	return 0;
}

