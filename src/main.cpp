#include "Menu.h"
#include <iostream> //brauchen wir das?
#include <pcl/console/parse.h>


int main() //DEBUG RAUS!
{
	Menu menu;
	std::map<std::string, std::function<void()>>  funcMap = {
		{ "extrinsics", std::function<void(void)>(std::bind(&Menu::extrinsics, &menu))},
		{ "intrinsics", std::function<void(void)>(std::bind(&Menu::intrinsics, &menu))},
		{ "record", std::function<void(void)>(std::bind(&Menu::record, &menu))},
		{"registration",std::function<void(void)>(std::bind(&Menu::registration, &menu))},
		{"merge",std::function<void(void)>(std::bind(&Menu::merge, &menu))},
		{ "hausdorff",  std::function<void(void)>(std::bind(&Menu::hausdorff, &menu))},
		{"features",std::function<void(void)>(std::bind(&Menu::features, &menu))},
		{"tolerances",std::function<void(void)>(std::bind(&Menu::tolerances, &menu))},
		{"view", std::function<void(void)>(std::bind(&Menu::view, &menu))},
		{"snap", std::function<void(void)>(std::bind(&Menu::snap, &menu))},
		{"reconnect",std::function<void(void)>(std::bind(&Menu::reconnect, &menu))},
		{"settings",std::function<void(void)>(std::bind(&Menu::settings, &menu))}
	};
	/*cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100);
	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(14, 14, 3,2, dictionary);
	cv::Mat boardImage;
	board->draw(cv::Size(10000, 10000), boardImage, 10, 1);
	cv::imwrite("charucoBild.jpg",boardImage);*/
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

