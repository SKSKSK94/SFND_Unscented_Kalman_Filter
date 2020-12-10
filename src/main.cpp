/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

//#include "render/render.h"
#include "highway.h"
#include "matplotlib-cpp/matplotlibcpp.h"

int main(int argc, char** argv)
{

	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);

	// set camera position and angle
	viewer->initCameraParameters();
	float x_pos = 0;
	viewer->setCameraPosition ( x_pos-26, 0, 15.0, x_pos+25, 0, 0, 0, 0, 1);

	Highway highway(viewer);

	//initHighway(viewer);

	int frame_per_sec = 30;
	int sec_interval = 10;
	int frame_count = 0;
	int time_us = 0;

	double egoVelocity = 25;

	bool visNIS = true;

	while (frame_count < (frame_per_sec*sec_interval))
	{
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		//stepHighway(egoVelocity,time_us, frame_per_sec, viewer);
		highway.stepHighway(egoVelocity,time_us, frame_per_sec, viewer);
		viewer->spinOnce(1000/frame_per_sec);
		frame_count++;
		time_us = 1000000*frame_count/frame_per_sec;		
	}

	if(visNIS)
	{
		std::vector<Car> cars = highway.traffic;
		bool useRadar = cars[0].ukf.use_radar_;
		bool useLidar = cars[0].ukf.use_laser_;
		
		int NISlength = (useRadar && useLidar) ? max(cars[0].ukf.radarNIS.size(), cars[0].ukf.lidarNIS.size()) :
			((useRadar ? cars[0].ukf.radarNIS.size() : cars[0].ukf.lidarNIS.size()));

		int numUseSensors = (useRadar && useLidar) ? 2 : 1;

		double lidarThreshold = 5.991; // chi-square distribution(0.05) with 2 DOF in measurement space
		std::vector<double> lidarThresholdVec(NISlength, lidarThreshold);
		double radarThreshold = 7.815; // chi-square distribution(0.05) with 3 DOF in measurement space
		std::vector<double> radarThresholdVec(NISlength, radarThreshold);

		MatrixXd overThreshold(numUseSensors, cars.size()); // overThreshold(0, -) = lidar, overThreshold(1, -) = radar
		overThreshold.fill(0);

		std::string str;

		matplotlibcpp::figure_size(1600,900);
		
		for(int i = 0 ; i < cars.size(); ++i)
		{
			if (useLidar && useRadar)
			{
				for (int j = 0; j < cars[i].ukf.lidarNIS.size(); ++j)
					if ( cars[i].ukf.lidarNIS[j] > lidarThreshold) overThreshold(0, i)++;
				for (int j = 0; j < cars[i].ukf.radarNIS.size(); ++j)
					if ( cars[i].ukf.radarNIS[j] > radarThreshold) overThreshold(1, i)++;

				matplotlibcpp::subplot(numUseSensors, cars.size(), i + 1);
				matplotlibcpp::plot(cars[i].ukf.lidarNIS);
				matplotlibcpp::plot(lidarThresholdVec, "r--");
				matplotlibcpp::xlabel("frames");
				matplotlibcpp::ylabel("Lidar NIS");
				matplotlibcpp::grid(true);
				str = ""; str += "Car" + std::to_string(i+1) + " with over threshold " + 
								std::to_string(overThreshold(0, i)/cars[i].ukf.lidarNIS.size()*100) + "%";
				matplotlibcpp::title(str);

				matplotlibcpp::subplot(numUseSensors, cars.size(), i + 1 + cars.size());
				matplotlibcpp::plot(cars[i].ukf.radarNIS);
				matplotlibcpp::plot(radarThresholdVec, "r--");
				matplotlibcpp::xlabel("frames");
				matplotlibcpp::ylabel("Radar NIS");
				matplotlibcpp::grid(true);
				str = ""; str += "Car" + std::to_string(i+1) + " with over threshold " + 
								std::to_string(overThreshold(1, i)/cars[i].ukf.radarNIS.size()*100) + "%";
				matplotlibcpp::title(str);
			}
			else if (useLidar)
			{
				for (int j = 0; j < cars[i].ukf.lidarNIS.size(); ++j)
					if ( cars[i].ukf.lidarNIS[j] > lidarThreshold) overThreshold(0, i)++;

				matplotlibcpp::subplot(numUseSensors, cars.size(), i + 1);
				matplotlibcpp::plot(cars[i].ukf.lidarNIS);
				matplotlibcpp::plot(lidarThresholdVec, "r--");
				matplotlibcpp::xlabel("frames");
				matplotlibcpp::ylabel("Lidar NIS");
				matplotlibcpp::grid(true);				
				str = ""; str += "Car" + std::to_string(i+1) + " with over threshold " + 
								std::to_string(overThreshold(0, i)/cars[i].ukf.lidarNIS.size()*100) + "%";
				matplotlibcpp::title(str);
			}
			else
			{
				for (int j = 0; j < cars[i].ukf.radarNIS.size(); ++j)
					if ( cars[i].ukf.radarNIS[j] > radarThreshold) overThreshold(0, i)++;

				matplotlibcpp::subplot(numUseSensors, cars.size(), i + 1);
				matplotlibcpp::plot(cars[i].ukf.radarNIS);
				matplotlibcpp::plot(radarThresholdVec, "r--");
				matplotlibcpp::xlabel("frames");
				matplotlibcpp::ylabel("Radar NIS");
				matplotlibcpp::grid(true);
				str = ""; str += "Car" + std::to_string(i+1) + " with over threshold " + 
								std::to_string(overThreshold(0, i)/cars[i].ukf.radarNIS.size()*100) + "%";
				matplotlibcpp::title(str);
			}
		}

		matplotlibcpp::show(true);
		
	}
}