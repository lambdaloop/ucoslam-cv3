#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

//#include <opencv2/xfeatures2d.hpp>

int main(int argc, char * argv[]){

	if (argc<5){
		std::cout <<"Error. The format of this program is ./calibrate rows cols size file.yml img1.jpg [img2.jpg] [...]."<< std::endl;
		exit(-1);
	}


	int rows=atoi(argv[1]);
	int cols=atoi(argv[2]);
	cv::Size patternsize= cv::Size(cols,rows);
	
	int size=atoi(argv[3]);
	std::string doc=argv[4];

	std::vector <cv::Mat> images;
	std::vector<cv::Point2f> corners;

	std::vector< std::vector< cv::Point3f > > object_points;
	std::vector< std::vector< cv::Point2f > > image_points;
	for (int i=5; i<argc; i++){
		cv::Mat img = cv::imread(argv[i], cv::IMREAD_GRAYSCALE);
		if (img.empty())
		{
			std::cerr << "Error: could not read image '" << argv[i] << "'." << std::endl;
			exit(-1);
		}

		images.push_back(img);

	}	
	
	bool agree;
	char option;
	std::vector <cv::Mat> images_res;
	std::vector <cv::Mat> images_drawn;

	std::cout<< "Press y to accept and n to reject images"<<std::endl;
	for(int i=0; i<(int)images.size(); i++){
	
		bool found=cv::findChessboardCorners(images[i], patternsize, corners);
			images_drawn.push_back(images[i].clone());
		if(found){

			cv::cornerSubPix(images[i], corners, cv::Size(5, 5), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
      		cv::drawChessboardCorners(images_drawn[i], patternsize, corners, found);

		}

		images_res.push_back(images[i]);

		cv::resize(images_drawn[i], images_drawn[i], cv::Size(1024, 720));

		//cv::imshow("Corners drawn", images[i]);

		cv::imshow("RESIZED", images_drawn[i]);
		 
		char key=0;
	while(key!='y' && key!='n')
		key=cv::waitKey(0);
 
		if(key == 'y'){
			std::vector< cv::Point3f > obj;
			for(int k = 0; k < rows; k++){
				for(int m = 0; m < cols; m++)
					obj.push_back(cv::Point3f((float)m * size, (float)k * size, 0));
			}

			if (found) {
 				std::cout << i << ". Found corners!" << std::endl;
				image_points.push_back(corners);
				object_points.push_back(obj);
			}

		}
	}

	/*
	for(int i=0; i<(int)images.size(); i++){
		std::string name="Input image ";
		std::string j=std::to_string(i);
		//cv::imshow(name, images[i]);
		std::string out="output";
		std::string png=".png";
		out=out+j+png;
		std::cout << "----"<<out<<"---\n";
		cv::imwrite(out, images[i]);
	}*/
	//cv::waitKey(0);


	printf("Starting Calibration\n");
	cv::Mat K;
	cv::Mat D;
	std::vector< cv::Mat > rvecs, tvecs;

	calibrateCamera(object_points, image_points, images[0].size(), K, D, rvecs, tvecs);

	printf("Recording results...\n");
	
	cv::FileStorage fs(argv[4], cv::FileStorage::WRITE);

	/*fs << "Size of images:" << images[0].size;
	fs << "Camera Matrix:"<<K;
	fs << "distCoeffs:"<<D;
	fs << "rvecs:"<<rvecs;
	

	
*/

    fs << "image_width"<<images[0].cols;
    fs << "image_height"<<images[0].rows;
    fs << "camera_matrix"<<K;
    fs << "distortion_coefficients"<<D;
	fs.release();

}
