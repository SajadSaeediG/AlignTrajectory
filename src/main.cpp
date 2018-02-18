#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <string.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include <Eigen/Eigen>
#include <AlignTrajectory.h>
#include <Viewer.h>
#include <pangolin/pangolin.h>


using namespace cv;
using namespace std;

enum GTformat  { TUM, VISIM, ICLNUIM };

bool readFiles(const std::string& strGroundTruth,
               const std::string& strEstimate,
               std::vector<std::pair<double, Eigen::Matrix4d>>& vGroundTruthTimed,
               std::vector<std::pair<double, Eigen::Matrix4d>>& vEstimateTimed,
               const std::string& strGTformat);

Eigen::Matrix3d quat2mat(float qx, float qy, float qz, float qw);

int main(int argc, char* argv[])
{
    for(int i = 0; i < argc; ++i)
        printf("Argument %d : %s\n", i, argv[i]);

    if(argc < 4)
    {
        std::cout << "USAGE: ./align [ground_truth_trajectory] [estime_trajectory] [dataset format: TUM, VISIM, ICLNUIM]" << std::endl;
        return 1;
    }

    std::string strGroundTruth = std::string(argv[1]);
    std::string strEstimate    = std::string(argv[2]);
    std::string strGTformat    = std::string(argv[3]);
 
    int iDriftRange  = 1;   //for RPE evaluation, number of frames to evaluate the drift over

    std::vector<std::pair<double, Eigen::Matrix4d>> vGroundTruthTimed;
    std::vector<std::pair<double, Eigen::Matrix4d>> vEstimateTimed;
    std::vector<std::pair<double, Eigen::Matrix4d>> vTransformedTimed;


    
    std::vector<Eigen::Matrix4d> vGroundTruth;
    std::vector<Eigen::Matrix4d> vEstimate;
    std::vector<Eigen::Matrix4d> vTransformed;

    readFiles(strGroundTruth, strEstimate, vGroundTruthTimed, vEstimateTimed, strGTformat);

    // calculte ATE 
    float ate=0;
    AlignTrajectory align;
     
    bool bDoAssociation;
    if(vGroundTruthTimed.size() == vEstimateTimed.size())
        bDoAssociation = false;
    else
        bDoAssociation = true;


    Eigen::Matrix4d Mat = align.calculateATE(vGroundTruthTimed, vEstimateTimed, ate, bDoAssociation);

    std::cout << "M is : " << std::endl << Mat << std::endl;
    std::cout << "ATE is: " << std::endl << ate << std::endl;

    // generate the aligned trajectory
    Eigen::Matrix3d scaledRotation = Mat.block<3,3>(0,0);
    Eigen::Vector3d translation    = Mat.block<3,1>(0,3);
    vTransformedTimed.clear();

    for(int i = 0 ; i < vEstimateTimed.size(); i++)
    {
        std::pair<double, Eigen::Matrix4d> poseTimed;
        poseTimed = std::make_pair(vEstimateTimed[i].first, (Mat.inverse())*vEstimateTimed[i].second);
        vTransformedTimed.push_back(poseTimed);
    }

    // calculate RPE
    int iDeltaFrames;
    double  rpe_rmse;
 
    std::vector<Eigen::Matrix4d>  rpe = align.calculateRPE(vGroundTruthTimed, vEstimateTimed, iDriftRange, rpe_rmse, bDoAssociation);
    
    if(!(rpe.size() == 0))
        std::cout << "RPE is: " << std::endl << rpe_rmse << std::endl;
    else
        std::cout << "RPE is: " << std::endl << "N.A." << std::endl;

    // plot results
    Viewer* mpViewer;
    std::thread* mptViewer;

    mpViewer = new Viewer(&vGroundTruthTimed, &vEstimateTimed, &vTransformedTimed);
    mptViewer = new thread(&Viewer::Run, mpViewer);

    getchar();
    cout << "Done!" << endl;

    if(mpViewer)
    {
        mpViewer->RequestStop();
        while(!mpViewer->isStopped())
            sleep(1);
    }


/*    if(mpViewer)
        pangolin::BindToContext("Trajectory");*/

    return 0;
}

Eigen::Matrix3d quat2mat(float qx, float qy, float qz, float qw)
{
    Eigen::Matrix3d rot;
    
    rot(0,0) = 1.0;
    rot(0,1) = 0.0;
    rot(0,2) = 0.0;

    rot(1,0) = 0.0;
    rot(1,1) = 1.0;
    rot(1,2) = 0.0;

    rot(2,0) = 0.0;
    rot(2,1) = 0.0;
    rot(2,2) = 1.0;

    return rot;
}

bool readFiles(const std::string& strGroundTruth,
               const std::string& strEstimate,
               std::vector<std::pair<double, Eigen::Matrix4d>>& vGroundTruthTimed,
               std::vector<std::pair<double, Eigen::Matrix4d>>& vEstimateTimed,
               const std::string& strGTformat)
{
    std::ifstream gtFile(strGroundTruth);
    std::ifstream estimateFile(strEstimate);

    std::string line;
    boost::smatch match;

    if(strGTformat.compare("VISIM") == 0)
    {
	    while (std::getline(gtFile, line))
	    {
		if (line.size() == 0) {
		continue;
		}
		else if(boost::regex_match(line,match,boost::regex("^\\s*#.*$")))
		{
		continue;
		}
		else if (boost::regex_match(line,match,boost::regex("^([0-9]+),+([-0-9.e]+),+([-0-9.e]+),+([-0-9.e]+),+([-0-9.e]+),+([-0-9.e]+),+([-0-9.e]+),+([-0-9.e]+).*")))
		{
                    double time =  std::stod(match[1]); 
		    
                    float tx =  std::stof(match[2]);
		    float ty =  std::stof(match[4]);
		    float tz =  std::stof(match[3]);

		    float qx =  std::stof(match[6]);
		    float qy =  std::stof(match[7]);
		    float qz =  std::stof(match[8]);
		    float qw =  std::stof(match[5]);

		    Eigen::Matrix3d rot = quat2mat(qx, qy, qz, qw);

		    Eigen::Matrix4d pose6d;
		    pose6d.block<3,3>(0,0) = rot;
		    pose6d.block<3,1>(0,3) << tx, ty, tz;
		    pose6d.block<1,4>(3,0) << 0.0, 0.0, 0.0, 1.0;

                    std::pair<double, Eigen::Matrix4d> pose6dTimed (time, pose6d);

		    vGroundTruthTimed.push_back(pose6dTimed);
		 }
		 else
		 {
		     std::cerr << "Unknown line:" << line << std::endl;
		     return false;
		 }
	    }
	    std::cout << "Number of ground truth poses: " << vGroundTruthTimed.size() << std::endl;
    }



    if(strGTformat.compare("TUM") == 0)
    {
	    while (std::getline(gtFile, line))
	    {
		if (line.size() == 0) {
		continue;
		}
		else if(boost::regex_match(line,match,boost::regex("^\\s*#.*$")))
		{
		continue;
		}
                else if (boost::regex_match(line,match,boost::regex("^([0-9.e]+)\\s+([-0-9.e]+)\\s+([-0-9.e]+)\\s+([-0-9.e]+)\\s+([-0-9.e]+)\\s+([-0-9.e]+)\\s+([-0-9.e]+)\\s+([-0-9.e]+).*")))
		{

                    double time =  std::stod(match[1]); 

		    float tx =  std::stof(match[2]);
		    float ty =  std::stof(match[3]);
		    float tz =  std::stof(match[4]);

		    float qx =  std::stof(match[5]);
		    float qy =  std::stof(match[6]);
		    float qz =  std::stof(match[7]);
		    float qw =  std::stof(match[8]);

		    Eigen::Matrix3d rot = quat2mat(qx, qy, qz, qw);

		    Eigen::Matrix4d pose6d;
		    pose6d.block<3,3>(0,0) = rot;
		    pose6d.block<3,1>(0,3) << tx, ty, tz;
		    pose6d.block<1,4>(3,0) << 0.0, 0.0, 0.0, 1.0;

		    //vGroundTruth.push_back(pose6d);

                    std::pair<double, Eigen::Matrix4d> pose6dTimed (time, pose6d);

		    vGroundTruthTimed.push_back(pose6dTimed);

		 }
		 else
		 {
		     std::cerr << "Unknown line:" << line << std::endl;
		     return false;
		 }
	    }
	    std::cout << "Number of ground truth poses: " << vGroundTruthTimed.size() << std::endl;
    }

    while (std::getline(estimateFile, line))
    {
        if (line.size() == 0) {
        continue;
        }
        else if(boost::regex_match(line,match,boost::regex("^\\s*#.*$")))
        {
        continue;
        }
        else if (boost::regex_match(line,match,boost::regex("^([0-9.e]+)\\s+([-0-9.e]+)\\s+([-0-9.e]+)\\s+([-0-9.e]+)\\s+([-0-9.e]+)\\s+([-0-9.e]+)\\s+([-0-9.e]+)\\s+([-0-9.e]+).*")))
        {

            double time =  std::stod(match[1]); 

            float tx =  std::stof(match[2]);
            float ty =  std::stof(match[4]);
            float tz =  std::stof(match[3]);

            float qx =  std::stof(match[5]);
            float qy =  std::stof(match[6]);
            float qz =  std::stof(match[7]);
            float qw =  std::stof(match[8]);

            Eigen::Matrix3d rot = quat2mat(qx, qy, qz, qw);

            Eigen::Matrix4d pose6d;
            pose6d.block<3,3>(0,0) = rot;
            pose6d.block<3,1>(0,3) << tx, ty, tz;
            pose6d.block<1,4>(3,0) << 0.0, 0.0, 0.0, 1.0;

//            vEstimate.push_back(pose6d);

                    std::pair<double, Eigen::Matrix4d> pose6dTimed (time, pose6d);

		    vEstimateTimed.push_back(pose6dTimed);
         }
         else
         {
             std::cerr << "Unknown line:" << line << std::endl;
             return false;
         }
    }


    std::cout << "Number of estimate poses: " << vEstimateTimed.size() << std::endl;

    return true;
}
