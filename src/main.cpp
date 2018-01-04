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

//#include <boost/filesystem.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/regex.hpp>

#include <Eigen/Eigen>
#include <AlignTrajectory.h>
#include <Viewer.h>
#include <pangolin/pangolin.h>

#include <signal.h>

using namespace cv;
using namespace std;

bool readFiles(const std::string& strGT,
               const std::string& strEstimate,
               std::vector<Eigen::Vector3d>& vGroundTruth,
               std::vector<Eigen::Vector3d>& vEstimate);

void sigint(int a){printf("---");}

int main(int argc, char* argv[])
{
    for(int i = 0; i < argc; ++i)
        printf("Argument %d : %s\n", i, argv[i]);

    if(argc < 3)
    {
        std::cout << "USAGE: ./align ground_truth_trajectory estime_trajectory" << std::endl;
        return 1;
    }

    std::string strGT      = "../examples/cam0_gt.visim";
    std::string strEstimate = "../examples/orbslam_traj.txt";

    std::vector<Eigen::Vector3d> vGroundTruth;
    std::vector<Eigen::Vector3d> vEstimate;
    std::vector<Eigen::Vector3d> vTransformed;

    readFiles(strGT, strEstimate, vGroundTruth, vEstimate);

    // calculte alignment
    float ate=0;
    AlignTrajectory align;
    Eigen::Matrix4d Mat = align.getAlignment(vEstimate, vGroundTruth, ate);

    std::cout << "M is : " << std::endl << Mat << std::endl;
    std::cout << "ATE is: " << std::endl << ate << std::endl;

    // generated the aligned trajectory
    Eigen::Matrix3d scaledRotation = Mat.block<3,3>(0,0);
    Eigen::Vector3d translation    = Mat.block<3,1>(0,3);
    vTransformed.clear();

    for(int i = 0 ; i < vEstimate.size(); i++)
    {
        Eigen::Vector3d p;
        p = scaledRotation*vEstimate[i] + translation;
        vTransformed.push_back(p);
    }

    // plot results
    Viewer* mpViewer;
    std::thread* mptViewer;

    mpViewer = new Viewer(&vGroundTruth, &vEstimate, &vTransformed);
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

// read ground truth and estimate trajectory files. Modify this function to match you standard.
bool readFiles(const std::string& strGT,
               const std::string& strEstimate,
               std::vector<Eigen::Vector3d>& vGroundTruth,
               std::vector<Eigen::Vector3d>& vEstimate)
{
    std::ifstream gtFile(strGT);
    std::ifstream estimateFile(strEstimate);

    std::string line;
    boost::smatch match;

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
            float tx =  std::stof(match[2]);
            float ty =  std::stof(match[4]);
            float tz =  std::stof(match[3]);

            Eigen::Vector3d pose;
            pose << tx, ty, tz;
            vGroundTruth.push_back(pose);
         }
         else
         {
             std::cerr << "Unknown line:" << line << std::endl;
             return false;
         }
    }
    std::cout << "Number of ground truth poses: " << vGroundTruth.size() << std::endl;


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
            float tx =  std::stof(match[2]);
            float ty =  std::stof(match[4]);
            float tz =  std::stof(match[3]);

            Eigen::Vector3d pose;
            pose << tx, ty, tz;
            vEstimate.push_back(pose);
         }
         else
         {
             std::cerr << "Unknown line:" << line << std::endl;
             return false;
         }
    }


    std::cout << "Number of estimate poses: " << vEstimate.size() << std::endl;

    return true;
}
