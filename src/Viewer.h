#ifndef VIEWER_H
#define VIEWER_H

#include <mutex>
#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include <pangolin/pangolin.h>


class Viewer
{
public:
    Viewer(std::vector<Eigen::Matrix4d>* pGroundTruth, std::vector<Eigen::Matrix4d>* pEstimate, std::vector<Eigen::Matrix4d>* pTransformed);

    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    void Release();

//    void DrawTrajectory(std::vector<Eigen::Vector3d> gt);


    void DrawTrajectory(std::vector<Eigen::Vector3d>* vpTrajectory, GLfloat red, GLfloat green, GLfloat blue, bool bDrawPoint);
    std::vector<Eigen::Vector3d>* mpvGroundTruth;
    std::vector<Eigen::Vector3d>* mpvEstimate;
    std::vector<Eigen::Vector3d>* mpvTransformed;


private:
    bool Stop();


    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

};

#endif //VIEWER_H

