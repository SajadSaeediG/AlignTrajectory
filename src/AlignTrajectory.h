#ifndef ALIGNTRAJECTORY_H
#define ALIGNTRAJECTORY_H

#include <Eigen/Eigen>
#include <iostream>
#include <vector>
#include <mutex>
 

class AlignTrajectory
{
public:
    AlignTrajectory();
   // ~AlignTrajectory();
    Eigen::Matrix4d calculateATE(std::vector<Eigen::Matrix4d> gt, std::vector<Eigen::Matrix4d>, float& ate);
    Eigen::Matrix4d calculateATE(std::vector<std::pair<double, Eigen::Matrix4d>> gt, std::vector<std::pair<double, Eigen::Matrix4d>> es, float& ate, bool associate);

    std::vector<Eigen::Matrix4d> calculateRPE(std::vector<std::pair<double, Eigen::Matrix4d>> gt, std::vector<std::pair<double, Eigen::Matrix4d>> es, int iDelta, double& rpe_rmse, bool associate);
    std::vector<Eigen::Matrix4d> calculateRPE(std::vector<Eigen::Matrix4d> gt, std::vector<Eigen::Matrix4d> es, int iDelta, double& rpe_rmse);

private:

    bool Associate(const std::vector<std::pair<double, Eigen::Matrix4d>> & gt,
                                    const std::vector<std::pair<double, Eigen::Matrix4d>> & t,
                                    std::vector<Eigen::Matrix4d>& vGroundTruth,
                                    std::vector<Eigen::Matrix4d>& vEstimate);

    Eigen::MatrixXd ATERotation(Eigen::MatrixXd model, Eigen::MatrixXd data);

    double ATEScale(Eigen::MatrixXd model, Eigen::MatrixXd data, Eigen::MatrixXd rotation);

    Eigen::Vector3d ATETranslation(Eigen::MatrixXd model, Eigen::MatrixXd data, double scale, Eigen::MatrixXd rotation, float& error);

};

#endif
