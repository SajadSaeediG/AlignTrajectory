#include<AlignTrajectory.h>
#include <iomanip>      // std::setprecision
using namespace std;
using namespace Eigen;

AlignTrajectory:: AlignTrajectory()
{


}

Eigen::Matrix4d AlignTrajectory::calculateATE(std::vector<std::pair<double, Eigen::Matrix4d>> gt, std::vector<std::pair<double, Eigen::Matrix4d>> es, float& ate, bool associate)
{
    std::cout << "Aligning traectories ..." << std::endl;

    std::vector<Eigen::Matrix4d> vGt;
    std::vector<Eigen::Matrix4d> vEs;

    if(!associate)
    {
        if (gt.size() != es.size())
        {
            std::cout << "size of groundtruth poses: " << gt.size() << std::endl; 
            std::cout << "size of estimated poses: " << es.size() << std::endl; 
            std::cerr << "for no association, size of estimated and ground truth trajectories must be equal." << std::endl;
            return MatrixXd::Identity(4,4);
        } 
	else
        {

            for(std::vector<std::pair<double, Eigen::Matrix4d>>::iterator it = es.begin(); it != es.end(); ++it)
                vEs.push_back((*it).second);

            for(std::vector<std::pair<double, Eigen::Matrix4d>>::iterator it = gt.begin(); it != gt.end(); ++it)
                vGt.push_back((*it).second);

            return AlignTrajectory::calculateATE(vGt, vEs, ate);
        }
    }
    else
    {
        if(Associate(gt, es, vGt, vEs))
             return AlignTrajectory::calculateATE(vGt, vEs, ate);
        else
        {
            std::cerr << "asspciation failed!" << std::endl;
            return Matrix4d::Identity(4,4);
        }
    }
}


bool AlignTrajectory::Associate(const std::vector<std::pair<double, Eigen::Matrix4d>> & gt,
                                const std::vector<std::pair<double, Eigen::Matrix4d>> & es,
                                std::vector<Eigen::Matrix4d>& vGroundTruth,
                                std::vector<Eigen::Matrix4d>& vEstimate)
{
    for(uint index = 0; index < es.size(); index++)  {

        vEstimate.push_back(es[index].second);

        double time = es[index].first;
        //double time = t[index].second.S + (t[index].second.Ns)*std::pow(10,-9);
        //std::cout << t[index].second.S + (t[index].second.Ns)*std::pow(10,-9)  << std::endl;

        uint    closest_gt_Index = index;

        //int precision = std::numeric_limits<double>::max_digits10;

        double bestTimeDiff = 100000000000.0;
        for(uint index2 = index; index2 < gt.size(); index2++)
        {
            //double gt_time = gt[index2].second.S + (gt[index2].second.Ns)*std::pow(10,-9);
            double gt_time = gt[index2].first;
            double timeDiff =  gt_time - time;
            //std::cout <<  std::fixed << std::setprecision(precision) << "diff: " << fabs(timeDiff) << " " << gt_time  << " " << time << std::endl; 
            if(fabs(timeDiff) < bestTimeDiff)
            {
                closest_gt_Index = index2;
                bestTimeDiff = fabs(timeDiff);
            }
        }

        //std::cout <<  index << " " << closest_gt_Index <<  " " << bestTimeDiff << std::endl;
        //std::cout <<  es[index].first << " "  << gt[closest_gt_Index].first << std::endl;
        //std::cout <<  t[index].second.S << " " << (t[index].second.Ns) << " " << gt[closest_gt_Index].second.S << " " << (gt[closest_gt_Index].second.Ns) << std::endl;

        Eigen::Matrix4d gtPose;
        gtPose = gt[closest_gt_Index].second; //<block-size>(start-i, start-j)
        vGroundTruth.push_back(gtPose);
    }
    
    return (bool)(vGroundTruth.size() * vEstimate.size());
}


Eigen::Matrix4d AlignTrajectory::calculateATE(std::vector<Eigen::Matrix4d> gt, std::vector<Eigen::Matrix4d> es, float& ate)
{
    // convert pose vectors to Eigen matrices
    int N = gt.size();
    Eigen::MatrixXd gtMat(3,N);
    for (int i = 0; i < N; i++)
    {
        gtMat(0,i) = gt.at(i)(0,3);
        gtMat(1,i) = gt.at(i)(1,3);
        gtMat(2,i) = gt.at(i)(2,3);
    }


    int M = gt.size();
    Eigen::MatrixXd esMat(3,N);
    for (int i = 0; i < M; i++)
    {
        esMat(0,i) = es.at(i)(0,3);
        esMat(1,i) = es.at(i)(1,3);
        esMat(2,i) = es.at(i)(2,3);
    }


    // calculate the mean pose to zero-shift the poses
    Eigen::Vector3d gtMean = gtMat.rowwise().mean();
    Eigen::Vector3d esMean = esMat.rowwise().mean();

    Eigen::MatrixXd gtZeroMat(3,N);
    Eigen::MatrixXd esZeroMat(3,N);

    gtZeroMat = gtMat.colwise() - gtMean;
    esZeroMat = esMat.colwise() - esMean;

    // rotation, translation, scale, and absoulte trajector error (ate) 
    Matrix3d rotation    = ATERotation(gtZeroMat, esZeroMat);
    double   scale       = ATEScale(gtZeroMat, esZeroMat, rotation);
    Vector3d translation = ATETranslation(gtMat, esMat, scale, rotation, ate);

    cout << "Rotation is:    "<<  endl << rotation << endl<<endl;
    cout << "Scale is:       "<<  endl << scale << endl<<endl; 
    cout << "Translation is: "<<  endl << translation << endl<<endl; 
    cout << "Error is:       "<<  endl << ate << endl<<endl; 

    Matrix4d Mat;
    Mat = MatrixXd::Identity(4,4);

    Mat.block<3,3>(0,0) = scale*rotation;
    Mat.block<3,1>(0,3) = translation;
    Mat.block<1,4>(3,0) << 0,0,0,1;

    return Mat;
}


std::vector<Eigen::Matrix4d> AlignTrajectory::calculateRPE(std::vector<std::pair<double, Eigen::Matrix4d>> gt, std::vector<std::pair<double, Eigen::Matrix4d>> es, int iDelta, double& rpe_rmse, bool associate)
{
    std::vector<Eigen::Matrix4d> vGt;
    std::vector<Eigen::Matrix4d> vEs;

    if(!associate)
    {
        if (gt.size() != es.size())
        {
            std::cout << "size of groundtruth poses: " << gt.size() << std::endl; 
            std::cout << "size of estimated poses: " << es.size() << std::endl; 
            std::cerr << "for no association, size of estimated and ground truth trajectories must be equal." << std::endl;
            return {};
        } 
	else
        {
            for(std::vector<std::pair<double, Eigen::Matrix4d>>::iterator it = es.begin(); it != es.end(); ++it)
                vEs.push_back((*it).second);

            for(std::vector<std::pair<double, Eigen::Matrix4d>>::iterator it = gt.begin(); it != gt.end(); ++it)
                vGt.push_back((*it).second);

            return AlignTrajectory::calculateRPE(vGt, vEs, iDelta, rpe_rmse);
        }
    }
    else
    {
        if(Associate(gt, es, vGt, vEs))
             return AlignTrajectory::calculateRPE(vGt, vEs, iDelta, rpe_rmse);
        else
        {
            std::cerr << "asspciation failed!" << std::endl;
            return {};
        }
    }

}
std::vector<Eigen::Matrix4d> AlignTrajectory::calculateRPE(std::vector<Eigen::Matrix4d> gt, std::vector<Eigen::Matrix4d> es, int iDelta, double& rpe_rmse)
{
    std::vector<Eigen::Matrix4d>  rpe; 
    rpe.clear();

    Eigen::Matrix4d pose_gt_i;
    Eigen::Matrix4d pose_gt_j;
    Eigen::Matrix4d pose_gt_diff;

    Eigen::Matrix4d pose_es_i;
    Eigen::Matrix4d pose_es_j;
    Eigen::Matrix4d pose_es_diff;

    double norms = 0.0;
    int n = gt.size();
    int m = n-iDelta;


    for (int i = 0; i < n-iDelta; i++)
    {
        int j = i + iDelta;
        pose_gt_i    = gt.at(i);
        pose_gt_j    = gt.at(j);
        pose_gt_diff = pose_gt_i * pose_gt_j.inverse();  

        pose_es_i    = es.at(i);
        pose_es_j    = es.at(j);
        pose_es_diff = pose_es_i * pose_es_j.inverse(); 

        Eigen::Matrix4d E;
        E = pose_es_diff * pose_gt_diff.inverse();
        rpe.push_back(E); 

        Eigen::Vector3d v;
        v = E.block<3,1>(0,3);
        double length = v.norm();
        norms = norms + length*length;
    }

    rpe_rmse = sqrt(norms/m);
    return rpe; 
}


Eigen::MatrixXd AlignTrajectory::ATERotation(Eigen::MatrixXd model, Eigen::MatrixXd data)
{
    Eigen::MatrixXd w;
    w = Eigen::MatrixXd::Identity(3,3);

    int cols = model.cols();

    //Model = [M0, M1, ...], Data = [D0, D1, ...]
    // W = M0*D0' + M1*D1' + ...  = \sum{Mi*Di}
    for (int i = 0; i < cols; i++)
        w = w + model.col(i) * data.col(i).transpose();

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(w.transpose(), Eigen::ComputeThinU | Eigen::ComputeThinV);

    Matrix3d U = svd.matrixU();
    Matrix3d V = svd.matrixV();
    float detV = V.determinant();
    float detU = U.determinant();

    MatrixXd S;
    S = MatrixXd::Identity(3,3);

    if(detU * detV < 0)
        S(2,2) = -1;

    MatrixXd rot;
    rot = U * S * V.transpose();

    return rot;
}



double AlignTrajectory::ATEScale(Eigen::MatrixXd model, Eigen::MatrixXd data, Eigen::MatrixXd rotation)
{
    int cols = model.cols();
    MatrixXd rotatedModel;
    rotatedModel = rotation * model;

    double dots = 0.0;
    double norms = 0.0;
    double normi = 0.0;

    //Model = [M0, M1, ...], Rotated Data = [R0, R1, ...]
    // W = M0.D0' + M1.D1' + ...  = \sum{Mi.Di}
    for (int i = 0; i < cols; i++)
    {
       Vector3d v1 = data.col(i);
       Vector3d v2 = rotatedModel.col(i);
       Vector3d v3 = model.col(i);

       dots = dots + v1.transpose()*v2;
       normi = v3.norm();
       norms = norms + normi*normi;
    }

    // scale
//    return  1/(dots/norms);
    return  (dots/norms);
    //return 1;
}


Eigen::Vector3d AlignTrajectory::ATETranslation(Eigen::MatrixXd model, Eigen::MatrixXd data, double scale, Eigen::MatrixXd rotation, float& ate)
{
    int N = model.cols();
    Vector3d translation = data.rowwise().mean() - (scale*rotation)*(model.rowwise().mean());
    MatrixXd rotatedModel(3,N);
    rotatedModel = (scale*rotation)*model;

    // error matrix E = [E1, E2, ...]
    MatrixXd errorMat(3, N);
    errorMat = (rotatedModel.colwise() + translation) - data;

    // Absoute Trajectory Error (ATE) = |||E1|| + ||E2||+ ... = \sum(||Ei||)
    for (int i = 0; i < N; i++)
        ate = ate + errorMat.col(i).norm();

    ate = ate/N;
    return   translation;

}


