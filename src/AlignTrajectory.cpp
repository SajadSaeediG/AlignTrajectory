#include<AlignTrajectory.h>

using namespace std;
using namespace Eigen;

AlignTrajectory:: AlignTrajectory()
{


}

Eigen::Matrix4d AlignTrajectory::getAlignment(std::vector<Eigen::Matrix4d> gt, std::vector<Eigen::Matrix4d> es, float& ate)
{
    std::cout << "Aligning traectories ..." << std::endl;

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
    //bool bOk             = prepareData(gt,  es, gtZeroMat, esZeroMat);
    Matrix3d rotation    = getRotation(gtZeroMat, esZeroMat);
    double   scale       = getScale(gtZeroMat, esZeroMat, rotation);
    Vector3d translation = getTranslation(gtMat, esMat, scale, rotation, ate);

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

std::vector<Eigen::Matrix4d> AlignTrajectory::getRPE(std::vector<Eigen::Matrix4d> gt, std::vector<Eigen::Matrix4d> es, int iDelta, double& rpe_rmse)
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
        pose_gt_diff = pose_gt_i.inverse() * pose_gt_j;  

        pose_es_i    = es.at(i);
        pose_es_j    = es.at(j);
        pose_es_diff = pose_es_i.inverse() * pose_es_j; 

        Eigen::Matrix4d E;
        E = pose_es_diff.inverse() * pose_gt_diff;
        rpe.push_back(E); 

        Eigen::Vector3d v;
        v = E.block<3,1>(0,3);
        double length = v.norm();
        norms = norms + length*length;
    }

    rpe_rmse = sqrt(norms/m);
    return rpe; 
}
/*
bool AlignTrajectory::prepareData(std::vector<Eigen::Vector3d> gt, std::vector<Eigen::Vector3d> es, Eigen::MatrixXd& gtZeroMat, Eigen::MatrixXd& esZeroMat)
{
//    // convert pose vectors to Eigen matrices
    int N = gt.size();
    Eigen::MatrixXd gtMat(3,N);
    for (int i = 0; i < N; i++)
    {
        gtMat(0,i) = gt.at(i)(0);
        gtMat(1,i) = gt.at(i)(1);
        gtMat(2,i) = gt.at(i)(2);
    }


    int M = gt.size();
    Eigen::MatrixXd esMat(3,N);
    for (int i = 0; i < M; i++)
    {
        esMat(0,i) = es.at(i)(0);
        esMat(1,i) = es.at(i)(1);
        esMat(2,i) = es.at(i)(2);
    }


    // calculate the mean pose to zero-shift the poses
    Eigen::Vector3d gtMean = gtMat.rowwise().mean();
    Eigen::Vector3d esMean = esMat.rowwise().mean();

    //Eigen::MatrixXd gtZeroMat(3,N);
    //Eigen::MatrixXd esZeroMat(3,N);

    gtZeroMat = gtMat.colwise() - gtMean;
    esZeroMat = esMat.colwise() - esMean;


    return true;
}
*/

Eigen::MatrixXd AlignTrajectory::getRotation(Eigen::MatrixXd model, Eigen::MatrixXd data)
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



double AlignTrajectory::getScale(Eigen::MatrixXd model, Eigen::MatrixXd data, Eigen::MatrixXd rotation)
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
    return  1/(dots/norms);
    //return 1;
}


Eigen::Vector3d AlignTrajectory::getTranslation(Eigen::MatrixXd model, Eigen::MatrixXd data, double scale, Eigen::MatrixXd rotation, float& ate)
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


