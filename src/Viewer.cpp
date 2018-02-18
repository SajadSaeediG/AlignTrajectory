#include <Viewer.h>
#include <mutex>

using namespace std;

//Viewer::Viewer(std::vector<Eigen::Matrix4d>* pGroundTruth, std::vector<Eigen::Matrix4d>* pEstimate, std::vector<Eigen::Matrix4d>* pTransformed):
Viewer::Viewer(std::vector<std::pair<double, Eigen::Matrix4d>>* pGroundTruth,  std::vector<std::pair<double, Eigen::Matrix4d>>* pEstimate, std::vector<std::pair<double, Eigen::Matrix4d>>* pTransformed):
    mbFinishRequested(false),
    mbFinished(true),
    mbStopped(true),
    mbStopRequested(false)
{
    int iGTSize = pGroundTruth->size();
    int iESSize = pEstimate->size();
    int iTRSize = pTransformed->size(); 

    mpvGroundTruth = new std::vector<Eigen::Vector3d>;
    mpvEstimate = new std::vector<Eigen::Vector3d>;
    mpvTransformed = new std::vector<Eigen::Vector3d>;

    for(int i =0; i<iGTSize; i++)
        mpvGroundTruth->push_back((pGroundTruth->at(i)).second.block<3,1>(0,3));

    for(int i =0; i<iESSize; i++)
        mpvEstimate->push_back((pEstimate->at(i)).second.block<3,1>(0,3));

    for(int i =0; i<iTRSize; i++)
        mpvTransformed->push_back((pTransformed->at(i)).second.block<3,1>(0,3));
}

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("Trajectory",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(195));
    pangolin::Var<bool> menuGroundTruth("menu.Groun Truth",true,true);
    pangolin::Var<bool> menuEstimate("menu.Estimate",true,true);
    pangolin::Var<bool> menuTransformed("menu.Transformed Estimate",true,true);
    pangolin::Var<bool> menuPointVsLine("menu.Point vs Line",false,true);
    pangolin::Var<bool> menuReset("menu.Redo",false,false);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,500,500,512,389,0.1,1000),
                pangolin::ModelViewLookAt(-0.0, -0.7, -1.8, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

//    pangolin::OpenGlMatrix Twc;
//    Twc.SetIdentity();

    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        //s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0.0, -1.7, -1.8, 0,0,0,0.0,-1.0, 0.0));
        //s_cam.Follow(Twc);

        d_cam.Activate(s_cam);
        glClearColor(1.0f,1.0f,1.0f,1.0f);

        if(menuGroundTruth)
            DrawTrajectory(mpvGroundTruth, 0.0, 1.0, 0.0, menuPointVsLine);

        if(menuEstimate)
            DrawTrajectory(mpvEstimate, 0.0, 0.0, 1.0, menuPointVsLine);

        if(menuTransformed)
            DrawTrajectory(mpvTransformed, 1.0, 0.0, 0.0, menuPointVsLine);

        pangolin::FinishFrame();

        usleep(3000);

        if(pangolin::ShouldQuit())
            break;

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
       }

        if(CheckFinish())
           break;
    }
    SetFinish();

}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}


void Viewer::DrawTrajectory(std::vector<Eigen::Vector3d>* vpTrajectory,
                            GLfloat red, GLfloat green, GLfloat blue,
                            bool bDrawPoint)
{
    if(vpTrajectory->empty())
        return;

    glPointSize(3.0);

    if(bDrawPoint)
        glBegin(GL_POINTS);
    else
        glBegin(GL_LINE_STRIP);

    glColor3f(red, green, blue);
    for(size_t i=0, iend=vpTrajectory->size(); i<iend;i++)
    {
       Eigen::Vector3d pos = (*vpTrajectory)[i];
       glVertex3f(pos(0),pos(1),pos(2));
    }
    glEnd();
}
