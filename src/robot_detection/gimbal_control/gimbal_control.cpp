#include "gimbal_control.h"
#include <opencv2/core/eigen.hpp>


using namespace cv;
using namespace Eigen;

AngleSolve::AngleSolve()
{
    F_MAT=(Mat_<float>(3, 3) <<   1579.60532, 0.000000000000, 627.56545,
                                            0.000000000000, 1579.86788, 508.65311,
                                            0.000000000000, 0.000000000000, 1.000000000000);
    C_MAT=(Mat_<float>(1, 5) << -0.09082  , 0.22923  , -0.00020  , 0.00013 , 0.00000);

    big_w = 0;
    big_h = 0;
    small_w = 0.135;
    small_h = 0.057;

}

Eigen::Vector3d AngleSolve::transformPos2_World(Vector3d &Pos)
{
    //for debug
    ab_pitch=ab_yaw=ab_roll=0;

    Mat camera2_tuoluo = (Mat_<double>(3,1) << CV_PI/2,0,0);
    Mat eular = (Mat_<double>(3,1) << -ab_pitch/180*CV_PI, -ab_yaw/180*CV_PI, -ab_roll/180*CV_PI);
    Mat rotated_mat,coordinate_mat;
    Rodrigues(camera2_tuoluo,coordinate_mat);
    Rodrigues(eular,rotated_mat);

    cv2eigen(rotated_mat,rotated_matrix);
    cv2eigen(coordinate_mat,coordinate_matrix);





    return coordinate_matrix*(rotated_matrix*Pos);
}

Eigen::Vector3d AngleSolve::transformPos2_Camera(Eigen::Vector3d &Pos)
{
    return rotated_matrix.inverse()*(coordinate_matrix.inverse()*Pos);
}

Eigen::Vector3d AngleSolve::gravitySolve(Vector3d &Pos)
{
    //at world coordinate system
    double height;

    double del_ta = pow(bullet_speed, 4) + 2 * 9.8 * Pos(1, 0) * bullet_speed * bullet_speed - 9.8 * 9.8 * Pos(2, 0) * Pos(2, 0);
    double t_2 = (9.8 * Pos(1, 0) + bullet_speed * bullet_speed - sqrt(del_ta)) / (0.5 * 9.8 * 9.8);
    height = 0.5 * 9.8 * t_2;

    return Vector3d(Pos(0,0),Pos(1,0), Pos(2,0) + height);

}

Eigen::Vector3d AngleSolve::airResistanceSolve(Vector3d &Pos)
{
    //at world coordinate system
    auto y = (float)Pos(1,0);
    auto x = (float)sqrt(Pos(0,0)*Pos(0,0)+Pos(2,0)*Pos(2,0));
    float y_temp, y_actual, dy;
    float a;
    y_temp = y;
    // by iteration
    for (int i = 0; i < 20; i++)
    {
        a = (float)atan2(y_temp, x);
        y_actual = BulletModel(x, bullet_speed, a);
        dy = y - y_actual;
        y_temp = y_temp + dy;
        if (fabsf(dy) < 0.001) {
            break;
        }
        printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,y_temp,dy);
    }

    return Vector3d(Pos(0,0),y_temp,Pos(2,0));
}

Eigen::Vector3d AngleSolve::pnpSolve(Point2f *p, int type, int method = SOLVEPNP_IPPE)
{
    double w = type == SMALL ? small_w : big_w;
    double h = type == SMALL ? small_h : big_h;
    cv::Point2f lu, ld, ru, rd;
    std::vector<cv::Point3d> ps = {
            {-w / 2 , -h / 2, 0.},
            {w / 2 , -h / 2, 0.},
            {w / 2 , h / 2, 0.},
            {-w / 2 , h / 2, 0.}
    };


    vector<cv::Point2f> pu;
    pu.push_back(p[3]);
    pu.push_back(p[2]);
    pu.push_back(p[1]);
    pu.push_back(p[0]);

    cv::Mat rvec;
    cv::Mat tvec;
    Eigen::Vector3d tv;

    cv::solvePnP(ps, pu, F_MAT, C_MAT, rvec, tvec,false,method);
    cv::cv2eigen(tvec, tv);



    // offset++



    return tv;
}

float AngleSolve::BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
    float y;
    fly_time = (float)((exp(SMALL_AIR_K * x) - 1) / (SMALL_AIR_K * v * cos(angle)));
    y = (float)(v * sin(angle) * fly_time - GRAVITY * fly_time * fly_time / 2);
    printf("fly_time:%f\n",fly_time);
    return y;
}

void AngleSolve::yawPitchSolve(Vector3d &Pos)
{
    send.yaw = atan2(Pos(0,0) ,
                     -1 * Pos(1,0)) / CV_PI*180.0 - ab_yaw;
    send.pitch = atan2(Pos(2,0) ,
                       sqrt(Pos(0,0)*Pos(0,0) + Pos(1,0)*Pos(1,0))) / CV_PI*180.0 - ab_pitch;
}

double AngleSolve::getFlyTime()
{
    return fly_time * 1000;
}

void AngleSolve::getAngle(Armor &aimArmor)
{

    ////sample////
    Vector3d po;
    po << 1,0,0;
    /////////////
    Vector3d aimPosition,worldPosition,world_dropPosition,camera_dropPosition;
    aimPosition = pnpSolve(aimArmor.armor_pt4, aimArmor.type);//use PnP to get aim position

    worldPosition = transformPos2_World(aimPosition);//transform aim position to world coordinate system

    //world_dropPosition = gravitySolve(worldPosition);//calculate gravity

    world_dropPosition = airResistanceSolve(worldPosition);//calculate gravity and air resistance

    camera_dropPosition = transformPos2_Camera(world_dropPosition);//transform position to camera coordinate system to get angle

    yawPitchSolve(camera_dropPosition);//get need yaw and pitch

    ////output result/////
    std::cout<<worldPosition[0]<<std::endl;
    std::cout<<worldPosition[1]<<std::endl;
    std::cout<<worldPosition[2]<<std::endl;
    /////////////////////

}

//
void AngleSolve::getAngle(Eigen::Vector3d predicted_position)
{
    Vector3d world_dropPosition,camera_dropPosition;



    world_dropPosition = airResistanceSolve(predicted_position);//calculate gravity and air resistance

    camera_dropPosition = transformPos2_Camera(world_dropPosition);//transform position to camera coordinate system to get angle

    yawPitchSolve(camera_dropPosition);//get need yaw and pitch

    ////output result/////
    std::cout<<predicted_position[0]<<std::endl;
    std::cout<<predicted_position[1]<<std::endl;
    std::cout<<predicted_position[2]<<std::endl;
    /////////////////////

}