#include "gimbal_control.h"
#include <opencv2/core/eigen.hpp>


using namespace cv;
using namespace Eigen;

namespace robot_detection{

AngleSolve::AngleSolve()
    {
        cv::FileStorage fs("/home/lmx2/vision_ws_2/src/robot_detection/vision_data/control_data.yaml", cv::FileStorage::READ);

        fs["big_w"] >> big_w;
        fs["big_h"] >> big_h;
        fs["small_w"] >> small_w;
        fs["small_h"] >> small_h;
        fs["buff_w"] >> buff_w;
        fs["buff_h"] >> buff_h;


        fs["F_MAT"] >> F_MAT;
        fs["C_MAT"] >> C_MAT;
        cv::cv2eigen(F_MAT,F_EGN);
        cv::cv2eigen(C_MAT,C_EGN);

        cv::Mat temp;
        fs["RotationMatrix_cam2imu"] >> temp;
        cv::cv2eigen(temp,RotationMatrix_cam2imu);

        fs.release();
    }

// zyx
Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d &theta)
{
    Eigen::Matrix3d R_x;    // 计算旋转矩阵的X分量
    R_x <<
        1,              0,               0,
            0,  cos(theta[0]),  -sin(theta[0]),
            0,  sin(theta[0]),   cos(theta[0]);

    Eigen::Matrix3d R_y;    // 计算旋转矩阵的Y分量
    R_y <<
        cos(theta[1]),   0, sin(theta[1]),
            0,   1,             0,
            -sin(theta[1]),  0, cos(theta[1]);

    Eigen::Matrix3d R_z;    // 计算旋转矩阵的Z分量
    R_z <<
        cos(theta[2]), -sin(theta[2]), 0,
            sin(theta[2]),  cos(theta[2]), 0,
            0,              0,             1;
    Eigen::Matrix3d R = R_z * R_y * R_x;
    return R;
}

// xyz,固定相机和IMU两个坐标系的转换
Eigen::Matrix3d eulerAnglesToRotationMatrix2(Eigen::Vector3d &theta)
{
    Eigen::Matrix3d R_x;    // 计算旋转矩阵的X分量
    R_x <<
        1,              0,               0,
            0,  cos(theta[0]),  -sin(theta[0]),
            0,  sin(theta[0]),   cos(theta[0]);

    Eigen::Matrix3d R_y;    // 计算旋转矩阵的Y分量
    R_y <<
        cos(theta[1]),   0, sin(theta[1]),
            0,   1,             0,
            -sin(theta[1]),  0, cos(theta[1]);

    Eigen::Matrix3d R_z;    // 计算旋转矩阵的Z分量
    R_z <<
        cos(theta[2]), -sin(theta[2]), 0,
            sin(theta[2]),  cos(theta[2]), 0,
            0,              0,             1;
    Eigen::Matrix3d R = R_x * R_y * R_z;
    return R;
}

    void AngleSolve::init(float r, float p, float y, float speed)
    {
        ab_roll = r;
        ab_pitch = p;
        ab_yaw = y;
        bullet_speed = speed;

        Eigen::Vector3d theta = {r,p,y};
        RotationMatrix_imu = eulerAnglesToRotationMatrix(theta);
    }

    Eigen::Vector3d AngleSolve::cam2imu(Vector3d cam_pos)
    {
        // cam 2 imu(init),through test get, xyz-rpy
        Vector3d pos_tmp;
        pos_tmp = RotationMatrix_cam2imu * cam_pos;

//        pos_tmp = {cam_pos[2],cam_pos[0],cam_pos[1]};
//        std::cout<<"tmp_pos: "<<pos_tmp<<std::endl;

        Vector3d imu_pos;
        imu_pos = RotationMatrix_imu * pos_tmp;

//    std::cout<<"imu_pos: "<<imu_pos<<std::endl;
        return imu_pos;
    }

    Eigen::Vector3d AngleSolve::imu2cam(Vector3d imu_pos)
    {
        Vector3d tmp_pos;
        tmp_pos = RotationMatrix_imu.inverse() * imu_pos;

        Vector3d cam_pos;
        cam_pos = RotationMatrix_cam2imu.inverse() * tmp_pos;
        return cam_pos;
    }

    cv::Point2f AngleSolve::cam2pixel(Eigen::Vector3d cam_pos)
    {
        Vector3d tmp_pixel;
        tmp_pixel = F_EGN * cam_pos;
//         std::cout<<"tmp_pixel: "<<tmp_pixel<<std::endl;
        cv::Point2f pixel_pos = Point2f((float)tmp_pixel[0]/tmp_pixel[2],(float)tmp_pixel[1]/tmp_pixel[2]);

        return pixel_pos;
    }

    cv::Point2f AngleSolve::imu2pixel(Vector3d imu_pos)
    {
        Vector3d tmp_pos;
        tmp_pos = RotationMatrix_imu.inverse() * imu_pos;

        Vector3d cam_pos;
        cam_pos = RotationMatrix_cam2imu.inverse() * tmp_pos;

        // std::cout<<"cam_pos_in_fuc_imu2pixel: "<<cam_pos<<std::endl;

        Vector3d tmp_pixel;
        tmp_pixel = F_EGN * cam_pos;
        // std::cout<<"tmp_pixel: "<<tmp_pixel<<std::endl;
        cv::Point2f pixel_pos = Point2f((float)tmp_pixel[0]/tmp_pixel[2],(float)tmp_pixel[1]/tmp_pixel[2]);

        return pixel_pos;
    }

    Eigen::Vector3d AngleSolve::pixel2imu(Armor armor, int method)
    {
        armor.camera_position = pnpSolve(armor.armor_pt4,armor.type,method);
        Eigen::Vector3d imu_pos = cam2imu(armor.camera_position);
        return imu_pos;
    }

    Eigen::Vector3d AngleSolve::pixel2cam(Armor armor, int method)
    {
        armor.camera_position = pnpSolve(armor.armor_pt4,armor.type,method);
        return armor.camera_position;
    }

    Eigen::Vector3d AngleSolve::transformPos2_World(Vector3d &Pos)
    {
        //for debug
        ab_pitch=ab_yaw=ab_roll=0;

        Mat camera2_tuoluo = (Mat_<double>(3,1) << 0,0,0);
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
        float y = -(float)Pos(2,0);
        float x = (float)sqrt(Pos(0,0)*Pos(0,0)+Pos(1,0)*Pos(1,0));
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
            //printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,y_temp,dy);
        }

        return Vector3d(Pos(0,0),Pos(1,0),-y_temp);
    }

    Eigen::Vector3d AngleSolve::pnpSolve(Point2f *p, int type, int method = SOLVEPNP_IPPE)
    {
        double w = type == SMALL ? small_w : big_w;
        double h = type == SMALL ? small_h : big_h;
        cv::Point2f lu, ld, ru, rd;
        std::vector<cv::Point3d> ps = {
                {-w / 2 , -h / 2, 0.},
                { w / 2 , -h / 2, 0.},
                { w / 2 ,  h / 2, 0.},
                {-w / 2 ,  h / 2, 0.}
        };

        std::vector<cv::Point2f> pu;
        pu.clear();
        pu.push_back(p[3]);
        pu.push_back(p[2]);
        pu.push_back(p[1]);
        pu.push_back(p[0]);

        cv::Mat rvec;
        cv::Mat tvec;
        Eigen::Vector3d tv;

        cv::solvePnP(ps, pu, F_MAT, C_MAT, rvec, tvec/*, SOLVEPNP_IPPE*/);
        cv::cv2eigen(tvec, tv);

//    Mat R;
//    Rodrigues(rvec,R);
//
//    // offset++
        std::cout<<"distance:   "<<tv.norm()<<std::endl;

#ifdef SHOW_MEASURE_RRECT
        Mat pnp_check = _src.clone();
        Mat rv_mat;
        Eigen::Matrix<double,3,3> rv;
        cv::Rodrigues(rvec,rv_mat);
        cv::cv2eigen(rv_mat,rv);
        std::cout<<"rv"<<rv<<std::endl;

        Eigen::Vector3d imuPoint = {-w / 2 , -h / 2, 0.};
        Eigen::Vector3d armorPoint = rv*imuPoint + tv;//in camera coordinate
        cv::Point2f m_lu,m_ld,m_ru,m_rd;
        m_lu = cam2pixel(armorPoint);

        imuPoint = {-w / 2 , h / 2, 0.};
        armorPoint = rv*imuPoint + tv;
        m_ld = cam2pixel(armorPoint);

        imuPoint = {w / 2 , -h / 2, 0.};
        armorPoint = rv*imuPoint + tv;
        m_ru = cam2pixel(armorPoint);

        imuPoint = {w / 2 , h / 2, 0.};
        armorPoint = rv*imuPoint + tv;
        m_rd = cam2pixel(armorPoint);

        circle(pnp_check,m_lu,3,Scalar(0,255,0),-1);
        circle(pnp_check,m_ld,3,Scalar(255,255,0),-1);
        circle(pnp_check,m_ru,3,Scalar(0,0,255),-1);
        circle(pnp_check,m_rd,3,Scalar(0,255,255),-1);
        line(pnp_check,m_lu,m_ld,Scalar(0,0,0),2);
        line(pnp_check,m_ld,m_rd,Scalar(255,0,0),2);
        line(pnp_check,m_rd,m_ru,Scalar(255,0,255),2);
        line(pnp_check,m_ru,m_lu,Scalar(255,255,0),2);
        std::cout<<"m_lu:"<<m_lu<<std::endl;
        std::cout<<"m_ld:"<<m_ld<<std::endl;
        std::cout<<"m_ru:"<<m_ru<<std::endl;
        std::cout<<"m_rd:"<<m_rd<<std::endl;
        std::cout<<"tvec:"<<cam2pixel(tv)<<std::endl;

imshow("pnp_check",pnp_check);
#endif

        return tv;
    }

    float AngleSolve::BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
        float y;
        fly_time = (float)((exp(SMALL_AIR_K * x) - 1) / (SMALL_AIR_K * v * cos(angle)));
        y = (float)(v * sin(angle) * fly_time - GRAVITY * fly_time * fly_time / 2);
        //printf("fly_time:%f\n",fly_time);
        return y;
    }

    Eigen::Vector3d AngleSolve::yawPitchSolve(Vector3d &Pos)
    {
        Eigen::Vector3d rpy;
        rpy[2] = atan2(Pos(1,0) ,
                       Pos(0,0)) / CV_PI*180.0;
        rpy[1] = -atan2(Pos(2,0) ,
                       Pos(0,0)) / CV_PI*180.0;
        rpy[0] = ab_roll;
        return rpy;
    }

    double AngleSolve::getFlyTime()
    {
        return fly_time * 1000;
    }

    double AngleSolve::getFlyTime(Eigen::Vector3d &pos)
    {
        return pos.norm() / bullet_speed;
    }

    void AngleSolve::getAngle1(Armor &aimArmor)
    {

        ////sample////
        Vector3d po;
        po << 1,0,0;
        /////////////
        Vector3d aimPosition,worldPosition,world_dropPosition,camera_dropPosition;
        aimPosition = pnpSolve(aimArmor.armor_pt4, aimArmor.type, SOLVEPNP_IPPE);//use PnP to get aim position

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



    void AngleSolve::getAngle(Eigen::Vector3d predicted_position)
    {
        Vector3d world_dropPosition;

        world_dropPosition = airResistanceSolve(predicted_position);//calculate gravity and air resistance

        yawPitchSolve(world_dropPosition);//get need yaw and pitch

    }
}