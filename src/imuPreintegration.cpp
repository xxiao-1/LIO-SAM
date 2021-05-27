#include "utility.h"
#include "../include/dynamic_factor.h"
#include "../include/ChaFactor.h"
#include "../include/ChaBias.h"
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include "lio_sam/chassis_data.h"
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

using namespace gtsam;
using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using gtsam::symbol_shorthand::K; // steerAngle ratio  (k)

class TransformFusion : public ParamServer {
public:
    std::mutex mtx;

    ros::Subscriber subImuOdometry;
    ros::Subscriber subLaserOdometry;

    ros::Publisher pubImuOdometry;
    ros::Publisher pubImuPath;

    Eigen::Affine3f lidarOdomAffine;
    Eigen::Affine3f imuOdomAffineFront;
    Eigen::Affine3f imuOdomAffineBack;

    tf::TransformListener tfListener;
    tf::StampedTransform lidar2Baselink;

    double lidarOdomTime = -1;


    deque <nav_msgs::Odometry> imuOdomQueue;

    TransformFusion() {
        if (lidarFrame != baselinkFrame) {
            try {
                tfListener.waitForTransform(lidarFrame, baselinkFrame, ros::Time(0), ros::Duration(3.0));
                tfListener.lookupTransform(lidarFrame, baselinkFrame, ros::Time(0), lidar2Baselink);
            }
            catch (tf::TransformException ex) {
                ROS_ERROR("%s", ex.what());
            }
        }

        // 接收mapOptmization发布的odometry, 值：transformToBeMapped，低频率，回环修正后结果
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry", 5,
                                                            &TransformFusion::lidarOdometryHandler, this,
                                                            ros::TransportHints().tcpNoDelay());
        // 接收imu+chassis+lidar优化后的结果
        subImuOdometry = nh.subscribe<nav_msgs::Odometry>(odomTopic + "_incremental", 2000,
                                                          &TransformFusion::imuOdometryHandler, this,
                                                          ros::TransportHints().tcpNoDelay());

        // 将接收到的两个消息融合输出laserOdometry
        pubImuOdometry = nh.advertise<nav_msgs::Odometry>(odomTopic, 2000);
        // 同为laserOdometry，展示上次优化到现在的轨迹变化
        pubImuPath = nh.advertise<nav_msgs::Path>("lio_sam/imu/path", 1);
    }

    Eigen::Affine3f odom2affine(nav_msgs::Odometry odom) {
        double x, y, z, roll, pitch, yaw;
        x = odom.pose.pose.position.x;
        y = odom.pose.pose.position.y;
        z = odom.pose.pose.position.z;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(odom.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        return pcl::getTransformation(x, y, z, roll, pitch, yaw);
    }

    void lidarOdometryHandler(const nav_msgs::Odometry::ConstPtr &odomMsg) {
        std::lock_guard <std::mutex> lock(mtx);

        lidarOdomAffine = odom2affine(*odomMsg);

        lidarOdomTime = odomMsg->header.stamp.toSec();
    }

    void imuOdometryHandler(const nav_msgs::Odometry::ConstPtr &odomMsg) {
        // static tf
        static tf::TransformBroadcaster tfMap2Odom;
        static tf::Transform map_to_odom = tf::Transform(tf::createQuaternionFromRPY(0, 0, 0), tf::Vector3(0, 0, 0));
        tfMap2Odom.sendTransform(tf::StampedTransform(map_to_odom, odomMsg->header.stamp, mapFrame, odometryFrame));

        std::lock_guard <std::mutex> lock(mtx);

        imuOdomQueue.push_back(*odomMsg);

        // get latest odometry (at current IMU stamp)
        if (lidarOdomTime == -1)
            return;
        while (!imuOdomQueue.empty()) {
            if (imuOdomQueue.front().header.stamp.toSec() <= lidarOdomTime)
                imuOdomQueue.pop_front();
            else
                break;
        }
        Eigen::Affine3f imuOdomAffineFront = odom2affine(imuOdomQueue.front());
        Eigen::Affine3f imuOdomAffineBack = odom2affine(imuOdomQueue.back());
        Eigen::Affine3f imuOdomAffineIncre = imuOdomAffineFront.inverse() * imuOdomAffineBack;
        Eigen::Affine3f imuOdomAffineLast = lidarOdomAffine * imuOdomAffineIncre;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(imuOdomAffineLast, x, y, z, roll, pitch, yaw);

        // publish latest odometry
        nav_msgs::Odometry laserOdometry = imuOdomQueue.back();
        laserOdometry.pose.pose.position.x = x;
        laserOdometry.pose.pose.position.y = y;
        laserOdometry.pose.pose.position.z = z;
        laserOdometry.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
        pubImuOdometry.publish(laserOdometry);

        // publish tf
        static tf::TransformBroadcaster tfOdom2BaseLink;
        tf::Transform tCur;
        tf::poseMsgToTF(laserOdometry.pose.pose, tCur);
        if (lidarFrame != baselinkFrame)
            tCur = tCur * lidar2Baselink;
        tf::StampedTransform odom_2_baselink = tf::StampedTransform(tCur, odomMsg->header.stamp, odometryFrame,
                                                                    baselinkFrame);
        tfOdom2BaseLink.sendTransform(odom_2_baselink);

        // publish IMU path
        static nav_msgs::Path imuPath;
        static double last_path_time = -1;
        double imuTime = imuOdomQueue.back().header.stamp.toSec();
        if (imuTime - last_path_time > 0.1) {
            last_path_time = imuTime;
            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.stamp = imuOdomQueue.back().header.stamp;
            pose_stamped.header.frame_id = odometryFrame;
            pose_stamped.pose = laserOdometry.pose.pose;
            imuPath.poses.push_back(pose_stamped);
            while (!imuPath.poses.empty() && imuPath.poses.front().header.stamp.toSec() < lidarOdomTime - 1.0)
                imuPath.poses.erase(imuPath.poses.begin());
            if (pubImuPath.getNumSubscribers() != 0) {
                imuPath.header.stamp = imuOdomQueue.back().header.stamp;
                imuPath.header.frame_id = odometryFrame;
                pubImuPath.publish(imuPath);
            }
        }
    }
};

class IMUPreintegration : public ParamServer {
public:

    std::mutex mtx;
    struct DynamicMeasurement {
        double time;
        gtsam::Vector3 velocity;
        gtsam::Vector3 angle;
    };
    ros::Subscriber subImu;
    ros::Subscriber subOdometry;
    ros::Publisher pubImuOdometry;

    bool systemInitialized = false;

    gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
    gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
    gtsam::noiseModel::Diagonal::shared_ptr correctionNoise3;
    gtsam::Vector noiseModelBetweenBias;

    gtsam::PreintegratedImuMeasurements *imuIntegratorOpt_;
    gtsam::PreintegratedImuMeasurements *imuIntegratorImu_;

    std::deque <sensor_msgs::Imu> imuQueOpt;
    std::deque <sensor_msgs::Imu> imuQueImu;

    gtsam::Pose3 prevPose_;
    gtsam::Vector3 prevVel_;
    gtsam::NavState prevState_;
    gtsam::imuBias::ConstantBias prevBias_;


    gtsam::NavState prevStateOdom;
    gtsam::imuBias::ConstantBias prevBiasOdom;

    ros::Subscriber subChassis;
    ros::Publisher pubChaOdometry;
    gtsam::noiseModel::Diagonal::shared_ptr chassisVelNoise;
    std::deque <DynamicMeasurement> chaQueOpt;
    std::deque <DynamicMeasurement> chaQueCha;
    gtsam::PreintegratedChaMeasurements *chaIntegratorOpt_;
    gtsam::PreintegratedChaMeasurements *chaIntegratorCha_;
    gtsam::ChaNavState prevStateCha_;
    gtsam::chaBias::ConstantBias prevBiasCha_;
    gtsam::ChaNavState prevStateCha;
    gtsam::chaBias::ConstantBias prevBiasCha;
    double lastChaT_cha = -1;
    double lastChaT_opt = -1;


    int steerAngleRatio = 17.4;
    bool doneFirstOpt = false;
    double lastImuT_imu = -1;
    double lastImuT_opt = -1;
    gtsam::ISAM2 optimizer;
    gtsam::NonlinearFactorGraph graphFactors;
    gtsam::Values graphValues;

    const double delta_t = 0;
    double tmp_imu_ang_x=0,tmp_imu_ang_y=0,tmp_imu_ang_z=0;
    double tmp_imu_ang_time=0;
    double vel = 0, vx = 0, vy = 0, vz = 0;
    int key = 1;
    ofstream myfileImu;
    ofstream myfileCha;

    gtsam::Pose3 imu2Lidar = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0),
                                          gtsam::Point3(-extTrans.x(), -extTrans.y(), -extTrans.z()));
    gtsam::Pose3 lidar2Imu = gtsam::Pose3(gtsam::Rot3(1, 0, 0, 0),
                                          gtsam::Point3(extTrans.x(), extTrans.y(), extTrans.z()));

    IMUPreintegration() {
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &IMUPreintegration::imuHandler, this,
                                                ros::TransportHints().tcpNoDelay());
        subOdometry = nh.subscribe<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 5,
                                                       &IMUPreintegration::odometryHandler, this,
                                                       ros::TransportHints().tcpNoDelay());
        subChassis = nh.subscribe<lio_sam::chassis_data>("/chassis_msgs", 2000, &IMUPreintegration::chassisHandler,
                                                         this, ros::TransportHints().tcpNoDelay());

        //两个去处：
        // 1、作为lidar去畸变，同时作为匹配的初值intial guess
        //2 被接收用来发布imu_path 和 odometry/imu
        // 是imu预测的值，根据imu lidar chassis图优化结果进行了调整
        pubImuOdometry = nh.advertise<nav_msgs::Odometry>(odomTopic + "_incremental", 2000);
        pubChaOdometry = nh.advertise<nav_msgs::Odometry>("chassis_incremental", 2000);

        boost::shared_ptr <gtsam::PreintegrationParams> p = gtsam::PreintegrationParams::MakeSharedU(imuGravity);
        p->accelerometerCovariance =
                gtsam::Matrix33::Identity(3, 3) * pow(imuAccNoise, 2); // acc white noise in continuous
        p->gyroscopeCovariance =
                gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        p->integrationCovariance = gtsam::Matrix33::Identity(3, 3) *
                                   pow(1e-4, 2); // error committed in integrating position from velocities
        gtsam::imuBias::ConstantBias prior_imu_bias(
                (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());; // assume zero initial bias

        priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2).finished()); // rad,rad,rad,m, m, m
        priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 1e4); // m/s
        chassisVelNoise = gtsam::noiseModel::Isotropic::Sigma(0.5, 1e4); // m/s
        priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3); // 1e-2 ~ 1e-3 seems to be good
        correctionNoise = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
        correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
        correctionNoise3 = gtsam::noiseModel::Diagonal::Sigmas(
                (gtsam::Vector(6) << 1, 1, 1, 100, 100, 100).finished()); // rad,rad,rad,m, m, m
        noiseModelBetweenBias = (gtsam::Vector(6)
                << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();

        imuIntegratorImu_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for IMU message thread
        imuIntegratorOpt_ = new gtsam::PreintegratedImuMeasurements(p, prior_imu_bias); // setting up the IMU integration for optimization

        // use chassis
        boost::shared_ptr <gtsam::ChaPreintegrationParams> p_cha = gtsam::ChaPreintegrationParams::MakeSharedU();
        p_cha->wheelspeedsensorCovariance = gtsam::Matrix33::Identity(3, 3) * pow(0.002, 2); // acc white noise in continuous
        p_cha->gyroscopeCovariance = gtsam::Matrix33::Identity(3, 3) * pow(imuGyrNoise, 2); // gyro white noise in continuous
        gtsam::chaBias::ConstantBias prior_cha_bias(
                (gtsam::Vector(6) << 0, 0, 0, 0, 0, 0).finished());// assume zero initial bias
        chaIntegratorCha_ = new gtsam::PreintegratedChaMeasurements(p_cha, prior_cha_bias); // setting up the IMU integration for IMU message thread
        chaIntegratorOpt_ = new gtsam::PreintegratedChaMeasurements(p_cha, prior_cha_bias); // setting up the IMU integration for optimization
        // std::cout<<"1------------------------init chassis params end"<<std::endl;
    }

    void resetOptimization() {
        gtsam::ISAM2Params optParameters;
        optParameters.relinearizeThreshold = 0.1;
        optParameters.relinearizeSkip = 1;
        optimizer = gtsam::ISAM2(optParameters);

        gtsam::NonlinearFactorGraph newGraphFactors;
        graphFactors = newGraphFactors;

        gtsam::Values NewGraphValues;
        graphValues = NewGraphValues;
    }

    void resetParams() {
        lastImuT_imu = -1;
        doneFirstOpt = false;
        systemInitialized = false;
    }

    void chassisHandler(const lio_sam::chassis_data::ConstPtr &chassis_msg) {
        if(useChassis == false){
            return;
        }
        std::lock_guard <std::mutex> lock(mtx);
        DynamicMeasurement thisChassis = vehicleDynamicsModel(chassis_msg->header.stamp.toSec(), chassis_msg->Velocity,
                                                               chassis_msg->SteeringAngle);
        chaQueOpt.push_back(thisChassis);
        chaQueCha.push_back(thisChassis);

        if (doneFirstOpt == false)
            return;

        double chaTime = thisChassis.time;
        double dt = (lastChaT_cha < 0) ? (1.0 / 500.0) : (chaTime - lastChaT_cha);
        lastChaT_cha = chaTime;
        // integrate this single chassis message
        chaIntegratorCha_->integrateMeasurement(
                thisChassis.velocity,thisChassis.angle, dt);

        // predict state and publish odometry
        const bool CHASSIS_ODOMETRY=true;
        if(CHASSIS_ODOMETRY){
            // predict odometry
            gtsam::ChaNavState currentChaState = chaIntegratorCha_->predict(prevStateCha, prevBiasCha);// TODO

            // publish odometry
            nav_msgs::Odometry odometryCha;
            odometryCha.header.stamp = chassis_msg->header.stamp;
            odometryCha.header.frame_id = odometryFrame;
            odometryCha.child_frame_id = "odom_cha";

            // transform cha pose to lidar
            gtsam::Pose3 chaPose = gtsam::Pose3(currentChaState.quaternion(), currentChaState.position());
            gtsam::Pose3 lidarPose = chaPose.compose(imu2Lidar); //平移变换

            odometryCha.pose.pose.position.x = lidarPose.translation().x();
            odometryCha.pose.pose.position.y = lidarPose.translation().y();
            odometryCha.pose.pose.position.z = lidarPose.translation().z();
            odometryCha.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
            odometryCha.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
            odometryCha.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
            odometryCha.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();

            odometryCha.twist.twist.linear.x = 0;
            odometryCha.twist.twist.linear.y = 0;
            odometryCha.twist.twist.linear.z =0;
            odometryCha.twist.twist.angular.x = 0;
            odometryCha.twist.twist.angular.y = 0;
            odometryCha.twist.twist.angular.z = 0;
            pubChaOdometry.publish(odometryCha);
            myfileCha.open("/home/xxiao/data/lio-sam/cha.txt", ios::app);
            myfileCha.precision(10);
            ROS_DEBUG("cha success");
            myfileCha<<  odometryCha.header.stamp <<" ";
            myfileCha <<odometryCha.pose.pose.position.x<< " " <<odometryCha.pose.pose.position.y<< " "<<odometryCha.pose.pose.position.z;
            myfileCha <<" " << odometryCha.pose.pose.orientation.x << " " << odometryCha.pose.pose.orientation.y << " " << odometryCha.pose.pose.orientation.z<<" "<<odometryCha.pose.pose.orientation.w ;
            myfileCha<< "\n";
            myfileCha.close();
        }
    }

    DynamicMeasurement vehicleDynamicsModel(double t, double Velocity, double Steer) {
        DynamicMeasurement chassis_out;
        chassis_out.time = t;
        // std::cout<<"velocity is "<<Velocity<<"-------steer is "<<Steer<<std::endl;
        double steer = 0, bias = 0;
        double beta;
        const double k1 = 30082 * 2;//front tyre
        const double k2 = 31888 * 2;//rear tyre
        const double mass = 1096;//zhiliang
        const double len = 2.3;
        const double len_a = 1.0377;//qianzhou
        const double len_b = 1.2623;//houzhou
        const double i0 = 17.4;
        const double K = mass * (len_a / k2 - len_b / k1) / (len * len);

        vel = Velocity / 3.6;//速度
        steer = -(Steer + bias) * M_PI / 180;//方向盘转角

        //!dynamics
        beta = (1 + mass * vel * vel * len_a / (2 * len * len_b * k2)) * len_b * steer / i0 / len / (1 - K * vel * vel);
        vy = vel * sin(beta);//changed for xiaomi_d you:x shang:y hou:z
        vx = vel * cos(beta);
//        rz = vel * steer / i0 / len / (1 - K * vel * vel);
        chassis_out.velocity = {vx, vy, vz};
        //ROS_INFO("chassis vel is %f,%f,%f", vx, vy, vz);
        //chassis_out.angle = {rx, ry, rz};
        //// std::cout<<"after Velocity is"<< chassis_out.velocity<<"==angel is "<<chassis_out.angle<<std::endl;
//        ROS_INFO("chassis angular_vel is %f,%f,%f",rx,ry,rz);//right,front,yaw
        // use latest imu angle velocity (imu msg time - chassis msg time = -0.003825s)
        chassis_out.angle = {tmp_imu_ang_x, tmp_imu_ang_y, tmp_imu_ang_z};
        return chassis_out;
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr &odomMsg) {
        std::lock_guard <std::mutex> lock(mtx);

        double currentCorrectionTime = ROS_TIME(odomMsg);

        // make sure we have imu data to integrate
        if (imuQueOpt.empty())
            return;

        float p_x = odomMsg->pose.pose.position.x;
        float p_y = odomMsg->pose.pose.position.y;
        float p_z = odomMsg->pose.pose.position.z;
        float r_x = odomMsg->pose.pose.orientation.x;
        float r_y = odomMsg->pose.pose.orientation.y;
        float r_z = odomMsg->pose.pose.orientation.z;
        float r_w = odomMsg->pose.pose.orientation.w;
        bool degenerate = (int) odomMsg->pose.covariance[0] == 1 ? true : false;
        gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(r_w, r_x, r_y, r_z),
                                              gtsam::Point3(p_x, p_y, p_z));


        // 0. initialize system
        if (systemInitialized == false) {
            resetOptimization();

            // pop old IMU message
            while (!imuQueOpt.empty()) {
                if (ROS_TIME(&imuQueOpt.front()) < currentCorrectionTime - delta_t) {
                    lastImuT_opt = ROS_TIME(&imuQueOpt.front());
                    imuQueOpt.pop_front();
                } else
                    break;
            }

            // pop old chassis message
            while (!chaQueOpt.empty() && useChassis) {
                if (chaQueOpt.front().time < currentCorrectionTime - delta_t) {
                    lastChaT_opt = chaQueOpt.front().time;
                    chaQueOpt.pop_front();
                } else
                    break;
            }

            // initial pose
            prevPose_ = lidarPose.compose(lidar2Imu);
            gtsam::PriorFactor <gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
            graphFactors.add(priorPose);
            // initial velocity
            prevVel_ = gtsam::Vector3(0, 0, 0);
            gtsam::PriorFactor <gtsam::Vector3> priorVel(V(0), prevVel_, priorVelNoise);
            graphFactors.add(priorVel);
            // initial bias
            prevBias_ = gtsam::imuBias::ConstantBias();
            gtsam::PriorFactor <gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
            graphFactors.add(priorBias);
            if(useChassis){
                // initial chassis bias
                prevBiasCha_ = gtsam::chaBias::ConstantBias();
                gtsam::PriorFactor <gtsam::chaBias::ConstantBias> priorBiasCha(K(0), prevBiasCha_, priorBiasNoise);
                graphFactors.add(priorBiasCha);
                graphValues.insert(K(0), prevBiasCha_);
            }
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_); //TODO USING CHASSIS?
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            imuIntegratorImu_->resetIntegrationAndSetBias(prevBias_);
            imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

            if(useChassis){
                chaIntegratorCha_->resetIntegrationAndSetBias(prevBiasCha_);
                chaIntegratorOpt_->resetIntegrationAndSetBias(prevBiasCha_);
            }

            key = 1;
            systemInitialized = true;
            return;
        }


        // reset graph for speed
        if (key == 100) {
            // get updated noise before reset
            gtsam::noiseModel::Gaussian::shared_ptr updatedPoseNoise = gtsam::noiseModel::Gaussian::Covariance(
                    optimizer.marginalCovariance(X(key - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedVelNoise = gtsam::noiseModel::Gaussian::Covariance(
                    optimizer.marginalCovariance(V(key - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasNoise = gtsam::noiseModel::Gaussian::Covariance(
                    optimizer.marginalCovariance(B(key - 1)));
            gtsam::noiseModel::Gaussian::shared_ptr updatedBiasChaNoise=gtsam::noiseModel::Gaussian::Covariance(
                    optimizer.marginalCovariance(B(key - 1)));
            if(useChassis){
                updatedBiasChaNoise = gtsam::noiseModel::Gaussian::Covariance(
                        optimizer.marginalCovariance(K(key - 1)));
            }
            // reset graph
            resetOptimization();
            // add pose
            gtsam::PriorFactor <gtsam::Pose3> priorPose(X(0), prevPose_, updatedPoseNoise);
            graphFactors.add(priorPose);
            // add velocity
            gtsam::PriorFactor <gtsam::Vector3> priorVel(V(0), prevVel_, updatedVelNoise);
            graphFactors.add(priorVel);
            // add bias
            gtsam::PriorFactor <gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, updatedBiasNoise);
            graphFactors.add(priorBias);
            if(useChassis){
                // add cha bias
                gtsam::PriorFactor <gtsam::chaBias::ConstantBias> priorBiasCha(K(0), prevBiasCha_, updatedBiasChaNoise);
                graphFactors.add(priorBiasCha);
                graphValues.insert(K(0), prevBiasCha_);
            }
            // add values
            graphValues.insert(X(0), prevPose_);
            graphValues.insert(V(0), prevVel_);
            graphValues.insert(B(0), prevBias_);
            // optimize once
            optimizer.update(graphFactors, graphValues);
            graphFactors.resize(0);
            graphValues.clear();

            key = 1;
        }

        // std::cout<<"3.0-----second graph begin "<<std::endl;
        // 1. integrate imu data and optimize
        while (!imuQueOpt.empty()) {
            // pop and integrate imu data that is between two optimizations
            sensor_msgs::Imu *thisImu = &imuQueOpt.front();
            double imuTime = ROS_TIME(thisImu);
            if (imuTime < currentCorrectionTime - delta_t) {
                double dt = (lastImuT_opt < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_opt);
                imuIntegratorOpt_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y,
                                       thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y,
                                       thisImu->angular_velocity.z), dt);

                lastImuT_opt = imuTime;
                imuQueOpt.pop_front();
            } else
                break;
        }
        // std::cout<<"3.01-----add imu end "<<std::endl;
        // 2. integrate chassis data and optimize
        if(useChassis){
            while (!chaQueOpt.empty()) {
                // pop and integrate chassis data that is between two optimizations
                DynamicMeasurement *thisChassis= &chaQueOpt.front();
                double chaTime = thisChassis->time;
                // std::cout<<"3.02-----the chaTime is "<<chaTime<<std::endl;
                if (chaTime < currentCorrectionTime - delta_t) {
                    double dt = (lastChaT_opt < 0) ? (1.0 / 500.0) : (chaTime - lastChaT_opt);
                    // std::cout<<"3.02-----the dt is "<<dt<<std::endl;
                    gtsam::Vector3 v=thisChassis->velocity;
                    // std::cout<<"velocity is "<<v<<std::endl;
                    gtsam::Vector3 ang=thisChassis->angle;
                    // std::cout<<"angle is "<<ang<<std::endl;
                    chaIntegratorOpt_->integrateMeasurement(v, ang, dt);
                    // std::cout<<"3.03-----the integrateMeasurement end"<<std::endl;
                    lastChaT_opt = chaTime;
                    chaQueOpt.pop_front();
                } else
                    break;
            }
        }

       // std::cout<<"3-------------------integrated chassis end"<<std::endl;
        // add imu factor to graph
        const gtsam::PreintegratedImuMeasurements &preint_imu = dynamic_cast<const gtsam::PreintegratedImuMeasurements &>(*imuIntegratorOpt_);
        gtsam::ImuFactor imu_factor(X(key - 1), V(key - 1), X(key), V(key), B(key - 1), preint_imu);
        graphFactors.add(imu_factor);
        imu_factor.print("imufactor");
        // add imu bias between factor
        graphFactors.add(
                gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(key - 1), B(key), gtsam::imuBias::ConstantBias(),
                                                                   gtsam::noiseModel::Diagonal::Sigmas(
                                                                           sqrt(imuIntegratorOpt_->deltaTij()) *
                                                                           noiseModelBetweenBias)));
        // add pose factor
        gtsam::Pose3 curPose = lidarPose.compose(lidar2Imu);
        gtsam::PriorFactor <gtsam::Pose3> pose_factor(X(key), curPose, degenerate ? correctionNoise2 : correctionNoise);
        // mock lidar not good
        // gtsam::PriorFactor <gtsam::Pose3> pose_factor(X(key), curPose, correctionNoise3);
        graphFactors.add(pose_factor);

        // insert predicted values
        gtsam::NavState propState_ = imuIntegratorOpt_->predict(prevState_, prevBias_);
        graphValues.insert(X(key), propState_.pose());
        graphValues.insert(V(key), propState_.v());
        graphValues.insert(B(key), prevBias_);

        // add chassis factor
        if (useChassis) {
            const gtsam::PreintegratedChaMeasurements &preint_cha = dynamic_cast<const gtsam::PreintegratedChaMeasurements &>(*chaIntegratorOpt_);
            gtsam::ChaFactor cha_factor(X(key - 1), X(key), K(key - 1), preint_cha);
            graphFactors.add(cha_factor);
            cha_factor.print();
            // add cha bias between factor
            graphFactors.add(
                    gtsam::BetweenFactor<gtsam::chaBias::ConstantBias>(K(key - 1), K(key), gtsam::chaBias::ConstantBias(),
                                                                       gtsam::noiseModel::Diagonal::Sigmas(
                                                                               sqrt(chaIntegratorOpt_->deltaTij()) *
                                                                               noiseModelBetweenBias)));
            graphValues.insert(K(key), prevBiasCha_);
        }

        // optimize
        optimizer.update(graphFactors, graphValues);
        optimizer.update();
        graphFactors.resize(0);
        graphValues.clear();
        // Overwrite the beginning of the preintegration for the next step.
        gtsam::Values result = optimizer.calculateEstimate();
        prevPose_ = result.at<gtsam::Pose3>(X(key));
        prevVel_ = result.at<gtsam::Vector3>(V(key));
        prevState_ = gtsam::NavState(prevPose_, prevVel_);
        prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(key));
        // Reset the optimization preintegration object.
        imuIntegratorOpt_->resetIntegrationAndSetBias(prevBias_);

        if(useChassis){
            prevStateCha_=gtsam::ChaNavState(prevPose_);
            prevBiasCha_=result.at<gtsam::chaBias::ConstantBias>(K(key));
            chaIntegratorOpt_->resetIntegrationAndSetBias(prevBiasCha_);
        }
        // check optimization
        if (failureDetection(prevVel_, prevBias_)) {
            resetParams();
            return;
        }

        // 2. after optiization, re-propagate imu odometry preintegration
        prevStateOdom = prevState_;
        prevBiasOdom = prevBias_;

        // first pop imu message older than current correction data
        double lastImuQT = -1;
        while (!imuQueImu.empty() && ROS_TIME(&imuQueImu.front()) < currentCorrectionTime - delta_t) {
            lastImuQT = ROS_TIME(&imuQueImu.front());
            imuQueImu.pop_front();
        }
        // repropogate imuIntegratorImu_利用imuQueImu中的imu信息重新积分
        if (!imuQueImu.empty()) {
            // reset bias use the newly optimized bias
            imuIntegratorImu_->resetIntegrationAndSetBias(prevBiasOdom);
            // integrate imu message from the beginning of this optimization
            for (int i = 0; i < (int) imuQueImu.size(); ++i) {
                sensor_msgs::Imu *thisImu = &imuQueImu[i];
                double imuTime = ROS_TIME(thisImu);
                double dt = (lastImuQT < 0) ? (1.0 / 500.0) : (imuTime - lastImuQT);

                imuIntegratorImu_->integrateMeasurement(
                        gtsam::Vector3(thisImu->linear_acceleration.x, thisImu->linear_acceleration.y,
                                       thisImu->linear_acceleration.z),
                        gtsam::Vector3(thisImu->angular_velocity.x, thisImu->angular_velocity.y,
                                       thisImu->angular_velocity.z), dt);
                lastImuQT = imuTime;
            }
        }
        if(useChassis){
            prevBiasCha = prevBiasCha_;
            prevStateCha = prevStateCha_;
            double lastChaQT = -1;
            DynamicMeasurement *thisChassis=&chaQueCha.front();
            while (!chaQueCha.empty() && thisChassis->time < currentCorrectionTime - delta_t) {
                lastChaQT = thisChassis->time;
                chaQueCha.pop_front();
            }
            if (!chaQueCha.empty()) {
                // reset bias use the newly optimized bias
                chaIntegratorCha_->resetIntegrationAndSetBias(prevBiasCha);
                // integrate chassis message from the beginning of this optimization
                for (int i = 0; i < (int) chaQueCha.size(); ++i) {
                    DynamicMeasurement *thisChassis = &chaQueCha[i];
                    double chaTime = thisChassis->time;
                    double dt = (lastChaQT < 0) ? (1.0 / 500.0) : (chaTime - lastChaQT);

                    chaIntegratorCha_->integrateMeasurement(thisChassis->velocity,thisChassis->angle, dt);
                    lastChaQT = chaTime;
                }
            }
        }

        ++key;
        doneFirstOpt = true;
    }

    bool failureDetection(const gtsam::Vector3 &velCur, const gtsam::imuBias::ConstantBias &biasCur) {
        Eigen::Vector3f vel(velCur.x(), velCur.y(), velCur.z());
        if (vel.norm() > 30) {
            ROS_WARN("Large velocity, reset IMU-preintegration!");
            return true;
        }

        Eigen::Vector3f ba(biasCur.accelerometer().x(), biasCur.accelerometer().y(), biasCur.accelerometer().z());
        Eigen::Vector3f bg(biasCur.gyroscope().x(), biasCur.gyroscope().y(), biasCur.gyroscope().z());
        if (ba.norm() > 1.0 || bg.norm() > 1.0) {
            ROS_WARN("Large bias, reset IMU-preintegration!");
            return true;
        }

        return false;
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr &imu_raw) {
        std::lock_guard <std::mutex> lock(mtx);

        sensor_msgs::Imu thisImu = imuConverter(*imu_raw); //旋转变换 将imu坐标系转到lidar坐标系

        tmp_imu_ang_x=thisImu.angular_velocity.x;
        tmp_imu_ang_y=thisImu.angular_velocity.y;
        tmp_imu_ang_z=thisImu.angular_velocity.z;

        imuQueOpt.push_back(thisImu);
        imuQueImu.push_back(thisImu);

        if (doneFirstOpt == false)
            return;

        double imuTime = ROS_TIME(&thisImu);
        double dt = (lastImuT_imu < 0) ? (1.0 / 500.0) : (imuTime - lastImuT_imu);
        lastImuT_imu = imuTime;
         // integrate this single imu message
        imuIntegratorImu_->integrateMeasurement(
                gtsam::Vector3(thisImu.linear_acceleration.x, thisImu.linear_acceleration.y,
                               thisImu.linear_acceleration.z),
                gtsam::Vector3(thisImu.angular_velocity.x, thisImu.angular_velocity.y, thisImu.angular_velocity.z), dt);
        // predict odometry
        gtsam::NavState currentState = imuIntegratorImu_->predict(prevStateOdom, prevBiasOdom);

        // publish odometry
        nav_msgs::Odometry odometry;
        odometry.header.stamp = thisImu.header.stamp;
        odometry.header.frame_id = odometryFrame;
        odometry.child_frame_id = "odom_imu";

        // transform imu pose to lidar
        gtsam::Pose3 imuPose = gtsam::Pose3(currentState.quaternion(), currentState.position());
        gtsam::Pose3 lidarPose = imuPose.compose(imu2Lidar); //平移变换

        odometry.pose.pose.position.x = lidarPose.translation().x();
        odometry.pose.pose.position.y = lidarPose.translation().y();
        odometry.pose.pose.position.z = lidarPose.translation().z();
        odometry.pose.pose.orientation.x = lidarPose.rotation().toQuaternion().x();
        odometry.pose.pose.orientation.y = lidarPose.rotation().toQuaternion().y();
        odometry.pose.pose.orientation.z = lidarPose.rotation().toQuaternion().z();
        odometry.pose.pose.orientation.w = lidarPose.rotation().toQuaternion().w();

        odometry.twist.twist.linear.x = currentState.velocity().x();
        odometry.twist.twist.linear.y = currentState.velocity().y();
        odometry.twist.twist.linear.z = currentState.velocity().z();
        odometry.twist.twist.angular.x = thisImu.angular_velocity.x + prevBiasOdom.gyroscope().x();
        odometry.twist.twist.angular.y = thisImu.angular_velocity.y + prevBiasOdom.gyroscope().y();
        odometry.twist.twist.angular.z = thisImu.angular_velocity.z + prevBiasOdom.gyroscope().z();
        pubImuOdometry.publish(odometry);
        //save odometry
        myfileImu.open("/home/xxiao/data/lio-sam/imu.txt", ios::app);
        myfileImu.precision(10);
        ROS_DEBUG("imu success");
        myfileImu<<  thisImu.header.stamp<<" ";
        myfileImu <<odometry.pose.pose.position.x<< " " <<odometry.pose.pose.position.y<< " "<<odometry.pose.pose.position.z;
        myfileImu <<" " << odometry.pose.pose.orientation.x << " " << odometry.pose.pose.orientation.y << " " << odometry.pose.pose.orientation.z<<" "<<odometry.pose.pose.orientation.w ;
        myfileImu<< "\n";
        myfileImu.close();
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "roboat_loam");

    IMUPreintegration ImuP;

    TransformFusion TF;

    ROS_INFO("\033[1;32m----> IMU Preintegration Started.\033[0m");

    ros::MultiThreadedSpinner spinner(4);
    spinner.spin();

    return 0;
}
