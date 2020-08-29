#include <slam/action_model.hpp>
#include <lcmtypes/particle_t.hpp>
#include <common/angle_functions.hpp>
#include <common/pose_trace.hpp>
#include <cassert>
#include <cmath>
#include <iostream>


ActionModel::ActionModel(void)
{
    //////////////// TODO: Handle any initialization for your ActionModel /////////////////////////
    alpha1 = .01; // 0.015; //.001;//.05; // were all .001
    alpha2 = .01; //0.015;
    alpha3 = .01; //0.015;
    alpha4 = .01; //0.015;

    pre_odometry.x = 0;
    pre_odometry.y = 0;
    pre_odometry.theta = 0;

    p[0] = 0;
    p[1] = 0;
    p[2] = 0;
    var[0] = 0.0;
    var[1] = 0.0;
    var[2] = 0.0;

    delta_rot1 = 0;
    delta_trans = 0;
    delta_rot2 = 0;
    delta_rot1_hat = 0;
    delta_trans_hat = 0;
    delta_rot2_hat = 0;

    delta_x = 0;
    delta_y = 0;
    delta_theta = 0;
}


bool ActionModel::updateAction(const pose_xyt_t& odometry)
{
    ////////////// TODO: Implement code here to compute a new distribution of the motion of the robot ////////////////
    //PoseTrace poses = ?;
    //ose_xyt_t pre_odometry = poses.poseAt(poses.size()-1);

    delta_x = odometry.x - pre_odometry.x;
    delta_y = odometry.y - pre_odometry.y;
    delta_theta = odometry.theta - pre_odometry.theta;

    /*   
    delta_rot1 = atan2(odometry.y - pre_odometry.y, odometry.x - pre_odometry.x) - pre_odometry.theta;
    delta_trans = sqrt((pre_odometry.x-odometry.x)*(pre_odometry.x-odometry.x) + (pre_odometry.y-odometry.y)*(pre_odometry.y-odometry.y));
    delta_rot2 = odometry.theta - pre_odometry.theta - delta_rot1;

    var[0] = alpha1*delta_rot1*delta_rot1 + alpha2*delta_trans*delta_trans;
    var[1] = alpha3*delta_trans*delta_trans + alpha4*delta_rot1*delta_rot1 + alpha4*delta_rot2*delta_rot2;
    var[2] = alpha1*delta_rot2*delta_rot2 + alpha2*delta_trans*delta_trans;
    */

    // ALL THESE .05 FOR COMPETITION, .01 for low spread for report
    var[0] = .05; // .1 WORKS FOR LOCALIZATION ONLY
    var[1] = .05;
    var[2] = .05;
    //


    // check if previous odometry is close to new odometry
    float epsilon = .000001; // made this 100 times smaller
    float x_dist = (odometry.x - pre_odometry.x)*(odometry.x - pre_odometry.x);
    float y_dist = (odometry.y - pre_odometry.y)*(odometry.y - pre_odometry.y);
    float theta_dist = (odometry.theta - pre_odometry.theta)*(odometry.theta - pre_odometry.theta);
    float distance = sqrt( x_dist + y_dist + theta_dist );

    //
    // set previous odometry values to current odometry values
    pre_odometry.utime = odometry.utime;
    pre_odometry.x = odometry.x;
    pre_odometry.y = odometry.y;
    pre_odometry.theta = odometry.theta;
    //

    if (distance > epsilon) {
        return true;
    }
    return false;
}


particle_t ActionModel::applyAction(const particle_t& sample)
{
    ////////////// TODO: Implement your code for sampling new poses from the distribution computed in updateAction //////////////////////
    // Make sure you create a new valid particle_t. Don't forget to set the new time and new parent_pose.

    std::normal_distribution<float> p0(0.0,var[0]);
    std::normal_distribution<float> p1(0.0,var[1]);
    std::normal_distribution<float> p2(0.0,var[2]);
    //


    p[0] = p0(generator);
    p[1] = p1(generator);
    p[2] = p2(generator);


    particle_t new_sample;

    /*
    delta_rot1_hat = delta_rot1 - p[0];
    delta_trans_hat = delta_trans - p[1];
    delta_rot2_hat = delta_rot2 - p[2];

    new_sample.pose.x = sample.pose.x + delta_trans_hat * cos(sample.pose.theta + delta_rot1_hat); // changed all parent pose to pose
    new_sample.pose.y = sample.pose.y + delta_trans_hat * sin(sample.pose.theta + delta_rot1_hat); // thought this might be minus
    new_sample.pose.theta = sample.pose.theta + delta_rot1_hat + delta_rot2_hat;
    */


    /*
    new_sample.pose.x = pre_odometry.x + p[0];
    new_sample.pose.y = pre_odometry.y + p[1];
    new_sample.pose.theta = pre_odometry.theta + p[2];
    */

    //
    new_sample.pose.x = sample.pose.x + delta_x + p[0]; // these were all pre_odometry // might try pose next
    new_sample.pose.y = sample.pose.y + delta_y + p[1];
    new_sample.pose.theta = sample.pose.theta + delta_theta + p[2];
    //

    new_sample.parent_pose = sample.pose;

    new_sample.weight = sample.weight;

    //printf("Prev X: %f\n", sample.parent_pose.x);
    //printf("New X: %f\n", new_sample.pose.x);

    return new_sample;
}
