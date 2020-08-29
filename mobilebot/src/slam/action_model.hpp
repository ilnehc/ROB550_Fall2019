#ifndef SLAM_ACTION_MODEL_HPP
#define SLAM_ACTION_MODEL_HPP

#include <lcmtypes/pose_xyt_t.hpp>
#include <random>

struct particle_t;

/**
* ActionModel implements the sampling-based odometry action model for estimating the motion of the robot between
* time t and t'.
* 
* An action model is used to propagate a sample from the prior distribution, x, into
* the proposal distribution, x', based on the supplied motion estimate of the robot
* in the time interval [t, t'].
*
* To use the ActionModel, a two methods exist:
*
*   - bool updateAction(const pose_xyt_t& odometry);
*   - particle_t applyAction(const particle_t& sample);
*
* updateAction() provides the most recent odometry data so the action model can update the distributions from
* which it will sample.
*
* applyAction() applies the action to the provided sample and returns a new sample that can be part of the proposal 
* distribution for the particle filter.
*/
class ActionModel
{
public:
    
    /**
    * Constructor for ActionModel.
    */
    ActionModel(void);
    
    /**
    * updateAction sets up the motion model for the current update for the localization.
    * After initialization, calls to applyAction() will be made, so all distributions based on sensor data
    * should be created here.
    *
    * \param    odometry            Current odometry data from the robot
    * \return   The pose transform distribution representing the uncertainty of the robot's motion.
    */
    bool updateAction(const pose_xyt_t& odometry);
    
    /**
    * applyAction applies the motion to the provided sample and returns a new sample that
    * can be part of the proposal distribution for the particle filter.
    *
    * \param    sample          Sample to be moved
    * \return   New sample based on distribution from the motion model at the current update.
    */
    particle_t applyAction(const particle_t& sample);

    pose_xyt_t pre_odometry;

    
private:
    
    ////////// TODO: Add private member variables needed for you implementation ///////////////////
    
    float alpha1;
    float alpha2;
    float alpha3;
    float alpha4;

    float p[3];
    float var[3];

    float delta_rot1;
    float delta_trans;
    float delta_rot2;
    float delta_rot1_hat;
    float delta_trans_hat;
    float delta_rot2_hat;

    std::default_random_engine generator;
    float delta_x;
    float delta_y;
    float delta_theta;

    //std::normal_distribution<float> p0;
    //std::normal_distribution<float> p1;
    //std::normal_distribution<float> p2;


    
};

#endif // SLAM_ACTION_MODEL_HPP
