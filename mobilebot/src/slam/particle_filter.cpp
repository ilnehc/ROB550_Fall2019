#include <slam/particle_filter.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <cassert>
#include <chrono>
#include <time.h>
#include <math.h>

#include <common/grid_utils.hpp>

//
#include <sys/time.h>

int64_t utime_now (void){
    struct timeval tv;
    gettimeofday (&tv, NULL);
    return (int64_t) tv.tv_sec * 1000000 + tv.tv_usec;
}
//

ParticleFilter::ParticleFilter(int numParticles)
: kNumParticles_ (numParticles)
{
    assert(kNumParticles_ > 1);
    posterior_.resize(kNumParticles_);
}

/*
void ParticleFilter::applyEpsilon(float epsilon) {
    particles_t prior = particles();


    for (int i = 0; i < prior.num_particles; i++) {
        particle_t particle = prior.particles[i];
        
    }
}*/

double ParticleFilter::my_rand(double min, double max) {
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator (seed);
    std::uniform_real_distribution<double> distribution(min,max);
    return distribution(generator);
}



void ParticleFilter::initializeFilterAtPose(const pose_xyt_t& pose)
{
    std::cout << "X: " << pose.x << " Y: " << pose.y << "\n";
    /*
    for (int i=0;i<30;i++) {
        for (int j=0; j<30; j++) {
            pose_xyt_t temp_pose;
            //temp_pose.x = -4.9 + (10.0/30.0)*j;
            //temp_pose.y = -4.9 + (10.0/30.0)*i;
            temp_pose.x = -2.5 + (5.0/30.0)*j; 
            temp_pose.y = -1.5 + (5.0/30.0)*i;
            posterior_[i*30+j].pose = temp_pose;
            posterior_[i*30+j].weight = 1.0 / kNumParticles_;
            posterior_[i*30+j].parent_pose = temp_pose;

        }
    }

    actionModel_.pre_odometry.x = 2.5;
    actionModel_.pre_odometry.y = 2.5;
    actionModel_.pre_odometry.theta = pose.theta;
    */

       
    ///////////// TODO: Implement your method for initializing the particles in the particle filter /////////////////

    //for i in kNumParticles, initialize particle to start pose // no need to adjust by epsilon, done in action model
    for (int i = 0; i < kNumParticles_; i++) {

        //set particle pose to initial pose
        posterior_[i].pose = pose;
        posterior_[i].weight = 1.0 / kNumParticles_;
        posterior_[i].parent_pose = pose;  
    }

    actionModel_.pre_odometry.x = pose.x;
    actionModel_.pre_odometry.y = pose.y;
    actionModel_.pre_odometry.theta = pose.theta;

    printf("Initialized particle filter to pose\n");
    // to-do try printing pose for next time

    /*
    total_time = 0;
    num_times = 0;
    */
}


pose_xyt_t ParticleFilter::updateFilter(const pose_xyt_t&      odometry,
                                        const lidar_t& laser,
                                        const OccupancyGrid&   map)
{
    uint64_t start_time = utime_now();

    //printf("Made it into update filter\n");
    // Only update the particles if motion was detected. If the robot didn't move, then
    // obviously don't do anything.
    bool hasRobotMoved = actionModel_.updateAction(odometry);
    
    //printf("Updated action model if robot moved\n");

    if(hasRobotMoved)
    {
        //printf("checked if robot has moved\n");
        auto prior = resamplePosteriorDistribution(); // this just resamples. done
    
        auto proposal = computeProposalDistribution(prior); // this applies action model. done.
        /*
        for (auto p : proposal) {
            printf("proposal w:%f\n", p.weight);
        }
        */
       // printf("computed proposal\n");
        posterior_ = computeNormalizedPosterior(proposal, laser, map); // this uses likelihood sensor model and normalizes.
        /*
        for (auto p : posterior_) {
            printf("posterior w:%f\n", p.weight);
        } 
        */
        //printf("computed posterior\n");
        posteriorPose_ = estimatePosteriorPose(posterior_); // from all samples and weights, get pose estimate
       // printf("estimated new pose\n");
    }
    
    posteriorPose_.utime = odometry.utime;

    uint64_t time_elapsed = utime_now() - start_time;
    /*
    if (!num_times_init) {
        num_times_init = true;
        num_times = 1;
        total_time = 1;
    }

    total_time += time_elapsed;

    num_times++;
    */

    //std::cout << "Avg Time of one filter update in microsec: " << total_time / num_times << "\n";
    std::cout << time_elapsed << "\n";


    return posteriorPose_;

}


pose_xyt_t ParticleFilter::poseEstimate(void) const
{
    return posteriorPose_;
}


particles_t ParticleFilter::particles(void) const
{
    particles_t particles;
    particles.num_particles = posterior_.size();
    particles.particles = posterior_;
    return particles;
}


std::vector<particle_t> ParticleFilter::resamplePosteriorDistribution(void)
{
    //////////// TODO: Implement your algorithm for resampling from the posterior distribution ///////////////////
    
    std::vector<particle_t> prior = posterior_;
    std::vector<particle_t> new_particles;

    //particle_t best_particle;
    /* set all to best weight for lulz
    double max_weight = 0;
    for (auto p : prior) {
        double w = p.weight;
        if (w > max_weight) {
            max_weight = w;
            best_particle = p;
        }
    }

    for (int i = 1; i < kNumParticles_; i++) {
        new_particles.push_back(best_particle);
    }
    */

    //
    // Low variance resampling
    int M = kNumParticles_;
    double r = my_rand(0,1.0/M);
    double c = prior[0].weight;
    int i = 0; // is 1 in notes

    for (int m = 0; m < M; m++ ) {
        double U = r + m * (1.0/M);
        while (U > c) {
            i++;

            c += prior[i].weight;

        }

        new_particles.push_back(prior[i]);
    }
    //

    return new_particles;
}


std::vector<particle_t> ParticleFilter::computeProposalDistribution(const std::vector<particle_t>& prior)
{
    //////////// TODO: Implement your algorithm for creating the proposal distribution by sampling from the ActionModel
    std::vector<particle_t> proposal;

    // all we are doing is applying action model
    for ( auto particle : prior ) {
        auto new_particle = actionModel_.applyAction(particle);
        proposal.push_back(new_particle);
    }

    return proposal;
}


std::vector<particle_t> ParticleFilter::computeNormalizedPosterior(const std::vector<particle_t>& proposal,
                                                                   const lidar_t& laser,
                                                                   const OccupancyGrid&   map)
{
    /////////// TODO: Implement your algorithm for computing the normalized posterior distribution using the 
    ///////////       particles in the proposal distribution
    std::vector<particle_t> posterior;

    // Get weights
    double weight_sum = 0.0;
    double epsilon = .000001;
    for (auto particle : proposal) {
        double w = sensorModel_.likelihood(particle, laser, map);
        //double w = 1;
        if (w < epsilon) w = epsilon;
        particle.weight = w;
        weight_sum += w;
        posterior.push_back(particle);
    }

    //printf("Weight sum: %f\n", weight_sum);

    // Normalize weights
    for (int i = 0; i < kNumParticles_; i++) {
        posterior[i].weight = posterior[i].weight / weight_sum;
        //printf("Likelihood: %f\n", posterior[i].weight);
        //printf("Particle x: %f", posterior_[i].pose.x );
    }

    return posterior;
}


pose_xyt_t ParticleFilter::estimatePosteriorPose(const std::vector<particle_t>& posterior)
{
    //////// TODO: Implement your method for computing the final pose estimate based on the posterior distribution
    pose_xyt_t pose;
    //
    double x_sum = 0;
    double y_sum = 0;
    double sin_sum = 0;
    double cos_sum = 0;
    //

    //double max_weight = 0;

    for (auto particle : posterior) {
        double w = particle.weight;
        /*
        if (w > max_weight) {
            max_weight = w;
            pose = particle.pose;
        }
        */
        //
        x_sum += w * particle.pose.x;
        y_sum += w * particle.pose.y;
        sin_sum += w * sin(particle.pose.theta);
        cos_sum += w * cos(particle.pose.theta);
        //
    }

    //
    pose.x = x_sum;
    pose.y = y_sum;
    pose.theta = atan2(sin_sum, cos_sum);
    //

    return pose;
}
