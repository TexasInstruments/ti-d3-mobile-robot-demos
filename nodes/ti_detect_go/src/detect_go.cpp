/*
 *  Copyright (C) 2022 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "detect_go.h"

static DetectGoNode *detect_go_node;

DetectGoNode::DetectGoNode(ros::NodeHandle *nh,
                           ros::NodeHandle *private_nh)
{
    // parse params from launch file
    private_nh->param("radar_track_topic", m_radar_track_topic, std::string("ti_mmwave/radar_trackarray"));
    private_nh->param("out_vel_topic", m_out_vel_topic, std::string("nav_vel"));
    private_nh->param("control_cycle_hz", m_control_cycle_hz, 10.0f);
    private_nh->param("speed_linear_fast", SPEED_LINEAR_FAST, 0.8f);
    private_nh->param("speed_linear_slow", SPEED_LINEAR_SLOW, 0.4f);
    private_nh->param("speed_linear_stop", SPEED_LINEAR_STOP, 0.0f);
    private_nh->param("zone_stop_xlim", ZONE_STOP_XLIM, 0.9f);
    private_nh->param("zone_slow_xlim", ZONE_SLOW_XLIM, 1.8f);
    private_nh->param("zone_yabs", ZONE_YABS, 0.6f);

    // subscriber, publisher, timer
    m_sub_track = nh->subscribe(m_radar_track_topic, 10, &DetectGoNode::trackarray_callback, this);
    m_pub = nh->advertise<geometry_msgs::Twist>(m_out_vel_topic, 10);
    m_timer = nh->createTimer(ros::Duration(1.0/m_control_cycle_hz), &DetectGoNode::control_cycle, this);

    // initialize
    m_state = STOP;
    m_state_ts = ros::Time::now();

    // define zones
    m_zone_stop = {0.0, ZONE_STOP_XLIM, -ZONE_YABS, ZONE_YABS};
    m_zone_slow = {ZONE_STOP_XLIM, ZONE_SLOW_XLIM, -ZONE_YABS, ZONE_YABS};

}

DetectGoNode::~DetectGoNode() { }

void DetectGoNode::trackarray_callback(const ti_mmwave_tracker_rospkg::RadarTrackArray::ConstPtr& msg)
{
    m_last_radar_trackarray = std::move(msg);
}

void DetectGoNode::control_cycle(const ros::TimerEvent& e)
{
    if (m_last_radar_trackarray == nullptr)
    {
        return;
    }

    // print_radar_trackarray();

    // occupancy flags for each zone
    m_flag_slow = check_zone(m_zone_slow);
    m_flag_stop = check_zone(m_zone_stop);
    // ROS_INFO("flag_slow, flag_stop: %d, %d", m_flag_slow, m_flag_stop);

    geometry_msgs::Twist out_vel;

    switch (m_state)
    {
        case fwdfast:
            out_vel.linear.x = SPEED_LINEAR_FAST;

            if (check_fwdfast_to_stop()) {
                change_state(STOP);
            }

            if (check_fwdfast_to_fwdslow()) {
                change_state(FWDSLOW);
            }
            break;

        case FWDSLOW:
            out_vel.linear.x = SPEED_LINEAR_SLOW;

            if (check_fwdslow_to_stop()) {
                change_state(STOP);
            }

            if (check_fwdslow_to_fwdfast()) {
                change_state(fwdfast);
            }
            break;

        case STOP:
            out_vel.linear.x = SPEED_LINEAR_STOP;

            if (check_stop_to_fwdfast()) {
                change_state(fwdfast);
            }

            if (check_stop_to_fwdslow()) {
                change_state(FWDSLOW);
            }
            break;
    }

    m_pub.publish(out_vel);
}

void DetectGoNode::print_radar_trackarray()
{
    geometry_msgs::Point32 track;
    int32_t num_tracks = m_last_radar_trackarray->num_tracks;

    for (int32_t idx = 0; idx < num_tracks; idx++) {
        track.x = m_last_radar_trackarray->track[idx].posx;
        track.y = m_last_radar_trackarray->track[idx].posy;
        track.z = m_last_radar_trackarray->track[idx].posz;
        ROS_INFO("x,y,z: %f, %f, %f", track.x, track.y, track.z);
    }
    ROS_INFO("num_tracks = %d", num_tracks);
}

bool DetectGoNode::check_zone(Zone2d_t zone)
{
    geometry_msgs::Point32 track;
    int32_t num_tracks = m_last_radar_trackarray->num_tracks;
    bool occupancy = false;

    for (int32_t idx = 0; idx < num_tracks; idx++) {
        track.x = m_last_radar_trackarray->track[idx].posx;
        track.y = m_last_radar_trackarray->track[idx].posy;
        track.z = m_last_radar_trackarray->track[idx].posz;

        if ((track.x > zone.xmin) && (track.x < zone.xmax) &&
           (track.y > zone.ymin) && (track.y < zone.ymax))
        {
            occupancy = true;
            break;
        }
    }

    return occupancy;
}

bool DetectGoNode::check_stop_to_fwdfast()
{
    return !m_flag_stop && !m_flag_slow;
}

bool DetectGoNode::check_stop_to_fwdslow()
{
    return !m_flag_stop && m_flag_slow;
}

bool DetectGoNode::check_fwdslow_to_stop()
{
    return m_flag_stop;
}

bool DetectGoNode::check_fwdslow_to_fwdfast()
{
    return !m_flag_stop && !m_flag_slow;
}

bool DetectGoNode::check_fwdfast_to_stop()
{
    return m_flag_stop;
}

bool DetectGoNode::check_fwdfast_to_fwdslow()
{
    return !m_flag_stop && m_flag_slow;
}

void DetectGoNode::change_state(state_t state_new)
{
    m_state = state_new;
    m_state_ts = ros::Time::now();
}

/**
 * Main
 */
int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "app_detect_go");
        ros::NodeHandle nh;
        ros::NodeHandle private_nh("~");
        detect_go_node = new DetectGoNode(&nh, &private_nh);
        ros::spin();
        ros::shutdown();
        return EXIT_SUCCESS;
    }
    catch (std::runtime_error& e)
    {
        return EXIT_FAILURE;
    }
}
