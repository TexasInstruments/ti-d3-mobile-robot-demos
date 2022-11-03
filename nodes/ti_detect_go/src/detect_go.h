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

#ifndef _APP_DETECT_GO_NODE_H_
#define _APP_DETECT_GO_NODE_H_

#include <stdio.h>
#include <utility>
#include <ros/ros.h>
#include <std_msgs/Time.h>
#include <geometry_msgs/Twist.h>
#include <ti_mmwave_tracker_rospkg/RadarTrackArray.h>
#include <geometry_msgs/Point32.h>

enum state_t
{   fwdfast,
    FWDSLOW,
    STOP
};

typedef struct
{
    float xmin;
    float xmax;
    float ymin;
    float ymax;
} Zone2d_t;

typedef struct
{
    state_t state;
    ros::Time ts;
} State_t;

class DetectGoNode
{
    public:
        DetectGoNode(ros::NodeHandle *nh, ros::NodeHandle *private_nh);
        ~DetectGoNode();

    private:
        // void pcl_callback(const sensor_msgs::PointCloud2::ConstPtr& msg);
        void trackarray_callback(const ti_mmwave_tracker_rospkg::RadarTrackArray::ConstPtr& msg);
        void control_cycle(const ros::TimerEvent& e);
        void change_state(state_t state_new);
        void print_radar_trackarray();

        bool check_zone(Zone2d_t zone);
        bool check_fwdfast_to_stop();
        bool check_fwdfast_to_fwdslow();
        bool check_fwdslow_to_stop();
        bool check_fwdslow_to_fwdfast();
        bool check_stop_to_fwdfast();
        bool check_stop_to_fwdslow();

        std::string m_radar_pcl_topic;
        std::string m_radar_track_topic;
        std::string m_out_vel_topic;
        float m_control_cycle_hz;
        float SPEED_LINEAR_FAST;
        float SPEED_LINEAR_SLOW;
        float SPEED_LINEAR_STOP;
        float ZONE_STOP_XLIM;
        float ZONE_SLOW_XLIM;
        float ZONE_YABS;
        // const ros::Duration SCAN_DURATION = ros::Duration(0.1); // TODO: add a logic

        Zone2d_t m_zone_stop;
        Zone2d_t m_zone_slow;
        state_t m_state;
        ros::Time m_state_ts;
        ros::Subscriber m_sub_track;
        ros::Publisher m_pub;
        ros::Timer m_timer;
        ti_mmwave_tracker_rospkg::RadarTrackArray::ConstPtr m_last_radar_trackarray;
        bool m_flag_slow;
        bool m_flag_stop;
};

#endif /* _APP_DETECT_GO_NODE_H_ */