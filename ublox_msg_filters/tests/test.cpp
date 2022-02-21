/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <gtest/gtest.h>

#include "ros/ros.h"
#include "ublox_msg_filters/exact_time.h"

using namespace ublox_msg_filters;

struct Msg
{
  u_int32_t iTOW;
  int data;
};
typedef boost::shared_ptr<Msg> MsgPtr;
typedef boost::shared_ptr<Msg const> MsgConstPtr;

class Helper
{
public:
  Helper() {}

  void cb()
  {
    ++count_;
  }

  void cb3(const MsgConstPtr &msg1, const MsgConstPtr &msg2, const MsgConstPtr &msg3)
  {
    ++count_;
    stamp1_ = msg1->iTOW;
    stamp2_ = msg2->iTOW;
    stamp3_ = msg3->iTOW;
  }

  void dropcb()
  {
    ++drop_count_;
  }

  int32_t count_ = 0;
  int32_t drop_count_ = 0;
  uint32_t stamp1_ = 0;
  uint32_t stamp2_ = 0;
  uint32_t stamp3_ = 0;
};

typedef ublox_msg_filters::ExactTime<Msg, Msg> Policy2;
typedef ublox_msg_filters::ExactTime<Msg, Msg, Msg> Policy3;
typedef Synchronizer<Policy2> Sync2;
typedef Synchronizer<Policy3> Sync3;

//////////////////////////////////////////////////////////////////////////////////////////////////
// From here on we assume that testing the 3-message version is sufficient, so as not to duplicate
// tests for everywhere from 2-9
//////////////////////////////////////////////////////////////////////////////////////////////////
TEST(ExactTime, multipleTimes)
{
  Sync3 sync(2);
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb3, &h, _1, _2, _3));
  MsgPtr m(boost::make_shared<Msg>());
  m->iTOW = 0;

  sync.add<0>(m);
  ASSERT_EQ(h.count_, 0);

  m = boost::make_shared<Msg>();
  m->iTOW = 1;
  sync.add<1>(m);
  ASSERT_EQ(h.count_, 0);
  sync.add<0>(m);
  ASSERT_EQ(h.count_, 0);
  sync.add<2>(m);
  ASSERT_EQ(h.count_, 1);
  ASSERT_EQ(1, h.stamp1_);
  ASSERT_EQ(1, h.stamp2_);
  ASSERT_EQ(1, h.stamp3_);
}

TEST(ExactTime, queueSize)
{
  Sync3 sync(1);
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  MsgPtr m(boost::make_shared<Msg>());
  m->iTOW = 0;

  sync.add<0>(m);
  ASSERT_EQ(h.count_, 0);
  sync.add<1>(m);
  ASSERT_EQ(h.count_, 0);

  m = boost::make_shared<Msg>();
  m->iTOW = 1;
  sync.add<1>(m);
  ASSERT_EQ(h.count_, 0);

  m = boost::make_shared<Msg>();
  m->iTOW = 0;
  sync.add<1>(m);
  ASSERT_EQ(h.count_, 0);
  sync.add<2>(m);
  ASSERT_EQ(h.count_, 0);
}

TEST(ExactTime, dropCallback)
{
  Sync2 sync(1);
  Helper h;
  sync.registerCallback(boost::bind(&Helper::cb, &h));
  sync.getPolicy()->registerDropCallback(boost::bind(&Helper::dropcb, &h));
  MsgPtr m(boost::make_shared<Msg>());

  m->iTOW = 0;

  sync.add<0>(m);
  ASSERT_EQ(h.drop_count_, 0);
  m->iTOW = 1;
  sync.add<0>(m);

  ASSERT_EQ(h.drop_count_, 1);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "blah");

  ros::Time::init();
  ros::Time::setNow(ros::Time());

  return RUN_ALL_TESTS();
}
