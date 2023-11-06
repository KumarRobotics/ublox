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

#include <rclcpp/rclcpp.hpp>
#include "message_filters/synchronizer.h"
#include "ublox_msg_filters/exact_time.h"

using namespace ublox_msg_filters::sync_policies;

struct Msg
{
  uint32_t i_tow;
  int data;
};
typedef std::shared_ptr<Msg> MsgPtr;
typedef std::shared_ptr<Msg const> MsgConstPtr;

class Helper
{
public:
  Helper()
  : count_(0)
  , drop_count_(0)
  {}

  void cb()
  {
    ++count_;
  }

  void dropcb()
  {
    ++drop_count_;
  }

  int32_t count_;
  int32_t drop_count_;
};

typedef ExactTime<Msg, Msg> Policy2;
typedef ExactTime<Msg, Msg, Msg> Policy3;
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
  sync.registerCallback(std::bind(&Helper::cb, &h));
  MsgPtr m(std::make_shared<Msg>());
  m->i_tow = 0;

  sync.add<0>(m);
  ASSERT_EQ(h.count_, 0);

  m = std::make_shared<Msg>();
  m->i_tow = 100000000;
  sync.add<1>(m);
  ASSERT_EQ(h.count_, 0);
  sync.add<0>(m);
  ASSERT_EQ(h.count_, 0);
  sync.add<2>(m);
  ASSERT_EQ(h.count_, 1);
}

TEST(ExactTime, queueSize)
{
  Sync3 sync(1);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h));
  MsgPtr m(std::make_shared<Msg>());
  m->i_tow = 0;

  sync.add<0>(m);
  ASSERT_EQ(h.count_, 0);
  sync.add<1>(m);
  ASSERT_EQ(h.count_, 0);

  m = std::make_shared<Msg>();
  m->i_tow = 100000000;
  sync.add<1>(m);
  ASSERT_EQ(h.count_, 0);

  m = std::make_shared<Msg>();
  m->i_tow = 0;
  sync.add<1>(m);
  ASSERT_EQ(h.count_, 0);
  sync.add<2>(m);
  ASSERT_EQ(h.count_, 0);
}

TEST(ExactTime, dropCallback)
{
  Sync2 sync(1);
  Helper h;
  sync.registerCallback(std::bind(&Helper::cb, &h));
  sync.getPolicy()->registerDropCallback(std::bind(&Helper::dropcb, &h));
  MsgPtr m(std::make_shared<Msg>());
  m->i_tow = 0;

  sync.add<0>(m);
  ASSERT_EQ(h.drop_count_, 0);
  m->i_tow = 100000000;
  sync.add<0>(m);

  ASSERT_EQ(h.drop_count_, 1);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  return RUN_ALL_TESTS();
}


