// Copyright 2024 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity.pb.h>

#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>

#include <gz/transport/Node.hh>

// Simple application that provides a `/remove` service and prints out the
// request's entity name. This works in conjuction with
// test_remove_node.launch.py
int main()
{
  std::mutex m;
  std::condition_variable cv;
  bool test_complete = false;

  gz::transport::Node node;
  auto cb = std::function(
    [&](
      const gz::msgs::Entity & _req,
      gz::msgs::Boolean & _res) -> bool {
      std::cout << _req.name() << std::endl;
      _res.set_data(true);

      {
        std::lock_guard<std::mutex> lk(m);
        test_complete = true;
      }
      cv.notify_one();
      return true;
    });

  node.Advertise("/world/default/remove", cb);
  // wait until we receive a message.
  std::unique_lock<std::mutex> lk(m);
  cv.wait(lk, [&] {return test_complete;});
  // Sleep so that the service response can be sent before exiting.
  std::this_thread::sleep_for(std::chrono::seconds(1));
}
