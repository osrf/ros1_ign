// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#ifndef SERVICE_FACTORY_HPP_
#define SERVICE_FACTORY_HPP_

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <gz/transport/Node.hh>

#include <rclcpp/rclcpp.hpp>

#include "ros_gz_bridge/convert_decl.hpp"

#include "service_factory_interface.hpp"

namespace ros_gz_bridge
{

template<typename RosResT>
bool
send_response_on_error(RosResT & ros_response);

<<<<<<< HEAD
template<typename RosServiceT, typename IgnRequestT, typename IgnReplyT>
=======
template<typename RosServiceT, typename GzRequestT, typename GzReplyT>
>>>>>>> 63b651a (Garden EOL (#662))
class ServiceFactory : public ServiceFactoryInterface
{
public:
  ServiceFactory(
    const std::string & ros_type_name, const std::string & gz_req_type_name,
    const std::string & gz_rep_type_name)
  : ros_type_name_(ros_type_name),
    gz_req_type_name_(gz_req_type_name),
    gz_rep_type_name_(gz_rep_type_name)
  {}

  rclcpp::ServiceBase::SharedPtr
  create_ros_service(
    rclcpp::Node::SharedPtr ros_node,
    std::shared_ptr<gz::transport::Node> gz_node,
    const std::string & service_name) override
  {
    return ros_node->create_service<RosServiceT>(
      service_name,
      [gz_node = std::move(gz_node), service_name](
        std::shared_ptr<rclcpp::Service<RosServiceT>> srv_handle,
        std::shared_ptr<rmw_request_id_t> reqid,
        std::shared_ptr<typename RosServiceT::Request> ros_req)
      {
<<<<<<< HEAD
        std::function<void(const IgnReplyT &, bool)> callback;
=======
        std::function<void(const GzReplyT &, bool)> callback;
>>>>>>> 63b651a (Garden EOL (#662))
        callback = [
          srv_handle = std::move(srv_handle),
          reqid
        ](
<<<<<<< HEAD
          const IgnReplyT & reply,
=======
          const GzReplyT & reply,
>>>>>>> 63b651a (Garden EOL (#662))
          const bool result)
        {
          typename RosServiceT::Response ros_res;
          if (!result) {
            if (send_response_on_error(ros_res)) {
              srv_handle->send_response(*reqid, ros_res);
            }
          }
          convert_gz_to_ros(reply, ros_res);
          srv_handle->send_response(*reqid, ros_res);
        };
<<<<<<< HEAD
        IgnRequestT gz_req;
=======
        GzRequestT gz_req;
>>>>>>> 63b651a (Garden EOL (#662))
        convert_ros_to_gz(*ros_req, gz_req);
        gz_node->Request(
          service_name,
          gz_req,
          callback);
      });
  }

private:
  std::string ros_type_name_;
  std::string gz_req_type_name_;
  std::string gz_rep_type_name_;
};

}  // namespace ros_gz_bridge

#endif  // SERVICE_FACTORY_HPP_
