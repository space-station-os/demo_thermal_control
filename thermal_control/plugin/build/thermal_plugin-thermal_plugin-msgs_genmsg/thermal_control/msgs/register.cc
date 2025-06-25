/*
 * Copyright (C) 2023 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/* This file was automatically generated.
 * Do not edit this directly
 */

#include "gz/msgs/Factory.hh"
#include "gz/msgs/MessageFactory.hh"
#include "thermal_control/msgs/MessageTypes.hh"

#include <array>

namespace {
    using NamedFactoryFn = std::pair<std::string, gz::msgs::MessageFactory::FactoryFn>;

    std::array<NamedFactoryFn, 2> kFactoryFunctions = {{
  {"thermal_control.msgs.ThermalNodeState",
    []()->std::unique_ptr<google::protobuf::Message>{return std::make_unique<thermal_control::msgs::ThermalNodeState>();}},
  {"thermal_control.msgs.ThermalNodeState_V",
    []()->std::unique_ptr<google::protobuf::Message>{return std::make_unique<thermal_control::msgs::ThermalNodeState_V>();}},
}};
}  // namespace

namespace thermal_control::msgs {
int RegisterAll() {
  size_t registered = 0;
  for (const auto &entry: kFactoryFunctions) {
    gz::msgs::Factory::Register(entry.first, entry.second);
    registered++;
  }
  return registered;
}

static int kMessagesRegistered = RegisterAll();
}  // namespace thermal_control::msgs
