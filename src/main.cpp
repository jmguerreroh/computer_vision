/*
 Copyright (c) 2023 José Miguel Guerrero Hernández

 Licensed under the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0) License;
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     https://creativecommons.org/licenses/by-sa/4.0/

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
*/

#include "computer_vision/CVSubscriber.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto cv_node = std::make_shared<CVSubscriber>();
  rclcpp::spin(cv_node);

  rclcpp::shutdown();
  return 0;
}
