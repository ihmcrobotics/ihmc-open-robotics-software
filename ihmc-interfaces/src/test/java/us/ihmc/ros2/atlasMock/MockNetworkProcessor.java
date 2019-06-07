/*
 * Copyright 2017 Florida Institute for Human and Machine Cognition (IHMC)
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
 */
package us.ihmc.ros2.atlasMock;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;

import java.io.IOException;

public class MockNetworkProcessor
{
   public static void main(String[] args) throws IOException, InterruptedException
   {
      Ros2Node node = new Ros2Node(PubSubImplementation.FAST_RTPS, "MockNetworkProcessor");

      // Preallocate message for packing
      RobotConfigurationData robotConfigurationData = new RobotConfigurationData();

      node.createSubscription(RobotConfigurationData.getPubSubType().get(), subscriber -> {
         try
         {
            if (subscriber.takeNextData(robotConfigurationData, null))
            {
               // Access message data
               // long nanosec = robotConfigurationData.getHeader().getStamp().getNanosec();
               long nanosec = robotConfigurationData.getMonotonicTime();
            }
         }
         catch (Exception e)
         {
         }
      }, "/robot_configuration_data");

      Thread.currentThread().join();
   }
}
