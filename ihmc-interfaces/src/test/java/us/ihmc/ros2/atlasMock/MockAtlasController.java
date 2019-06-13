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
import controller_msgs.msg.dds.RobotConfigurationDataPubSubType;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.ros2.Ros2Publisher;

import java.io.IOException;

public class MockAtlasController
{
   public static void main(String[] args) throws IOException, InterruptedException
   {
      Ros2Node node = new Ros2Node(PubSubImplementation.FAST_RTPS, "MockAtlasController");
      Ros2Publisher<RobotConfigurationData> publisher = node.createPublisher(new RobotConfigurationDataPubSubType(), "/robot_configuration_data");

      for (int i = 0; true; i++)
      {
         // Preallocate message data structure for packing
         RobotConfigurationData robotConfigurationData = new RobotConfigurationData();

         // Pack message with data
//         robotConfigurationData.getHeader().getStamp().setNanosec(i);
         robotConfigurationData.setMonotonicTime(i);
         System.out.println("Publishing timestamp: " + i);

         // Publish message, thread safe, copies data into another preallocated holder for sending
         publisher.publish(robotConfigurationData);

         Thread.sleep(1000);
      }
   }
}
