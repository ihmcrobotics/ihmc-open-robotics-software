package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;

public enum BaseForControl
{
   @RosEnumValueDocumentation(documentation = "The hand is controlled with respect to the chest. In other words, the controlled hand moves along with the chest.")
   CHEST,
   @RosEnumValueDocumentation(documentation = "The hand is controlled with respect to the estimated world. In other words, the controlled hand will remain fixed in world even if the robot starts moving.")
   WORLD,
   @RosEnumValueDocumentation(documentation = "The hand is controlled with respect to the middle of the feet. In other words, the controlled hand moves along with the robot when walking but is not affected by swaying.")
   WALKING_PATH
}
