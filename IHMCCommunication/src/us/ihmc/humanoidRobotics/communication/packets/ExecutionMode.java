package us.ihmc.humanoidRobotics.communication.packets;

import us.ihmc.communication.annotations.ros.RosEnumValueDocumentation;

public enum ExecutionMode
{
   @RosEnumValueDocumentation(documentation = "This message will override the previous.")
   OVERRIDE,
   @RosEnumValueDocumentation(documentation = "The previous message will first be executed before executing this message. When sending a series of queued messages, the very first has to be declared as OVERRIDE.")
   QUEUE
}
