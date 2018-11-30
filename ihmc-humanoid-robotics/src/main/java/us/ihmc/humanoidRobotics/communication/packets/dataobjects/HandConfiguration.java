package us.ihmc.humanoidRobotics.communication.packets.dataobjects;

import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;

/**
* @author twan
*         Date: 6/8/13
*/
public enum HandConfiguration
{
   @RosEnumValueDocumentation(documentation = "stops the fingers at their current position")
   STOP,
   @RosEnumValueDocumentation(documentation = "fully opens the fingers")
   OPEN,
   @RosEnumValueDocumentation(documentation = "fully closes the fingers")
   CLOSE,
   @RosEnumValueDocumentation(documentation = "fully closes the fingers applying maximum force")
   CRUSH,
   @RosEnumValueDocumentation(documentation = "closes all but one finger to create a hook")
   HOOK,
   @RosEnumValueDocumentation(documentation = "sets gripper to use a standard grasp")
   BASIC_GRIP,
   @RosEnumValueDocumentation(documentation = "sets gripper to use a pinch grasp where the thumb and fingers come together when closed")
   PINCH_GRIP,
   @RosEnumValueDocumentation(documentation = "sets gripper to use a wide-spread finger grasp")
   WIDE_GRIP,
   @RosEnumValueDocumentation(documentation = "sets gripper to use a scissor grasp where the index and middle finger come together when closed")
   SCISSOR_GRIP,
   @RosEnumValueDocumentation(documentation = "sets all fingers to their zero position")
   RESET,
   @RosEnumValueDocumentation(documentation = "fully open all fingers except the thumb")
   OPEN_FINGERS,
   @RosEnumValueDocumentation(documentation = "fully open the thumb only")
   OPEN_THUMB,
   @RosEnumValueDocumentation(documentation = "fully close all fingers except the thumb")
   CLOSE_FINGERS,
   @RosEnumValueDocumentation(documentation = "fully close the thumb only")
   CLOSE_THUMB,
   OPEN_INDEX,
   OPEN_MIDDLE,
   HALF_CLOSE,
   CONNECT,
   CRUSH_INDEX,
   CRUSH_MIDDLE,
   CRUSH_THUMB,
   INVERT_POWER,
   T_SPREAD,
   BEND_BACKWARD,
   CALIBRATE,
   FINGER_MANIPULATION,
   PRE_CREEPY_GRASP,
   PARTIAL_CREEPY_GRASP,
   CREEPY_GRASPING,
   CREEPY_GRASPING_HARD,
   SLOW_CLOSE;

   public final static HandConfiguration[] values = values();

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static HandConfiguration fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
