package us.ihmc.humanoidRobotics.communication.packets.dataobjects;

import us.ihmc.tools.DocumentedEnum;

/**
* @author twan
*         Date: 6/8/13
*/
public enum FingerState implements DocumentedEnum<FingerState>
{
   STOP,
   CONNECT,
   OPEN,
   CLOSE,
   CRUSH,
   HOOK,
   BASIC_GRIP,
   PINCH_GRIP,
   WIDE_GRIP,
   SCISSOR_GRIP,
   RESET,
   OPEN_INDEX,
   OPEN_MIDDLE,
   OPEN_FINGERS,
   OPEN_THUMB,
   CLOSE_FINGERS,
   CLOSE_THUMB,
   HALF_CLOSE,
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

   @Override public String getDocumentation(FingerState var)
   {
      switch (var)
      {
      case STOP:
         return "stops the fingers at their current position";
      case OPEN:
         return "fully opens the fingers";
      case CLOSE:
         return "fully closes the fingers";
      case CRUSH:
         return "fully closes the fingers applying maximum force";
      case HOOK:
         return "closes all but one finger to create a hook";
      case BASIC_GRIP:
         return "sets gripper to use a standard grasp";
      case PINCH_GRIP:
         return "sets gripper to use a pinch grasp where the thumb and fingers come together when closed";
      case WIDE_GRIP:
         return "sets gripper to use a wide-spread finger grasp";
      case SCISSOR_GRIP:
         return "sets gripper to use a scissor grasp where the index and middle finger come together when closed";
      case RESET:
         return "sets all fingers to their zero position (usually fully open)";
      case OPEN_FINGERS:
         return "fully open all fingers except the thumb";
      case OPEN_THUMB:
         return "fully open the thumb only";
      case CLOSE_FINGERS:
         return "fully close all fingers except the thumb";
      case CLOSE_THUMB:
         return "fully close the thumb only";
      default:
         return "no documentation available";
      }
   }

   @Override public FingerState[] getDocumentedValues()
   {
      return new FingerState[]
            {
                  STOP,
                  OPEN,
                  CLOSE,
                  CRUSH,
                  HOOK,
                  BASIC_GRIP,
                  PINCH_GRIP,
                  WIDE_GRIP,
                  SCISSOR_GRIP,
                  RESET,
                  OPEN_FINGERS,
                  OPEN_THUMB,
                  CLOSE_FINGERS,
                  CLOSE_THUMB
            };
   }
}
