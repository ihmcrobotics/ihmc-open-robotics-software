package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros.generators.RosExportedField;
import us.ihmc.communication.ros.generators.RosMessagePacket;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.robotSide.RobotSide;

@RosMessagePacket(documentation = "Packet for commanding the hands to perform various predefined grasps."
      + " A message with a unique id equals to 0 will be interpreted as invalid and will not be processed by the controller.", rosPackage = RosMessagePacket.CORE_IHMC_PACKAGE, topic = "/control/hand_desired_configuration")
public class HandDesiredConfigurationMessage extends Packet<HandDesiredConfigurationMessage>
{
   public static final byte ROBOT_SIDE_LEFT = 0;
   public static final byte ROBOT_SIDE_RIGHT = 1;

   public static final byte HAND_CONFIGURATION_STOP = 0;
   public static final byte HAND_CONFIGURATION_OPEN = 1;
   public static final byte HAND_CONFIGURATION_CLOSE = 2;
   public static final byte HAND_CONFIGURATION_CRUSH = 3;
   public static final byte HAND_CONFIGURATION_HOOK = 4;
   public static final byte HAND_CONFIGURATION_BASIC_GRIP = 5;
   public static final byte HAND_CONFIGURATION_PINCH_GRIP = 6;
   public static final byte HAND_CONFIGURATION_WIDE_GRIP = 7;
   public static final byte HAND_CONFIGURATION_SCISSOR_GRIP = 8;
   public static final byte HAND_CONFIGURATION_RESET = 9;
   public static final byte HAND_CONFIGURATION_OPEN_FINGERS = 10;
   public static final byte HAND_CONFIGURATION_OPEN_THUMB = 11;
   public static final byte HAND_CONFIGURATION_CLOSE_FINGERS = 12;
   public static final byte HAND_CONFIGURATION_CLOSE_THUMB = 13;
   public static final byte HAND_CONFIGURATION_OPEN_INDEX = 14;
   public static final byte HAND_CONFIGURATION_OPEN_MIDDLE = 15;
   public static final byte HAND_CONFIGURATION_HALF_CLOSE = 16;
   public static final byte HAND_CONFIGURATION_CONNECT = 17;
   public static final byte HAND_CONFIGURATION_CRUSH_INDEX = 18;
   public static final byte HAND_CONFIGURATION_CRUSH_MIDDLE = 19;
   public static final byte HAND_CONFIGURATION_CRUSH_THUMB = 20;
   public static final byte HAND_CONFIGURATION_INVERT_POWER = 21;
   public static final byte HAND_CONFIGURATION_T_SPREAD = 22;
   public static final byte HAND_CONFIGURATION_BEND_BACKWARD = 23;
   public static final byte HAND_CONFIGURATION_CALIBRATE = 24;
   public static final byte HAND_CONFIGURATION_FINGER_MANIPULATION = 25;
   public static final byte HAND_CONFIGURATION_PRE_CREEPY_GRASP = 26;
   public static final byte HAND_CONFIGURATION_PARTIAL_CREEPY_GRASP = 27;
   public static final byte HAND_CONFIGURATION_CREEPY_GRASPING = 28;
   public static final byte HAND_CONFIGURATION_CREEPY_GRASPING_HARD = 29;
   public static final byte HAND_CONFIGURATION_SLOW_CLOSE = 30;

   @RosExportedField(documentation = "Specifies the side of the robot that will execute the trajectory")
   public byte robotSide;
   @RosExportedField(documentation = "Specifies the grasp to perform")
   public byte desiredHandConfiguration;

   /**
    * Empty constructor for serialization. Set the id of the message to
    * {@link Packet#VALID_MESSAGE_DEFAULT_ID}.
    */
   public HandDesiredConfigurationMessage()
   {
      setUniqueId(VALID_MESSAGE_DEFAULT_ID);
   }

   @Override
   public void set(HandDesiredConfigurationMessage other)
   {
      robotSide = other.robotSide;
      desiredHandConfiguration = other.desiredHandConfiguration;
      setPacketInformation(other);
   }

   public byte getDesiredHandConfiguration()
   {
      return desiredHandConfiguration;
   }

   public byte getRobotSide()
   {
      return robotSide;
   }

   @Override
   public boolean equals(Object other)
   {
      return ((other instanceof HandDesiredConfigurationMessage) && this.epsilonEquals((HandDesiredConfigurationMessage) other, 0));
   }

   @Override
   public String toString()
   {
      return RobotSide.fromByte(robotSide).toString() + " State= " + HandConfiguration.fromByte(desiredHandConfiguration).toString();
   }

   @Override
   public boolean epsilonEquals(HandDesiredConfigurationMessage other, double epsilon)
   {
      boolean ret = (this.getRobotSide() == other.getRobotSide());
      ret &= (this.getDesiredHandConfiguration() == other.getDesiredHandConfiguration());

      return ret;
   }
}
