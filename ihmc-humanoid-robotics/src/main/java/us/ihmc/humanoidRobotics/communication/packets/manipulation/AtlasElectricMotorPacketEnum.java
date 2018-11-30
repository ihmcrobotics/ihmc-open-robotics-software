package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.ros.generators.RosEnumValueDocumentation;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Enum for the various joints in the Atlas forearm. Note the joint names correspond to their
 * orientation in the canonical 'zero pose' where the arms are held out perpendicular to the
 * sides of the robot.
*/
public enum AtlasElectricMotorPacketEnum
{
   @RosEnumValueDocumentation(documentation = "left wrist roll")
   L_ARM_WRY(0),
   @RosEnumValueDocumentation(documentation = "left upper wrist pitch")
   L_ARM_WRX(1),
   @RosEnumValueDocumentation(documentation = "left lower wrist pitch")
   L_ARM_WRY2(2),
   @RosEnumValueDocumentation(documentation = "right wrist roll")
   R_ARM_WRY(3),
   @RosEnumValueDocumentation(documentation = "right upper wrist pitch")
   R_ARM_WRX(4),
   @RosEnumValueDocumentation(documentation = "right lower wrist pitch")
   R_ARM_WRY2(5);

   public int id;

   AtlasElectricMotorPacketEnum(int id)
   {
      this.id = id;
   }

   public int getId()
   {
      return id;
   }

   public static final AtlasElectricMotorPacketEnum[] values = values();

   public RobotSide getRobotSide()
   {
      switch(this)
      {
      case L_ARM_WRX:
      case L_ARM_WRY:
      case L_ARM_WRY2:
         return RobotSide.LEFT;
         
      case R_ARM_WRX:
      case R_ARM_WRY:
      case R_ARM_WRY2:
         return RobotSide.RIGHT;
         
      default:
         return null;
      }
   }

   public byte toByte()
   {
      return (byte) ordinal();
   }

   public static AtlasElectricMotorPacketEnum fromByte(byte enumAsByte)
   {
      return values[enumAsByte];
   }
}
