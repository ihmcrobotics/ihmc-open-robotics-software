package us.ihmc.humanoidRobotics.communication.packets.manipulation;

import us.ihmc.communication.packetAnnotations.ClassDocumentation;
import us.ihmc.tools.DocumentedEnum;
import us.ihmc.robotics.robotSide.RobotSide;

/**
* Created by dstephen on 3/26/15.
*/
@ClassDocumentation("Enum for the various joints in the Atlas forearm. Note the joint names correspond to their\n"
                                  + "orientation in the canonical 'zero pose' where the arms are held out perpendicular to the\n"
                                  + "sides of the robot.")
public enum AtlasElectricMotorPacketEnum implements DocumentedEnum<AtlasElectricMotorPacketEnum>
{
   L_ARM_WRY(0), L_ARM_WRX(1), L_ARM_WRY2(2), R_ARM_WRY(3), R_ARM_WRX(4), R_ARM_WRY2(5);

   public int id;

   AtlasElectricMotorPacketEnum(int id)
   {
      this.id = id;
   }

   public int getId()
   {
      return id;
   }

   @Override
   public String getDocumentation(AtlasElectricMotorPacketEnum var)
   {
      switch(var)
      {
      case L_ARM_WRX:
         return "left wrist roll";
      case L_ARM_WRY:
         return "left upper wrist pitch";
      case L_ARM_WRY2:
         return "left lower wrist pitch";
      case R_ARM_WRX:
         return "right wrist roll";
      case R_ARM_WRY:
         return "right upper wrist pitch";
      case R_ARM_WRY2:
         return "right lower wrist pitch";
      default:
         return "no documentation available";
      }
   }

   public static final AtlasElectricMotorPacketEnum[] values = values();

   @Override
   public AtlasElectricMotorPacketEnum[] getDocumentedValues()
   {
      return values;
   }
   
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
}
