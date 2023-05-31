package us.ihmc.valkyrie.hands.zimmer;

import com.google.common.base.CaseFormat;

import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class ZimmerGripperModel implements HandModel
{
   @Override
   public ZimmerJointName[] getHandJointNames()
   {
      return ZimmerJointName.values;
   }

   public static enum ZimmerJointName implements HandJointName
   {
      gripper_finger1_joint, gripper_finger2_joint;

      public static final ZimmerJointName[] values = values();

      @Override
      public String getJointName(RobotSide robotSide)
      {
         return robotSide.getCamelCaseName() + "_" + name();
      }

      public String getCamelCaseJointName(RobotSide side)
      {
         return side.getCamelCaseName() + CaseFormat.LOWER_UNDERSCORE.to(CaseFormat.UPPER_CAMEL, name());
      }

      public String getPascalCaseJointName(RobotSide side)
      {
         return side.getPascalCaseName() + CaseFormat.LOWER_UNDERSCORE.to(CaseFormat.UPPER_CAMEL, name());
      }

      public ZimmerFingerName getFingerName()
      {
         switch (this)
         {
            case gripper_finger1_joint:
               return ZimmerFingerName.finger1;
            case gripper_finger2_joint:
               return ZimmerFingerName.finger2;
            default:
               throw new IllegalStateException("Unexpected value: " + this);
         }
      }

      @Override
      public int getIndex(RobotSide robotSide)
      {
         return ordinal();
      }
   }

   public static enum ZimmerFingerName
   {
      finger1, finger2;
   }
}
