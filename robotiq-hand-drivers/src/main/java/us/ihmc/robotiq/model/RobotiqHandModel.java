package us.ihmc.robotiq.model;

import us.ihmc.robotics.partNames.FingerName;
import us.ihmc.avatar.handControl.packetsAndConsumers.HandModel;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class RobotiqHandModel implements HandModel
{
   public enum RobotiqHandJointNameMinimal implements HandJointName
   {
	  PALM_FINGER_1_JOINT, FINGER_1_JOINT_1, FINGER_1_JOINT_2, FINGER_1_JOINT_3,
     PALM_FINGER_2_JOINT, FINGER_2_JOINT_1, FINGER_2_JOINT_2, FINGER_2_JOINT_3,
     FINGER_MIDDLE_JOINT_1, FINGER_MIDDLE_JOINT_2, FINGER_MIDDLE_JOINT_3; //palm_finger_middle_joint is fixed

     public static final RobotiqHandJointNameMinimal[] values = RobotiqHandJointNameMinimal.values();

      public int getHandJointAngleIndex()
      {
         switch (this)
         {
            case PALM_FINGER_1_JOINT:
            case PALM_FINGER_2_JOINT:
            case FINGER_MIDDLE_JOINT_1: //fixed
               return 0;
            case FINGER_1_JOINT_1:
            case FINGER_2_JOINT_1:
            case FINGER_MIDDLE_JOINT_2:
               return 1;
            case FINGER_1_JOINT_2:
            case FINGER_2_JOINT_2:
            case FINGER_MIDDLE_JOINT_3:
               return 2;
            case FINGER_1_JOINT_3:
            case FINGER_2_JOINT_3:
               return 3;
            default:
               return -1;
         }
      }

      @Override
      public FingerName getFinger(RobotSide robotSide)
      {
         switch (this)
         {
            case PALM_FINGER_1_JOINT:
            case FINGER_1_JOINT_1:
            case FINGER_1_JOINT_2:
            case FINGER_1_JOINT_3:
               return robotSide == RobotSide.LEFT ? FingerName.MIDDLE : FingerName.INDEX;
            case PALM_FINGER_2_JOINT:
            case FINGER_2_JOINT_1:
            case FINGER_2_JOINT_2:
            case FINGER_2_JOINT_3:
               return robotSide == RobotSide.LEFT ? FingerName.INDEX : FingerName.MIDDLE;
            case FINGER_MIDDLE_JOINT_1:
            case FINGER_MIDDLE_JOINT_2:
            case FINGER_MIDDLE_JOINT_3:
               return FingerName.THUMB;
         }
         return null;
      }

      @Override
      public String getJointName(RobotSide robotSide)
      {
         return robotSide.getShortLowerCaseName() + "_" + this.toString().toLowerCase();
      }

      @Override
      public HandJointName[] getValues()
      {
         return values;
      }
   }

   @Override
   public HandJointName[] getHandJointNames()
   {
      return RobotiqHandJointNameMinimal.values;
   }
}
