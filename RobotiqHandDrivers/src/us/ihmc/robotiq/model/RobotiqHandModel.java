package us.ihmc.robotiq.model;

import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandJointName;
import us.ihmc.darpaRoboticsChallenge.handControl.packetsAndConsumers.HandModel;
import us.ihmc.utilities.humanoidRobot.partNames.FingerName;

public class RobotiqHandModel implements HandModel
{
	//TODO: All these names need to change
	//assumes a right hand convention for index, middle
	public enum RobotiqHandJointNameMinimal implements HandJointName
   {
	  PALM_FINGER_1_JOINT, FINGER_1_JOINT_1, FINGER_1_JOINT_2, FINGER_1_JOINT_3,
      PALM_FINGER_2_JOINT, FINGER_2_JOINT_1, FINGER_2_JOINT_2, FINGER_2_JOINT_3,
      FINGER_MIDDLE_JOINT_1, FINGER_MIDDLE_JOINT_2, FINGER_MIDDLE_JOINT_3; //palm_finger_middle_joint is fixed
      
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
      
      public FingerName getFinger()
      {
         switch (this)
         {
         case PALM_FINGER_1_JOINT:
         case FINGER_1_JOINT_1:
         case FINGER_1_JOINT_2:
         case FINGER_1_JOINT_3:
            return FingerName.INDEX;
         case PALM_FINGER_2_JOINT:
         case FINGER_2_JOINT_1:
         case FINGER_2_JOINT_2:
         case FINGER_2_JOINT_3:
            return FingerName.MIDDLE;
         case FINGER_MIDDLE_JOINT_1:
         case FINGER_MIDDLE_JOINT_2:
         case FINGER_MIDDLE_JOINT_3:
            return FingerName.THUMB;
         }
         return null;
      }

      public String toLowerCase()
      {
         return this.toString().toLowerCase();
      }

      @Override
      public HandJointName[] getValues()
      {
    	  return RobotiqHandJointNameMinimal.values();
      }
   }
	
	@Override
	public HandJointName[] getHandJointNames()
	{
		return RobotiqHandJointNameMinimal.values();
	}

}
