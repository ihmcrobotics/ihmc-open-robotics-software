package us.ihmc.valkyrie.configuration;

import us.ihmc.SdfLoader.partNames.NeckJointName;

public abstract class ValkyrieSliderBoardControlledNeckJoints
{  
   public static double getFullyExtendedPositionLimit(NeckJointName joint)
   {
      //Have to do this with IF statements rather than a switch  because in NeckJointName the toString method changes the 
      //strings.
      if(joint == NeckJointName.LOWER_NECK_PITCH)
      {
         return -0.0074;
      }
      else if(joint==NeckJointName.UPPER_NECK_PITCH)
      {
         return -0.03;
      }
      else if(joint==NeckJointName.NECK_YAW)
      {
         return Math.PI/3 - 0.03;
      }
      else
      {
         throw new RuntimeException("Invalid neck joint name.");
      }
   }
   
   public static double getFullyFlexedPositionLimit(NeckJointName joint)
   {
      if(joint==NeckJointName.NECK_YAW)
      {
         return -Math.PI/3 + 0.03;
      }
      else if(joint==NeckJointName.UPPER_NECK_PITCH)
      {
         return 0.66;
      }
      else if(joint==NeckJointName.LOWER_NECK_PITCH)
      {
         return -1.5708;
      }
      else
      {
         throw new RuntimeException("Invalid neck joint name.");
      }
   }
   
   public static NeckJointName[] getNeckJointsControlledBySliderBoard()
   {
      return new NeckJointName[]{
            NeckJointName.UPPER_NECK_PITCH, NeckJointName.LOWER_NECK_PITCH, NeckJointName.NECK_YAW
      };
   }
}
