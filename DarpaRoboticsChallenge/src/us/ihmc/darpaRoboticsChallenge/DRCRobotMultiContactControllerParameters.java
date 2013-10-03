package us.ihmc.darpaRoboticsChallenge;

import static us.ihmc.darpaRoboticsChallenge.ros.ROSAtlasJointMap.*;


public class DRCRobotMultiContactControllerParameters extends DRCRobotWalkingControllerParameters
{
   @Override
   public String[] getDefaultHeadOrientationControlJointNames()
   {
      return new String[] {jointNames[neck_ay]}; 
   }

   @Override
   public String[] getDefaultChestOrientationControlJointNames()
   {
      return new String[] {jointNames[back_lbz], jointNames[back_ubx], jointNames[back_mby]};
   }

   @Override
   public String getJointNameForExtendedPitchRange()
   {
      return null;
   }

}
