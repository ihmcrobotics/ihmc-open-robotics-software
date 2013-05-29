package us.ihmc.darpaRoboticsChallenge;


public class DRCRobotDrivingControllerParameters extends DRCRobotWalkingControllerParameters
{
   @Override
   public String[] getDefaultHeadOrientationControlJointNames()
   {
      return new String[] {"back_lbz", "neck_ay"};
   }

   @Override
   public String[] getDefaultChestOrientationControlJointNames()
   {
      return new String[] {"back_ubx", "back_mby"};
   }

   @Override
   public String getJointNameForExtendedPitchRange()
   {
      return null;
   }

}
