package us.ihmc.darpaRoboticsChallenge;


public class DRCRobotMultiContactControllerParameters extends DRCRobotWalkingControllerParameters
{
   @Override
   public String[] getHeadOrientationControlJointNames()
   {
      return new String[] {"neck_ay"}; 
   }

   @Override
   public String[] getChestOrientationControlJointNames()
   {
      return new String[] {"back_lbz", "back_ubx", "back_mby"};
   }

   @Override
   public String getJointNameForExtendedPitchRange()
   {
      return null;
   }

}
