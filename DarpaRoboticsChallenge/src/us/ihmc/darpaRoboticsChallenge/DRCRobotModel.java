package us.ihmc.darpaRoboticsChallenge;

public enum DRCRobotModel
{
   ATLAS_NO_HANDS, ATLAS_IROBOT_HANDS, ATLAS_SANDIA_HANDS, ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS;
   
   public static DRCRobotModel getDefaultRobotModel()
   {
      return ATLAS_NO_HANDS;
   }
}
