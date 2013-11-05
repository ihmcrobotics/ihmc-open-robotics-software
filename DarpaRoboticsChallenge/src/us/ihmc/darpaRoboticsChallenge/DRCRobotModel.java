package us.ihmc.darpaRoboticsChallenge;

public enum DRCRobotModel
{
   ATLAS_NO_HANDS, ATLAS_NO_HANDS_ADDED_MASS, ATLAS_IROBOT_HANDS, ATLAS_SANDIA_HANDS, ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS,ATLAS_V3_IROBOT_HANDS;
   
   public static DRCRobotModel getDefaultRobotModel()
   {
      return DRCConfigParameters.robotModelToUse;
   }
   
   public String getModelName()
   {
      switch(this)
      {
      case ATLAS_NO_HANDS:
      case ATLAS_NO_HANDS_ADDED_MASS:
      case ATLAS_IROBOT_HANDS:
      case ATLAS_V3_IROBOT_HANDS:
      case ATLAS_SANDIA_HANDS:
      case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS:
         return "atlas";
      default:
         throw new RuntimeException("Unkown model");
      }
   }
}
