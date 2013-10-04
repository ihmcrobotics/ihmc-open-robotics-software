package us.ihmc.darpaRoboticsChallenge;

public enum DRCRobotModel
{
   ATLAS_NO_HANDS, ATLAS_IROBOT_HANDS, ATLAS_SANDIA_HANDS, ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS, V1;
   
   public static DRCRobotModel getDefaultRobotModel()
   {
      return ATLAS_SANDIA_HANDS;
   }
   
   public String getModelName()
   {
      switch(this)
      {
      case ATLAS_NO_HANDS:
      case ATLAS_IROBOT_HANDS:
      case ATLAS_SANDIA_HANDS:
      case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS:
         return "atlas";
      case V1:
         return "V1";
      default:
         throw new RuntimeException("Unkown model");
      }
   }
}
