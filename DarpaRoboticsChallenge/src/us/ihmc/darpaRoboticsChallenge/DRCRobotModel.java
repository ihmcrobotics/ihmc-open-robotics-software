package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.darpaRoboticsChallenge.handControl.DRCHandModel;

public enum DRCRobotModel
{
   ATLAS_NO_HANDS, ATLAS_NO_HANDS_ADDED_MASS, ATLAS_IROBOT_HANDS, ATLAS_SANDIA_HANDS, ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS,ATLAS_V3_IROBOT_HANDS;
   
   public static DRCRobotModel getDefaultRobotModel()
   {
      return DRCLocalConfigParameters.robotModelToUse;
   }
   
   public DRCHandModel getHandModel()
   {
	   switch(this)
	      {
	      case ATLAS_IROBOT_HANDS:
	      case ATLAS_V3_IROBOT_HANDS:
	    	  return DRCHandModel.IROBOT;
	    	  
	      case ATLAS_SANDIA_HANDS:
	      case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS:
	    	  return DRCHandModel.SANDIA;
	    	  
	      case ATLAS_NO_HANDS:
	      case ATLAS_NO_HANDS_ADDED_MASS:
	      default:
	    	  return DRCHandModel.NONE;
	      }
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
