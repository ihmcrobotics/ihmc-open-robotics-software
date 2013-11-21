package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.darpaRoboticsChallenge.handControl.DRCHandModel;

public enum DRCRobotModel
{
   ATLAS_NO_HANDS, ATLAS_NO_HANDS_ADDED_MASS, ATLAS_SANDIA_HANDS, ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS, ATLAS_V3_IROBOT_HANDS,
   ATLAS_V3_IROBOT_HANDS_ADDED_MASS, ATLAS_CALIBRATION, ATLAS_IHMC_PARAMETERS;

   public static DRCRobotModel getDefaultRobotModel()
   {
      return DRCLocalConfigParameters.robotModelToUse;
   }

   public boolean hasIRobotHands()
   {
      switch (this)
      {
         case ATLAS_V3_IROBOT_HANDS :
         case ATLAS_V3_IROBOT_HANDS_ADDED_MASS :
         case ATLAS_IHMC_PARAMETERS :
            return true;

         default :
            return false;
      }
   }

   public DRCHandModel getHandModel()
   {
      switch (this)
      {
         case ATLAS_V3_IROBOT_HANDS :
         case ATLAS_V3_IROBOT_HANDS_ADDED_MASS :
         case ATLAS_IHMC_PARAMETERS :
            return DRCHandModel.IROBOT;

         case ATLAS_SANDIA_HANDS :
         case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS :
            return DRCHandModel.SANDIA;

         case ATLAS_NO_HANDS :
         case ATLAS_NO_HANDS_ADDED_MASS :
         case ATLAS_CALIBRATION :
         default :
            return DRCHandModel.NONE;
      }
   }

   public String getModelName()
   {
      switch (this)
      {
         case ATLAS_NO_HANDS :
         case ATLAS_NO_HANDS_ADDED_MASS :
         case ATLAS_V3_IROBOT_HANDS :
         case ATLAS_V3_IROBOT_HANDS_ADDED_MASS :
         case ATLAS_IHMC_PARAMETERS :
         case ATLAS_SANDIA_HANDS :
         case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS :
         case ATLAS_CALIBRATION :
            return "atlas";

         default :
            throw new RuntimeException("Unkown model");
      }
   }

}
