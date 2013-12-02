package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.darpaRoboticsChallenge.handControl.DRCHandModel;

public enum DRCRobotModel
{
   ATLAS_NO_HANDS, ATLAS_NO_HANDS_ADDED_MASS, ATLAS_SANDIA_HANDS, ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS, ATLAS_IROBOT_HANDS,
   ATLAS_IROBOT_HANDS_ADDED_MASS,  ATLAS_IROBOT_HANDS_WITH_EXTENSION, ATLAS_IROBOT_HANDS_WITH_EXTENSION_ROTATED, ATLAS_IROBOT_HANDS_ADDED_MASS_COMXZ, ATLAS_IROBOT_HANDS_ADDED_MASS_COMXYZ,ATLAS_CALIBRATION,ATLAS_RHOOK_HAND, ATLAS_IHMC_PARAMETERS, ATLAS_;

   public static DRCRobotModel getDefaultRobotModel()
   {
      return DRCLocalConfigParameters.robotModelToUse;
   }
   
   public boolean hasIRobotHands()
   {
      switch (this)
      {
         case ATLAS_IROBOT_HANDS :
         case ATLAS_IROBOT_HANDS_ADDED_MASS :
         case ATLAS_IROBOT_HANDS_WITH_EXTENSION:
         case ATLAS_IHMC_PARAMETERS:
         case ATLAS_IROBOT_HANDS_ADDED_MASS_COMXZ:
         case ATLAS_IROBOT_HANDS_ADDED_MASS_COMXYZ:
         case ATLAS_CALIBRATION :
         case ATLAS_RHOOK_HAND:
         case ATLAS_IROBOT_HANDS_WITH_EXTENSION_ROTATED:
            return true;

         default :
            return false;
      }
   }

   public DRCHandModel getHandModel()
   {
      switch (this)
      {
         case ATLAS_IROBOT_HANDS :
         case ATLAS_IROBOT_HANDS_ADDED_MASS :
         case ATLAS_IROBOT_HANDS_WITH_EXTENSION:
         case ATLAS_IHMC_PARAMETERS :
         case ATLAS_IROBOT_HANDS_ADDED_MASS_COMXZ:
         case ATLAS_IROBOT_HANDS_ADDED_MASS_COMXYZ:
         case ATLAS_CALIBRATION :
         case ATLAS_RHOOK_HAND:
         case ATLAS_IROBOT_HANDS_WITH_EXTENSION_ROTATED:
            return DRCHandModel.IROBOT;

         case ATLAS_SANDIA_HANDS :
         case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS :
            return DRCHandModel.SANDIA;

         case ATLAS_NO_HANDS :
         case ATLAS_NO_HANDS_ADDED_MASS :
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
         case ATLAS_IROBOT_HANDS :
         case ATLAS_IROBOT_HANDS_ADDED_MASS :
         case ATLAS_IROBOT_HANDS_WITH_EXTENSION:
         case ATLAS_IROBOT_HANDS_ADDED_MASS_COMXZ:
         case ATLAS_IROBOT_HANDS_ADDED_MASS_COMXYZ:
         case ATLAS_IHMC_PARAMETERS :
         case ATLAS_SANDIA_HANDS :
         case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS :
         case ATLAS_CALIBRATION :
         case ATLAS_RHOOK_HAND:
         case ATLAS_IROBOT_HANDS_WITH_EXTENSION_ROTATED:
            return "atlas";

         default :
            throw new RuntimeException("Unkown model");
      }
   }

}
