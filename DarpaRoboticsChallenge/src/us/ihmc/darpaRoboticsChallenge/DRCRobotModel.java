package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.darpaRoboticsChallenge.handControl.DRCHandModel;

public enum DRCRobotModel
{
//   ATLAS_NO_HANDS, ATLAS_NO_HANDS_ADDED_MASS, ATLAS_SANDIA_HANDS, ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS, ATLAS_IROBOT_HANDS, ATLAS_IROBOT_HANDS_ADDED_MASS, ATLAS_IROBOT_HANDS_WITH_EXTENSION, 
//   ATLAS_IROBOT_HANDS_WITH_EXTENSION_ROTATED_ADDED_MASS, ATLAS_IROBOT_HANDS_ADDED_MASS_COMXZ, ATLAS_IROBOT_HANDS_ADDED_MASS_COMXYZ, ATLAS_CALIBRATION, ATLAS_RHOOK_HAND, ATLAS_IHMC_PARAMETERS, 
//   ATLAS_IROBOT_LEFT_HAND_WITH_EXTENSION_ROTATED_RIGHT_HAND_HOOK_ADDED_MASS,ATLAS_V3_IROBOT_HANDS_RIGHT_8_INCH_EXTENSION_LEFT_4_INCH_ROTATED_ADDED_MASS, ATLAS_HOOK_HANDS, DEBRIS_TASK_MODEL,
   ATLAS_SANDIA_HANDS, ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS,
   DRC_NO_HANDS, DRC_HANDS, DRC_EXTENDED_HANDS, DRC_HOOKS,  DRC_TASK_HOSE;

   public static DRCRobotModel getDefaultRobotModel()
   {
      return DRCLocalConfigParameters.robotModelToUse;
   }

   public boolean hasIRobotHands()
   {
      switch (this)
      {
//      case ATLAS_IROBOT_HANDS:
//      case ATLAS_IROBOT_HANDS_ADDED_MASS:
//      case ATLAS_IROBOT_HANDS_WITH_EXTENSION:
//      case ATLAS_IHMC_PARAMETERS:
//      case ATLAS_IROBOT_HANDS_ADDED_MASS_COMXZ:
//      case ATLAS_IROBOT_HANDS_ADDED_MASS_COMXYZ:
//      case ATLAS_CALIBRATION:
//      case ATLAS_RHOOK_HAND:
//      case ATLAS_IROBOT_HANDS_WITH_EXTENSION_ROTATED_ADDED_MASS:
//      case ATLAS_IROBOT_LEFT_HAND_WITH_EXTENSION_ROTATED_RIGHT_HAND_HOOK_ADDED_MASS:
//      case ATLAS_V3_IROBOT_HANDS_RIGHT_8_INCH_EXTENSION_LEFT_4_INCH_ROTATED_ADDED_MASS:
//      case DEBRIS_TASK_MODEL:
         
      case DRC_TASK_HOSE:
      case DRC_HANDS:
      case DRC_EXTENDED_HANDS:
         return true;

      default:
         return false;
      }
   }

   public boolean hasIRobotHandsOnExtensions()
   {
      switch (this)
      {
//      case ATLAS_IROBOT_HANDS_WITH_EXTENSION:
//      case ATLAS_IROBOT_HANDS_WITH_EXTENSION_ROTATED_ADDED_MASS:
//      case ATLAS_V3_IROBOT_HANDS_RIGHT_8_INCH_EXTENSION_LEFT_4_INCH_ROTATED_ADDED_MASS:
//      case DEBRIS_TASK_MODEL:
         
      case DRC_EXTENDED_HANDS:
         return true;

      default:
         return false;
      }
   }

   public boolean hasHookHands()
   {
      switch (this)
      {
//      case ATLAS_RHOOK_HAND:
//      case ATLAS_HOOK_HANDS:
         
      case DRC_TASK_HOSE:
      case DRC_HOOKS:
         return true;

      default:
         return false;
      }
   }

   public DRCHandModel getHandModel()
   {
      switch (this)
      {
//      case ATLAS_IROBOT_HANDS:
//      case ATLAS_IROBOT_HANDS_ADDED_MASS:
//      case ATLAS_IROBOT_HANDS_WITH_EXTENSION:
//      case ATLAS_IHMC_PARAMETERS:
//      case ATLAS_IROBOT_HANDS_ADDED_MASS_COMXZ:
//      case ATLAS_IROBOT_HANDS_ADDED_MASS_COMXYZ:
//      case ATLAS_CALIBRATION:
//      case ATLAS_RHOOK_HAND:
//      case ATLAS_IROBOT_HANDS_WITH_EXTENSION_ROTATED_ADDED_MASS:
//      case ATLAS_IROBOT_LEFT_HAND_WITH_EXTENSION_ROTATED_RIGHT_HAND_HOOK_ADDED_MASS:
//      case ATLAS_V3_IROBOT_HANDS_RIGHT_8_INCH_EXTENSION_LEFT_4_INCH_ROTATED_ADDED_MASS:
//      case DEBRIS_TASK_MODEL:
         
      case DRC_HANDS:
      case DRC_EXTENDED_HANDS:
      case DRC_TASK_HOSE:
         return DRCHandModel.IROBOT;

      case ATLAS_SANDIA_HANDS:
      case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS:
         return DRCHandModel.SANDIA;

//      case ATLAS_NO_HANDS:
//      case ATLAS_NO_HANDS_ADDED_MASS:
         
      case DRC_NO_HANDS:
      case DRC_HOOKS:
      default:
         return DRCHandModel.NONE;
      }
   }

   public String getModelName()
   {
      switch (this)
      {
//      case ATLAS_NO_HANDS:
//      case ATLAS_NO_HANDS_ADDED_MASS:
//      case ATLAS_IROBOT_HANDS:
//      case ATLAS_IROBOT_HANDS_ADDED_MASS:
//      case ATLAS_IROBOT_HANDS_WITH_EXTENSION:
//      case ATLAS_IROBOT_HANDS_ADDED_MASS_COMXZ:
//      case ATLAS_IROBOT_HANDS_ADDED_MASS_COMXYZ:
//      case ATLAS_IHMC_PARAMETERS:
      case ATLAS_SANDIA_HANDS:
      case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS:
//      case ATLAS_CALIBRATION:
//      case ATLAS_RHOOK_HAND:
//      case ATLAS_IROBOT_HANDS_WITH_EXTENSION_ROTATED_ADDED_MASS:
//      case ATLAS_V3_IROBOT_HANDS_RIGHT_8_INCH_EXTENSION_LEFT_4_INCH_ROTATED_ADDED_MASS:
//      case DEBRIS_TASK_MODEL:
//      case ATLAS_HOOK_HANDS:
//         
      case DRC_NO_HANDS:
      case DRC_HANDS:
      case DRC_EXTENDED_HANDS:
      case DRC_HOOKS:
      case DRC_TASK_HOSE:
         return "atlas";

      default:
         throw new RuntimeException("Unkown model");
      }
   }

}
