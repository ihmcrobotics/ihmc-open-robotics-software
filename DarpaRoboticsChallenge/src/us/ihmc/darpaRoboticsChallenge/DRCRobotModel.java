package us.ihmc.darpaRoboticsChallenge;

import us.ihmc.atlas.AtlasArmControllerParameters;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.atlas.AtlasPhysicalProperties;
import us.ihmc.atlas.AtlasWalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.darpaRoboticsChallenge.acsell.ACSELLArmControlParameters;
import us.ihmc.darpaRoboticsChallenge.acsell.ACSELLPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.acsell.ACSELLWalkingControllerParameters;
import us.ihmc.darpaRoboticsChallenge.acsell.AxlJointMap;
import us.ihmc.darpaRoboticsChallenge.acsell.BonoJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotPhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.handControl.DRCHandModel;
import us.ihmc.darpaRoboticsChallenge.valkyrie.ValkyrieArmControllerParameters;
import us.ihmc.darpaRoboticsChallenge.valkyrie.ValkyrieJointMap;
import us.ihmc.darpaRoboticsChallenge.valkyrie.ValkyriePhysicalProperties;
import us.ihmc.darpaRoboticsChallenge.valkyrie.ValkyrieWalkingControllerParameters;

public enum DRCRobotModel
{
   ATLAS_NO_HANDS_ADDED_MASS, ATLAS_SANDIA_HANDS, ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS,
   DRC_NO_HANDS, DRC_HANDS, DRC_EXTENDED_HANDS, DRC_HOOKS,  DRC_TASK_HOSE, DRC_EXTENDED_HOOKS,
   VALKYRIE,
   AXL, BONO;
   
   public enum RobotType
   {
      ATLAS, VALKYRIE, ACSELL
   }
   
//   public static DRCRobotModel getDefaultRobotModel()
//   {
//      return DRCLocalConfigParameters.robotModelToUse;
//   }
   
   public ArmControllerParameters getArmControllerParameters()
   {
      switch (this)
      {
      case VALKYRIE:
         return new ValkyrieArmControllerParameters();
         
      case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS:
      case ATLAS_NO_HANDS_ADDED_MASS:
      case ATLAS_SANDIA_HANDS:
      case DRC_EXTENDED_HANDS:
      case DRC_EXTENDED_HOOKS:
      case DRC_HANDS:
      case DRC_HOOKS:
      case DRC_NO_HANDS:
      case DRC_TASK_HOSE:
         return new AtlasArmControllerParameters(DRCLocalConfigParameters.RUNNING_ON_REAL_ROBOT);
         
      case AXL:
      case BONO:
         return new ACSELLArmControlParameters();
         
      default:
         throw new RuntimeException("Unkown model");
      }
   }
   
   public WalkingControllerParameters getWalkingControlParamaters()
   {
      switch (this)
      {
      case VALKYRIE:
         return new ValkyrieWalkingControllerParameters();
         
      case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS:
      case ATLAS_NO_HANDS_ADDED_MASS:
      case ATLAS_SANDIA_HANDS:
      case DRC_EXTENDED_HANDS:
      case DRC_EXTENDED_HOOKS:
      case DRC_HANDS:
      case DRC_HOOKS:
      case DRC_NO_HANDS:
      case DRC_TASK_HOSE:
         return new AtlasWalkingControllerParameters();
         
      case AXL:
      case BONO:
         return new ACSELLWalkingControllerParameters(this);
         
      default:
         throw new RuntimeException("Unkown model");
      }
   }
   
   public DRCRobotPhysicalProperties getPhysicalProperties()
   {
      switch (this)
      {
      case VALKYRIE:
         return new ValkyriePhysicalProperties();
      case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS:
      case ATLAS_NO_HANDS_ADDED_MASS:
      case ATLAS_SANDIA_HANDS:
      case DRC_EXTENDED_HANDS:
      case DRC_EXTENDED_HOOKS:
      case DRC_HANDS:
      case DRC_HOOKS:
      case DRC_NO_HANDS:
      case DRC_TASK_HOSE:
         return new AtlasPhysicalProperties();
      case AXL:
      case BONO:
         return new ACSELLPhysicalProperties();
      default:
         throw new RuntimeException("Unkown model");
      }
   }

   public DRCRobotJointMap getJointMap(boolean addLoadsOfContactPoints, boolean addLoadsOfContactPointsToFeetOnly)
   {
      switch (this)
      {
      case VALKYRIE:
         return new ValkyrieJointMap();

      case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS:
      case ATLAS_NO_HANDS_ADDED_MASS:
      case ATLAS_SANDIA_HANDS:
      case DRC_EXTENDED_HANDS:
      case DRC_EXTENDED_HOOKS:
      case DRC_HANDS:
      case DRC_HOOKS:
      case DRC_NO_HANDS:
      case DRC_TASK_HOSE:
         return new AtlasJointMap(this, addLoadsOfContactPoints, addLoadsOfContactPointsToFeetOnly);
         
      case AXL:
         return new AxlJointMap();
         
      case BONO:
         return new BonoJointMap();
         
      default:
         throw new RuntimeException("Unkown model");
      }
   }
   
   public boolean hasIRobotHands()
   {
      switch (this)
      {
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
      case DRC_TASK_HOSE:
      case DRC_HOOKS:
      case DRC_EXTENDED_HOOKS:
         return true;

      default:
         return false;
      }
   }

   public DRCHandModel getHandModel()
   {
      switch (this)
      {
      case DRC_HANDS:
      case DRC_EXTENDED_HANDS:
      case DRC_TASK_HOSE:
         return DRCHandModel.IROBOT;

      case ATLAS_SANDIA_HANDS:
      case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS:
         return DRCHandModel.SANDIA;

      case ATLAS_NO_HANDS_ADDED_MASS:
      case DRC_NO_HANDS:
      case DRC_HOOKS:
      case DRC_EXTENDED_HOOKS:
      default:
         return DRCHandModel.NONE;
      }
   }

   public RobotType getType()
   {
      switch (this)
      {
      case ATLAS_NO_HANDS_ADDED_MASS:
      case ATLAS_SANDIA_HANDS:
      case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS:
      case DRC_NO_HANDS:
      case DRC_HANDS:
      case DRC_EXTENDED_HANDS:
      case DRC_HOOKS:
      case DRC_TASK_HOSE:
      case DRC_EXTENDED_HOOKS:
         return RobotType.ATLAS;
         
      case VALKYRIE:
         return RobotType.VALKYRIE;
         
      case AXL:
      case BONO:
         return RobotType.ACSELL;

      default:
         throw new RuntimeException("Unkown model");
      }
   }
   
   public String getModelName()
   {
      switch (this)
      {
      case ATLAS_NO_HANDS_ADDED_MASS:
      case ATLAS_SANDIA_HANDS:
      case ATLAS_INVISIBLE_CONTACTABLE_PLANE_HANDS:
      case DRC_NO_HANDS:
      case DRC_HANDS:
      case DRC_EXTENDED_HANDS:
      case DRC_HOOKS:
      case DRC_TASK_HOSE:
      case DRC_EXTENDED_HOOKS:
         return "atlas";
         
      case VALKYRIE:
         return "V1";
         
      case AXL:
         return "axl";
         
      case BONO:
         return "bono";

      default:
         throw new RuntimeException("Unkown model");
      }
   }

}
