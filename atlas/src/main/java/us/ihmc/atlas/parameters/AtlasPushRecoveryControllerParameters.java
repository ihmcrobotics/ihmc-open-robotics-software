package us.ihmc.atlas.parameters;

import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;

public class AtlasPushRecoveryControllerParameters implements PushRecoveryControllerParameters
{
   private final AtlasMomentumOptimizationSettings momentumOptimizationSettings;

   public AtlasPushRecoveryControllerParameters(RobotTarget target, AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
   {
      momentumOptimizationSettings = new AtlasMomentumOptimizationSettings(jointMap, contactPointParameters.getNumberOfContactableBodies());
   }

   @Override
   public MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      return momentumOptimizationSettings;
   }

   @Override
   public double getMaxStepLength()
   {
      return 1.0;
   }

   @Override
   public double getMaxStepWidth()
   {
      return 0.6;
   }

   @Override
   public double getMinStepWidth()
   {
      return 0.15;
   }

   @Override
   public double getMaxBackwardsStepLength()
   {
      return 0.6;
   }
}
