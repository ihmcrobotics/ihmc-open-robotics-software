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

public class AtlasPushRecoveryControllerParameters extends PushRecoveryControllerParameters
{
   private final AtlasJointMap jointMap;
   private final AtlasMomentumOptimizationSettings momentumOptimizationSettings;

   private AtlasSteppingParameters steppingParameters;

   private final JointLimitParameters spineJointLimitParameters;
   private final JointLimitParameters kneeJointLimitParameters;
   private final JointLimitParameters ankleJointLimitParameters;

   public AtlasPushRecoveryControllerParameters(RobotTarget target, AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
   {
      this.jointMap = jointMap;

      momentumOptimizationSettings = new AtlasMomentumOptimizationSettings(jointMap, contactPointParameters.getNumberOfContactableBodies());

      steppingParameters = new AtlasSteppingParameters(jointMap);

      spineJointLimitParameters = new JointLimitParameters();
      spineJointLimitParameters.setMaxAbsJointVelocity(9.0);
      spineJointLimitParameters.setJointLimitDistanceForMaxVelocity(Math.toRadians(30.0));
      spineJointLimitParameters.setJointLimitFilterBreakFrequency(15.0);
      spineJointLimitParameters.setVelocityControlGain(30.0);

      kneeJointLimitParameters = new JointLimitParameters();
      kneeJointLimitParameters.setMaxAbsJointVelocity(5.0);
      kneeJointLimitParameters.setJointLimitDistanceForMaxVelocity(Math.toRadians(30.0));
      kneeJointLimitParameters.setJointLimitFilterBreakFrequency(15.0);
      kneeJointLimitParameters.setVelocityControlGain(60.0);
      kneeJointLimitParameters.setVelocityDeadbandSize(0.6);

      ankleJointLimitParameters = new JointLimitParameters();
      ankleJointLimitParameters.setMaxAbsJointVelocity(5.0);
      ankleJointLimitParameters.setJointLimitDistanceForMaxVelocity(Math.toRadians(20.0));
      ankleJointLimitParameters.setJointLimitFilterBreakFrequency(10.0);
      ankleJointLimitParameters.setVelocityControlGain(90.0);
      ankleJointLimitParameters.setVelocityDeadbandSize(0.6);
      ankleJointLimitParameters.setRangeOfMotionMarginFraction(0.02);
   }

   @Override
   public MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      return momentumOptimizationSettings;
   }

   /** {@inheritDoc} */
   @Override
   public String[] getJointsWithRestrictiveLimits()
   {
      String bkxName = jointMap.getSpineJointName(SpineJointName.SPINE_ROLL);
      String bkyName = jointMap.getSpineJointName(SpineJointName.SPINE_PITCH);
      String leftKnyName = jointMap.getLegJointName(RobotSide.LEFT, LegJointName.KNEE_PITCH);
      String rightKnyName = jointMap.getLegJointName(RobotSide.RIGHT, LegJointName.KNEE_PITCH);
      String leftAkyName = jointMap.getLegJointName(RobotSide.LEFT, LegJointName.ANKLE_PITCH);
      String rightAkyName = jointMap.getLegJointName(RobotSide.RIGHT, LegJointName.ANKLE_PITCH);
      String[] joints = {bkxName, bkyName, leftKnyName, rightKnyName, leftAkyName, rightAkyName};
      return joints;
   }

   /** {@inheritDoc} */
   @Override
   public JointLimitParameters getJointLimitParametersForJointsWithRestrictiveLimits(String jointName)
   {
      if (jointMap.getSpineJointName(jointName) == SpineJointName.SPINE_ROLL || jointMap.getSpineJointName(jointName) == SpineJointName.SPINE_PITCH)
         return spineJointLimitParameters;
      else if (jointMap.getLegJointName(jointName) != null)
      {
         if (jointMap.getLegJointName(jointName).getRight() == LegJointName.KNEE_PITCH)
            return kneeJointLimitParameters;
         else if (jointMap.getLegJointName(jointName).getRight() == LegJointName.ANKLE_PITCH)
            return ankleJointLimitParameters;
      }

      return null;
   }

   /** {@inheritDoc} */
   @Override
   public boolean enableHeightFeedbackControl()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public SteppingParameters getSteppingParameters()
   {
      return steppingParameters;
   }
}
