package us.ihmc.atlas.parameters;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.*;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.PushRecoveryControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.WrenchBasedFootSwitchFactory;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PID3DConfiguration;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDSE3Configuration;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchFactory;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class AtlasPushRecoveryControllerParameters extends PushRecoveryControllerParameters
{
   private final RobotTarget target;
   private final boolean runningOnRealRobot;
   private final SideDependentList<RigidBodyTransform> handPosesWithRespectToChestFrame = new SideDependentList<RigidBodyTransform>();

   // USE THESE FOR Real Atlas Robot and sims when controlling pelvis height instead of CoM.
   private final double minimumHeightAboveGround;// = 0.625;
   private double nominalHeightAboveGround;// = 0.705;
   private final double maximumHeightAboveGround;// = 0.765 + 0.08;

   private final AtlasJointMap jointMap;
   private final AtlasMomentumOptimizationSettings momentumOptimizationSettings;
   private final double massScale;

   private JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters;
   private LegConfigurationParameters legConfigurationParameters;
   private ToeOffParameters toeOffParameters;
   private SwingTrajectoryParameters swingTrajectoryParameters;
   private ICPOptimizationParameters icpOptimizationParameters;
   private AtlasSteppingParameters steppingParameters;
   private LeapOfFaithParameters leapOfFaithParameters;

   private final JointLimitParameters spineJointLimitParameters;
   private final JointLimitParameters kneeJointLimitParameters;
   private final JointLimitParameters ankleJointLimitParameters;

   public AtlasPushRecoveryControllerParameters(RobotTarget target, AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
   {
      this.target = target;
      this.jointMap = jointMap;
      this.massScale = Math.pow(jointMap.getModelScale(), jointMap.getMassScalePower());

      momentumOptimizationSettings = new AtlasMomentumOptimizationSettings(jointMap, contactPointParameters.getNumberOfContactableBodies());

      minimumHeightAboveGround = jointMap.getModelScale() * (0.625 + 0.08);
      nominalHeightAboveGround = jointMap.getModelScale() * (0.705 + 0.08);
      maximumHeightAboveGround = jointMap.getModelScale() * (0.736 + 0.08);

      runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      jointPrivilegedConfigurationParameters = new AtlasJointPrivilegedConfigurationParameters(runningOnRealRobot);
      legConfigurationParameters = new AtlasLegConfigurationParameters(runningOnRealRobot);
      toeOffParameters = new AtlasToeOffParameters(jointMap);
      swingTrajectoryParameters = new AtlasSwingTrajectoryParameters(target, jointMap.getModelScale());
      steppingParameters = new AtlasSteppingParameters(jointMap);
      leapOfFaithParameters = new AtlasLeapOfFaithParameters(runningOnRealRobot);

      icpOptimizationParameters = new AtlasICPOptimizationParameters(runningOnRealRobot);

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

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTransform transform = new RigidBodyTransform();

         double x = 0.20;
         double y = robotSide.negateIfRightSide(0.35); // 0.30);
         double z = -0.40;
         Vector3D translation = new Vector3D(x, y, z);
         translation.scale(jointMap.getModelScale());
         transform.getTranslation().set(translation);

         RotationMatrix rotation = new RotationMatrix();
         double yaw = 0.0; // robotSide.negateIfRightSide(-1.7);
         double pitch = 0.7;
         double roll = 0.0; // robotSide.negateIfRightSide(-0.8);
         rotation.setYawPitchRoll(yaw, pitch, roll);
         transform.getRotation().set(rotation);

         handPosesWithRespectToChestFrame.put(robotSide, transform);
      }
   }

   @Override
   public double minimumHeightAboveAnkle()
   {
      return minimumHeightAboveGround;
   }

   @Override
   public double nominalHeightAboveAnkle()
   {
      return nominalHeightAboveGround;
   }

   @Override
   public double maximumHeightAboveAnkle()
   {
      return maximumHeightAboveGround;
   }

   @Override
   public double getMaximumLegLengthForSingularityAvoidance()
   {
      return jointMap.getPhysicalProperties().getShinLength() + jointMap.getPhysicalProperties().getThighLength();
   }

   @Override
   public MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      return momentumOptimizationSettings;
   }

   /** {@inheritDoc} */
   @Override
   public double getCoPErrorThresholdForHighCoPDamping()
   {
      return Double.POSITIVE_INFINITY; //0.075;
   }

   /** {@inheritDoc} */
   @Override
   public double getMaxAllowedDistanceCMPSupport()
   {
      return 0.04 * jointMap.getModelScale();
   }

   /** {@inheritDoc} */
   @Override
   public boolean useCenterOfMassVelocityFromEstimator()
   {
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public boolean usePelvisHeightControllerOnly()
   {
      return false;
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
   public boolean controlToeDuringSwing()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public JointPrivilegedConfigurationParameters getJointPrivilegedConfigurationParameters()
   {
      return jointPrivilegedConfigurationParameters;
   }

   /** {@inheritDoc} */
   @Override
   public LegConfigurationParameters getLegConfigurationParameters()
   {
      return legConfigurationParameters;
   }

   /** {@inheritDoc} */
   @Override
   public boolean enableHeightFeedbackControl()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public ToeOffParameters getToeOffParameters()
   {
      return toeOffParameters;
   }

   /** {@inheritDoc} */
   @Override
   public SwingTrajectoryParameters getSwingTrajectoryParameters()
   {
      return swingTrajectoryParameters;
   }

   /** {@inheritDoc} */
   @Override
   public ICPOptimizationParameters getICPOptimizationParameters()
   {
      return icpOptimizationParameters;
   }

   /** {@inheritDoc} */
   @Override
   public SteppingParameters getSteppingParameters()
   {
      return steppingParameters;
   }

   @Override
   public double getMinSwingTrajectoryClearanceFromStanceFoot()
   {
      return 0.15;
   }

   /**
    * Maximum velocity of the CoM height. Desired height velocity will be set to this if it is exceeded.
    * Not a very clean variable and probably should not be here, but here it is...
    */
   @Override
   public double getMaximumVelocityCoMHeight()
   {
      return 0.5;
   }
}
