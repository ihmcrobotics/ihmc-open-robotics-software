package us.ihmc.atlas.parameters;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeSlippingDetectorParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
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
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchFactory;

public class AtlasWalkingControllerParameters extends WalkingControllerParameters
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
   private final ICPAngularMomentumModifierParameters angularMomentumModifierParameters;
   private final double massScale;

   private TObjectDoubleHashMap<String> jointHomeConfiguration = null;
   private Map<String, Pose3D> bodyHomeConfiguration = null;

   private final JointPrivilegedConfigurationParameters jointPrivilegedConfigurationParameters;
   private final LegConfigurationParameters legConfigurationParameters;
   private final ToeOffParameters toeOffParameters;
   private final SwingTrajectoryParameters swingTrajectoryParameters;
   private final ICPOptimizationParameters icpOptimizationParameters;
   private final AtlasSteppingParameters steppingParameters;
   private final LeapOfFaithParameters leapOfFaithParameters;

   public AtlasWalkingControllerParameters(RobotTarget target, AtlasJointMap jointMap, AtlasContactPointParameters contactPointParameters)
   {
      this.target = target;
      this.jointMap = jointMap;
      this.massScale = Math.pow(jointMap.getModelScale(), jointMap.getMassScalePower());

      momentumOptimizationSettings = new AtlasMomentumOptimizationSettings(jointMap, contactPointParameters.getNumberOfContactableBodies());
      angularMomentumModifierParameters = new ICPAngularMomentumModifierParameters();

      minimumHeightAboveGround = jointMap.getModelScale() * (0.625 + 0.08);
      nominalHeightAboveGround = jointMap.getModelScale() * (0.705 + 0.08);
      maximumHeightAboveGround = jointMap.getModelScale() * (0.845 + 0.08);

      runningOnRealRobot = target == RobotTarget.REAL_ROBOT;

      jointPrivilegedConfigurationParameters = new AtlasJointPrivilegedConfigurationParameters(runningOnRealRobot);
      legConfigurationParameters = new AtlasLegConfigurationParameters(runningOnRealRobot);
      toeOffParameters = new AtlasToeOffParameters(jointMap);
      swingTrajectoryParameters = new AtlasSwingTrajectoryParameters(target, jointMap.getModelScale());
      steppingParameters = new AtlasSteppingParameters(jointMap);
      leapOfFaithParameters = new AtlasLeapOfFaithParameters(runningOnRealRobot);

      icpOptimizationParameters = new AtlasICPOptimizationParameters(runningOnRealRobot);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyTransform transform = new RigidBodyTransform();

         double x = 0.20;
         double y = robotSide.negateIfRightSide(0.35); // 0.30);
         double z = -0.40;
         Vector3D translation = new Vector3D(x, y, z);
         translation.scale(jointMap.getModelScale());
         transform.setTranslation(translation);

         RotationMatrix rotation = new RotationMatrix();
         double yaw = 0.0; // robotSide.negateIfRightSide(-1.7);
         double pitch = 0.7;
         double roll = 0.0; // robotSide.negateIfRightSide(-0.8);
         rotation.setYawPitchRoll(yaw, pitch, roll);
         transform.setRotation(rotation);

         handPosesWithRespectToChestFrame.put(robotSide, transform);
      }
   }

   @Override
   public boolean usePelvisHeightControllerOnly()
   {
      return false;
   }

   @Override
   public double getOmega0()
   {
      // TODO probably need to be tuned.
      return (runningOnRealRobot ? 3.4 : 3.0) / Math.sqrt(jointMap.getModelScale()); // 3.0 seems more appropriate.
      //      return 3.0;
   }

   /** {@inheritDoc} */
   @Override
   public boolean enableToeOffSlippingDetection()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public ToeSlippingDetectorParameters getToeSlippingDetectorParameters()
   {
      return new ToeSlippingDetectorParameters();
   }

   /** @inheritDoc */
   @Override
   public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
   {
      return true; // TODO Seems to work well but still need to be heavily tested on the robot.
   }

   @Override
   public boolean allowAutomaticManipulationAbort()
   {
      return true;
   }

   @Override
   public double getICPErrorThresholdToSpeedUpSwing()
   {
      return jointMap.getModelScale() * 0.05;
   }

   @Override
   public double getMinimumSwingTimeForDisturbanceRecovery()
   {
      if (runningOnRealRobot)
         return 0.6;
      else
         return 0.3;
   }

   // USE THESE FOR DRC Atlas Model TASK 2 UNTIL WALKING WORKS BETTER WITH OTHERS.
   //   private final double minimumHeightAboveGround = 0.785;
   //   private double nominalHeightAboveGround = 0.865;
   //   private final double maximumHeightAboveGround = 0.925;

   //   // USE THESE FOR VRC Atlas Model TASK 2 UNTIL WALKING WORKS BETTER WITH OTHERS.
   //   private double minimumHeightAboveGround = 0.68;
   //   private double nominalHeightAboveGround = 0.76;
   //   private double maximumHeightAboveGround = 0.82;

   //   // USE THESE FOR IMPROVING WALKING, BUT DONT CHECK THEM IN UNTIL IT IMPROVED WALKING THROUGH MUD.
   //   private double minimumHeightAboveGround = 0.68;
   //   private double nominalHeightAboveGround = 0.80;  // NOTE: used to be 0.76, jojo
   //   private double maximumHeightAboveGround = 0.84;  // NOTE: used to be 0.82, jojo

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
   public double defaultOffsetHeightAboveAnkle()
   {
      double defaultOffset = runningOnRealRobot ? 0.035 : 0.0;
      return defaultOffset * jointMap.getModelScale();
   }

   public void setNominalHeightAboveAnkle(double nominalHeightAboveAnkle)
   {
      this.nominalHeightAboveGround = nominalHeightAboveAnkle;
   }

   @Override
   public double getMaximumLegLengthForSingularityAvoidance()
   {
      return jointMap.getPhysicalProperties().getShinLength() + jointMap.getPhysicalProperties().getThighLength();
   }

   @Override
   public ICPControlGains createICPControlGains()
   {
      ICPControlGains gains = new ICPControlGains();

      double kpParallel = 2.5;
      double kpOrthogonal = 1.5;
      double ki = 0.0;
      double kiBleedOff = 0.0;

      gains.setKpParallelToMotion(kpParallel);
      gains.setKpOrthogonalToMotion(kpOrthogonal);
      gains.setKi(ki);
      gains.setIntegralLeakRatio(kiBleedOff);

      //      if (runningOnRealRobot) gains.setFeedbackPartMaxRate(1.0);
      return gains;
   }

   @Override
   public PDGains getCoMHeightControlGains()
   {
      PDGains gains = new PDGains();

      double kp = 40.0;
      double zeta = runningOnRealRobot ? 0.4 : 0.8;
      double maxAcceleration = 0.5 * 9.81;
      double maxJerk = maxAcceleration / 0.05;

      gains.setKp(kp);
      gains.setZeta(zeta);
      gains.setMaximumFeedback(maxAcceleration);
      gains.setMaximumFeedbackRate(maxJerk);

      return gains;
   }

   /** {@inheritDoc} */
   @Override
   public List<GroupParameter<PIDGainsReadOnly>> getJointSpaceControlGains()
   {
      List<GroupParameter<PIDGainsReadOnly>> jointspaceGains = new ArrayList<>();
      jointspaceGains.add(new GroupParameter<>("SpineJoints", jointMap.getSpineJointNamesAsStrings()));
      jointspaceGains.add(new GroupParameter<>("NeckJoints", jointMap.getNeckJointNamesAsStrings()));
      jointspaceGains.add(new GroupParameter<>("ArmJoints", jointMap.getArmJointNamesAsStrings()));
      return jointspaceGains;
   }

   /** {@inheritDoc} */
   @Override
   public List<GroupParameter<PID3DConfiguration>> getTaskspaceOrientationControlGains()
   {
      List<GroupParameter<PID3DConfiguration>> taskspaceAngularGains = new ArrayList<>();

      PID3DConfiguration chestAngularGainConfiguration = new PID3DConfiguration(GainCoupling.XY, false);
      taskspaceAngularGains.add(new GroupParameter<>("Chest", chestAngularGainConfiguration, jointMap.getChestName()));

      PID3DConfiguration headAngularGainConfiguration = new PID3DConfiguration(GainCoupling.XYZ, false);
      taskspaceAngularGains.add(new GroupParameter<>("Head", headAngularGainConfiguration, jointMap.getHeadName()));

      PID3DConfiguration handAngularGainConfiguration = new PID3DConfiguration(GainCoupling.XYZ, false);
      taskspaceAngularGains.add(new GroupParameter<>("Hand", handAngularGainConfiguration, jointMap.getHandNames()));

      PID3DConfiguration pelvisAngularGainConfiguration = new PID3DConfiguration(GainCoupling.XY, false);
      taskspaceAngularGains.add(new GroupParameter<>("Pelvis", pelvisAngularGainConfiguration, jointMap.getPelvisName()));

      return taskspaceAngularGains;
   }

   /** {@inheritDoc} */
   @Override
   public List<GroupParameter<PID3DConfiguration>> getTaskspacePositionControlGains()
   {
      List<GroupParameter<PID3DConfiguration>> taskspaceLinearGains = new ArrayList<>();

      PID3DConfiguration handLinearGainConfiguration = new PID3DConfiguration(GainCoupling.XYZ, false);
      taskspaceLinearGains.add(new GroupParameter<>("Hand", handLinearGainConfiguration, jointMap.getHandNames()));

      return taskspaceLinearGains;
   }

   /** {@inheritDoc} */
   @Override
   public Map<String, RigidBodyControlMode> getDefaultControlModesForRigidBodies()
   {
      Map<String, RigidBodyControlMode> defaultControlModes = new HashMap<>();
      defaultControlModes.put(jointMap.getChestName(), RigidBodyControlMode.TASKSPACE);
      return defaultControlModes;
   }

   /** {@inheritDoc} */
   @Override
   public TObjectDoubleHashMap<String> getOrCreateJointHomeConfiguration()
   {
      if (jointHomeConfiguration != null)
         return jointHomeConfiguration;

      jointHomeConfiguration = new TObjectDoubleHashMap<String>();

      jointHomeConfiguration.put(jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), 0.0);
      jointHomeConfiguration.put(jointMap.getSpineJointName(SpineJointName.SPINE_ROLL), 0.0);
      jointHomeConfiguration.put(jointMap.getSpineJointName(SpineJointName.SPINE_YAW), 0.0);

      jointHomeConfiguration.put(jointMap.getNeckJointName(NeckJointName.PROXIMAL_NECK_PITCH), runningOnRealRobot ? 0.7 : 0.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), robotSide.negateIfRightSide(0.785398));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(-0.52379));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), 2.33708);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), robotSide.negateIfRightSide(2.35619));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH), -0.337807);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL), robotSide.negateIfRightSide(0.207730));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SECOND_WRIST_PITCH), -0.026599);
      }

      return jointHomeConfiguration;
   }

   /** {@inheritDoc} */
   @Override
   public Map<String, Pose3D> getOrCreateBodyHomeConfiguration()
   {
      if (bodyHomeConfiguration != null)
         return bodyHomeConfiguration;

      bodyHomeConfiguration = new HashMap<String, Pose3D>();

      Pose3D homeChestPoseInPelvisZUpFrame = new Pose3D();
      if (runningOnRealRobot)
      {
         homeChestPoseInPelvisZUpFrame.appendPitchRotation(Math.toRadians(10.0));
      }
      bodyHomeConfiguration.put(jointMap.getChestName(), homeChestPoseInPelvisZUpFrame);

      return bodyHomeConfiguration;
   }

   @Override
   public PIDSE3Configuration getSwingFootControlGains()
   {
      return new PIDSE3Configuration(GainCoupling.XY, false);
   }

   @Override
   public PIDSE3Configuration getHoldPositionFootControlGains()
   {
      return new PIDSE3Configuration(GainCoupling.XY, false);
   }

   @Override
   public PIDSE3Configuration getToeOffFootControlGains()
   {
      return new PIDSE3Configuration(GainCoupling.XY, false);
   }

   @Override
   public double getDefaultTransferTime()
   {
      return (runningOnRealRobot ? 0.8 : 0.25); //Math.sqrt(jointMap.getModelScale()) *
   }

   @Override
   public double getDefaultSwingTime()
   {
      return (runningOnRealRobot ? 1.2 : 0.6); //Math.sqrt(jointMap.getModelScale()) *
   }

   @Override
   public FootSwitchFactory getFootSwitchFactory()
   {
      WrenchBasedFootSwitchFactory footSwitchFactory = new WrenchBasedFootSwitchFactory();
      double contactThresholdForce = 5.0;
      if (target == RobotTarget.GAZEBO)
         contactThresholdForce = 50.0;
      else if (target == RobotTarget.REAL_ROBOT)
         contactThresholdForce = 80.0;

      footSwitchFactory.setDefaultContactThresholdForce(massScale * contactThresholdForce);
      footSwitchFactory.setDefaultCoPThresholdFraction(0.02);
      footSwitchFactory.setDefaultSecondContactThresholdForceIgnoringCoP(massScale * 220.0);
      return footSwitchFactory;
   }

   @Override
   public String[] getJointsToIgnoreInController()
   {
      return new String[0];
   }

   @Override
   public MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      return momentumOptimizationSettings;
   }

   @Override
   public ICPAngularMomentumModifierParameters getICPAngularMomentumModifierParameters()
   {
      return angularMomentumModifierParameters;
   }

   @Override
   public double getMaxICPErrorBeforeSingleSupportX()
   {
      return 0.035 * jointMap.getModelScale();
   }

   @Override
   public double getMaxICPErrorBeforeSingleSupportY()
   {
      return 0.015 * jointMap.getModelScale();
   }

   @Override
   public boolean finishSingleSupportWhenICPPlannerIsDone()
   {
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public double getHighCoPDampingDurationToPreventFootShakies()
   {
      return -1.0; // 0.5;
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
      String[] joints = {bkxName, bkyName};
      return joints;
   }

   /** {@inheritDoc} */
   @Override
   public JointLimitParameters getJointLimitParametersForJointsWithRestictiveLimits()
   {
      JointLimitParameters parameters = new JointLimitParameters();
      parameters.setMaxAbsJointVelocity(9.0);
      parameters.setJointLimitDistanceForMaxVelocity(30.0 * Math.PI / 180.0);
      parameters.setJointLimitFilterBreakFrequency(15.0);
      parameters.setVelocityControlGain(30.0);
      return parameters;
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

   /** {@inheritDoc} */
   @Override
   public LeapOfFaithParameters getLeapOfFaithParameters()
   {
      return leapOfFaithParameters;
   }

   @Override
   public double getMinSwingTrajectoryClearanceFromStanceFoot()
   {
      return 0.15;
   }
}
