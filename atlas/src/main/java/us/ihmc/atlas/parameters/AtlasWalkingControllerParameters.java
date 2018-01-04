package us.ihmc.atlas.parameters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.commons.lang3.tuple.ImmutableTriple;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.atlas.AtlasJointMap;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commonWalkingControlModules.configurations.LegConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeSlippingDetectorParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.PID3DGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDGains;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;



public class AtlasWalkingControllerParameters extends WalkingControllerParameters
{
   private final RobotTarget target;
   private final boolean runningOnRealRobot;
   private final SideDependentList<RigidBodyTransform> handPosesWithRespectToChestFrame = new SideDependentList<RigidBodyTransform>();

// USE THESE FOR Real Atlas Robot and sims when controlling pelvis height instead of CoM.
   private final double minimumHeightAboveGround;// = 0.625;
   private double       nominalHeightAboveGround;// = 0.705;
   private final double maximumHeightAboveGround;// = 0.765 + 0.08;

   private final AtlasJointMap jointMap;
   private final AtlasMomentumOptimizationSettings momentumOptimizationSettings;
   private final ICPAngularMomentumModifierParameters angularMomentumModifierParameters;
   private final double massScale;

   private TObjectDoubleHashMap<String> jointHomeConfiguration = null;
   private Map<String, Pose3D> bodyHomeConfiguration = null;
   private ArrayList<String> positionControlledJoints = null;
   private List<ImmutableTriple<String, JointAccelerationIntegrationParametersReadOnly, List<String>>> integrationSettings = null;

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

      minimumHeightAboveGround = jointMap.getModelScale() * ( 0.625 );
      nominalHeightAboveGround = jointMap.getModelScale() * ( 0.705 );
      maximumHeightAboveGround = jointMap.getModelScale() * ( 0.765 + 0.08 );

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
         double y = robotSide.negateIfRightSide(0.35);    // 0.30);
         double z = -0.40;
         Vector3D translation = new Vector3D(x, y, z);
         translation.scale(jointMap.getModelScale());
         transform.setTranslation(translation);

         RotationMatrix rotation = new RotationMatrix();
         double yaw = 0.0;    // robotSide.negateIfRightSide(-1.7);
         double pitch = 0.7;
         double roll = 0.0;    // robotSide.negateIfRightSide(-0.8);
         rotation.setYawPitchRoll(yaw, pitch, roll);
         transform.setRotation(rotation);

         handPosesWithRespectToChestFrame.put(robotSide, transform);
      }
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
      return jointMap.getPhysicalProperties().getShinLength()  + jointMap.getPhysicalProperties().getThighLength();
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
      gains.setKiBleedOff(kiBleedOff);

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
      List<String> spineNames = new ArrayList<>();
      List<String> neckNames = new ArrayList<>();
      List<String> armNames = new ArrayList<>();

      Arrays.stream(jointMap.getSpineJointNames()).forEach(n -> spineNames.add(jointMap.getSpineJointName(n)));
      Arrays.stream(jointMap.getNeckJointNames()).forEach(n -> neckNames.add(jointMap.getNeckJointName(n)));
      for (RobotSide side : RobotSide.values)
      {
         Arrays.stream(jointMap.getArmJointNames()).forEach(n -> armNames.add(jointMap.getArmJointName(side, n)));
      }

      PIDGains spineGains = createSpineControlGains();
      PIDGains neckGains = createNeckControlGains();
      PIDGains armGains = createArmControlGains();

      List<GroupParameter<PIDGainsReadOnly>> jointspaceGains = new ArrayList<>();
      jointspaceGains.add(new GroupParameter<>("_SpineJointGains", spineGains, spineNames));
      jointspaceGains.add(new GroupParameter<>("_NeckJointGains", neckGains, neckNames));
      jointspaceGains.add(new GroupParameter<>("_ArmJointGains", armGains, armNames));

      return jointspaceGains;
   }

   private PIDGains createSpineControlGains()
   {
      PIDGains spineGains = new PIDGains();

      double kp = 50.0;
      double zeta = runningOnRealRobot ? 0.3 : 0.8;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      spineGains.setKp(kp);
      spineGains.setZeta(zeta);
      spineGains.setKi(ki);
      spineGains.setMaxIntegralError(maxIntegralError);
      spineGains.setMaximumFeedback(maxAccel);
      spineGains.setMaximumFeedbackRate(maxJerk);

      return spineGains;
   }

   private PIDGains createNeckControlGains()
   {
      PIDGains gains = new PIDGains();

      double kp = 40.0;
      double zeta = runningOnRealRobot ? 0.4 : 0.8;
      double maxAccel = runningOnRealRobot ? 6.0 : 36.0;
      double maxJerk = runningOnRealRobot ? 60.0 : 540.0;

      gains.setKp(kp);
      gains.setZeta(zeta);
      gains.setMaximumFeedback(maxAccel);
      gains.setMaximumFeedbackRate(maxJerk);

      return gains;
   }

   private PIDGains createArmControlGains()
   {
      PIDGains armGains = new PIDGains();

      double kp = runningOnRealRobot ? 40.0 : 80.0;
      double zeta = runningOnRealRobot ? 0.3 : 0.6;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 20.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      armGains.setKp(kp);
      armGains.setZeta(zeta);
      armGains.setKi(ki);
      armGains.setMaxIntegralError(maxIntegralError);
      armGains.setMaximumFeedback(maxAccel);
      armGains.setMaximumFeedbackRate(maxJerk);

      return armGains;
   }

   /** {@inheritDoc} */
   @Override
   public List<GroupParameter<PID3DGainsReadOnly>> getTaskspaceOrientationControlGains()
   {
      List<GroupParameter<PID3DGainsReadOnly>> taskspaceAngularGains = new ArrayList<>();

      PID3DGains chestAngularGains = createChestOrientationControlGains();
      List<String> chestGainBodies = new ArrayList<>();
      chestGainBodies.add(jointMap.getChestName());
      taskspaceAngularGains.add(new GroupParameter<>("Chest", chestAngularGains, chestGainBodies));

      PID3DGains headAngularGains = createHeadOrientationControlGains();
      List<String> headGainBodies = new ArrayList<>();
      headGainBodies.add(jointMap.getHeadName());
      taskspaceAngularGains.add(new GroupParameter<>("Head", headAngularGains, headGainBodies));

      PID3DGains handAngularGains = createHandOrientationControlGains();
      List<String> handGainBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         handGainBodies.add(jointMap.getHandName(robotSide));
      }
      taskspaceAngularGains.add(new GroupParameter<>("Hand", handAngularGains, handGainBodies));

      PID3DGains pelvisAngularGains = createPelvisOrientationControlGains();
      List<String> pelvisGainBodies = new ArrayList<>();
      pelvisGainBodies.add(jointMap.getPelvisName());
      taskspaceAngularGains.add(new GroupParameter<>("Pelvis", pelvisAngularGains, pelvisGainBodies));

      return taskspaceAngularGains;
   }

   private PID3DGains createPelvisOrientationControlGains()
   {
      double kpXY = 40.0;
      double kpZ = 40.0;
      double zetaXY = runningOnRealRobot ? 0.2 : 0.8;
      double zetaZ = runningOnRealRobot ? 0.5 : 0.8;
      double maxAccel = runningOnRealRobot ? 12.0 : 36.0;
      double maxJerk = runningOnRealRobot ? 180.0 : 540.0;

      DefaultPID3DGains gains = new DefaultPID3DGains(GainCoupling.XY, false);
      gains.setProportionalGains(kpXY, kpXY, kpZ);
      gains.setDampingRatios(zetaXY, zetaXY, zetaZ);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
   }

   private PID3DGains createHeadOrientationControlGains()
   {
      double kp = 40.0;
      double zeta = runningOnRealRobot ? 0.4 : 0.8;
      double maxAccel = runningOnRealRobot ? 6.0 : 36.0;
      double maxJerk = runningOnRealRobot ? 60.0 : 540.0;

      DefaultPID3DGains gains = new DefaultPID3DGains(GainCoupling.XYZ, false);
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
   }

   private PID3DGains createChestOrientationControlGains()
   {
      double kpXY = 40.0;
      double kpZ = 40.0;
      double zetaXY = runningOnRealRobot ? 0.5 : 0.8;
      double zetaZ = runningOnRealRobot ? 0.22 : 0.8;
      double maxAccel = runningOnRealRobot ? 6.0 : 36.0;
      double maxJerk = runningOnRealRobot ? 60.0 : 540.0;
      double maxProportionalError = 10.0 * Math.PI/180.0;

      DefaultPID3DGains gains = new DefaultPID3DGains(GainCoupling.XY, false);
      gains.setProportionalGains(kpXY, kpXY, kpZ);
      gains.setDampingRatios(zetaXY, zetaXY, zetaZ);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);
      gains.setMaxProportionalError(maxProportionalError);

      return gains;
   }

   private PID3DGains createHandOrientationControlGains()
   {
      double kp = runningOnRealRobot ? 40.0 :100.0;
      // When doing position control, the damping here seems to result into some kind of spring.
      double zeta = runningOnRealRobot ? 0.0 : 1.0;
      double maxAccel = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains(GainCoupling.XYZ, false);
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
   }

   /** {@inheritDoc} */
   @Override
   public List<GroupParameter<PID3DGainsReadOnly>> getTaskspacePositionControlGains()
   {
      List<GroupParameter<PID3DGainsReadOnly>> taskspaceLinearGains = new ArrayList<>();

      PID3DGains handLinearGains = createHandPositionControlGains();
      List<String> handGainBodies = new ArrayList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         handGainBodies.add(jointMap.getHandName(robotSide));
      }
      taskspaceLinearGains.add(new GroupParameter<>("Hand", handLinearGains, handGainBodies));

      return taskspaceLinearGains;
   }

   private PID3DGains createHandPositionControlGains()
   {
      double kp = runningOnRealRobot ? 40.0 :100.0;
      // When doing position control, the damping here seems to result into some kind of spring.
      double zeta = runningOnRealRobot ? 0.0 : 1.0;
      double maxAccel = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      DefaultPID3DGains gains = new DefaultPID3DGains(GainCoupling.XYZ, false);
      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
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

      jointHomeConfiguration.put(jointMap.getNeckJointName(NeckJointName.PROXIMAL_NECK_PITCH), 0.0);

      for (RobotSide robotSide : RobotSide.values)
      {
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_YAW), robotSide.negateIfRightSide(0.785398));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SHOULDER_ROLL), robotSide.negateIfRightSide(-0.1));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_PITCH), 3.00);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.ELBOW_ROLL), robotSide.negateIfRightSide(1.8));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.FIRST_WRIST_PITCH), -0.30);
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.WRIST_ROLL), robotSide.negateIfRightSide(0.70));
         jointHomeConfiguration.put(jointMap.getArmJointName(robotSide, ArmJointName.SECOND_WRIST_PITCH), 0.15);
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
      bodyHomeConfiguration.put(jointMap.getChestName(), homeChestPoseInPelvisZUpFrame);

      return bodyHomeConfiguration;
   }

   @Override
   public PIDSE3Gains getSwingFootControlGains()
   {
      double kpXY = 150.0;
      double kpZ = 200.0;
      double zetaXYZ = runningOnRealRobot ? 0.7 : 0.7;

      double kpXYOrientation = 200.0;
      double kpZOrientation = 200.0;
      double zetaOrientation = 0.7;

      // Reduce maxPositionAcceleration from 30 to 6 to prevent too high acceleration when hitting joint limits.
      double maxPositionAcceleration = runningOnRealRobot ? 20.0 : Double.POSITIVE_INFINITY;
//      double maxPositionAcceleration = runningOnRealRobot ? 30.0 : Double.POSITIVE_INFINITY;
      double maxPositionJerk = runningOnRealRobot ? 300.0 : Double.POSITIVE_INFINITY;
      double maxOrientationAcceleration = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxOrientationJerk = runningOnRealRobot ? 1500.0 : Double.POSITIVE_INFINITY;

      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains(GainCoupling.XY, false);
      gains.setPositionProportionalGains(kpXY, kpXY, kpZ);
      gains.setPositionDampingRatios(zetaXYZ);
      gains.setPositionMaxFeedbackAndFeedbackRate(maxPositionAcceleration, maxPositionJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatios(zetaOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxOrientationAcceleration, maxOrientationJerk);

      return gains;
   }

   @Override
   public PIDSE3Gains getHoldPositionFootControlGains()
   {
      double kpXY = 100.0;
      double kpZ = 0.0;
      double zetaXYZ = runningOnRealRobot ? 0.2 : 1.0;
      double kpXYOrientation = runningOnRealRobot ? 100.0 : 175.0;
      double kpZOrientation = runningOnRealRobot ? 100.0 : 200.0;
      double zetaOrientation = runningOnRealRobot ? 0.2 : 1.0;
      // Reduce maxPositionAcceleration from 10 to 6 to prevent too high acceleration when hitting joint limits.
      double maxLinearAcceleration = runningOnRealRobot ? 6.0 : Double.POSITIVE_INFINITY;
//      double maxLinearAcceleration = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = runningOnRealRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = runningOnRealRobot ? 1500.0 : Double.POSITIVE_INFINITY;

      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains(GainCoupling.XY, false);
      gains.setPositionProportionalGains(kpXY, kpXY, kpZ);
      gains.setPositionDampingRatios(zetaXYZ);
      gains.setPositionMaxFeedbackAndFeedbackRate(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatios(zetaOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxAngularAcceleration, maxAngularJerk);

      return gains;
   }

   @Override
   public PIDSE3Gains getToeOffFootControlGains()
   {
      double kpXY = 100.0;
      double kpZ = 100.0;
      double zetaXYZ = runningOnRealRobot ? 0.4 : 0.4;
      double kpXYOrientation = runningOnRealRobot ? 200.0 : 200.0;
      double kpZOrientation = runningOnRealRobot ? 200.0 : 200.0;
      double zetaOrientation = runningOnRealRobot ? 0.4 : 0.4;
      // Reduce maxPositionAcceleration from 10 to 6 to prevent too high acceleration when hitting joint limits.
      double maxLinearAcceleration = runningOnRealRobot ? 6.0 : Double.POSITIVE_INFINITY;
//      double maxLinearAcceleration = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxLinearJerk = runningOnRealRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxAngularAcceleration = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxAngularJerk = runningOnRealRobot ? 1500.0 : Double.POSITIVE_INFINITY;

      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains(GainCoupling.XY, false);
      gains.setPositionProportionalGains(kpXY, kpXY, kpZ);
      gains.setPositionDampingRatios(zetaXYZ);
      gains.setPositionMaxFeedbackAndFeedbackRate(maxLinearAcceleration, maxLinearJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatios(zetaOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxAngularAcceleration, maxAngularJerk);

      return gains;
   }

   public double getSwingMaxHeightForPushRecoveryTrajectory()
   {
      return 0.15 * jointMap.getModelScale();
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
   public double getContactThresholdForce()
   {
      switch (target)
      {
         case REAL_ROBOT :
            return massScale * 80.0;

         case GAZEBO :
            return massScale * 50.0;

         case SCS:
            return massScale * 5.0;

         default :
            throw new RuntimeException();
      }
   }

   @Override
   public double getSecondContactThresholdForceIgnoringCoP()
   {
      return massScale * 220.0;
   }

   @Override
   public double getCoPThresholdFraction()
   {
      return 0.02;
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
   public double getContactThresholdHeight()
   {
      return jointMap.getModelScale() * 0.05;
   }

   @Override
   public FootSwitchType getFootSwitchType()
   {
      return FootSwitchType.WrenchBased;
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
      return 0.06 * jointMap.getModelScale();
   }

   /** {@inheritDoc} */
   @Override
   public boolean useCenterOfMassVelocityFromEstimator()
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
      parameters.setJointLimitDistanceForMaxVelocity(30.0 * Math.PI/180.0);
      parameters.setJointLimitFilterBreakFrequency(15.0);
      parameters.setVelocityControlGain(30.0);
      return parameters;
   }

   /** {@inheritDoc} */
   @Override
   public boolean useOptimizationBasedICPController()
   {
      return false;
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
   public boolean controlHeightWithMomentum()
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
   public boolean alwaysAllowMomentum()
   {
      return false;
   }

   @Override
   public double getMinSwingTrajectoryClearanceFromStanceFoot()
   {
      return 0.15;
   }
}
