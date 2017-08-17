package us.ihmc.steppr.controlParameters;

import java.util.HashMap;
import java.util.Map;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.pidGains.GainCoupling;
import us.ihmc.robotics.controllers.pidGains.YoPID3DGains;
import us.ihmc.robotics.controllers.pidGains.YoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultYoPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.SymmetricYoPIDSE3Gains;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.steppr.parameters.BonoPhysicalProperties;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class BonoWalkingControllerParameters extends WalkingControllerParameters
{

   private final SideDependentList<RigidBodyTransform> handPosesWithRespectToChestFrame = new SideDependentList<RigidBodyTransform>();

   private final boolean runningOnRealRobot;
   private final DRCRobotJointMap jointMap;
   private final ToeOffParameters toeOffParameters;
   private final SwingTrajectoryParameters swingTrajectoryParameters;
   private final BonoSteppingParameters steppingParameters;

   public BonoWalkingControllerParameters(DRCRobotJointMap jointMap, boolean runningOnRealRobot)
   {
      this.jointMap = jointMap;
      this.runningOnRealRobot = runningOnRealRobot;
      this.toeOffParameters = new BonoToeOffParameters();
      this.swingTrajectoryParameters = new BonoSwingTrajectoryParameters(runningOnRealRobot);
      this.steppingParameters = new BonoSteppingParameters(runningOnRealRobot);

      for (RobotSide robotSide : RobotSide.values())
      {
         handPosesWithRespectToChestFrame.put(robotSide, new RigidBodyTransform());
      }
   }

   @Override
   public double getOmega0()
   {
      return 3.4;
   }

   @Override
   public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
   {
      return false;
   }

   @Override
   public boolean allowAutomaticManipulationAbort()
   {
      return false;
   }

   @Override
   public double getICPErrorThresholdToSpeedUpSwing()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getMinimumSwingTimeForDisturbanceRecovery()
   {
      return getDefaultSwingTime();
   }

   private final double minimumHeightAboveGround = 0.595;
   private double nominalHeightAboveGround = 0.670+0.020-0.03-0.02;//+0.010;//+0.020;
   private final double maximumHeightAboveGround = 0.79;//Hip height fully upright//0.735;
   //private final double additionalOffsetHeightBono = 0.15;
   private final double additionalOffsetHeightBono = 0.16; //Spring Ankle

   @Override
   public double minimumHeightAboveAnkle()
   {
      return minimumHeightAboveGround + additionalOffsetHeightBono;
   }

   @Override
   public double nominalHeightAboveAnkle()
   {
      return nominalHeightAboveGround + additionalOffsetHeightBono;
   }

   @Override
   public double maximumHeightAboveAnkle()
   {
      return maximumHeightAboveGround + additionalOffsetHeightBono;
   }

   @Override
   public double defaultOffsetHeightAboveAnkle()
   {
      return 0.0;
   }

   public void setNominalHeightAboveAnkle(double nominalHeightAboveAnkle)
   {
      this.nominalHeightAboveGround = nominalHeightAboveAnkle;
   }

   @Override
   public ICPControlGains createICPControlGains()
   {
      ICPControlGains gains = new ICPControlGains();

      double kpParallel = 1.5;
      double kpOrthogonal = 1.8;
      double ki = 4.0;
      double kiBleedOff = 0.9;

      gains.setKpParallelToMotion(kpParallel);
      gains.setKpOrthogonalToMotion(kpOrthogonal);
      gains.setKi(ki);
      gains.setKiBleedOff(kiBleedOff);

      return gains;
   }

   @Override
   public YoPDGains createCoMHeightControlGains(YoVariableRegistry registry)
   {
      YoPDGains gains = new YoPDGains("CoMHeight", registry);

      double kp = runningOnRealRobot ? 40.0 : 50.0;
      double zeta = runningOnRealRobot ? 0.4 : 1.0;
      double maxAcceleration = 0.5 * 9.81;
      double maxJerk = maxAcceleration / 0.05;

      gains.setKp(kp);
      gains.setZeta(zeta);
      gains.setMaximumFeedback(maxAcceleration);
      gains.setMaximumFeedbackRate(maxJerk);
      gains.createDerivativeGainUpdater(true);

      return gains;
   }

   private YoPID3DGains createPelvisOrientationControlGains(YoVariableRegistry registry)
   {
      SymmetricYoPIDSE3Gains gains = new SymmetricYoPIDSE3Gains("PelvisOrientation", registry);

      double kp = 100;//600.0;
      double zeta = 0.4;//0.8;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = Double.POSITIVE_INFINITY;
      double maxJerk = Double.POSITIVE_INFINITY;

      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setIntegralGains(ki, maxIntegralError);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
   }

   private YoPID3DGains createChestControlGains(YoVariableRegistry registry)
   {
      SymmetricYoPIDSE3Gains gains = new SymmetricYoPIDSE3Gains("ChestOrientation", registry);

      double kp = runningOnRealRobot ? 100.0 : 100.0;
      double zeta = runningOnRealRobot ? 0.7 : 0.8;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 12.0 : 18.0;
      double maxJerk = runningOnRealRobot ? 180.0 : 270.0;

      gains.setProportionalGains(kp);
      gains.setDampingRatios(zeta);
      gains.setIntegralGains(ki, maxIntegralError);
      gains.setMaxFeedbackAndFeedbackRate(maxAccel, maxJerk);

      return gains;
   }

   private YoPIDGains createSpineControlGains(YoVariableRegistry registry)
   {
      double kp = 250.0;
      double zeta = 0.6;
      double ki = 0.0;
      double maxIntegralError = 0.0;
      double maxAccel = runningOnRealRobot ? 20.0 : Double.POSITIVE_INFINITY;
      double maxJerk = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;

      YoPIDGains spineGains = new YoPIDGains("SpineJointspace", registry);
      spineGains.setKp(kp);
      spineGains.setZeta(zeta);
      spineGains.setKi(ki);
      spineGains.setMaximumIntegralError(maxIntegralError);
      spineGains.setMaximumFeedback(maxAccel);
      spineGains.setMaximumFeedbackRate(maxJerk);
      spineGains.createDerivativeGainUpdater(true);

      return spineGains;
   }

   private Map<String, YoPIDGains> jointspaceGains = null;
   /** {@inheritDoc} */
   @Override
   public Map<String, YoPIDGains> getOrCreateJointSpaceControlGains(YoVariableRegistry registry)
   {
      if (jointspaceGains != null)
         return jointspaceGains;

      jointspaceGains = new HashMap<>();

      YoPIDGains spineGains = createSpineControlGains(registry);
      for (SpineJointName name : jointMap.getSpineJointNames())
         jointspaceGains.put(jointMap.getSpineJointName(name), spineGains);

      return jointspaceGains;
   }

   private Map<String, YoPID3DGains> taskspaceAngularGains = null;
   /** {@inheritDoc} */
   @Override
   public Map<String, YoPID3DGains> getOrCreateTaskspaceOrientationControlGains(YoVariableRegistry registry)
   {
      if (taskspaceAngularGains != null)
         return taskspaceAngularGains;

      taskspaceAngularGains = new HashMap<>();

      YoPID3DGains chestAngularGains = createChestControlGains(registry);
      taskspaceAngularGains.put(jointMap.getChestName(), chestAngularGains);

      YoPID3DGains pelvisAngularGains = createPelvisOrientationControlGains(registry);
      taskspaceAngularGains.put(jointMap.getPelvisName(), pelvisAngularGains);

      return taskspaceAngularGains;
   }

   private TObjectDoubleHashMap<String> jointHomeConfiguration = null;
   /** {@inheritDoc} */
   @Override
   public TObjectDoubleHashMap<String> getOrCreateJointHomeConfiguration()
   {
      if (jointHomeConfiguration != null)
         return jointHomeConfiguration;

      jointHomeConfiguration = new TObjectDoubleHashMap<String>();

      for (SpineJointName name : jointMap.getSpineJointNames())
         jointHomeConfiguration.put(jointMap.getSpineJointName(name), 0.0);

      for (NeckJointName name : jointMap.getNeckJointNames())
         jointHomeConfiguration.put(jointMap.getNeckJointName(name), 0.0);

      return jointHomeConfiguration;
   }

   @Override
   public YoPIDSE3Gains createSwingFootControlGains(YoVariableRegistry registry)
   {
      double kpXY = 75.0;
      double kpZ = 100.0; // 200.0 Trying to smash the ground there
      double zetaXYZ = 0.3;
      double kpXYOrientation = 100.0; // 300 not working
      double kpZOrientation = 100.0;
      double zetaXYOrientation = 0.3;
      double zetaZOrientation = runningOnRealRobot ? 0.3 : 0.7;
      double maxPositionAcceleration = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
      double maxPositionJerk = runningOnRealRobot ? 150.0 : Double.POSITIVE_INFINITY;
      double maxOrientationAcceleration = runningOnRealRobot ? 100.0 : Double.POSITIVE_INFINITY;
      double maxOrientationJerk = runningOnRealRobot ? 1500.0 : Double.POSITIVE_INFINITY;

      DefaultPIDSE3Gains gains = new DefaultPIDSE3Gains(GainCoupling.XY, false);
      gains.setPositionProportionalGains(kpXY, kpXY, kpZ);
      gains.setPositionDampingRatios(zetaXYZ);
      gains.setPositionMaxFeedbackAndFeedbackRate(maxPositionAcceleration, maxPositionJerk);
      gains.setOrientationProportionalGains(kpXYOrientation, kpXYOrientation, kpZOrientation);
      gains.setOrientationDampingRatios(zetaXYOrientation, zetaXYOrientation, zetaZOrientation);
      gains.setOrientationMaxFeedbackAndFeedbackRate(maxOrientationAcceleration, maxOrientationJerk);

      return new DefaultYoPIDSE3Gains("SwingFoot", gains, registry);
   }

   @Override
   public YoPIDSE3Gains createHoldPositionFootControlGains(YoVariableRegistry registry)
   {
      double kpXY = 100.0;
      double kpZ = 0.0;
      double zetaXYZ = runningOnRealRobot ? 0.2 : 1.0;
      double kpXYOrientation = runningOnRealRobot ? 40.0 : 100.0;
      double kpZOrientation = runningOnRealRobot ? 40.0 : 100.0;
      double zetaOrientation = runningOnRealRobot ? 0.2 : 1.0;
      double maxLinearAcceleration = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
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

      return new DefaultYoPIDSE3Gains("HoldFoot", gains, registry);
   }

   @Override
   public YoPIDSE3Gains createToeOffFootControlGains(YoVariableRegistry registry)
   {
      double kpXY = 100.0;
      double kpZ = 0.0;
      double zetaXYZ = runningOnRealRobot ? 0.4 : 0.4;
      double kpXYOrientation = runningOnRealRobot ? 200.0 : 200.0;
      double kpZOrientation = runningOnRealRobot ? 200.0 : 200.0;
      double zetaOrientation = runningOnRealRobot ? 0.4 : 0.4;
      double maxLinearAcceleration = runningOnRealRobot ? 10.0 : Double.POSITIVE_INFINITY;
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

      return new DefaultYoPIDSE3Gains("ToeOffFoot", gains, registry);
   }

   @Override
   public boolean doPrepareManipulationForLocomotion()
   {
      return true;
   }

   @Override
   public double getDefaultTransferTime()
   {
      if (runningOnRealRobot)
         return 0.25;//0.15;//0.3;////1.0; //.5;
      return 0.25; // 1.5; //
   }

   @Override
   public double getDefaultSwingTime()
   {
      if (runningOnRealRobot)
         return 1.0;//0.7; //1.0
      return 0.6; // 1.5; //
   }

   @Override
   public double getMaximumLegLengthForSingularityAvoidance()
   {
      return BonoPhysicalProperties.legLength;
   }

   @Override
   public double getContactThresholdForce()
   {
      return 90.0;
   }

   @Override
   public double getSecondContactThresholdForceIgnoringCoP()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public double getCoPThresholdFraction()
   {
      return Double.NaN;
   }

   @Override
   public String[] getJointsToIgnoreInController()
   {
      if (!runningOnRealRobot)
         return null;

      String[] defaultChestOrientationControlJointNames = new String[] { jointMap.getSpineJointName(SpineJointName.SPINE_YAW),
            jointMap.getSpineJointName(SpineJointName.SPINE_PITCH), jointMap.getSpineJointName(SpineJointName.SPINE_ROLL) };

      return defaultChestOrientationControlJointNames;
   }

   @Override
   public MomentumOptimizationSettings getMomentumOptimizationSettings()
   {
      return new BonoMomentumOptimizationSettings(jointMap);
   }

   @Override
   public ICPAngularMomentumModifierParameters getICPAngularMomentumModifierParameters()
   {
      return null;
   }

   @Override
   public double getContactThresholdHeight()
   {
      return 0.05;
   }

   @Override
   public FootSwitchType getFootSwitchType()
   {
      return FootSwitchType.WrenchBased;
   }

   @Override
   public double getMaxICPErrorBeforeSingleSupportX()
   {
      return 0.025;
   }

   @Override
   public double getMaxICPErrorBeforeSingleSupportY()
   {
      return 0.025;
   }

   @Override
   public boolean finishSingleSupportWhenICPPlannerIsDone()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   public double getHighCoPDampingDurationToPreventFootShakies()
   {
      return -1.0;
   }

   /** {@inheritDoc} */
   @Override
   public double getCoPErrorThresholdForHighCoPDamping()
   {
      return Double.POSITIVE_INFINITY;
   }

   @Override
   public boolean useOptimizationBasedICPController()
   {
      return false;
   }

   @Override
   public ToeOffParameters getToeOffParameters()
   {
      return toeOffParameters;
   }

   @Override
   public SwingTrajectoryParameters getSwingTrajectoryParameters()
   {
      return swingTrajectoryParameters;
   }

   /** {@inheritDoc} */
   @Override
   public SteppingParameters getSteppingParameters()
   {
      return steppingParameters;
   }
}
