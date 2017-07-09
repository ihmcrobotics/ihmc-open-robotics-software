package us.ihmc.commonWalkingControlModules.controlModules.foot.toeOffCalculator;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.robotics.controllers.YoOrientationPIDGainsInterface;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

public class CentroidProjectionToeOffCalculatorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private static final double footLengthForControl = 0.22;
   private static final double footWidthForControl = 0.11;
   private static final double toeWidthForControl = 0.0825;

   private ToeOffCalculator toeOffCalculator;
   private YoVariableRegistry parentRegistry;

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private final SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();

   @Before
   public void setUp()
   {
      parentRegistry = new YoVariableRegistry("parentRegistryTEST");

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.0;
         List<Point2D> contactPointsInSoleFrame = new ArrayList<>();
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, -toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, -footWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, footWidthForControl / 2.0));
         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
         FramePose startingPose = new FramePose();
         startingPose.setToZero(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(0.20));
         contactableFoot.setSoleFrame(startingPose);
         contactableFeet.put(robotSide, contactableFoot);

         RigidBody foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         List<FramePoint2d> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, parentRegistry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);
      }

   }

   @After
   public void tearDown()
   {

   }



	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testConstructor()
   {
      toeOffCalculator = new CentroidProjectionToeOffCalculator(contactStates, contactableFeet, getWalkingControllerParameters(), parentRegistry);
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testClear()
   {
      toeOffCalculator = new CentroidProjectionToeOffCalculator(contactStates, contactableFeet, getWalkingControllerParameters(), parentRegistry);
      toeOffCalculator.clear();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetExitCMP()
   {
      RobotSide trailingSide = RobotSide.LEFT;
      FramePoint exitCMP = new FramePoint();
      exitCMP.setToZero(contactableFeet.get(trailingSide).getSoleFrame());
      exitCMP.setX(0.05);

      toeOffCalculator = new CentroidProjectionToeOffCalculator(contactStates, contactableFeet, getWalkingControllerParameters(), parentRegistry);
      toeOffCalculator.setExitCMP(exitCMP, trailingSide);
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeToeOffContactPoint()
   {
      RobotSide trailingSide = RobotSide.LEFT;

      FramePoint exitCMP = new FramePoint();
      FramePoint2d desiredCMP = new FramePoint2d();

      exitCMP.setToZero(contactableFeet.get(trailingSide).getSoleFrame());
      desiredCMP.setToZero(contactableFeet.get(trailingSide).getSoleFrame());

      exitCMP.setX(0.05);
      desiredCMP.setX(0.05);

      toeOffCalculator = new CentroidProjectionToeOffCalculator(contactStates, contactableFeet, getWalkingControllerParameters(), parentRegistry);
      toeOffCalculator.setExitCMP(exitCMP, trailingSide);
      toeOffCalculator.computeToeOffContactPoint(desiredCMP, trailingSide);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetToeOffContactPoint()
   {
      RobotSide trailingSide = RobotSide.LEFT;

      FramePoint exitCMP = new FramePoint();
      FramePoint2d desiredCMP = new FramePoint2d();
      FramePoint2d toeOffPoint = new FramePoint2d();

      exitCMP.setToZero(contactableFeet.get(trailingSide).getSoleFrame());
      desiredCMP.setToZero(contactableFeet.get(trailingSide).getSoleFrame());
      toeOffPoint.setToZero(contactableFeet.get(trailingSide).getSoleFrame());

      exitCMP.setX(0.05);
      desiredCMP.setX(0.05);

      toeOffCalculator = new CentroidProjectionToeOffCalculator(contactStates, contactableFeet, getWalkingControllerParameters(), parentRegistry);
      toeOffCalculator.setExitCMP(exitCMP, trailingSide);
      toeOffCalculator.computeToeOffContactPoint(desiredCMP, trailingSide);
      toeOffCalculator.getToeOffContactPoint(toeOffPoint, trailingSide);
   }

   public WalkingControllerParameters getWalkingControllerParameters()
   {
      return new WalkingControllerParameters()
      {
         @Override
         public SideDependentList<RigidBodyTransform> getDesiredHandPosesWithRespectToChestFrame()
         {
            return null;
         }

         @Override
         public String[] getDefaultChestOrientationControlJointNames()
         {
            return new String[0];
         }

         @Override
         public double getOmega0()
         {
            return 0;
         }

         @Override
         public double getAnkleHeight()
         {
            return 0;
         }

         @Override
         public double getLegLength()
         {
            return 0;
         }

         @Override
         public double getMinLegLengthBeforeCollapsingSingleSupport()
         {
            return 0;
         }

         @Override
         public double getMinMechanicalLegLength()
         {
            return 0;
         }

         @Override
         public double minimumHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public double nominalHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public double maximumHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public double defaultOffsetHeightAboveAnkle()
         {
            return 0;
         }

         @Override
         public double pelvisToAnkleThresholdForWalking()
         {
            return 0;
         }

         @Override
         public double getTimeToGetPreparedForLocomotion()
         {
            return 0;
         }

         @Override
         public boolean doToeOffIfPossible()
         {
            return false;
         }

         @Override
         public boolean doToeOffIfPossibleInSingleSupport()
         {
            return false;
         }

         @Override
         public boolean checkECMPLocationToTriggerToeOff()
         {
            return false;
         }

         @Override
         public double getMinStepLengthForToeOff()
         {
            return 0;
         }

         @Override
         public boolean doToeOffWhenHittingAnkleLimit()
         {
            return false;
         }

         @Override
         public double getMaximumToeOffAngle()
         {
            return 0;
         }

         @Override
         public boolean doToeTouchdownIfPossible()
         {
            return false;
         }

         @Override
         public double getToeTouchdownAngle()
         {
            return 0;
         }

         @Override
         public boolean doHeelTouchdownIfPossible()
         {
            return false;
         }

         @Override
         public double getHeelTouchdownAngle()
         {
            return 0;
         }

         @Override
         public boolean allowShrinkingSingleSupportFootPolygon()
         {
            return false;
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
         public double getMinimumSwingTimeForDisturbanceRecovery()
         {
            return 0;
         }

         @Override
         public boolean useOptimizationBasedICPController()
         {
            return false;
         }

         @Override
         public double getICPErrorThresholdToSpeedUpSwing()
         {
            return 0;
         }

         @Override
         public ICPControlGains createICPControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoPDGains createPelvisICPBasedXYControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoOrientationPIDGainsInterface createPelvisOrientationControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoPDGains createCoMHeightControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public boolean getCoMHeightDriftCompensation()
         {
            return false;
         }

         @Override
         public YoPDGains createUnconstrainedJointsControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoOrientationPIDGainsInterface createChestControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoSE3PIDGainsInterface createSwingFootControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoSE3PIDGainsInterface createHoldPositionFootControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoSE3PIDGainsInterface createToeOffFootControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoSE3PIDGainsInterface createEdgeTouchdownFootControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public double getSwingHeightMaxForPushRecoveryTrajectory()
         {
            return 0;
         }

         @Override
         public boolean doPrepareManipulationForLocomotion()
         {
            return false;
         }

         @Override
         public boolean controlHeadAndHandsWithSliders()
         {
            return false;
         }

         @Override
         public double getDefaultTransferTime()
         {
            return 0;
         }

         @Override
         public double getDefaultSwingTime()
         {
            return 0;
         }

         @Override
         public double getSpineYawLimit()
         {
            return 0;
         }

         @Override
         public double getSpinePitchUpperLimit()
         {
            return 0;
         }

         @Override
         public double getSpinePitchLowerLimit()
         {
            return 0;
         }

         @Override
         public double getSpineRollLimit()
         {
            return 0;
         }

         @Override
         public boolean isSpinePitchReversed()
         {
            return false;
         }

         @Override
         public double getFoot_start_toetaper_from_back()
         {
            return 0;
         }

         @Override
         public double getSideLengthOfBoundingBoxForFootstepHeight()
         {
            return 0;
         }

         @Override
         public double getDesiredTouchdownHeightOffset()
         {
            return 0;
         }

         @Override
         public double getDesiredTouchdownVelocity()
         {
            return 0;
         }

         @Override
         public double getDesiredTouchdownAcceleration()
         {
            return 0;
         }

         @Override
         public double getContactThresholdForce()
         {
            return 0;
         }

         @Override
         public double getSecondContactThresholdForceIgnoringCoP()
         {
            return 0;
         }

         @Override
         public LinkedHashMap<NeckJointName, ImmutablePair<Double, Double>> getSliderBoardControlledNeckJointsWithLimits()
         {
            return null;
         }

         @Override
         public SideDependentList<LinkedHashMap<String, ImmutablePair<Double, Double>>> getSliderBoardControlledFingerJointsWithLimits()
         {
            return null;
         }

         @Override
         public double getCoPThresholdFraction()
         {
            return 0;
         }

         @Override
         public String[] getJointsToIgnoreInController()
         {
            return new String[0];
         }

         @Override
         public MomentumOptimizationSettings getMomentumOptimizationSettings()
         {
            return null;
         }

         @Override
         public boolean doFancyOnToesControl()
         {
            return false;
         }

         @Override
         public FootSwitchType getFootSwitchType()
         {
            return null;
         }

         @Override
         public double getContactThresholdHeight()
         {
            return 0;
         }

         @Override
         public double getMaxICPErrorBeforeSingleSupportX()
         {
            return 0;
         }

         @Override
         public double getMaxICPErrorBeforeSingleSupportY()
         {
            return 0;
         }

         @Override
         public boolean finishSingleSupportWhenICPPlannerIsDone()
         {
            return false;
         }

         @Override
         public void useInverseDynamicsControlCore()
         {

         }

         @Override
         public void useVirtualModelControlCore()
         {

         }

         @Override
         public double getHighCoPDampingDurationToPreventFootShakies()
         {
            return 0;
         }

         @Override
         public double getCoPErrorThresholdForHighCoPDamping()
         {
            return 0;
         }

         @Override
         public double getFootForwardOffset()
         {
            return 0;
         }

         @Override
         public double getFootBackwardOffset()
         {
            return 0;
         }

         @Override
         public double getFootWidth()
         {
            return 0;
         }

         @Override
         public double getToeWidth()
         {
            return 0;
         }

         @Override
         public double getFootLength()
         {
            return 0;
         }

         @Override
         public double getActualFootWidth()
         {
            return 0;
         }

         @Override
         public double getActualFootLength()
         {
            return 0;
         }

         @Override
         public double getFootstepArea()
         {
            return 0;
         }

         @Override
         public String[] getDefaultHeadOrientationControlJointNames()
         {
            return new String[0];
         }

         @Override
         public YoOrientationPIDGainsInterface createHeadOrientationControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public YoPIDGains createHeadJointspaceControlGains(YoVariableRegistry registry)
         {
            return null;
         }

         @Override
         public double[] getInitialHeadYawPitchRoll()
         {
            return new double[0];
         }

         @Override
         public boolean isNeckPositionControlled()
         {
            return false;
         }

         @Override
         public double getNeckPitchUpperLimit()
         {
            return 0;
         }

         @Override
         public double getNeckPitchLowerLimit()
         {
            return 0;
         }

         @Override
         public double getHeadYawLimit()
         {
            return 0;
         }

         @Override
         public double getHeadRollLimit()
         {
            return 0;
         }

         @Override
         public double getTrajectoryTimeHeadOrientation()
         {
            return 0;
         }

         @Override
         public double getMaxStepLength()
         {
            return 0;
         }

         @Override
         public double getDefaultStepLength()
         {
            return 0;
         }

         @Override
         public double getMaxStepWidth()
         {
            return 0;
         }

         @Override
         public double getMinStepWidth()
         {
            return 0;
         }

         @Override
         public double getInPlaceWidth()
         {
            return 0;
         }

         @Override
         public double getDesiredStepForward()
         {
            return 0;
         }

         @Override
         public double getStepPitch()
         {
            return 0;
         }

         @Override
         public double getMaxStepUp()
         {
            return 0;
         }

         @Override
         public double getMaxStepDown()
         {
            return 0;
         }

         @Override
         public double getMaxSwingHeightFromStanceFoot()
         {
            return 0;
         }

         @Override
         public double getMaxAngleTurnOutwards()
         {
            return 0;
         }

         @Override
         public double getMaxAngleTurnInwards()
         {
            return 0;
         }

         @Override
         public double getMinAreaPercentForValidFootstep()
         {
            return 0;
         }

         @Override
         public double getDangerAreaPercentForValidFootstep()
         {
            return 0;
         }
      };
   }
}
