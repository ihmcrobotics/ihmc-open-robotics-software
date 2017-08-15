package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.ContinuousCMPICPPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.FootstepTestHelper;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ContinuousCMPBasedICPPlanner;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlPlane;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlPolygons;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.stateEstimation.FootSwitchType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class ICPAdjustmentOptimizationControllerTest
{
   private final YoVariableRegistry registry = new YoVariableRegistry("robert");
   private final YoDouble omega = new YoDouble("omega", registry);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 0.0001;

   private static final double footLengthForControl = 0.25;
   private static final double footWidthForControl = 0.12;
   private static final double toeWidthForControl = 0.12;

   private static final double singleSupportDuration = 2.0;
   private static final double doubleSupportDuration = 1.0;

   private final SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();
   private final SideDependentList<FramePose> footPosesAtTouchdown = new SideDependentList<>(new FramePose(), new FramePose());
   private final SideDependentList<ReferenceFrame> ankleFrames = new SideDependentList<>();
   private final SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();

   private BipedSupportPolygons bipedSupportPolygons;
   private ICPControlPolygons icpControlPolygons;

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testStepping()
   {
      setupContactableFeet();
      setupBipedSupportPolygons();

      double omega0 = 0.3;
      omega.set(omega0);

      ContinuousCMPBasedICPPlanner icpPlanner = new ContinuousCMPBasedICPPlanner(bipedSupportPolygons, contactableFeet,
                                                                                 icpPlannerParameters.getNumberOfFootstepsToConsider(), registry, null);
      icpPlanner.initializeParameters(icpPlannerParameters);
      icpPlanner.setOmega0(omega.getDoubleValue());
      icpPlanner.clearPlan();

      ICPAdjustmentOptimizationController icpOptimizationController = new ICPAdjustmentOptimizationController(icpPlannerParameters, icpOptimizationParameters,
            walkingControllerParameters, bipedSupportPolygons, icpControlPolygons, contactableFeet, 0.001, registry, null);
      icpOptimizationController.clearPlan();

      double stepLength = 0.2;
      double stepWidth = 0.1;
      FootstepTestHelper footstepTestHelper = new FootstepTestHelper(contactableFeet);
      List<Footstep> footsteps = footstepTestHelper.createFootsteps(stepWidth, stepLength, 3);
      FootstepTiming defaultTiming = new FootstepTiming(singleSupportDuration, doubleSupportDuration);
      icpPlanner.setFinalTransferDuration(doubleSupportDuration);

      for (int i = 0; i < footsteps.size(); i++)
      {
         icpOptimizationController.addFootstepToPlan(footsteps.get(i), defaultTiming);
         icpPlanner.addFootstepToPlan(footsteps.get(i), defaultTiming);
      }

      RobotSide supportSide = footsteps.get(0).getRobotSide().getOppositeSide();

      icpPlanner.setSupportLeg(supportSide);
      icpPlanner.initializeForSingleSupport(0.0);
      icpOptimizationController.initializeForSingleSupport(0.0, supportSide, omega0);

      icpPlanner.updateCurrentPlan(true);
      double currentTime = 0.5;
      FramePoint2D desiredICP = new FramePoint2D();
      FrameVector2D desiredICPVelocity = new FrameVector2D();
      FramePoint2D perfectCMP = new FramePoint2D();
      icpPlanner.compute(currentTime);
      icpPlanner.getDesiredCapturePointPosition(desiredICP);
      icpPlanner.getDesiredCapturePointVelocity(desiredICPVelocity);
      icpPlanner.getDesiredCentroidalMomentumPivotPosition(perfectCMP);
      icpOptimizationController.compute(currentTime, desiredICP, desiredICPVelocity, perfectCMP, desiredICP, omega0);

      FramePoint2D desiredCMP = new FramePoint2D();
      icpOptimizationController.getDesiredCMP(desiredCMP);
      PrintTools.debug("Desired CMP = " + desiredCMP);
      PrintTools.debug("Perfect CMP = " + perfectCMP);
   }

   private void setupContactableFeet()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.084;

         List<Point2D> contactPointsInSoleFrame = new ArrayList<>();
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(footLengthForControl / 2.0, -toeWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, -footWidthForControl / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLengthForControl / 2.0, footWidthForControl / 2.0));

         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
         FramePose startingPose = footPosesAtTouchdown.get(robotSide);
         startingPose.setToZero(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(0.15));
         contactableFoot.setSoleFrame(startingPose);

         contactableFeet.put(robotSide, contactableFoot);
      }
   }

   private void setupBipedSupportPolygons()
   {
      SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         ReferenceFrame ankleFrame = contactableFoot.getFrameAfterParentJoint();
         ankleFrames.put(robotSide, ankleFrame);
         ankleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, ankleFrame, robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));

         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         RigidBody foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         List<FramePoint2D> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, registry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);
      }

      ReferenceFrame midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame", worldFrame, ankleZUpFrames.get(RobotSide.LEFT), ankleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();

      bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, ankleZUpFrames, registry, null);
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      ICPControlPlane icpControlPlane = new ICPControlPlane(omega, ReferenceFrame.getWorldFrame(), 9.81, registry);
      icpControlPolygons = new ICPControlPolygons(icpControlPlane, midFeetZUpFrame, registry, null);
   }

   private static final ICPOptimizationParameters icpOptimizationParameters = new ICPOptimizationParameters()
   {
      @Override public boolean useSimpleOptimization()
      {
         return false;
      }

      @Override public int getMaximumNumberOfFootstepsToConsider()
      {
         return 5;
      }

      @Override public int numberOfFootstepsToConsider()
      {
         return 0;
      }

      @Override public double getForwardFootstepWeight()
      {
         return 5.0;
      }

      @Override public double getLateralFootstepWeight()
      {
         return 5.0;
      }

      @Override public double getFootstepRegularizationWeight()
      {
         return 0.0001;
      }

      @Override public double getFeedbackForwardWeight()
      {
         return 2.0;
      }

      @Override public double getFeedbackLateralWeight()
      {
         return 2.0;
      }

      @Override public double getFeedbackRegularizationWeight()
      {
         return 0.0001;
      }

      @Override public double getFeedbackParallelGain()
      {
         return 2.0;
      }

      @Override public double getFeedbackOrthogonalGain()
      {
         return 3.0;
      }

      @Override public double getDynamicRelaxationWeight()
      {
         return 1000.0;
      }

      @Override public double getDynamicRelaxationDoubleSupportWeightModifier()
      {
         return 1.0;
      }

      @Override public double getAngularMomentumMinimizationWeight()
      {
         return 0.5;
      }

      @Override public boolean scaleStepRegularizationWeightWithTime()
      {
         return false;
      }

      @Override public boolean scaleFeedbackWeightWithGain()
      {
         return false;
      }

      @Override public boolean scaleUpcomingStepWeights()
      {
         return false;
      }

      @Override public boolean useFeedbackRegularization()
      {
         return true;
      }

      @Override public boolean useStepAdjustment()
      {
         return true;
      }

      @Override public boolean useTimingOptimization()
      {
         return false;
      }

      @Override public boolean useAngularMomentum()
      {
         return true;
      }

      @Override public boolean useFootstepRegularization()
      {
         return true;
      }

      @Override public double getMinimumFootstepWeight()
      {
         return 0.0001;
      }

      @Override public double getMinimumFeedbackWeight()
      {
         return 0.0001;
      }

      @Override
      public double getMinimumTimeRemaining()
      {
         return 0.001;
      }

      @Override public double getAdjustmentDeadband()
      {
         return 0.0;
      }
   };

   private static final ContinuousCMPICPPlannerParameters icpPlannerParameters = new ContinuousCMPICPPlannerParameters()
   {
      @Override
      public int getNumberOfCoPWayPointsPerFoot()
      {
         return 1;
      }

      @Override
      public List<Vector2D> getCoPOffsets()
      {
         Vector2D entryOffset = new Vector2D();
         Vector2D exitOffset = new Vector2D();

         List<Vector2D> copOffsets = new ArrayList<>();
         copOffsets.add(entryOffset);
         copOffsets.add(exitOffset);

         return copOffsets;
      }

      @Override
      public List<Vector2D> getCoPForwardOffsetBounds()
      {
         Vector2D entryOffset = new Vector2D();
         Vector2D exitOffset = new Vector2D();

         List<Vector2D> copOffsets = new ArrayList<>();
         copOffsets.add(entryOffset);
         copOffsets.add(exitOffset);

         return copOffsets;
      }
   };

   private static final WalkingControllerParameters walkingControllerParameters = new WalkingControllerParameters()
   {

      @Override
      public boolean useOptimizationBasedICPController()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public double nominalHeightAboveAnkle()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double minimumHeightAboveAnkle()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double maximumHeightAboveAnkle()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getSecondContactThresholdForceIgnoringCoP()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getOmega0()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public MomentumOptimizationSettings getMomentumOptimizationSettings()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public ICPAngularMomentumModifierParameters getICPAngularMomentumModifierParameters()
      {
         return null;
      }

      @Override
      public double getMinimumSwingTimeForDisturbanceRecovery()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMaxICPErrorBeforeSingleSupportY()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMaxICPErrorBeforeSingleSupportX()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getMaximumLegLengthForSingularityAvoidance()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public String[] getJointsToIgnoreInController()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public double getICPErrorThresholdToSpeedUpSwing()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getHighCoPDampingDurationToPreventFootShakies()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public FootSwitchType getFootSwitchType()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public double getDefaultTransferTime()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getDefaultSwingTime()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getContactThresholdHeight()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getContactThresholdForce()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getCoPThresholdFraction()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public double getCoPErrorThresholdForHighCoPDamping()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public boolean finishSingleSupportWhenICPPlannerIsDone()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public boolean doPrepareManipulationForLocomotion()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public double defaultOffsetHeightAboveAnkle()
      {
         // TODO Auto-generated method stub
         return 0;
      }

      @Override
      public YoSE3PIDGainsInterface createToeOffFootControlGains(YoVariableRegistry registry)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public YoSE3PIDGainsInterface createSwingFootControlGains(YoVariableRegistry registry)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public ICPControlGains createICPControlGains()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public YoSE3PIDGainsInterface createHoldPositionFootControlGains(YoVariableRegistry registry)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public YoPDGains createCoMHeightControlGains(YoVariableRegistry registry)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public boolean allowAutomaticManipulationAbort()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public ToeOffParameters getToeOffParameters()
      {
         return new ToeOffParameters()
         {
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
         };
      }

      @Override
      public SwingTrajectoryParameters getSwingTrajectoryParameters()
      {
         return new SwingTrajectoryParameters()
         {
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
            public double getMinMechanicalLegLength()
            {
               return 0;
            }
         };
      }

      @Override
      public ICPOptimizationParameters getICPOptimizationParameters()
      {
         return icpOptimizationParameters;
      }

      @Override
      public SteppingParameters getSteppingParameters()
      {
         // TODO Auto-generated method stub
         return null;
      }
   };
}
