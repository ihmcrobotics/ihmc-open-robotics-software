package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.simpleController;

import java.util.ArrayList;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.robotics.controllers.YoPDGains;
import us.ihmc.robotics.controllers.YoSE3PIDGainsInterface;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SimpleAdjustmentICPOptimizationControllerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 1e-3;

   private static final double footLength = 0.25;
   private static final double stanceWidth = 0.35;

   private final SideDependentList<FramePose> footPosesAtTouchdown = new SideDependentList<>(new FramePose(), new FramePose());
   private final SideDependentList<ReferenceFrame> ankleFrames = new SideDependentList<>();

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testStandingWithPerfectTracking() throws Exception
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");

      double feedbackGain = 2.0;
      TestICPOptimizationParameters optimizationParameters = new TestICPOptimizationParameters()
      {
         @Override
         public double getFeedbackParallelGain()
         {
            return feedbackGain;
         }

         @Override
         public double getFeedbackOrthogonalGain()
         {
            return feedbackGain;
         }

         @Override
         public double getDynamicRelaxationWeight()
         {
            return 10000.0;
         }

         @Override
         public boolean useAngularMomentum()
         {
            return false;
         }

         @Override
         public boolean useStepAdjustment()
         {
            return false;
         }
      };

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;
      SimpleAdjustmentICPOptimizationController controller = new SimpleAdjustmentICPOptimizationController(optimizationParameters, walkingControllerParameters,
                                                                                                           bipedSupportPolygons, null, contactableFeet,
                                                                                                           controlDT, registry, null);


      double omega = walkingControllerParameters.getOmega0();

      FramePoint2d desiredICP = new FramePoint2d(worldFrame, 0.03, 0.06);
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.01, 0.04);
      FrameVector2d desiredICPVelocity = new FrameVector2d();

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FrameVector2d icpError = new FrameVector2d();
      FramePoint2d currentICP = new FramePoint2d();
      currentICP.set(desiredICP);
      currentICP.add(icpError);

      controller.initializeForStanding(0.0);
      controller.compute(0.04, desiredICP, desiredICPVelocity, perfectCMP, currentICP, omega);

      FramePoint2d desiredCMP = new FramePoint2d();
      controller.getDesiredCMP(desiredCMP);

      Assert.assertTrue(desiredCMP.epsilonEquals(perfectCMP, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testTransferWithPerfectTracking() throws Exception
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");

      double feedbackGain = 2.0;
      TestICPOptimizationParameters optimizationParameters = new TestICPOptimizationParameters()
      {
         @Override
         public double getFeedbackParallelGain()
         {
            return feedbackGain;
         }

         @Override
         public double getFeedbackOrthogonalGain()
         {
            return feedbackGain;
         }

         @Override
         public double getDynamicRelaxationWeight()
         {
            return 10000.0;
         }

         @Override
         public boolean useAngularMomentum()
         {
            return false;
         }

         @Override
         public boolean useStepAdjustment()
         {
            return false;
         }
      };

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;
      SimpleAdjustmentICPOptimizationController controller = new SimpleAdjustmentICPOptimizationController(optimizationParameters, walkingControllerParameters,
                                                                                                           bipedSupportPolygons, null, contactableFeet, controlDT,
                                                                                                           registry, null);


      double omega = walkingControllerParameters.getOmega0();

      FramePoint2d desiredICP = new FramePoint2d(worldFrame, 0.03, 0.06);
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.01, 0.04);
      FrameVector2d desiredICPVelocity = new FrameVector2d();

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FrameVector2d icpError = new FrameVector2d();
      FramePoint2d currentICP = new FramePoint2d();
      currentICP.set(desiredICP);
      currentICP.add(icpError);

      controller.initializeForTransfer(0.0, RobotSide.LEFT, omega);
      controller.compute(0.04, desiredICP, desiredICPVelocity, perfectCMP, currentICP, omega);

      FramePoint2d desiredCMP = new FramePoint2d();
      controller.getDesiredCMP(desiredCMP);

      Assert.assertTrue(desiredCMP.epsilonEquals(perfectCMP, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testStandingConstrained() throws Exception
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");

      double feedbackGain = 2.0;
      TestICPOptimizationParameters optimizationParameters = new TestICPOptimizationParameters()
      {
         @Override
         public double getFeedbackParallelGain()
         {
            return feedbackGain;
         }

         @Override
         public double getFeedbackOrthogonalGain()
         {
            return feedbackGain;
         }

         @Override
         public double getDynamicRelaxationWeight()
         {
            return 10000.0;
         }

         @Override
         public boolean useAngularMomentum()
         {
            return false;
         }

         @Override
         public boolean useStepAdjustment()
         {
            return false;
         }
      };

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;
      SimpleAdjustmentICPOptimizationController controller = new SimpleAdjustmentICPOptimizationController(optimizationParameters, walkingControllerParameters,
                                                                                                           bipedSupportPolygons, null, contactableFeet, controlDT,
                                                                                                           registry, null);


      double omega = walkingControllerParameters.getOmega0();

      FramePoint2d desiredICP = new FramePoint2d(worldFrame, 0.03, 0.06);
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.01, 0.04);
      FrameVector2d desiredICPVelocity = new FrameVector2d();

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FrameVector2d icpError = new FrameVector2d(worldFrame, 0.03, 0.06);
      FramePoint2d currentICP = new FramePoint2d();
      currentICP.set(desiredICP);
      currentICP.add(icpError);

      controller.initializeForStanding(0.0);
         controller.compute(0.04, desiredICP, desiredICPVelocity, perfectCMP, currentICP, omega);

      FramePoint2d desiredCMP = new FramePoint2d();
      controller.getDesiredCMP(desiredCMP);

      FramePoint2d desiredCMPExpected = new FramePoint2d();
      desiredCMPExpected.set(icpError);
      desiredCMPExpected.scale(feedbackGain + 1.0);
      desiredCMPExpected.add(perfectCMP);

      double maxY = stanceWidth / 2.0;
      double maxX = footLength / 2.0;

      desiredCMPExpected.setX(Math.min(maxX, desiredCMPExpected.getX()));
      desiredCMPExpected.setY(Math.min(maxY, desiredCMPExpected.getY()));

      Assert.assertTrue(desiredCMP.epsilonEquals(desiredCMPExpected, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 21000)
   public void testStandingConstrainedWithAngularMomentum() throws Exception
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");

      double feedbackGain = 2.0;
      TestICPOptimizationParameters optimizationParameters = new TestICPOptimizationParameters()
      {
         @Override
         public double getFeedbackParallelGain()
         {
            return feedbackGain;
         }

         @Override
         public double getFeedbackOrthogonalGain()
         {
            return feedbackGain;
         }

         @Override
         public double getDynamicRelaxationWeight()
         {
            return 10000.0;
         }

         @Override
         public boolean useAngularMomentum()
         {
            return true;
         }

         @Override
         public double getAngularMomentumMinimizationWeight()
         {
            return 25.0;
         }

         @Override
         public boolean useStepAdjustment()
         {
            return false;
         }
      };

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;
      SimpleAdjustmentICPOptimizationController controller = new SimpleAdjustmentICPOptimizationController(optimizationParameters, walkingControllerParameters,
                                                                                                           bipedSupportPolygons, null, contactableFeet, controlDT,
                                                                                                           registry, null);


      double omega = walkingControllerParameters.getOmega0();

      FramePoint2d desiredICP = new FramePoint2d(worldFrame, 0.03, 0.06);
      FramePoint2d perfectCMP = new FramePoint2d(worldFrame, 0.01, 0.04);
      FrameVector2d desiredICPVelocity = new FrameVector2d();

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FrameVector2d icpError = new FrameVector2d(worldFrame, 0.03, 0.06);
      FramePoint2d currentICP = new FramePoint2d();
      currentICP.set(desiredICP);
      currentICP.add(icpError);

      controller.initializeForStanding(0.0);
      controller.compute(0.04, desiredICP, desiredICPVelocity, perfectCMP, currentICP, omega);

      FramePoint2d desiredCMP = new FramePoint2d();
      controller.getDesiredCMP(desiredCMP);

      FramePoint2d desiredCMPExpected = new FramePoint2d();
      desiredCMPExpected.set(icpError);
      desiredCMPExpected.scale(feedbackGain + 1.0);
      desiredCMPExpected.add(perfectCMP);

      Assert.assertTrue(desiredCMP.epsilonEquals(desiredCMPExpected, epsilon));
   }

   private SideDependentList<FootSpoof> setupContactableFeet(double footLength, double footWidth, double totalWidth)
   {
      SideDependentList<FootSpoof> contactableFeet = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         double xToAnkle = 0.0;
         double yToAnkle = 0.0;
         double zToAnkle = 0.084;

         List<Point2D> contactPointsInSoleFrame = new ArrayList<>();
         contactPointsInSoleFrame.add(new Point2D(footLength / 2.0, footWidth / 2.0));
         contactPointsInSoleFrame.add(new Point2D(footLength / 2.0, -footWidth / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
         contactPointsInSoleFrame.add(new Point2D(-footLength / 2.0, footWidth / 2.0));

         FootSpoof contactableFoot = new FootSpoof(sidePrefix + "Foot", xToAnkle, yToAnkle, zToAnkle, contactPointsInSoleFrame, 0.0);
         FramePose startingPose = footPosesAtTouchdown.get(robotSide);
         startingPose.setToZero(worldFrame);
         startingPose.setY(robotSide.negateIfRightSide(0.5 * (totalWidth - footWidth)));
         contactableFoot.setSoleFrame(startingPose);

         contactableFeet.put(robotSide, contactableFoot);
      }

      return contactableFeet;
   }

   private BipedSupportPolygons setupBipedSupportPolygons(SideDependentList<FootSpoof> contactableFeet, YoVariableRegistry registry)
   {
      SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<>();
      SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         ReferenceFrame ankleFrame = contactableFoot.getFrameAfterParentJoint();
         ankleFrames.put(robotSide, ankleFrame);
         ankleZUpFrames.put(robotSide, new ZUpFrame(worldFrame, ankleFrame, robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));

         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         RigidBody foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         List<FramePoint2d> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, registry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);
      }

      ReferenceFrame midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame", worldFrame, ankleZUpFrames.get(RobotSide.LEFT), ankleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();

      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(ankleZUpFrames, midFeetZUpFrame, ankleZUpFrames, registry, null);
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      return bipedSupportPolygons;
   }

   private class TestICPOptimizationParameters extends ICPOptimizationParameters
   {
      @Override
      public boolean useSimpleOptimization()
      {
         return true;
      }

      @Override
      public int numberOfFootstepsToConsider()
      {
         return 1;
      }

      @Override
      public double getForwardFootstepWeight()
      {
         return 10.0;
      }

      @Override
      public double getLateralFootstepWeight()
      {
         return 10.0;
      }

      @Override
      public double getFootstepRegularizationWeight()
      {
         return 0.0001;
      }

      @Override
      public double getFeedbackForwardWeight()
      {
         return 0.5;
      }

      @Override
      public double getFeedbackLateralWeight()
      {
         return 0.5;
      }

      @Override
      public double getFeedbackRegularizationWeight()
      {
         return 0.0001;
      }

      @Override
      public double getFeedbackParallelGain()
      {
         return 3.0;
      }

      @Override
      public double getFeedbackOrthogonalGain()
      {
         return 2.5;
      }

      @Override
      public double getDynamicRelaxationWeight()
      {
         return 500.0;
      }

      @Override
      public double getDynamicRelaxationDoubleSupportWeightModifier()
      {
         return 1.0;
      }

      @Override
      public double getAngularMomentumMinimizationWeight()
      {
         return 50;
      }

      @Override
      public boolean scaleStepRegularizationWeightWithTime()
      {
         return false;
      }

      @Override
      public boolean scaleFeedbackWeightWithGain()
      {
         return false;
      }

      @Override
      public boolean scaleUpcomingStepWeights()
      {
         return false;
      }

      @Override
      public boolean useFeedbackRegularization()
      {
         return false;
      }

      @Override
      public boolean useStepAdjustment()
      {
         return false;
      }

      @Override
      public boolean useAngularMomentum()
      {
         return false;
      }

      @Override
      public boolean useTimingOptimization()
      {
         return false;
      }

      @Override
      public boolean useFootstepRegularization()
      {
         return false;
      }

      @Override
      public double getMinimumFootstepWeight()
      {
         return 0.0001;
      }

      @Override
      public double getMinimumFeedbackWeight()
      {
         return 0.0001;
      }


      @Override
      public double getMinimumTimeRemaining()
      {
         return 0.0001;
      }

      @Override
      public double getAdjustmentDeadband()
      {
         return 0.03;
      }

      @Override
      public double getSafeCoPDistanceToEdge()
      {
         return 0.0;
      }
   }

   private class TestWalkingControllerParameters extends WalkingControllerParameters
   {
      @Override
      public double getOmega0()
      {
         return 3.0;
      }

      @Override
      public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
      {
         return false;
      }

      @Override
      public double getMinimumSwingTimeForDisturbanceRecovery()
      {
         return 0;
      }

      @Override
      public double getICPErrorThresholdToSpeedUpSwing()
      {
         return 0;
      }

      @Override
      public boolean allowAutomaticManipulationAbort()
      {
         return false;
      }

      @Override
      public boolean useOptimizationBasedICPController()
      {
         return false;
      }

      @Override
      public ICPControlGains createICPControlGains()
      {
         return null;
      }

      @Override
      public YoPDGains createCoMHeightControlGains(YoVariableRegistry registry)
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
      public boolean doPrepareManipulationForLocomotion()
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
      public ICPAngularMomentumModifierParameters getICPAngularMomentumModifierParameters()
      {
         return null;
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
      public ToeOffParameters getToeOffParameters()
      {
         return null;
      }

      @Override
      public SwingTrajectoryParameters getSwingTrajectoryParameters()
      {
         return null;
      }

      @Override
      public double getMaximumLegLengthForSingularityAvoidance()
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
      public ICPOptimizationParameters getICPOptimizationParameters()
      {
         return new TestICPOptimizationParameters();
      }

      @Override
      public SteppingParameters getSteppingParameters()
      {
         return null;
      }
   }
}
