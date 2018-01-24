package us.ihmc.commonWalkingControlModules.capturePoint.optimization.simpleController;

import java.util.ArrayList;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationController;
import us.ihmc.commonWalkingControlModules.configurations.ICPAngularMomentumModifierParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ICPOptimizationControllerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 1e-3;

   private static final double footLength = 0.25;
   private static final double stanceWidth = 0.35;

   private final SideDependentList<FramePose3D> footPosesAtTouchdown = new SideDependentList<>(new FramePose3D(), new FramePose3D());
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
         public double getDynamicsObjectiveWeight()
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
      ICPOptimizationController controller = new ICPOptimizationController(walkingControllerParameters, optimizationParameters, bipedSupportPolygons,
                                                                           null, contactableFeet, controlDT, registry, null);


      double omega = walkingControllerParameters.getOmega0();

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.03, 0.06);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.01, 0.04);
      FrameVector2D desiredICPVelocity = new FrameVector2D();

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FrameVector2D icpError = new FrameVector2D();
      FramePoint2D currentICP = new FramePoint2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);

      controller.initializeForStanding(0.0);
      controller.compute(0.04, desiredICP, desiredICPVelocity, perfectCMP, currentICP, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
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
         public double getDynamicsObjectiveWeight()
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
      ICPOptimizationController controller = new ICPOptimizationController(walkingControllerParameters, optimizationParameters, bipedSupportPolygons,
                                                                           null, contactableFeet, controlDT, registry, null);


      double omega = walkingControllerParameters.getOmega0();

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.03, 0.06);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.01, 0.04);
      FrameVector2D desiredICPVelocity = new FrameVector2D();

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FrameVector2D icpError = new FrameVector2D();
      FramePoint2D currentICP = new FramePoint2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);

      controller.initializeForTransfer(0.0, RobotSide.LEFT, omega);
      controller.compute(0.04, desiredICP, desiredICPVelocity, perfectCMP, currentICP, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
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
         public double getDynamicsObjectiveWeight()
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
      ICPOptimizationController controller = new ICPOptimizationController(walkingControllerParameters, optimizationParameters, bipedSupportPolygons,
                                                                           null, contactableFeet, controlDT, registry, null);

      double omega = walkingControllerParameters.getOmega0();

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.03, 0.06);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.01, 0.04);
      FrameVector2D desiredICPVelocity = new FrameVector2D();

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.03, 0.06);
      FramePoint2D currentICP = new FramePoint2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);

      controller.initializeForStanding(0.0);
      controller.compute(0.04, desiredICP, desiredICPVelocity, perfectCMP, currentICP, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
      controller.getDesiredCMP(desiredCMP);

      FramePoint2D desiredCMPExpected = new FramePoint2D();
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
         public double getDynamicsObjectiveWeight()
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
      ICPOptimizationController controller = new ICPOptimizationController(walkingControllerParameters, optimizationParameters, bipedSupportPolygons,
                                                                           null, contactableFeet, controlDT, registry, null);

      double omega = walkingControllerParameters.getOmega0();

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.03, 0.06);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.01, 0.04);
      FrameVector2D desiredICPVelocity = new FrameVector2D();

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.03, 0.06);
      FramePoint2D currentICP = new FramePoint2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);

      controller.initializeForStanding(0.0);
      controller.compute(0.04, desiredICP, desiredICPVelocity, perfectCMP, currentICP, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
      controller.getDesiredCMP(desiredCMP);

      FramePoint2D desiredCMPExpected = new FramePoint2D();
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
         FramePose3D startingPose = footPosesAtTouchdown.get(robotSide);
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
         List<FramePoint2D> contactFramePoints = contactableFoot.getContactPoints2d();
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
      public double getFootstepRateWeight()
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
      public double getFeedbackRateWeight()
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
      public double getDynamicsObjectiveWeight()
      {
         return 500.0;
      }

      @Override
      public double getDynamicsObjectiveDoubleSupportWeightModifier()
      {
         return 1.0;
      }

      @Override
      public double getAngularMomentumMinimizationWeight()
      {
         return 50;
      }

      @Override
      public boolean scaleStepRateWeightWithTime()
      {
         return false;
      }

      @Override
      public boolean scaleFeedbackWeightWithGain()
      {
         return false;
      }

      @Override
      public boolean useFeedbackRate()
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
      public boolean useFootstepRate()
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
      public PDGains getCoMHeightControlGains()
      {
         return null;
      }

      @Override
      public PIDSE3Gains getSwingFootControlGains()
      {
         return null;
      }

      @Override
      public PIDSE3Gains getHoldPositionFootControlGains()
      {
         return null;
      }

      @Override
      public PIDSE3Gains getToeOffFootControlGains()
      {
         return null;
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
         return new TestSteppingParameters();
      }
   }

   private class TestSteppingParameters implements SteppingParameters
   {

      @Override
      public double getMaxStepLength()
      {
         return 1.0;
      }

      @Override
      public double getDefaultStepLength()
      {
         return 0.5;
      }

      @Override
      public double getMaxStepWidth()
      {
         return 0.5;
      }

      @Override
      public double getMinStepWidth()
      {
         return 0.05;
      }

      @Override
      public double getInPlaceWidth()
      {
         return 0.2;
      }

      @Override
      public double getDesiredStepForward()
      {
         return 0.3;
      }

      @Override
      public double getStepPitch()
      {
         return 0;
      }

      @Override
      public double getMaxStepUp()
      {
         return 0.2;
      }

      @Override
      public double getMaxStepDown()
      {
         return 0.2;
      }

      @Override
      public double getMaxSwingHeightFromStanceFoot()
      {
         return 0;
      }

      @Override
      public double getMaxAngleTurnOutwards()
      {
         return 0.3;
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
         return 0.1;
      }

      @Override
      public double getToeWidth()
      {
         return 0.1;
      }

      @Override
      public double getFootLength()
      {
         return 0.2;
      }

      @Override
      public double getActualFootWidth()
      {
         return 0.1;
      }

      @Override
      public double getActualFootLength()
      {
         return 0.2;
      }
   }

}
