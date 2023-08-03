package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class HeuristicICPControllerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 1e-3;

   private static final double footLength = 0.25;
   private static final double stanceWidth = 0.35;

   private static boolean visualize = false;

   @BeforeEach
   public void setup()
   {
      visualize &= !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   }

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   public static ICPControllerInterface createICPController(TestICPControllerParameters icpControllerParameters,
                                                      double controlDT,
                                                      YoRegistry registry,
                                                      YoGraphicsListRegistry yoGraphicsListRegistry)
   {   
      return new HeuristicICPController(icpControllerParameters,
                                        controlDT,
                                        registry,
                                        yoGraphicsListRegistry);

   }

   public static TestICPControllerParameters createTestICPControllerParameters(double kpParallel, double kpOrthogonal)
   {
      TestICPControllerParameters icpControllerParameters = new TestICPControllerParameters()
      {
         private FeedbackProjectionOperator feedbackProjectionOperator;
         private FeedForwardAlphaCalculator feedForwardAlphaCalculator;

         @Override
         public ICPControlGainsReadOnly getICPFeedbackGains()
         {
            ICPControlGains gains = new ICPControlGains();
            gains.setKpParallelToMotion(kpParallel);
            gains.setKpOrthogonalToMotion(kpOrthogonal);

            return gains;
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
         public void createFeedForwardAlphaCalculator(YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
         {
            feedForwardAlphaCalculator = new ErrorBasedFeedForwardAlphaCalculator("", registry);
         }

         @Override
         public void createFeedbackProjectionOperator(YoRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
         {
            feedbackProjectionOperator = new CoPProjectionTowardsMidpoint(registry, yoGraphicsListRegistry);
         }

         @Override
         public FeedbackProjectionOperator getFeedbackProjectionOperator()
         {
            return feedbackProjectionOperator;
         }

         @Override
         public FeedForwardAlphaCalculator getFeedForwardAlphaCalculator()
         {
            return feedForwardAlphaCalculator;
         }
      };
      
      return icpControllerParameters;
   }

   public static void computeDesiredICPVelocityFromPerfectCMP(double omega, FramePoint2DReadOnly desiredICP, FramePoint2DReadOnly perfectCMP, FrameVector2D desiredICPVelocityToPack)
   {
      desiredICPVelocityToPack.set(desiredICP);
      desiredICPVelocityToPack.sub(perfectCMP);
      desiredICPVelocityToPack.scale(omega);
   }

   private static FramePoint2DBasics computeExpectedICPMeetupPoint(FramePoint2DReadOnly desiredICP,
                                                                   FramePoint2DReadOnly currentICP,
                                                                   FrameVector2DReadOnly desiredICPVelocity,
                                                                   FrameVector2D expectedControlICPVelocity)
   {
      FramePoint2DBasics expectedICPMeetupPoint = new FramePoint2D(desiredICP.getReferenceFrame());

      FrameLine2D desiredICPLine = new FrameLine2D(desiredICP, desiredICPVelocity);
      FrameLine2D expectedICPLine = new FrameLine2D(currentICP, expectedControlICPVelocity);

      boolean intersectionExists = desiredICPLine.intersectionWith(expectedICPLine, expectedICPMeetupPoint);
      if (!intersectionExists)
      {
         expectedICPMeetupPoint.setToNaN();
         return expectedICPMeetupPoint;
      }

      FrameVector2D desiredToMeetup = new FrameVector2D(expectedICPMeetupPoint);
      desiredToMeetup.sub(desiredICP);
      
      if (desiredToMeetup.length() < 0.002)
         return expectedICPMeetupPoint;

      if (desiredICPVelocity.dot(desiredToMeetup) < 0.0)
      {
         expectedICPMeetupPoint.setToNaN();
      }

      return expectedICPMeetupPoint;
   }

   @Test
   public void testKeepAwayFromEdgeIfNotNecessaryInSingleSupport() throws Exception
   {
      YoRegistry registry = new YoRegistry("ICPControllerTest");
      double feedbackGain = 2.0;
      TestICPControllerParameters icpControllerParameters = createTestICPControllerParameters(feedbackGain, feedbackGain);

      double stanceWidth = 0.4;
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, RobotSide.RIGHT, registry);
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();

      double controlDT = 0.001;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      ICPControllerInterface controller = createICPController(icpControllerParameters,
                                                              controlDT,
                                                              registry,
                                                              yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);
      ((YoBoolean) registry.findVariable("copProjectionUseCoPProjection")).set(true);

      ICPControllerTestVisualizer visualizer = null;
      if (visualize)
      {
         visualizer = new ICPControllerTestVisualizer(registry, yoGraphicsListRegistry);
      }

      double omega = 3.0;

      FrameVector2D desiredICPVelocity = new FrameVector2D(worldFrame);
      FramePoint2D desiredICP = new FramePoint2D(worldFrame);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame);

      FramePoint2D finalICP = new FramePoint2D();
      finalICP.setToNaN();

      desiredICP.set(0.0, 0.12);
      perfectCMP.set(0.0, -0.15);
      FramePoint2D perfectCoP = new FramePoint2D(perfectCMP);

      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);

      FramePoint2D currentICP = new FramePoint2D(worldFrame, 0.0, 0.10);

      FramePoint2D currentCoMPosition = new FramePoint2D(currentICP);

      if (visualize)
         visualizer.updateInputs(omega, supportPolygonInWorld, desiredICP, desiredICPVelocity, perfectCMP, perfectCoP, currentICP, currentCoMPosition);

      controller.initialize();
      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, finalICP, perfectCMP, currentICP, currentCoMPosition, omega);

      FrameVector2D expectedControlICPVelocity = new FrameVector2D(worldFrame);
      FramePoint2D desiredCMP = new FramePoint2D(controller.getDesiredCMP());
      FramePoint2D desiredCoP = new FramePoint2D(controller.getDesiredCoP());

      expectedControlICPVelocity.sub(currentICP, desiredCMP);
      expectedControlICPVelocity.scale(omega);

      FramePoint2DBasics expectedICPMeetupPoint = computeExpectedICPMeetupPoint(desiredICP, currentICP, desiredICPVelocity, expectedControlICPVelocity);

      if (visualize)
      {
         visualizer.updateOutputs(desiredCoP, desiredCMP, expectedControlICPVelocity, expectedICPMeetupPoint);
      }

      FrameLine2D lineFromICPTODesired = new FrameLine2D(desiredICP, currentICP);
      double distanceFromLineToProjection = lineFromICPTODesired.distance(desiredCMP);
      double distanceFromMiddleOfFootToCMP = desiredCMP.distance(perfectCMP);

      if (visualize)
      {
         ThreadTools.sleepForever();
      }

      Assert.assertTrue("distanceFromMiddleOfFootToCMP = " + distanceFromMiddleOfFootToCMP + ". It should be near zero.",
                        distanceFromMiddleOfFootToCMP < 0.015);
      Assert.assertTrue("distanceFromLineToProjection = " + distanceFromLineToProjection + ". It should be near zero.", distanceFromLineToProjection < 0.005);
   }

   @Test
   public void testProjectOnLineFromICPToDesired() throws Exception
   {
      YoRegistry registry = new YoRegistry("ICPControllerTest");
      double feedbackGain = 2.0;
      TestICPControllerParameters icpControllerParameters = createTestICPControllerParameters(feedbackGain, feedbackGain);

      double footLength = 0.25;
      double stanceWidth = 0.3;
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();

      double controlDT = 0.001;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      ICPControllerInterface controller = createICPController(icpControllerParameters,
                                                              controlDT,
                                                              registry,
                                                              yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);
      ((YoBoolean) registry.findVariable("copProjectionUseCoPProjection")).set(true);

      ICPControllerTestVisualizer visualizer = null;
      if (visualize)
      {
         visualizer = new ICPControllerTestVisualizer(registry, yoGraphicsListRegistry);
      }

      double omega = 3.0;

      FrameVector2D desiredICPVelocity = new FrameVector2D(worldFrame);
      FramePoint2D desiredICP = new FramePoint2D(worldFrame);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame);

      desiredICP.set(0.03, 0.06);
      perfectCMP.set(0.01, 0.05);

      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);

      FramePoint2D perfectCoP = new FramePoint2D(perfectCMP);

      FramePoint2D finalICP = new FramePoint2D();
      finalICP.setToNaN();

      FrameVector2D icpError = new FrameVector2D(desiredICPVelocity);
      icpError.normalize();
      icpError.scale(-0.1);

      FramePoint2D currentICP = new FramePoint2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);
      FramePoint2D currentCoMPosition = new FramePoint2D(currentICP);

      if (visualize)
         visualizer.updateInputs(omega, supportPolygonInWorld, desiredICP, desiredICPVelocity, perfectCMP, perfectCoP, currentICP, currentCoMPosition);

      controller.initialize();
      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, finalICP, perfectCMP, currentICP, currentCoMPosition, omega);

      FrameVector2D expectedControlICPVelocity = new FrameVector2D(worldFrame);
      FramePoint2D desiredCMP = new FramePoint2D(controller.getDesiredCMP());
      FramePoint2D desiredCoP = new FramePoint2D(controller.getDesiredCoP());

      expectedControlICPVelocity.sub(currentICP, desiredCMP);
      expectedControlICPVelocity.scale(omega);

      FramePoint2DBasics expectedICPMeetupPoint = computeExpectedICPMeetupPoint(desiredICP, currentICP, desiredICPVelocity, expectedControlICPVelocity);

      if (visualize)
      {
         visualizer.updateOutputs(desiredCoP, desiredCMP, expectedControlICPVelocity, expectedICPMeetupPoint);
      }

      FrameLine2D lineFromICPTODesired = new FrameLine2D(desiredICP, currentICP);
      double distanceFromLineToProjection = lineFromICPTODesired.distance(desiredCMP);

      if (visualize)
      {
         ThreadTools.sleepForever();
      }

      Assert.assertTrue("distanceFromLineToProjection = " + distanceFromLineToProjection + ". It should be near zero.", distanceFromLineToProjection < 0.005);
   }

   @Test
   public void testStandingWithPerfectTracking() throws Exception
   {
      YoRegistry registry = new YoRegistry("ICPControllerTest");
      double feedbackGain = 2.0;
      TestICPControllerParameters icpControllerParameters = createTestICPControllerParameters(feedbackGain, feedbackGain);

      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();

      double controlDT = 0.001;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      ICPControllerInterface controller = createICPController(icpControllerParameters,
                                                              controlDT,
                                                              registry,
                                                              yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);
      ((YoBoolean) registry.findVariable("copProjectionUseCoPProjection")).set(true);

      double omega = 3.0;

      FrameVector2D desiredICPVelocity = new FrameVector2D(worldFrame);
      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.03, 0.06);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.01, 0.04);

      FramePoint2D finalICP = new FramePoint2D();
      finalICP.setToNaN();

      desiredICP.set(0.03, 0.06);
      perfectCMP.set(0.01, 0.04);

      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);

      FramePoint2D perfectCoP = new FramePoint2D(perfectCMP);

      FrameVector2D icpError = new FrameVector2D();
      FramePoint2D currentICP = new FramePoint2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);
      FramePoint2D currentCoMPosition = new FramePoint2D(currentICP);

      controller.initialize();
      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, finalICP, perfectCMP, currentICP, currentCoMPosition, omega);

      FrameVector2D expectedControlICPVelocity = new FrameVector2D(worldFrame);
      FramePoint2D desiredCMP = new FramePoint2D(controller.getDesiredCMP());
      FramePoint2D desiredCoP = new FramePoint2D(controller.getDesiredCoP());

      expectedControlICPVelocity.sub(currentICP, desiredCMP);
      expectedControlICPVelocity.scale(omega);

      FramePoint2DBasics expectedICPMeetupPoint = computeExpectedICPMeetupPoint(desiredICP, currentICP, desiredICPVelocity, expectedControlICPVelocity);

      ICPControllerTestVisualizer visualizer = null;
      if (visualize)
      {
         visualizer = new ICPControllerTestVisualizer(registry, yoGraphicsListRegistry);
         visualizer.updateInputs(omega, supportPolygonInWorld, desiredICP, desiredICPVelocity, perfectCMP, perfectCoP, currentICP, currentCoMPosition);
         visualizer.updateOutputs(desiredCoP, desiredCMP, expectedControlICPVelocity, expectedICPMeetupPoint);
      }

      Assert.assertTrue(desiredCMP.epsilonEquals(perfectCMP, epsilon));

      if (visualize)
      {
         ThreadTools.sleepForever();
      }
   }

   @Test
   public void testTransferWithPerfectTracking() throws Exception
   {
      YoRegistry registry = new YoRegistry("ICPControllerTest");
      double feedbackGain = 2.0;
      TestICPControllerParameters icpControllerParameters = createTestICPControllerParameters(feedbackGain, feedbackGain);

      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();

      double controlDT = 0.001;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      ICPControllerInterface controller = createICPController(icpControllerParameters,
                                                              controlDT,
                                                              registry,
                                                              yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);
      ((YoBoolean) registry.findVariable("copProjectionUseCoPProjection")).set(true);

      double omega = 3.0;

      FramePoint2D finalICP = new FramePoint2D();
      finalICP.setToNaN();

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.03, 0.06);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.01, 0.04);
      FrameVector2D desiredICPVelocity = new FrameVector2D();

      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);

      FrameVector2D icpError = new FrameVector2D();
      FramePoint2D currentICP = new FramePoint2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);
      FramePoint2D currentCoM = new FramePoint2D(currentICP);

      controller.initialize();
      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, finalICP, perfectCMP, currentICP, currentCoM, omega);

      Assert.assertTrue(controller.getDesiredCMP().epsilonEquals(perfectCMP, epsilon));
   }

   private static SideDependentList<FootSpoof> setupContactableFeet(double footLength, double footWidth, double totalWidth)
   {
      SideDependentList<FramePose3D> footPosesAtTouchdown = new SideDependentList<>(new FramePose3D(), new FramePose3D());
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

   private static BipedSupportPolygons setupBipedSupportPolygons(SideDependentList<FootSpoof> contactableFeet, YoRegistry registry)
   {
      return setupBipedSupportPolygons(contactableFeet, null, registry);
   }

   private static BipedSupportPolygons setupBipedSupportPolygons(SideDependentList<FootSpoof> contactableFeet, RobotSide supportFoot, YoRegistry registry)
   {
      SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>();

      SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
      SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         if (contactableFoot != null)
         {
            soleZUpFrames.put(robotSide, new ZUpFrame(contactableFoot.getSoleFrame(), robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));

            String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
            RigidBodyBasics foot = contactableFoot.getRigidBody();
            ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
            soleFrames.put(robotSide, soleFrame);
            List<FramePoint2D> contactFramePoints = contactableFoot.getContactPoints2d();
            double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
            YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix
                  + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, registry);

            if ((supportFoot == null) || (supportFoot == robotSide))
               yoPlaneContactState.setFullyConstrained();
            else
               yoPlaneContactState.clear();
            contactStates.put(robotSide, yoPlaneContactState);
         }
      }

      ReferenceFrame midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame",
                                                            worldFrame,
                                                            soleZUpFrames.get(RobotSide.LEFT),
                                                            soleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();

      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(midFeetZUpFrame, soleZUpFrames, soleFrames, registry, null);
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      return bipedSupportPolygons;
   }

   public static class TestICPControllerParameters extends ICPControllerParameters
   {
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
      public ICPControlGainsReadOnly getICPFeedbackGains()
      {
         ICPControlGains gains = new ICPControlGains();
         gains.setKpParallelToMotion(3.0);
         gains.setKpOrthogonalToMotion(2.5);

         return gains;
      }
      @Override
      public double getDynamicsObjectiveWeight()
      {
         return 500.0;
      }

      @Override
      public double getAngularMomentumMinimizationWeight()
      {
         return 50;
      }

      @Override
      public boolean scaleFeedbackWeightWithGain()
      {
         return false;
      }

      @Override
      public boolean useAngularMomentum()
      {
         return false;
      }

      @Override
      public double getSafeCoPDistanceToEdge()
      {
         return 0.0;
      }
   }
   
}
