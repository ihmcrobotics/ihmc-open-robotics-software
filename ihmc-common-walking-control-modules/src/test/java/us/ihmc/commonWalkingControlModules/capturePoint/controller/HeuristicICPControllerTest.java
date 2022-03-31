package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment.StepAdjustmentParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDSE3Configuration;
import us.ihmc.robotics.referenceFrames.MidFrameZUpFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

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

   private ICPControllerInterface createICPController(TestICPControllerParameters icpControllerParameters,
                                                      double controlDT,
                                                      YoRegistry registry,
                                                      YoGraphicsListRegistry yoGraphicsListRegistry)
   {   
      return new HeuristicICPController(icpControllerParameters,
                                        controlDT,
                                        registry,
                                        yoGraphicsListRegistry);

   }

   public static void main(String[] args) throws Exception
   {
      HeuristicICPControllerTest test = new HeuristicICPControllerTest();
      //      test.visualizeRandom();
//      test.visualizeCasesOfInterest();
      
      double footLength = 0.25;
      double footWidth = 0.1;
      double stanceWidth = 0.35;
      test.visualizeOverGrid(footLength, footWidth, stanceWidth);
   }

   public void visualizeOverGrid(double footLength, double footWidth, double stanceWidth) throws Exception
   {
      YoRegistry registry = new YoRegistry("ICPControllerTest");
      double kpParallel = 1.0;
      double kpOrthogonal = 2.0;
      TestICPControllerParameters icpControllerParameters = createTestICPControllerParameters(kpParallel, kpOrthogonal);

      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, footWidth, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();

      double controlDT = 0.001;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      ICPControllerInterface controller = createICPController(icpControllerParameters,
                                                              controlDT,
                                                              registry,
                                                              yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      ICPControllerTestVisualizer visualizer = new ICPControllerTestVisualizer(60000, registry, yoGraphicsListRegistry);

      ICPControllerTestCase testCase = new ICPControllerTestCase();

      double omega = 3.0;
      testCase.setOmega(omega);

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.15, 0.0);
      FramePoint2D perfectCoP = new FramePoint2D(worldFrame, -0.25, 0.0);
      FrameVector2D perfectCMPOffset = new FrameVector2D(worldFrame, 0.0, 0.0);

      FramePoint2D perfectCMP = new FramePoint2D(perfectCoP);
      perfectCMP.add(perfectCMPOffset);

      FramePoint2D currentICP = new FramePoint2D(desiredICP);
      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.0, 0.0);
      currentICP.add(icpError);

      FramePoint2D currentCoMPosition = new FramePoint2D(worldFrame, -0.02, 0.0);

      FrameVector2D desiredICPVelocity = new FrameVector2D(worldFrame);
      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);

      testCase.setSupportPolygonInWorld(supportPolygonInWorld);
      testCase.setDesiredICP(desiredICP);
      testCase.setPerfectCoP(perfectCoP);
      testCase.setPerfectCMPOffset(perfectCMPOffset);
      testCase.setDesiredICPVelocity(desiredICPVelocity);
      testCase.setCurrentICP(currentICP);
      testCase.setCurrentCoMPosition(currentCoMPosition);

      solveAndVisualize(controller, visualizer, testCase);

      double xLowerBound = -0.2;
      double xUpperBound = 0.4;
      double yBounds = 0.2;
      double yIncrement = 0.002;
      double xIncrement = 0.002;

      boolean incrementY=true;
      double y = -yBounds;

      for (double x = xLowerBound; x< xUpperBound; x=x+xIncrement)
      {
         while(true)
         {
            testCase = new ICPControllerTestCase(testCase);
            currentICP.set(x, y);
            testCase.setCurrentICP(currentICP);

            computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);
            testCase.setDesiredICPVelocity(desiredICPVelocity);

            solveAndVisualize(controller, visualizer, testCase);

            if (incrementY)
            {
               y = y + yIncrement;
               if (y >= yBounds)
               {
                  incrementY = !incrementY;
                  break;
               }
            }
            else
            {
               y = y - yIncrement;
               if (y <= -yBounds)
               {
                  incrementY = !incrementY;
                  break;
               }
            }
         }
      }

      visualizer.cropBuffer();
      ThreadTools.sleepForever();
   }
   
   public void visualizeCasesOfInterest() throws Exception
   {
      YoRegistry registry = new YoRegistry("ICPControllerTest");
      double kpParallel = 1.0;
      double kpOrthogonal = 2.0;
      TestICPControllerParameters icpControllerParameters = createTestICPControllerParameters(kpParallel, kpOrthogonal);
      
      double footLength = 10.25;
      double footWidth = 10.1;
      double stanceWidth = 10.35;
      
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, footWidth, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();

      double controlDT = 0.001;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      ICPControllerInterface controller = createICPController(icpControllerParameters,
                                                              controlDT,
                                                              registry,
                                                              yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      ICPControllerTestVisualizer visualizer = new ICPControllerTestVisualizer(registry, yoGraphicsListRegistry);

      ICPControllerTestCase testCase = new ICPControllerTestCase();

      double omega = 3.0;
      testCase.setOmega(omega);

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.04, 0.06);
      FramePoint2D perfectCoP = new FramePoint2D(worldFrame, 0.0, 0.06); //0.01, 0.01);
      FrameVector2D perfectCMPOffset = new FrameVector2D(worldFrame, 0.0, 0.0);

      FramePoint2D perfectCMP = new FramePoint2D(perfectCoP);
      perfectCMP.add(perfectCMPOffset);

      FramePoint2D currentICP = new FramePoint2D(desiredICP);
      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.0, 0.0);
      currentICP.add(icpError);

      FramePoint2D currentCoMPosition = new FramePoint2D(worldFrame, -0.02, 0.0);

      FrameVector2D desiredICPVelocity = new FrameVector2D(worldFrame);
      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);

      testCase.setSupportPolygonInWorld(supportPolygonInWorld);
      testCase.setDesiredICP(desiredICP);
      testCase.setPerfectCoP(perfectCoP);
      testCase.setPerfectCMPOffset(perfectCMPOffset);
      testCase.setDesiredICPVelocity(desiredICPVelocity);
      testCase.setCurrentICP(currentICP);
      testCase.setCurrentCoMPosition(currentCoMPosition);

      solveAndVisualize(controller, visualizer, testCase);

      for (int i = 0; i < 20; i++)
      {
         testCase = new ICPControllerTestCase(testCase);
         desiredICP.add(0.01, 0.0);
         testCase.setDesiredICP(desiredICP);

         computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);
         testCase.setDesiredICPVelocity(desiredICPVelocity);
         
         solveAndVisualize(controller, visualizer, testCase);
      }

      testCase = new ICPControllerTestCase(testCase);
      desiredICP.set(0.04, 0.06);
      testCase.setDesiredICP(desiredICP);
      
      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);
      testCase.setDesiredICPVelocity(desiredICPVelocity);
      solveAndVisualize(controller, visualizer, testCase);

      for (int i = 0; i < 20; i++)
      {
         testCase = new ICPControllerTestCase(testCase);
         currentICP.add(0.01, 0.0);
         testCase.setCurrentICP(currentICP);
         computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);
         testCase.setDesiredICPVelocity(desiredICPVelocity);
         solveAndVisualize(controller, visualizer, testCase);
      }
      
      testCase = new ICPControllerTestCase(testCase);
      desiredICP.set(0.1, 0.1);
      testCase.setDesiredICP(desiredICP);
      currentICP.set(0.1, 0.1);
      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);
      testCase.setDesiredICPVelocity(desiredICPVelocity);
      solveAndVisualize(controller, visualizer, testCase);

      for (int i = 0; i < 20; i++)
      {
         testCase = new ICPControllerTestCase(testCase);
         currentICP.add(0.01, 0.0);
         testCase.setCurrentICP(currentICP);
         computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);
         testCase.setDesiredICPVelocity(desiredICPVelocity);
         solveAndVisualize(controller, visualizer, testCase);
      }
      
      ThreadTools.sleepForever();
   }

   private TestICPControllerParameters createTestICPControllerParameters(double kpParallel, double kpOrthogonal)
   {
      TestICPControllerParameters icpControllerParameters = new TestICPControllerParameters()
      {
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
      };
      
      return icpControllerParameters;
   }

   public void computeDesiredICPVelocityFromPerfectCMP(double omega, FramePoint2D desiredICP, FramePoint2D perfectCMP, FrameVector2D desiredICPVelocity)
   {
      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);
   }

   public void visualizeRandom() throws Exception
   {
      YoRegistry registry = new YoRegistry("ICPControllerTest");
      double kpParallel = 1.0;
      double kpOrthogonal = 2.0;
      TestICPControllerParameters icpControllerParameters = createTestICPControllerParameters(kpParallel, kpOrthogonal);

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

      ICPControllerTestVisualizer visualizer = null;

      visualizer = new ICPControllerTestVisualizer(registry, yoGraphicsListRegistry);

      double omega = 3.0;

      ArrayList<ICPControllerTestCase> testCases = new ArrayList<>();

      int numberOfTests = 300;
      Random random = new Random(1776L);
      //      Random random = new Random();

      for (int i = 0; i < numberOfTests; i++)
      {
         ICPControllerTestCase testCase = new ICPControllerTestCase();
         testCase.setOmega(omega);

         FramePoint2D desiredICP = new FramePoint2D(worldFrame, EuclidCoreRandomTools.nextPoint2D(random, 0.2));
         FramePoint2D perfectCoP = new FramePoint2D(worldFrame, EuclidCoreRandomTools.nextPoint2D(random, 0.06));
         FrameVector2D perfectCMPOffset = new FrameVector2D(worldFrame, EuclidCoreRandomTools.nextVector2D(random));
         perfectCMPOffset.scale(0.04);

         FramePoint2D perfectCMP = new FramePoint2D(perfectCoP);
         perfectCMP.add(perfectCMPOffset);

         testCase.setPerfectCMPOffset(perfectCMPOffset);

         FramePoint2D currentICP = new FramePoint2D(desiredICP);
         FrameVector2D icpError = new FrameVector2D(worldFrame, EuclidCoreRandomTools.nextPoint2D(random, 0.1));
         currentICP.add(icpError);

         FramePoint2D currentCoMPosition = new FramePoint2D(worldFrame, EuclidCoreRandomTools.nextPoint2D(random, 0.2));

         FrameVector2D desiredICPVelocity = new FrameVector2D(worldFrame);
         computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);

         testCase.setSupportPolygonInWorld(supportPolygonInWorld);
         testCase.setDesiredICP(desiredICP);
         testCase.setDesiredICPVelocity(desiredICPVelocity);
         testCase.setPerfectCoP(perfectCoP);
         testCase.setCurrentICP(currentICP);
         testCase.setCurrentCoMPosition(currentCoMPosition);

         testCases.add(testCase);
      }

      solveAndVisualize(controller, visualizer, testCases);

      ThreadTools.sleepForever();
   }

   private void solveAndVisualize(ICPControllerInterface controller,
                                  ICPControllerTestVisualizer visualizer,
                                  ArrayList<ICPControllerTestCase> testCases)
   {
      for (int i = 0; i < testCases.size(); i++)
      {
         ICPControllerTestCase testCase = testCases.get(i);

         solveAndVisualize(controller, visualizer, testCase);
      }
   }

   private void solveAndVisualize(ICPControllerInterface controller,
                                  ICPControllerTestVisualizer visualizer,
                                  ICPControllerTestCase testCase)
   {
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = testCase.getSupportPolygonInWorld();
      double omega = testCase.getOmega();

      FramePoint2DReadOnly desiredICP = testCase.getDesiredICP();
      FramePoint2DReadOnly perfectCoP = testCase.getPerfectCoP();
      FrameVector2DReadOnly perfectCMPOffset = testCase.getPerfectCMPOffset();
      FramePoint2DReadOnly currentICP = testCase.getCurrentICP();

      FramePoint2DReadOnly currentCoMPosition = testCase.getCurrentCoMPosition();
      FrameVector2DReadOnly desiredICPVelocity = testCase.getDesiredICPVelocity();

      FramePoint2D perfectCMP = new FramePoint2D(perfectCoP);
      perfectCMP.add(perfectCMPOffset);

      controller.initialize();
      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, perfectCoP, perfectCMPOffset, currentICP, currentCoMPosition, omega);

      FrameVector2D expectedControlICPVelocity = new FrameVector2D(worldFrame);
      FramePoint2D desiredCMP = new FramePoint2D(worldFrame);
      FramePoint2D desiredCoP = new FramePoint2D(worldFrame);

      controller.getDesiredCMP(desiredCMP);
      controller.getDesiredCoP(desiredCoP);

      testCase.setDesiredCMP(desiredCMP);
      testCase.setDesiredCoP(desiredCoP);

      expectedControlICPVelocity.sub(currentICP, desiredCMP);
      expectedControlICPVelocity.scale(omega);
      
      visualizer.updateInputs(omega, supportPolygonInWorld, desiredICP, desiredICPVelocity, perfectCMP, perfectCoP, currentICP, currentCoMPosition);
      visualizer.updateOutputs(desiredCoP, desiredCMP, expectedControlICPVelocity);
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

      ICPControllerTestVisualizer visualizer = null;
      if (visualize)
      {
         visualizer = new ICPControllerTestVisualizer(registry, yoGraphicsListRegistry);
      }

      double omega = 3.0;

      FrameVector2D desiredICPVelocity = new FrameVector2D(worldFrame);
      FramePoint2D desiredICP = new FramePoint2D(worldFrame);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame);

      desiredICP.set(0.0, 0.12);
      perfectCMP.set(0.0, -0.15);
      FramePoint2D perfectCoP = new FramePoint2D(perfectCMP);

      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);

      FramePoint2D currentICP = new FramePoint2D(worldFrame, 0.0, 0.10);

      FramePoint2D currentCoMPosition = new FramePoint2D(currentICP);

      if (visualize)
         visualizer.updateInputs(omega, supportPolygonInWorld, desiredICP, desiredICPVelocity, perfectCMP, perfectCoP, currentICP, currentCoMPosition);

      controller.initialize();
      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition, omega);

      FrameVector2D expectedControlICPVelocity = new FrameVector2D(worldFrame);
      FramePoint2D desiredCMP = new FramePoint2D(worldFrame);
      FramePoint2D desiredCoP = new FramePoint2D(worldFrame);

      controller.getDesiredCMP(desiredCMP);
      controller.getDesiredCoP(desiredCoP);

      expectedControlICPVelocity.sub(currentICP, desiredCMP);
      expectedControlICPVelocity.scale(omega);

      if (visualize)
         visualizer.updateOutputs(desiredCoP, desiredCMP, expectedControlICPVelocity);

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
      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition, omega);

      FrameVector2D expectedControlICPVelocity = new FrameVector2D(worldFrame);
      FramePoint2D desiredCMP = new FramePoint2D(worldFrame);
      FramePoint2D desiredCoP = new FramePoint2D(worldFrame);

      controller.getDesiredCMP(desiredCMP);
      controller.getDesiredCoP(desiredCoP);

      expectedControlICPVelocity.sub(currentICP, desiredCMP);
      expectedControlICPVelocity.scale(omega);

      if (visualize)
         visualizer.updateOutputs(desiredCoP, desiredCMP, expectedControlICPVelocity);

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

      double omega = 3.0;

      FrameVector2D desiredICPVelocity = new FrameVector2D(worldFrame);
      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.03, 0.06);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.01, 0.04);

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
      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition, omega);

      FrameVector2D expectedControlICPVelocity = new FrameVector2D(worldFrame);
      FramePoint2D desiredCMP = new FramePoint2D(worldFrame);
      FramePoint2D desiredCoP = new FramePoint2D(worldFrame);

      controller.getDesiredCMP(desiredCMP);
      controller.getDesiredCoP(desiredCoP);

      expectedControlICPVelocity.sub(currentICP, desiredCMP);
      expectedControlICPVelocity.scale(omega);

      ICPControllerTestVisualizer visualizer = null;
      if (visualize)
      {
         visualizer = new ICPControllerTestVisualizer(registry, yoGraphicsListRegistry);
         visualizer.updateInputs(omega, supportPolygonInWorld, desiredICP, desiredICPVelocity, perfectCMP, perfectCoP, currentICP, currentCoMPosition);
         visualizer.updateOutputs(desiredCoP, desiredCMP, expectedControlICPVelocity);
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

      double omega = 3.0;

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
      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoM, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
      controller.getDesiredCMP(desiredCMP);

      Assert.assertTrue(desiredCMP.epsilonEquals(perfectCMP, epsilon));
   }

   private SideDependentList<FootSpoof> setupContactableFeet(double footLength, double footWidth, double totalWidth)
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

   private BipedSupportPolygons setupBipedSupportPolygons(SideDependentList<FootSpoof> contactableFeet, YoRegistry registry)
   {
      return setupBipedSupportPolygons(contactableFeet, null, registry);
   }

   private BipedSupportPolygons setupBipedSupportPolygons(SideDependentList<FootSpoof> contactableFeet, RobotSide supportFoot, YoRegistry registry)
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

   private class TestICPControllerParameters extends ICPControllerParameters
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
