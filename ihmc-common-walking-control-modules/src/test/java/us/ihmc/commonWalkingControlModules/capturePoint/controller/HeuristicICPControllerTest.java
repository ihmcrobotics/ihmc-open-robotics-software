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
import us.ihmc.euclid.referenceFrame.FrameLineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
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
   private static boolean testHeuristicController = !false;

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

   private ICPControllerInterface createICPController(TestWalkingControllerParameters walkingControllerParameters,
                                                      TestICPControllerParameters icpControllerParameters,
                                                      BipedSupportPolygons bipedSupportPolygons,
                                                      Object object,
                                                      SideDependentList<FootSpoof> contactableFeet,
                                                      double controlDT,
                                                      YoRegistry registry,
                                                      YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      if (testHeuristicController)
         return new HeuristicICPController(walkingControllerParameters,
                                           icpControllerParameters,
                                           bipedSupportPolygons,
                                           null,
                                           contactableFeet,
                                           controlDT,
                                           registry,
                                           yoGraphicsListRegistry);

      else
         return new ICPController(walkingControllerParameters,
                                  icpControllerParameters,
                                  bipedSupportPolygons,
                                  null,
                                  contactableFeet,
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

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();

      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, footWidth, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      ICPControllerInterface controller = createICPController(walkingControllerParameters,
                                                              icpControllerParameters,
                                                              bipedSupportPolygons,
                                                              null,
                                                              contactableFeet,
                                                              controlDT,
                                                              registry,
                                                              yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      ICPControllerTestVisualizer visualizer = new ICPControllerTestVisualizer(4000, registry, yoGraphicsListRegistry);

      ICPControllerTestCase testCase = new ICPControllerTestCase();

      double omega = walkingControllerParameters.getOmega0();
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

      testCase.setDesiredICP(desiredICP);
      testCase.setPerfectCoP(perfectCoP);
      testCase.setPerfectCMPOffset(perfectCMPOffset);
      testCase.setDesiredICPVelocity(desiredICPVelocity);
      testCase.setCurrentICP(currentICP);
      testCase.setCurrentCoMPosition(currentCoMPosition);

      solveAndVisualize(bipedSupportPolygons, controller, visualizer, testCase);

      for (double x = -0.2; x< 0.4; x=x+0.01)
      {
         for (double y = -0.2; y<0.2; y = y + 0.01)
         {
            testCase = new ICPControllerTestCase(testCase);
            currentICP.set(x, y);
            testCase.setCurrentICP(currentICP);

            computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);
            testCase.setDesiredICPVelocity(desiredICPVelocity);

            solveAndVisualize(bipedSupportPolygons, controller, visualizer, testCase);
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

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      
      double footLength = 10.25;
      double footWidth = 10.1;
      double stanceWidth = 10.35;
      
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, footWidth, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      ICPControllerInterface controller = createICPController(walkingControllerParameters,
                                                              icpControllerParameters,
                                                              bipedSupportPolygons,
                                                              null,
                                                              contactableFeet,
                                                              controlDT,
                                                              registry,
                                                              yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      ICPControllerTestVisualizer visualizer = new ICPControllerTestVisualizer(registry, yoGraphicsListRegistry);

      ICPControllerTestCase testCase = new ICPControllerTestCase();

      double omega = walkingControllerParameters.getOmega0();
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

      testCase.setDesiredICP(desiredICP);
      testCase.setPerfectCoP(perfectCoP);
      testCase.setPerfectCMPOffset(perfectCMPOffset);
      testCase.setDesiredICPVelocity(desiredICPVelocity);
      testCase.setCurrentICP(currentICP);
      testCase.setCurrentCoMPosition(currentCoMPosition);

      solveAndVisualize(bipedSupportPolygons, controller, visualizer, testCase);

      for (int i = 0; i < 20; i++)
      {
         testCase = new ICPControllerTestCase(testCase);
         desiredICP.add(0.01, 0.0);
         testCase.setDesiredICP(desiredICP);

         computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);
         testCase.setDesiredICPVelocity(desiredICPVelocity);
         
         solveAndVisualize(bipedSupportPolygons, controller, visualizer, testCase);
      }

      testCase = new ICPControllerTestCase(testCase);
      desiredICP.set(0.04, 0.06);
      testCase.setDesiredICP(desiredICP);
      
      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);
      testCase.setDesiredICPVelocity(desiredICPVelocity);
      solveAndVisualize(bipedSupportPolygons, controller, visualizer, testCase);

      for (int i = 0; i < 20; i++)
      {
         testCase = new ICPControllerTestCase(testCase);
         currentICP.add(0.01, 0.0);
         testCase.setCurrentICP(currentICP);
         computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);
         testCase.setDesiredICPVelocity(desiredICPVelocity);
         solveAndVisualize(bipedSupportPolygons, controller, visualizer, testCase);
      }
      
      testCase = new ICPControllerTestCase(testCase);
      desiredICP.set(0.1, 0.1);
      testCase.setDesiredICP(desiredICP);
      currentICP.set(0.1, 0.1);
      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);
      testCase.setDesiredICPVelocity(desiredICPVelocity);
      solveAndVisualize(bipedSupportPolygons, controller, visualizer, testCase);

      for (int i = 0; i < 20; i++)
      {
         testCase = new ICPControllerTestCase(testCase);
         currentICP.add(0.01, 0.0);
         testCase.setCurrentICP(currentICP);
         computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);
         testCase.setDesiredICPVelocity(desiredICPVelocity);
         solveAndVisualize(bipedSupportPolygons, controller, visualizer, testCase);
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

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      ICPControllerInterface controller = createICPController(walkingControllerParameters,
                                                              icpControllerParameters,
                                                              bipedSupportPolygons,
                                                              null,
                                                              contactableFeet,
                                                              controlDT,
                                                              registry,
                                                              yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      ICPControllerTestVisualizer visualizer = null;

      visualizer = new ICPControllerTestVisualizer(registry, yoGraphicsListRegistry);

      double omega = walkingControllerParameters.getOmega0();

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

         testCase.setDesiredICP(desiredICP);
         testCase.setDesiredICPVelocity(desiredICPVelocity);
         testCase.setPerfectCoP(perfectCoP);
         testCase.setCurrentICP(currentICP);
         testCase.setCurrentCoMPosition(currentCoMPosition);

         testCases.add(testCase);
      }

      solveAndVisualize(bipedSupportPolygons, controller, visualizer, testCases);

      ThreadTools.sleepForever();
   }

   private void solveAndVisualize(BipedSupportPolygons bipedSupportPolygons,
                                  ICPControllerInterface controller,
                                  ICPControllerTestVisualizer visualizer,
                                  ArrayList<ICPControllerTestCase> testCases)
   {
      for (int i = 0; i < testCases.size(); i++)
      {
         ICPControllerTestCase testCase = testCases.get(i);

         solveAndVisualize(bipedSupportPolygons, controller, visualizer, testCase);
      }
   }

   private void solveAndVisualize(BipedSupportPolygons bipedSupportPolygons,
                                  ICPControllerInterface controller,
                                  ICPControllerTestVisualizer visualizer,
                                  ICPControllerTestCase testCase)
   {
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
      controller.compute(desiredICP, desiredICPVelocity, perfectCoP, perfectCMPOffset, currentICP, currentCoMPosition, omega);

      FramePoint2D desiredCMP = new FramePoint2D(worldFrame);
      FramePoint2D desiredCoP = new FramePoint2D(worldFrame);

      controller.getDesiredCMP(desiredCMP);
      controller.getDesiredCoP(desiredCoP);

      testCase.setDesiredCMP(desiredCMP);
      testCase.setDesiredCoP(desiredCoP);

      visualizer.updateInputs(omega, bipedSupportPolygons, desiredICP, desiredICPVelocity, perfectCMP, perfectCoP, currentICP, currentCoMPosition);
      visualizer.updateOutputs(desiredCoP, desiredCMP);
   }

   @Test
   public void testKeepAwayFromEdgeIfNotNecessaryInSingleSupport() throws Exception
   {
      YoRegistry registry = new YoRegistry("ICPControllerTest");
      double feedbackGain = 2.0;
      TestICPControllerParameters icpControllerParameters = createTestICPControllerParameters(feedbackGain, feedbackGain);

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();

      double stanceWidth = 0.4;
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, RobotSide.RIGHT, registry);

      double controlDT = 0.001;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      ICPControllerInterface controller = createICPController(walkingControllerParameters,
                                                              icpControllerParameters,
                                                              bipedSupportPolygons,
                                                              null,
                                                              contactableFeet,
                                                              controlDT,
                                                              registry,
                                                              yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      ICPControllerTestVisualizer visualizer = null;
      if (visualize)
      {
         visualizer = new ICPControllerTestVisualizer(registry, yoGraphicsListRegistry);
      }

      double omega = walkingControllerParameters.getOmega0();

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
         visualizer.updateInputs(omega, bipedSupportPolygons, desiredICP, desiredICPVelocity, perfectCMP, perfectCoP, currentICP, currentCoMPosition);

      controller.initialize();
      controller.compute(desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
      FramePoint2D desiredCoP = new FramePoint2D();
      controller.getDesiredCMP(desiredCMP);
      controller.getDesiredCoP(desiredCoP);

      if (visualize)
         visualizer.updateOutputs(desiredCoP, desiredCMP);

      FrameLine2D lineFromICPTODesired = new FrameLine2D(desiredICP, currentICP);
      double distanceFromLineToProjection = lineFromICPTODesired.distance(desiredCMP);
      System.out.println("distanceFromLineToProjection = " + distanceFromLineToProjection);

      double distanceFromMiddleOfFootToCMP = desiredCMP.distance(perfectCMP);
      System.out.println("distanceFromMiddleOfFootToCMP = " + distanceFromMiddleOfFootToCMP);

      if (visualize)
      {
         ThreadTools.sleepForever();
      }

      Assert.assertTrue("distanceFromMiddleOfFootToCMP = " + distanceFromMiddleOfFootToCMP + ". It should be near zero.",
                        distanceFromMiddleOfFootToCMP < 0.005);
      Assert.assertTrue("distanceFromLineToProjection = " + distanceFromLineToProjection + ". It should be near zero.", distanceFromLineToProjection < 0.005);
   }

   @Test
   public void testProjectOnLineFromICPToDesired() throws Exception
   {
      YoRegistry registry = new YoRegistry("ICPControllerTest");
      double feedbackGain = 2.0;
      TestICPControllerParameters icpControllerParameters = createTestICPControllerParameters(feedbackGain, feedbackGain);

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();

      double footLength = 0.25;
      double stanceWidth = 0.3;
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      ICPControllerInterface controller = createICPController(walkingControllerParameters,
                                                              icpControllerParameters,
                                                              bipedSupportPolygons,
                                                              null,
                                                              contactableFeet,
                                                              controlDT,
                                                              registry,
                                                              yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      ICPControllerTestVisualizer visualizer = null;
      if (visualize)
      {
         visualizer = new ICPControllerTestVisualizer(registry, yoGraphicsListRegistry);
      }

      double omega = walkingControllerParameters.getOmega0();

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
         visualizer.updateInputs(omega, bipedSupportPolygons, desiredICP, desiredICPVelocity, perfectCMP, perfectCoP, currentICP, currentCoMPosition);

      controller.initialize();
      controller.compute(desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
      FramePoint2D desiredCoP = new FramePoint2D();
      controller.getDesiredCMP(desiredCMP);
      controller.getDesiredCoP(desiredCoP);

      if (visualize)
         visualizer.updateOutputs(desiredCoP, desiredCMP);

      FrameLine2D lineFromICPTODesired = new FrameLine2D(desiredICP, currentICP);
      double distanceFromLineToProjection = lineFromICPTODesired.distance(desiredCMP);
      System.out.println("distanceFromLineToProjection = " + distanceFromLineToProjection);

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

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      ICPControllerInterface controller = createICPController(walkingControllerParameters,
                                                              icpControllerParameters,
                                                              bipedSupportPolygons,
                                                              null,
                                                              contactableFeet,
                                                              controlDT,
                                                              registry,
                                                              yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      double omega = walkingControllerParameters.getOmega0();

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
      controller.compute(desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
      FramePoint2D desiredCoP = new FramePoint2D();
      controller.getDesiredCMP(desiredCMP);
      controller.getDesiredCoP(desiredCoP);

      ICPControllerTestVisualizer visualizer = null;
      if (visualize)
      {
         visualizer = new ICPControllerTestVisualizer(registry, yoGraphicsListRegistry);
         visualizer.updateInputs(omega, bipedSupportPolygons, desiredICP, desiredICPVelocity, perfectCMP, perfectCoP, currentICP, currentCoMPosition);
         visualizer.updateOutputs(desiredCoP, desiredCMP);
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

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      ICPControllerInterface controller = createICPController(walkingControllerParameters,
                                                              icpControllerParameters,
                                                              bipedSupportPolygons,
                                                              null,
                                                              contactableFeet,
                                                              controlDT,
                                                              registry,
                                                              yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      double omega = walkingControllerParameters.getOmega0();

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
      controller.compute(desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoM, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
      controller.getDesiredCMP(desiredCMP);

      Assert.assertTrue(desiredCMP.epsilonEquals(perfectCMP, epsilon));
   }

   @Test
   public void testStandingConstrained() throws Exception
   {
      YoRegistry registry = new YoRegistry("ICPControllerTest");
      double feedbackGain = 2.0;
      TestICPControllerParameters optimizationParameters = createTestICPControllerParameters(feedbackGain, feedbackGain);

      double footWidth = 0.1;
      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, footWidth, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      ICPControllerInterface controller = createICPController(walkingControllerParameters,
                                                              optimizationParameters,
                                                              bipedSupportPolygons,
                                                              null,
                                                              contactableFeet,
                                                              controlDT,
                                                              registry,
                                                              yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      double omega = walkingControllerParameters.getOmega0();

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.03, 0.06);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.01, 0.04);
      FrameVector2D desiredICPVelocity = new FrameVector2D();

      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);

      FramePoint2D perfectCoP = new FramePoint2D(perfectCMP);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.03, 0.06);
      FramePoint2D currentICP = new FramePoint2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);
      FramePoint2D currentCoMPosition = new FramePoint2D(currentICP);

      controller.initialize();
      controller.compute(desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
      FramePoint2D desiredCoP = new FramePoint2D();
      controller.getDesiredCMP(desiredCMP);
      controller.getDesiredCoP(desiredCoP);

      FramePoint2D desiredCMPExpected = new FramePoint2D();
      desiredCMPExpected.set(icpError);
      desiredCMPExpected.scale(feedbackGain + 1.0);
      desiredCMPExpected.add(perfectCMP);

      FramePoint2DReadOnly cmpToTest;
      if (bipedSupportPolygons.getSupportPolygonInWorld().isPointInside(desiredCMPExpected))
      {
         cmpToTest = desiredCMPExpected;
      }
      else
      {
         FrameLineSegment2D lineSegment2D = new FrameLineSegment2D();
         lineSegment2D.set(perfectCMP, desiredCMPExpected);
         cmpToTest = lineSegment2D.intersectionWith(bipedSupportPolygons.getSupportPolygonInWorld())[0];
      }

      ICPControllerTestVisualizer visualizer;
      if (visualize)
      {
         visualizer = new ICPControllerTestVisualizer(registry, yoGraphicsListRegistry);
         visualizer.updateInputs(omega, bipedSupportPolygons, desiredICP, desiredICPVelocity, perfectCMP, perfectCoP, currentICP, currentCoMPosition);
         visualizer.updateOutputs(desiredCMP, desiredCMP);
         ThreadTools.sleepForever();
      }

      EuclidFrameTestTools.assertFramePoint2DGeometricallyEquals(cmpToTest, desiredCMP, epsilon);
   }

   @Test
   public void testStandingUnconstrained() throws Exception
   {
      YoRegistry registry = new YoRegistry("ICPControllerTest");

      double feedbackGain = 2.0;
      TestICPControllerParameters optimizationParameters = createTestICPControllerParameters(feedbackGain, feedbackGain);

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(10.0, 5.0, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;
      ICPController controller = new ICPController(walkingControllerParameters,
                                                   optimizationParameters,
                                                   bipedSupportPolygons,
                                                   null,
                                                   contactableFeet,
                                                   controlDT,
                                                   registry,
                                                   null);
      new DefaultParameterReader().readParametersInRegistry(registry);

      double omega = walkingControllerParameters.getOmega0();

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.03, 0.06);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.01, 0.04);
      FrameVector2D desiredICPVelocity = new FrameVector2D();

      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.03, 0.06);
      FramePoint2D currentICP = new FramePoint2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);
      FramePoint2D currentCoMPosition = new FramePoint2D(currentICP);

      controller.initialize();
      controller.compute(desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
      controller.getDesiredCMP(desiredCMP);

      FramePoint2D desiredCMPExpected = new FramePoint2D();
      FrameVector2D expectedCoPFeedbackDelta = new FrameVector2D();
      FrameVector2D expectedCMPFeedbackDelta = new FrameVector2D();
      expectedCoPFeedbackDelta.set(icpError);
      expectedCoPFeedbackDelta.scale(feedbackGain + 1.0);

      desiredCMPExpected.set(perfectCMP);
      desiredCMPExpected.add(expectedCMPFeedbackDelta);
      desiredCMPExpected.add(expectedCoPFeedbackDelta);

      EuclidFrameTestTools.assertFrameVector2DGeometricallyEquals(icpError, controller.icpError, epsilon);
      EuclidFrameTestTools.assertFramePoint2DGeometricallyEquals(perfectCMP, controller.perfectCoP, epsilon);
      EuclidFrameTestTools.assertFramePoint2DGeometricallyEquals(perfectCMP, controller.perfectCMP, epsilon);
      EuclidFrameTestTools.assertFrameVector2DGeometricallyEquals(expectedCMPFeedbackDelta, controller.feedbackCMPDelta, epsilon);
      EuclidFrameTestTools.assertFrameVector2DGeometricallyEquals(expectedCoPFeedbackDelta, controller.feedbackCoPDelta, epsilon);
      EuclidFrameTestTools.assertFramePoint2DGeometricallyEquals(desiredCMPExpected, desiredCMP, epsilon);
   }

   @Test
   public void testStandingConstrainedWithAngularMomentum() throws Exception
   {
      YoRegistry registry = new YoRegistry("ICPControllerTest");

      double feedbackGain = 2.0;

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      double kpParallel = 1.0;
      double kpOrthogonal = 2.0;
      TestICPControllerParameters optimizationParameters = createTestICPControllerParameters(kpParallel, kpOrthogonal);
      
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      ICPControllerInterface controller = createICPController(walkingControllerParameters,
                                                              optimizationParameters,
                                                              bipedSupportPolygons,
                                                              null,
                                                              contactableFeet,
                                                              controlDT,
                                                              registry,
                                                              yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      double omega = walkingControllerParameters.getOmega0();

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.03, 0.06);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.01, 0.04);
      FrameVector2D desiredICPVelocity = new FrameVector2D();

      computeDesiredICPVelocityFromPerfectCMP(omega, desiredICP, perfectCMP, desiredICPVelocity);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.03, 0.06);
      FramePoint2D currentICP = new FramePoint2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);
      FramePoint2D currentCoMPosition = new FramePoint2D(currentICP);

      controller.initialize();
      controller.compute(desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
      controller.getDesiredCMP(desiredCMP);

      FramePoint2D desiredCMPExpected = new FramePoint2D();
      desiredCMPExpected.set(icpError);
      desiredCMPExpected.scale(feedbackGain + 1.0);
      desiredCMPExpected.add(perfectCMP);

      EuclidFrameTestTools.assertFramePoint2DGeometricallyEquals(desiredCMPExpected, desiredCMP, epsilon);
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
      public PDGains getCoMHeightControlGains()
      {
         return null;
      }

      @Override
      public PIDSE3Configuration getSwingFootControlGains()
      {
         return null;
      }

      @Override
      public PIDSE3Configuration getHoldPositionFootControlGains()
      {
         return null;
      }

      @Override
      public PIDSE3Configuration getToeOffFootControlGains()
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
      public FootSwitchFactory getFootSwitchFactory()
      {
         return null;
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
      public double getMaxICPErrorBeforeSingleSupportForwardX()
      {
         return 0;
      }

      @Override
      public double getMaxICPErrorBeforeSingleSupportInnerY()
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
      public ICPControllerParameters getICPControllerParameters()
      {
         return new TestICPControllerParameters();
      }

      @Override
      public StepAdjustmentParameters getStepAdjustmentParameters()
      {
         return null;
      }

      @Override
      public SteppingParameters getSteppingParameters()
      {
         return null;
      }
   }
}
