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
import us.ihmc.commonWalkingControlModules.capturePoint.optimization.ICPOptimizationParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
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

public class ICPControllerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 1e-3;

   private static final double footLength = 0.25;
   private static final double stanceWidth = 0.35;

   private static boolean visualize = true;

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

   public static void main(String[] args) throws Exception
   {
      ICPControllerTest test = new ICPControllerTest();
      test.visualizeRandom();
   }
   
   public void visualizeRandom() throws Exception
   {
      YoRegistry registry = new YoRegistry("ICPControllerTest");
      double feedbackGain = 2.0;
      TestICPOptimizationParameters optimizationParameters = createTestICPOptimizationParameters(feedbackGain);

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters, bipedSupportPolygons, null, contactableFeet, controlDT, registry, yoGraphicsListRegistry);
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
      FramePoint2D currentCoMPosition = new FramePoint2D(worldFrame);
      FramePoint2D currentICP = new FramePoint2D(worldFrame);
      FramePoint2D desiredCMP = new FramePoint2D(worldFrame);
      FramePoint2D desiredCoP = new FramePoint2D(worldFrame);

      int numberOfTests = 100;
      Random random = new Random(1776L);
      
      for (int i = 0; i < numberOfTests; i++)
      {
         
         currentCoMPosition.set(EuclidCoreRandomTools.nextPoint2D(random, 0.2));
         currentICP.set(EuclidCoreRandomTools.nextPoint2D(random, 0.2));

         desiredICP.set(EuclidCoreRandomTools.nextPoint2D(random, 0.2));
         perfectCMP.set(EuclidCoreRandomTools.nextPoint2D(random, 0.2));
         desiredICPVelocity.set(EuclidCoreRandomTools.nextPoint2D(random, 0.2));

         if (visualize)
            visualizer.updateInputs(bipedSupportPolygons, desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition);

         controller.initialize();
         controller.compute(desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition, omega);
         controller.getDesiredCMP(desiredCMP);
         controller.getDesiredCoP(desiredCoP);

         if (visualize)
            visualizer.updateOutputs(desiredCoP, desiredCMP);

         //      Assert.assertTrue(desiredCMP.epsilonEquals(perfectCMP, epsilon));
      }

      if (visualize)
      {
         ThreadTools.sleepForever();
      }
   }
   
   
   @Test
   public void testKeepAwayFromEdgeIfNotNecessaryInSingleSupport() throws Exception
   {
      YoRegistry registry = new YoRegistry("ICPControllerTest");
      double feedbackGain = 2.0;
      TestICPOptimizationParameters optimizationParameters = createTestICPOptimizationParameters(feedbackGain);

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      
      double stanceWidth = 0.4;
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, RobotSide.RIGHT, registry);
    
      double controlDT = 0.001;
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters, bipedSupportPolygons, null, contactableFeet, controlDT, registry, yoGraphicsListRegistry);
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

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FramePoint2D currentICP = new FramePoint2D(worldFrame, 0.0, 0.10);
     
      FramePoint2D currentCoMPosition = new FramePoint2D(currentICP);

      if (visualize)
         visualizer.updateInputs(bipedSupportPolygons, desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition);

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
      
      Assert.assertTrue("distanceFromMiddleOfFootToCMP = " + distanceFromMiddleOfFootToCMP + ". It should be near zero.", distanceFromMiddleOfFootToCMP < 0.005);
      Assert.assertTrue("distanceFromLineToProjection = " + distanceFromLineToProjection + ". It should be near zero.", distanceFromLineToProjection < 0.005);
   }
   
   
   @Test
   public void testProjectOnLineFromICPToDesired() throws Exception
   {
      YoRegistry registry = new YoRegistry("ICPControllerTest");
      double feedbackGain = 2.0;
      TestICPOptimizationParameters optimizationParameters = createTestICPOptimizationParameters(feedbackGain);

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      
      double footLength = 0.25;
      double stanceWidth = 0.3;
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters, bipedSupportPolygons, null, contactableFeet, controlDT, registry, yoGraphicsListRegistry);
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

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FrameVector2D icpError = new FrameVector2D(desiredICPVelocity);
      icpError.normalize();
      icpError.scale(-0.1);
      
      FramePoint2D currentICP = new FramePoint2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);
      FramePoint2D currentCoMPosition = new FramePoint2D(currentICP);

      if (visualize)
         visualizer.updateInputs(bipedSupportPolygons, desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition);

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
      TestICPOptimizationParameters optimizationParameters = createTestICPOptimizationParameters(feedbackGain);

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters, bipedSupportPolygons, null, contactableFeet, controlDT, registry, yoGraphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      ICPControllerTestVisualizer visualizer = null;
      if (visualize)
      {
         visualizer = new ICPControllerTestVisualizer(registry, yoGraphicsListRegistry);
      }

      double omega = walkingControllerParameters.getOmega0();

      FrameVector2D desiredICPVelocity = new FrameVector2D(worldFrame);
      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.03, 0.06);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.01, 0.04);

      desiredICP.set(0.03, 0.06);
      perfectCMP.set(0.01, 0.04);

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FrameVector2D icpError = new FrameVector2D();
      FramePoint2D currentICP = new FramePoint2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);
      FramePoint2D currentCoMPosition = new FramePoint2D(currentICP);

      if (visualize)
         visualizer.updateInputs(bipedSupportPolygons, desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition);

      controller.initialize();
      controller.compute(desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition, omega);

      FramePoint2D desiredCMP = new FramePoint2D();
      FramePoint2D desiredCoP = new FramePoint2D();
      controller.getDesiredCMP(desiredCMP);
      controller.getDesiredCoP(desiredCoP);

      if (visualize)
         visualizer.updateOutputs(desiredCoP, desiredCMP);

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
      TestICPOptimizationParameters optimizationParameters = createTestICPOptimizationParameters(feedbackGain);

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;
      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters, bipedSupportPolygons, null, contactableFeet, controlDT, registry, null);
      new DefaultParameterReader().readParametersInRegistry(registry);

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
      TestICPOptimizationParameters optimizationParameters = createTestICPOptimizationParameters(feedbackGain);

      double footWidth = 0.1;
      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, footWidth, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;

      YoGraphicsListRegistry graphicsListRegistry = new YoGraphicsListRegistry();
      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters, bipedSupportPolygons, null, contactableFeet, controlDT, registry, graphicsListRegistry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      boolean visualize = false;
      
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
         visualizer = new ICPControllerTestVisualizer(registry, graphicsListRegistry);
         visualizer.updateInputs(bipedSupportPolygons, desiredICP, desiredICPVelocity, perfectCMP, currentICP, currentCoMPosition);
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
      TestICPOptimizationParameters optimizationParameters = createTestICPOptimizationParameters(feedbackGain);

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(10.0, 5.0, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;
      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters, bipedSupportPolygons, null, contactableFeet, controlDT, registry, null);
      new DefaultParameterReader().readParametersInRegistry(registry);

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
      TestICPOptimizationParameters optimizationParameters = new TestICPOptimizationParameters()
      {
         @Override
         public ICPControlGainsReadOnly getICPFeedbackGains()
         {
            ICPControlGains gains = new ICPControlGains();
            gains.setKpOrthogonalToMotion(feedbackGain);
            gains.setKpParallelToMotion(feedbackGain);
            return gains;
         }

         @Override
         public double getDynamicsObjectiveWeight()
         {
            return 1000000.0;
         }

         @Override
         public boolean useAngularMomentum()
         {
            return true;
         }

         @Override
         public double getAngularMomentumMinimizationWeight()
         {
            return 1.0;
         }

         @Override
         public boolean allowStepAdjustment()
         {
            return false;
         }
      };

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      double controlDT = 0.001;
      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters, bipedSupportPolygons, null, contactableFeet, controlDT, registry, null);
      new DefaultParameterReader().readParametersInRegistry(registry);

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
            YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, registry);
            
            if ((supportFoot == null) || (supportFoot == robotSide))
               yoPlaneContactState.setFullyConstrained();
            else
               yoPlaneContactState.clear();
            contactStates.put(robotSide, yoPlaneContactState);
         }
      }

      ReferenceFrame midFeetZUpFrame = new MidFrameZUpFrame("midFeetZupFrame", worldFrame, soleZUpFrames.get(RobotSide.LEFT), soleZUpFrames.get(RobotSide.RIGHT));
      midFeetZUpFrame.update();

      BipedSupportPolygons bipedSupportPolygons = new BipedSupportPolygons(midFeetZUpFrame, soleZUpFrames, soleFrames, registry, null);
      bipedSupportPolygons.updateUsingContactStates(contactStates);

      return bipedSupportPolygons;
   }

   private TestICPOptimizationParameters createTestICPOptimizationParameters(double feedbackGain)
   {
      TestICPOptimizationParameters optimizationParameters = new TestICPOptimizationParameters()
      {
         @Override
         public ICPControlGainsReadOnly getICPFeedbackGains()
         {
            ICPControlGains gains = new ICPControlGains();
            gains.setKpParallelToMotion(feedbackGain);
            gains.setKpOrthogonalToMotion(feedbackGain);

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
         public boolean allowStepAdjustment()
         {
            return false;
         }
      };

      return optimizationParameters;
   }

   private class TestICPOptimizationParameters extends ICPOptimizationParameters
   {
      @Override
      public double getFeedbackDirectionWeight()
      {
         return 1000000.0;
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
      public boolean allowStepAdjustment()
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
