package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.AfterEach;
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
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
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
import us.ihmc.yoVariables.variable.YoDouble;

import static org.junit.jupiter.api.Assertions.*;

public class ICPControllerTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double epsilon = 1e-3;

   private static final double footLength = 0.25;
   private static final double stanceWidth = 0.35;

   private final SideDependentList<FramePose3D> footPosesAtTouchdown = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final SideDependentList<ReferenceFrame> soleZUpFrames = new SideDependentList<>();

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testStandingWithPerfectTracking() throws Exception
   {
      YoRegistry registry = new YoRegistry("robert");

      double feedbackGain = 2.0;
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
      };

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();

      double controlDT = 0.001;
      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters, null, contactableFeet, controlDT, registry, null);
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
      FramePoint2D currentCoMPosition = new FramePoint2D(currentICP);

      controller.initialize();
      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

      Assert.assertTrue(controller.getDesiredCMP().epsilonEquals(perfectCMP, epsilon));
   }

   @Test
   public void testStandingWithDifferentConstraintsOnMaximumFeedbackRate()
   {
      YoRegistry registry = new YoRegistry("robert");

      double feedbackGain = 2.0;
      ICPControlGains testGains = new ICPControlGains();
      testGains.setKpParallelToMotion(feedbackGain);
      testGains.setKpOrthogonalToMotion(feedbackGain);
      TestICPOptimizationParameters optimizationParameters = new TestICPOptimizationParameters()
      {
         @Override
         public ICPControlGainsReadOnly getICPFeedbackGains()
         {
            return testGains;
         }

         @Override
         public double getFeedbackRateWeight()
         {
            return 0.0;
         }
      };

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();

      double controlDT = 0.001;
      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters, null, contactableFeet, controlDT, registry, null);
      new DefaultParameterReader().readParametersInRegistry(registry);

      double omega = walkingControllerParameters.getOmega0();

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.0, 0.0);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.0, 0.0);
      FrameVector2D desiredICPVelocity = new FrameVector2D();

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.02, 0.02);
      FramePoint2D currentICP = new FramePoint2D();
      currentICP.add(desiredICP, icpError);
      FramePoint2D currentCoMPosition = new FramePoint2D(currentICP);

      // do an initial test to initialize the controller.
      FramePoint2D expectedCMP = new FramePoint2D();
      expectedCMP.set(icpError);
      expectedCMP.scale(1.0 + feedbackGain);
      expectedCMP.add(perfectCMP);

      controller.initialize();
      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

      EuclidCoreTestTools.assertEquals(expectedCMP, controller.getDesiredCMP(), epsilon);

      // ok, so now we're going to set the feedback max values. It should clamp to this. By having the error in the same direction, there should be little
      // Rate regularization
      double maxRate = 0.05;
      ((YoDouble) registry.findVariable("feedbackPartMaxRate")).set(maxRate);

      icpError = new FrameVector2D(worldFrame, -0.03, -0.03);
      currentICP = new FramePoint2D();
      currentICP.add(desiredICP, icpError);
      currentCoMPosition = new FramePoint2D(currentICP);

      FramePoint2D unclampedCMP = new FramePoint2D();
      unclampedCMP.set(icpError);
      unclampedCMP.scale(1.0 + feedbackGain);
      unclampedCMP.add(perfectCMP);

      FrameVector2D delta = new FrameVector2D();
      delta.sub(unclampedCMP, expectedCMP);
      delta.clipToMaxNorm(maxRate * controlDT);

      FramePoint2D expectedClampedCMP = new FramePoint2D();

      expectedClampedCMP.set(expectedCMP);
      expectedClampedCMP.add(delta);

      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

      EuclidCoreTestTools.assertEquals(expectedClampedCMP, controller.getDesiredCMP(), epsilon);

      // Add a bunch of random tests that continue to move the desired CMP around, but make sure to clamp it every time.
      Random random = new Random(1738L);
      for (int i = 0; i < 10; i++)
      {
         icpError = EuclidFrameRandomTools.nextFrameVector2D(random, worldFrame, new Point2D(0.05, 0.05));
         currentICP = new FramePoint2D();
         currentICP.add(desiredICP, icpError);
         currentCoMPosition = new FramePoint2D(currentICP);

         unclampedCMP = new FramePoint2D();
         unclampedCMP.set(icpError);
         unclampedCMP.scale(1.0 + feedbackGain);
         unclampedCMP.add(perfectCMP);

         delta.sub(unclampedCMP, expectedClampedCMP);
         delta.clipToMaxNorm(maxRate * controlDT);

         expectedClampedCMP.add(delta);

         controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

         EuclidCoreTestTools.assertEquals("iter " + i + " failed", expectedClampedCMP, controller.getDesiredCMP(), epsilon);
      }
   }

   @Test
   public void testStandingWithDifferentConstraintsOnMaximumFeedbackAmount()
   {
      YoRegistry registry = new YoRegistry("robert");

      double feedbackGain = 2.0;
      ICPControlGains testGains = new ICPControlGains();
      testGains.setKpParallelToMotion(feedbackGain);
      testGains.setKpOrthogonalToMotion(feedbackGain);
      TestICPOptimizationParameters optimizationParameters = new TestICPOptimizationParameters()
      {
         @Override
         public ICPControlGainsReadOnly getICPFeedbackGains()
         {
            return testGains;
         }
      };

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();

      double controlDT = 0.001;
      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters, null, contactableFeet, controlDT, registry, null);
      new DefaultParameterReader().readParametersInRegistry(registry);

      double omega = walkingControllerParameters.getOmega0();

      FramePoint2D desiredICP = new FramePoint2D(worldFrame, 0.0, 0.0);
      FramePoint2D perfectCMP = new FramePoint2D(worldFrame, 0.0, 0.0);
      FrameVector2D desiredICPVelocity = new FrameVector2D();

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      FrameVector2D icpError = new FrameVector2D(worldFrame, 0.02, 0.02);
      FramePoint2D currentICP = new FramePoint2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);
      FramePoint2D currentCoMPosition = new FramePoint2D(currentICP);

      // do an initial test to initialize the controller.
      FramePoint2D expectedCMP = new FramePoint2D();
      expectedCMP.set(icpError);
      expectedCMP.scale(1.0 + feedbackGain);
      expectedCMP.add(perfectCMP);

      controller.initialize();
      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

      EuclidCoreTestTools.assertEquals(expectedCMP, controller.getDesiredCMP(), epsilon);

      // ok, so now we're going to set the feedback max values. It should clamp to this. By having the error in the same direction, there should be little
      // Rate regularization
      ((YoDouble) registry.findVariable("feedbackPartMaxValueParallelToMotion")).set(0.03);
      ((YoDouble) registry.findVariable("feedbackPartMaxValueOrthogonalToMotion")).set(0.03);

      FramePoint2D expectedClampedCMP = new FramePoint2D();

      expectedClampedCMP.set(0.03, 0.03);

      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

      EuclidCoreTestTools.assertEquals(expectedClampedCMP, controller.getDesiredCMP(), epsilon);

      // Now clamp just the parallel to motion value. Since we're standing, this should do no clamping
      ((YoDouble) registry.findVariable("feedbackPartMaxValueParallelToMotion")).set(0.03);
      ((YoDouble) registry.findVariable("feedbackPartMaxValueOrthogonalToMotion")).set(Double.POSITIVE_INFINITY);

      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

      EuclidCoreTestTools.assertEquals(expectedCMP, controller.getDesiredCMP(), epsilon);

      // Now clamp just the parallel to motion value. Since we're standing, this should do full clamping
      ((YoDouble) registry.findVariable("feedbackPartMaxValueParallelToMotion")).set(Double.POSITIVE_INFINITY);
      ((YoDouble) registry.findVariable("feedbackPartMaxValueOrthogonalToMotion")).set(0.03);

      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

      EuclidCoreTestTools.assertEquals(expectedClampedCMP, controller.getDesiredCMP(), epsilon);

      // Now do everything in some motion, with no clamping
      desiredICP = new FramePoint2D(worldFrame, 0.03, 0.06);
      perfectCMP = new FramePoint2D(worldFrame, 0.01, 0.04);

      currentICP.add(desiredICP, icpError);

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      expectedCMP = new FramePoint2D();
      expectedCMP.set(icpError);
      expectedCMP.scale(1.0 + feedbackGain);
      expectedCMP.add(perfectCMP);

      ((YoDouble) registry.findVariable("feedbackPartMaxValueParallelToMotion")).set(Double.POSITIVE_INFINITY);
      ((YoDouble) registry.findVariable("feedbackPartMaxValueOrthogonalToMotion")).set(Double.POSITIVE_INFINITY);

      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

      EuclidCoreTestTools.assertEquals(expectedCMP, controller.getDesiredCMP(), epsilon);


      // Make the clamping non-symmetric. Have the desired dynamics be straight forward in X
      desiredICP = new FramePoint2D(worldFrame, 0.05, 0.0);
      perfectCMP = new FramePoint2D(worldFrame, 0.0, 0.0);

      currentICP.add(desiredICP, icpError);

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      expectedCMP = new FramePoint2D();
      expectedCMP.set(icpError);
      expectedCMP.scale(1.0 + feedbackGain);
      expectedCMP.add(perfectCMP);

      ((YoDouble) registry.findVariable("feedbackPartMaxValueParallelToMotion")).set(0.01);
      ((YoDouble) registry.findVariable("feedbackPartMaxValueOrthogonalToMotion")).set(0.04);

      FrameVector2D feedbackLimit = new FrameVector2D(worldFrame, 0.01, 0.04);

      expectedClampedCMP.set(feedbackLimit);
      expectedClampedCMP.add(perfectCMP);

      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

      EuclidCoreTestTools.assertEquals(expectedClampedCMP, controller.getDesiredCMP(), epsilon);

      // Make the clamping non-symmetric. Have the desired dynamics be straight sideways in Y, so a 90 degree rotation
      desiredICP = new FramePoint2D(worldFrame, 0.0, 0.05);
      perfectCMP = new FramePoint2D(worldFrame, 0.0, 0.0);

      currentICP.add(desiredICP, icpError);

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      expectedCMP = new FramePoint2D();
      expectedCMP.set(icpError);
      expectedCMP.scale(1.0 + feedbackGain);
      expectedCMP.add(perfectCMP);

      ((YoDouble) registry.findVariable("feedbackPartMaxValueParallelToMotion")).set(0.01);
      ((YoDouble) registry.findVariable("feedbackPartMaxValueOrthogonalToMotion")).set(0.04);

      feedbackLimit = new FrameVector2D(worldFrame, 0.04, 0.01);

      expectedClampedCMP.set(feedbackLimit);
      expectedClampedCMP.add(perfectCMP);

      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

      EuclidCoreTestTools.assertEquals(expectedClampedCMP, controller.getDesiredCMP(), epsilon);


      // Now do asymmetric clamping, while in motion at an angle
      desiredICP = new FramePoint2D(worldFrame, 0.02, 0.02);

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      ICPControllerHelper.Vector2dZUpFrame dynamicsFrame = new ICPControllerHelper.Vector2dZUpFrame("yes", worldFrame);
      dynamicsFrame.setXAxis(desiredICPVelocity);

      icpError = new FrameVector2D(dynamicsFrame, 0.02, 0.02);
      FrameVector2D feedback = new FrameVector2D(icpError);
      feedback.scale(1.0 + feedbackGain);
      feedback.setX(MathTools.clamp(feedback.getX(), 0.01));
      feedback.setY(MathTools.clamp(feedback.getY(), 0.04));

      icpError.changeFrame(worldFrame);
      feedback.changeFrame(worldFrame);

      currentICP.add(icpError, desiredICP);
      expectedClampedCMP.add(feedback, perfectCMP);

      ((YoDouble) registry.findVariable("feedbackPartMaxValueParallelToMotion")).set(0.01);
      ((YoDouble) registry.findVariable("feedbackPartMaxValueOrthogonalToMotion")).set(0.04);

      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

      EuclidCoreTestTools.assertEquals(expectedClampedCMP, controller.getDesiredCMP(), epsilon);

      // Now do asymmetric clamping, in random directions
      Random random = new Random(1738L);
      for (int i = 0; i < 1000; i++)
      {
         desiredICP = EuclidFrameRandomTools.nextFramePoint2D(random, worldFrame, 0.03);

         desiredICPVelocity.set(desiredICP);
         desiredICPVelocity.sub(perfectCMP);
         desiredICPVelocity.scale(omega);

         dynamicsFrame = new ICPControllerHelper.Vector2dZUpFrame("yes", worldFrame);
         dynamicsFrame.setXAxis(desiredICPVelocity);

         icpError = EuclidFrameRandomTools.nextFrameVector2D(random, dynamicsFrame, new Vector2D(0.02, 0.02));
         double parallelClamp = RandomNumbers.nextDouble(random, 0.01, 0.15);
         double orthogonalClamp = RandomNumbers.nextDouble(random, 0.01, 0.15);

         feedback = new FrameVector2D(icpError);
         feedback.scale(1.0 + feedbackGain);
         feedback.setX(MathTools.clamp(feedback.getX(), parallelClamp));
         feedback.setY(MathTools.clamp(feedback.getY(), orthogonalClamp));

         icpError.changeFrame(worldFrame);
         feedback.changeFrame(worldFrame);

         currentICP.add(icpError, desiredICP);
         expectedClampedCMP.add(feedback, perfectCMP);

         ((YoDouble) registry.findVariable("feedbackPartMaxValueParallelToMotion")).set(parallelClamp);
         ((YoDouble) registry.findVariable("feedbackPartMaxValueOrthogonalToMotion")).set(orthogonalClamp);

         controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

         EuclidCoreTestTools.assertEquals(expectedClampedCMP, controller.getDesiredCMP(), epsilon);
      }

      // Now do clamping with infinite parallel,
      desiredICP = new FramePoint2D(worldFrame, 0.05, 0.0);

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      dynamicsFrame = new ICPControllerHelper.Vector2dZUpFrame("yes", worldFrame);
      dynamicsFrame.setXAxis(desiredICPVelocity);

      double parallelClamp = Double.POSITIVE_INFINITY;
      double orthogonalClamp = 0.03;

      icpError = new FrameVector2D(dynamicsFrame, 0.02, 0.02);
      feedback = new FrameVector2D(icpError);
      feedback.scale(1.0 + feedbackGain);
      feedback.setY(MathTools.clamp(feedback.getY(), orthogonalClamp));

      icpError.changeFrame(worldFrame);
      feedback.changeFrame(worldFrame);

      currentICP.add(icpError, desiredICP);
      expectedClampedCMP.add(feedback, perfectCMP);

      ((YoDouble) registry.findVariable("feedbackPartMaxValueParallelToMotion")).set(parallelClamp);
      ((YoDouble) registry.findVariable("feedbackPartMaxValueOrthogonalToMotion")).set(orthogonalClamp);

      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

      EuclidCoreTestTools.assertEquals(expectedClampedCMP, controller.getDesiredCMP(), epsilon);

      // Now do clamping with infinite orthogonal,
      desiredICP = new FramePoint2D(worldFrame, 0.05, 0.0);

      desiredICPVelocity.set(desiredICP);
      desiredICPVelocity.sub(perfectCMP);
      desiredICPVelocity.scale(omega);

      dynamicsFrame = new ICPControllerHelper.Vector2dZUpFrame("yes", worldFrame);
      dynamicsFrame.setXAxis(desiredICPVelocity);

      parallelClamp = 0.02;
      orthogonalClamp = Double.POSITIVE_INFINITY;

      icpError = new FrameVector2D(dynamicsFrame, 0.02, 0.02);
      feedback = new FrameVector2D(icpError);
      feedback.scale(1.0 + feedbackGain);
      feedback.setX(MathTools.clamp(feedback.getX(), parallelClamp));

      icpError.changeFrame(worldFrame);
      feedback.changeFrame(worldFrame);

      currentICP.add(icpError, desiredICP);
      expectedClampedCMP.add(feedback, perfectCMP);

      ((YoDouble) registry.findVariable("feedbackPartMaxValueParallelToMotion")).set(parallelClamp);
      ((YoDouble) registry.findVariable("feedbackPartMaxValueOrthogonalToMotion")).set(orthogonalClamp);

      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

      EuclidCoreTestTools.assertEquals(expectedClampedCMP, controller.getDesiredCMP(), epsilon);
   }


   @Test
   public void testTransferWithPerfectTracking() throws Exception
   {
      YoRegistry registry = new YoRegistry("robert");

      double feedbackGain = 2.0;
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
      };

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();

      double controlDT = 0.001;
      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters, null, contactableFeet, controlDT, registry, null);
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
      FrameVector2D currentICPVelocity = new FrameVector2D();
      currentICP.set(desiredICP);
      currentICP.add(icpError);
      FramePoint2D currentCoM = new FramePoint2D(currentICP);

      controller.initialize();
      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoM, omega);

      Assert.assertTrue(controller.getDesiredCMP().epsilonEquals(perfectCMP, epsilon));
   }

   @Test
   public void testStandingConstrained() throws Exception
   {
      YoRegistry registry = new YoRegistry("robert");

      double feedbackGain = 2.0;
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
      };

      double footWidth = 0.1;
      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, footWidth, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();

      double controlDT = 0.001;
      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters, null, contactableFeet, controlDT, registry, null);
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
      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

      FramePoint2D desiredCMPExpected = new FramePoint2D();
      desiredCMPExpected.set(icpError);
      desiredCMPExpected.scale(feedbackGain + 1.0);
      desiredCMPExpected.add(perfectCMP);

      double maxY = stanceWidth / 2.0;// + footWidth / 2.0;
      double maxX = footLength / 2.0;

      desiredCMPExpected.setX(Math.min(maxX, desiredCMPExpected.getX()));
      desiredCMPExpected.setY(Math.min(maxY, desiredCMPExpected.getY()));

      EuclidFrameTestTools.assertGeometricallyEquals(desiredCMPExpected, controller.getDesiredCMP(), epsilon);
   }

   @Test
   public void testStandingUnconstrained() throws Exception
   {
      YoRegistry registry = new YoRegistry("robert");

      double feedbackGain = 2.0;
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
      };

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(10.0, 5.0, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();

      double controlDT = 0.001;
      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters, null, contactableFeet, controlDT, registry, null);
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
      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

      FramePoint2D desiredCMPExpected = new FramePoint2D();
      FrameVector2D expectedCoPFeedbackDelta = new FrameVector2D();
      FrameVector2D expectedCMPFeedbackDelta = new FrameVector2D();
      expectedCoPFeedbackDelta.set(icpError);
      expectedCoPFeedbackDelta.scale(feedbackGain + 1.0);

      desiredCMPExpected.set(perfectCMP);
      desiredCMPExpected.add(expectedCMPFeedbackDelta);
      desiredCMPExpected.add(expectedCoPFeedbackDelta);

      EuclidFrameTestTools.assertGeometricallyEquals(icpError, controller.icpError, epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals(perfectCMP, controller.perfectCoP, epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals(perfectCMP, controller.perfectCMP, epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals(expectedCMPFeedbackDelta, controller.feedbackCMPDelta, epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals(expectedCoPFeedbackDelta, controller.feedbackCoPDelta, epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals(desiredCMPExpected, controller.getDesiredCMP(), epsilon);
   }

   @Test
   public void testStandingConstrainedWithAngularMomentum() throws Exception
   {
      YoRegistry registry = new YoRegistry("robert");

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
      };

      TestWalkingControllerParameters walkingControllerParameters = new TestWalkingControllerParameters();
      SideDependentList<FootSpoof> contactableFeet = setupContactableFeet(footLength, 0.1, stanceWidth);
      BipedSupportPolygons bipedSupportPolygons = setupBipedSupportPolygons(contactableFeet, registry);
      FrameConvexPolygon2DReadOnly supportPolygonInWorld = bipedSupportPolygons.getSupportPolygonInWorld();

      double controlDT = 0.001;
      ICPController controller = new ICPController(walkingControllerParameters, optimizationParameters, null, contactableFeet, controlDT, registry, null);
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
      controller.compute(supportPolygonInWorld, desiredICP, desiredICPVelocity, new FramePoint2D(), perfectCMP, currentICP, currentCoMPosition, omega);

      FramePoint2D desiredCMPExpected = new FramePoint2D();
      desiredCMPExpected.set(icpError);
      desiredCMPExpected.scale(feedbackGain + 1.0);
      desiredCMPExpected.add(perfectCMP);

      EuclidFrameTestTools.assertGeometricallyEquals(desiredCMPExpected, controller.getDesiredCMP(), epsilon);
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

   private BipedSupportPolygons setupBipedSupportPolygons(SideDependentList<FootSpoof> contactableFeet, YoRegistry registry)
   {
      SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
      SideDependentList<YoPlaneContactState> contactStates = new SideDependentList<>();

      for (RobotSide robotSide : RobotSide.values)
      {
         FootSpoof contactableFoot = contactableFeet.get(robotSide);
         soleZUpFrames.put(robotSide, new ZUpFrame(contactableFoot.getSoleFrame(), robotSide.getCamelCaseNameForStartOfExpression() + "ZUp"));

         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         RigidBodyBasics foot = contactableFoot.getRigidBody();
         ReferenceFrame soleFrame = contactableFoot.getSoleFrame();
         soleFrames.put(robotSide, soleFrame);
         List<FramePoint2D> contactFramePoints = contactableFoot.getContactPoints2d();
         double coefficientOfFriction = contactableFoot.getCoefficientOfFriction();
         YoPlaneContactState yoPlaneContactState = new YoPlaneContactState(sidePrefix
               + "Foot", foot, soleFrame, contactFramePoints, coefficientOfFriction, registry);
         yoPlaneContactState.setFullyConstrained();
         contactStates.put(robotSide, yoPlaneContactState);
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

   private class TestICPOptimizationParameters extends ICPControllerParameters
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
      public ICPControllerParameters getICPControllerParameters()
      {
         return new TestICPOptimizationParameters();
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
