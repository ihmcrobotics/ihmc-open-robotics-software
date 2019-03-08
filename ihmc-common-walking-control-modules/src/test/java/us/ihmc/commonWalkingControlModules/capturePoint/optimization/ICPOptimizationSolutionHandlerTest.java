package us.ihmc.commonWalkingControlModules.capturePoint.optimization;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGains;
import us.ihmc.commonWalkingControlModules.capturePoint.ICPControlGainsReadOnly;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoFramePoint2D;
import us.ihmc.yoVariables.variable.YoFramePose3D;

public class ICPOptimizationSolutionHandlerTest
{
   private YoVariableRegistry registry = new YoVariableRegistry("robert");

   private ICPOptimizationParameters parameters;
   private ICPOptimizationSolutionHandler solutionHandler;
   private ICPOptimizationQPSolver solver;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   private void setupTest(double deadbandSize)
   {
      setupTest(deadbandSize, 0.02);
   }

   private void setupTest(double deadbandSize, double resolution)
   {
      parameters = new TestICPOptimizationParameters(deadbandSize, resolution);
      solutionHandler = new ICPOptimizationSolutionHandler(parameters, new YoBoolean("useICPControlPolygons", registry), "test", registry);
      solver = new ICPOptimizationQPSolver(4, false);
      new DefaultParameterReader().readParametersInRegistry(registry);
   }

   private Footstep createFootsteps(double length, double width, int numberOfSteps)
   {
      ArrayList<Footstep> upcomingFootsteps = new ArrayList<>();
      RobotSide robotSide = RobotSide.LEFT;
      for (int i = 0; i < numberOfSteps; i++)
      {
         FramePose3D footstepPose = new FramePose3D(ReferenceFrame.getWorldFrame());

         footstepPose.setPosition(length * (i + 1), robotSide.negateIfRightSide(0.5 * width), 0.0);
         upcomingFootsteps.add(new Footstep(robotSide, footstepPose, false));

         FramePoint2D referenceFootstepPosition = new FramePoint2D(footstepPose.getPosition());

         robotSide = robotSide.getOppositeSide();
      }

      return upcomingFootsteps.get(0);
   }

   private FramePoint2D createReferenceLocations(Footstep upcomingFootstep)
   {
      FramePoint2D referenceLocation = new FramePoint2D();
      upcomingFootstep.getPosition2d(referenceLocation);

      return referenceLocation;
   }

   @Test
   public void testWellWithinDeadband()
   {
      double scale = 0.2;
      double deadbandSize = 0.05;
      runDeadbandTest(scale, deadbandSize);
   }

   @Test
   public void testALittleWithinDeadband()
   {
      double scale = 0.9;
      double deadbandSize = 0.05;
      runDeadbandTest(scale, deadbandSize);
   }

   @Test
   public void testJustWithinDeadband()
   {
      double scale = 0.99;
      double deadbandSize = 0.05;
      runDeadbandTest(scale, deadbandSize);
   }

   @Test
   public void testRightOnDeadband()
   {
      double scale = 1.0;
      double deadbandSize = 0.05;
      runDeadbandTest(scale, deadbandSize);
   }

   @Test
   public void testJustOutsideDeadband()
   {
      double scale = 1.01;
      double deadbandSize = 0.05;
      runDeadbandTest(scale, deadbandSize);
   }

   @Test
   public void testALittleOutsideDeadband()
   {
      double scale = 1.05;
      double deadbandSize = 0.05;
      runDeadbandTest(scale, deadbandSize);
   }

   @Test
   public void testWellOutsideDeadband()
   {
      double scale = 1.5;
      double deadbandSize = 0.05;
      runDeadbandTest(scale, deadbandSize);
   }

   @Test
   public void testWithinDeadbandResolution()
   {
      double scale = 1.1;
      double deadbandSize = 0.05;
      double deadbandResolution = 0.02;

      setupTest(deadbandSize, deadbandResolution);

      double stepLength = 0.5;
      double stanceWidth = 0.2;
      int numberOfSteps = 3;
      YoFramePose3D foostepSolution = new YoFramePose3D("footstepSolution", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint2D unclippedFootstepSolution = new YoFramePoint2D("unclippedFootstepSolution", ReferenceFrame.getWorldFrame(), registry);
      FramePose3D foostepPose = new FramePose3D();
      FramePoint2D foostepXYSolution = new FramePoint2D();

      Footstep upcomingFootstep = createFootsteps(stepLength, stanceWidth, numberOfSteps);
      FramePoint2D referenceFootstepPosition = createReferenceLocations(upcomingFootstep);

      double recursionMultiplier = Math.exp(-3.0 * 0.5);
      setupFeedbackTask(10000.0, stanceWidth);
      setupFootstepAdjustmentTask(5.0, recursionMultiplier, referenceFootstepPosition);

      FrameVector2D currentICPError = new FrameVector2D(ReferenceFrame.getWorldFrame(), scale * deadbandSize, 0.0);
      currentICPError.scale(recursionMultiplier);

      FramePoint2D perfectCMP = new FramePoint2D(ReferenceFrame.getWorldFrame(), -0.1, 0.0);
      assertTrue(solver.compute(currentICPError, perfectCMP));

      solutionHandler.extractFootstepSolution(foostepSolution, unclippedFootstepSolution, upcomingFootstep.getFootstepPose(),  solver);
      FrameVector2D copFeedback = new FrameVector2D();
      solver.getCoPFeedbackDifference(copFeedback);

      // first solution should be just outside the deadband
      FramePoint2D expectedUnclippedSolution = new FramePoint2D(referenceFootstepPosition);
      expectedUnclippedSolution.add(scale * deadbandSize, 0.0);
      FramePoint2D expectedClippedSolution = new FramePoint2D(referenceFootstepPosition);


      FrameVector2D clippedAdjustment = new FrameVector2D(ReferenceFrame.getWorldFrame(), (scale - 1.0) * deadbandSize, 0.0);
      expectedClippedSolution.add(clippedAdjustment);

      FrameVector2D adjustment = new FrameVector2D();
      adjustment.set(scale * deadbandSize, 0.0);

      foostepPose.set(foostepSolution);
      foostepXYSolution.set(foostepPose.getPosition());

      assertTrue(foostepXYSolution.epsilonEquals(expectedClippedSolution, 1e-3));
      assertTrue(unclippedFootstepSolution.epsilonEquals(expectedUnclippedSolution, 1e-3));
      assertEquals(true, solutionHandler.wasFootstepAdjusted());
      assertTrue(TupleTools.epsilonEquals(clippedAdjustment, solutionHandler.getClippedFootstepAdjustment(), 1e-3));
      assertTrue(TupleTools.epsilonEquals(adjustment, solutionHandler.getFootstepAdjustment(), 1e-3));

      // this should now produce a test within the next adjustment
      currentICPError = new FrameVector2D(ReferenceFrame.getWorldFrame(), scale * deadbandSize + 0.5 * deadbandResolution, 0.0);
      currentICPError.scale(recursionMultiplier);

      solver.compute(currentICPError, perfectCMP);

      solutionHandler.extractFootstepSolution(foostepSolution, unclippedFootstepSolution, upcomingFootstep.getFootstepPose(), solver);

      // new solution should be clipped to the same value
      expectedUnclippedSolution = new FramePoint2D(referenceFootstepPosition);
      expectedUnclippedSolution.add(scale * deadbandSize + 0.5 * deadbandResolution, 0.0);

      adjustment = new FrameVector2D();
      adjustment.set(scale * deadbandSize + 0.5 * deadbandResolution, 0.0);

      foostepPose.set(foostepSolution);
      foostepXYSolution.set(foostepPose.getPosition());

      assertTrue(foostepXYSolution.epsilonEquals(expectedClippedSolution, 1e-3));
      assertTrue(unclippedFootstepSolution.epsilonEquals(expectedUnclippedSolution, 1e-3));
      assertEquals(false, solutionHandler.wasFootstepAdjusted());
      assertTrue(TupleTools.epsilonEquals(clippedAdjustment, solutionHandler.getClippedFootstepAdjustment(), 1e-3));
      assertTrue(TupleTools.epsilonEquals(adjustment, solutionHandler.getFootstepAdjustment(), 1e-3));

      // test zeroing stuff out
      solutionHandler.zeroAdjustment();
      assertFalse(solutionHandler.wasFootstepAdjusted());
      assertEquals(0.0, solutionHandler.getFootstepAdjustment().length(), 1e-3);
   }

   @Test
   public void testOutsideDeadbandResolution()
   {
      double scale = 1.1;
      double deadbandSize = 0.05;
      double deadbandResolution = 0.02;

      setupTest(deadbandSize, deadbandResolution);

      double stepLength = 0.5;
      double stanceWidth = 0.2;
      int numberOfSteps = 3;
      YoFramePose3D foostepSolution = new YoFramePose3D("footstepSolution", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint2D unclippedFootstepSolution = new YoFramePoint2D("unclippedFootstepSolution", ReferenceFrame.getWorldFrame(), registry);
      FramePose3D footstepPose = new FramePose3D();
      FramePoint2D footstepXYSolution = new FramePoint2D();

      Footstep upcomingFootstep = createFootsteps(stepLength, stanceWidth, numberOfSteps);
      FramePoint2D referenceFootstepPosition = createReferenceLocations(upcomingFootstep);

      double recursionMultiplier = Math.exp(-3.0 * 0.5);
      setupFeedbackTask(10000.0, stanceWidth);
      setupFootstepAdjustmentTask(5.0, recursionMultiplier, referenceFootstepPosition);

      FrameVector2D currentICPError = new FrameVector2D(ReferenceFrame.getWorldFrame(), scale * deadbandSize, 0.0);
      currentICPError.scale(recursionMultiplier);

      FramePoint2D perfectCMP = new FramePoint2D(ReferenceFrame.getWorldFrame(), -0.1, 0.0);
      assertTrue(solver.compute(currentICPError, perfectCMP));

      solutionHandler.extractFootstepSolution(foostepSolution, unclippedFootstepSolution, upcomingFootstep.getFootstepPose(), solver);
      FrameVector2D copFeedback = new FrameVector2D();
      solver.getCoPFeedbackDifference(copFeedback);

      // first solution should be just outside the deadband
      FramePoint2D expectedUnclippedSolution = new FramePoint2D(referenceFootstepPosition);
      expectedUnclippedSolution.add(scale * deadbandSize, 0.0);
      FramePoint2D expectedClippedSolution = new FramePoint2D(referenceFootstepPosition);


      FrameVector2D clippedAdjustment = new FrameVector2D(ReferenceFrame.getWorldFrame(), (scale - 1.0) * deadbandSize, 0.0);
      expectedClippedSolution.add(clippedAdjustment);

      FrameVector2D adjustment = new FrameVector2D();
      adjustment.set(scale * deadbandSize, 0.0);

      footstepPose.set(foostepSolution);
      footstepXYSolution.set(footstepPose.getPosition());

      assertTrue(footstepXYSolution.epsilonEquals(expectedClippedSolution, 1e-3));
      assertTrue(unclippedFootstepSolution.epsilonEquals(expectedUnclippedSolution, 1e-3));
      assertEquals(true, solutionHandler.wasFootstepAdjusted());
      assertTrue(TupleTools.epsilonEquals(clippedAdjustment, solutionHandler.getClippedFootstepAdjustment(), 1e-3));
      assertTrue(TupleTools.epsilonEquals(adjustment, solutionHandler.getFootstepAdjustment(), 1e-3));

      // this should now produce a test within the next adjustment
      currentICPError = new FrameVector2D(ReferenceFrame.getWorldFrame(), scale * deadbandSize + 1.5 * deadbandResolution, 0.0);
      currentICPError.scale(recursionMultiplier);

      solver.compute(currentICPError, perfectCMP);

      solutionHandler.extractFootstepSolution(foostepSolution, unclippedFootstepSolution, upcomingFootstep.getFootstepPose(), solver);

      // new solution should be clipped to the same value
      expectedUnclippedSolution = new FramePoint2D(referenceFootstepPosition);
      adjustment = new FrameVector2D(ReferenceFrame.getWorldFrame(), scale * deadbandSize + 1.5 * deadbandResolution, 0.0);
      expectedUnclippedSolution.add(adjustment);

      expectedClippedSolution = new FramePoint2D(referenceFootstepPosition);
      clippedAdjustment = new FrameVector2D(ReferenceFrame.getWorldFrame(), scale * deadbandSize + 1.5 * deadbandResolution - deadbandSize, 0.0);
      expectedClippedSolution.add(clippedAdjustment);

      footstepPose.set(foostepSolution);
      footstepXYSolution.set(footstepPose.getPosition());

      assertTrue(footstepXYSolution.epsilonEquals(expectedClippedSolution, 1e-3));
      assertTrue(unclippedFootstepSolution.epsilonEquals(expectedUnclippedSolution, 1e-3));
      assertEquals(true, solutionHandler.wasFootstepAdjusted());
      assertTrue(TupleTools.epsilonEquals(clippedAdjustment, solutionHandler.getClippedFootstepAdjustment(), 1e-3));
      assertTrue(TupleTools.epsilonEquals(adjustment, solutionHandler.getFootstepAdjustment(), 1e-3));

      // test zeroing stuff out
      solutionHandler.zeroAdjustment();
      assertFalse(solutionHandler.wasFootstepAdjusted());
      assertEquals(0.0, solutionHandler.getFootstepAdjustment().length(), 1e-3);
   }


   private void runDeadbandTest(double scale, double deadbandSize)
   {
      setupTest(deadbandSize);

      double stepLength = 0.5;
      double stanceWidth = 0.2;
      int numberOfSteps = 3;

      YoFramePose3D footstepSolution = new YoFramePose3D("footstepSolution", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint2D unclippedFootstepSolution = new YoFramePoint2D("unclippedFootstepSolution", ReferenceFrame.getWorldFrame(), registry);
      FramePose3D footstepPose = new FramePose3D();
      FramePoint2D footstepXYSolution = new FramePoint2D();

      Footstep upcomingFootstep = createFootsteps(stepLength, stanceWidth, numberOfSteps);
      FramePoint2D referenceFootstepPosition = createReferenceLocations(upcomingFootstep);

      double recursionMultiplier = Math.exp(-3.0 * 0.5);
      setupFeedbackTask(10000.0, stanceWidth);
      setupFootstepAdjustmentTask(5.0, recursionMultiplier, referenceFootstepPosition);

      FrameVector2D currentICPError = new FrameVector2D(ReferenceFrame.getWorldFrame(), scale * deadbandSize, 0.0);
      currentICPError.scale(recursionMultiplier);

      FramePoint2D perfectCMP = new FramePoint2D(ReferenceFrame.getWorldFrame(), -0.1, 0.0);
      solver.compute(currentICPError, perfectCMP);

      solutionHandler.extractFootstepSolution(footstepSolution, unclippedFootstepSolution, upcomingFootstep.getFootstepPose(), solver);
      FrameVector2D copFeedback = new FrameVector2D();
      solver.getCoPFeedbackDifference(copFeedback);

      // this should be within the deadband
      FramePoint2D expectedUnclippedSolution = new FramePoint2D(referenceFootstepPosition);
      expectedUnclippedSolution.add(scale * deadbandSize, 0.0);
      FramePoint2D expectedClippedSolution = new FramePoint2D(referenceFootstepPosition);

      boolean wasAdjusted = scale > 1.0;

      FrameVector2D clippedAdjustment = new FrameVector2D();
      if (wasAdjusted)
         clippedAdjustment.set((scale - 1.0) * deadbandSize, 0.0);
      expectedClippedSolution.add(clippedAdjustment);

      FrameVector2D adjustment = new FrameVector2D();
      adjustment.set(scale * deadbandSize, 0.0);

      footstepPose.set(footstepSolution);
      footstepXYSolution.set(footstepPose.getPosition());

      assertTrue(footstepXYSolution.epsilonEquals(expectedClippedSolution, 1e-3));
      assertTrue(unclippedFootstepSolution.epsilonEquals(expectedUnclippedSolution, 1e-3));
      assertEquals(wasAdjusted, solutionHandler.wasFootstepAdjusted());
      assertTrue(TupleTools.epsilonEquals(clippedAdjustment, solutionHandler.getClippedFootstepAdjustment(), 1e-3));
      assertTrue(TupleTools.epsilonEquals(adjustment, solutionHandler.getFootstepAdjustment(), 1e-3));

      // test zeroing stuff out
      solutionHandler.zeroAdjustment();
      assertFalse(solutionHandler.wasFootstepAdjusted());
      assertEquals(0.0, solutionHandler.getFootstepAdjustment().length(), 1e-3);
   }

   private void setupFeedbackTask(double weight, double stanceWidth)
   {
      double footLength = 0.2;
      double footWidth = 0.1;

      FrameConvexPolygon2D supportPolygon = new FrameConvexPolygon2D();
      supportPolygon.addVertex(new Point2D(0.5 * footLength, 0.5 * footWidth - 0.5 * stanceWidth));
      supportPolygon.addVertex(new Point2D(0.5 * footLength, -0.5 * footWidth - 0.5 * stanceWidth));
      supportPolygon.addVertex(new Point2D(-0.5 * footLength, 0.5 * footWidth - 0.5 * stanceWidth));
      supportPolygon.addVertex(new Point2D(-0.5 * footLength, -0.5 * footWidth - 0.5 * stanceWidth));
      supportPolygon.update();

      double feedbackGain = 2.5;
      double dynamicsWeight = 100000.0;

      solver.setFeedbackConditions(feedbackGain, weight, dynamicsWeight);
      solver.addSupportPolygon(supportPolygon);
   }

   private void setupFootstepAdjustmentTask(double weight, double recursionMultiplier, FramePoint2D referenceFootstepPosition)
   {
      solver.setFootstepAdjustmentConditions(recursionMultiplier, weight, referenceFootstepPosition);
   }


   private class TestICPOptimizationParameters extends ICPOptimizationParameters
   {
      private final double deadbandSize;
      private final double resolution;

      public TestICPOptimizationParameters(double deadbandSize, double resolution)
      {
         this.deadbandSize = deadbandSize;
         this.resolution = resolution;
      }

      @Override public double getForwardFootstepWeight()
      {
         return 5.0;
      }

      @Override public double getLateralFootstepWeight()
      {
         return 5.0;
      }

      @Override public double getFootstepRateWeight()
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

      @Override public double getFeedbackRateWeight()
      {
         return 0.0001;
      }

      @Override
      public ICPControlGainsReadOnly getICPFeedbackGains()
      {
         ICPControlGains gains = new ICPControlGains();
         gains.setKpParallelToMotion(2.0);
         gains.setKpOrthogonalToMotion(3.0);

         return gains;
      }

      @Override public double getDynamicsObjectiveWeight()
      {
         return 1000.0;
      }

      @Override public double getDynamicsObjectiveDoubleSupportWeightModifier()
      {
         return 1.0;
      }

      @Override public double getAngularMomentumMinimizationWeight()
      {
         return 0.5;
      }

      @Override public boolean scaleStepRateWeightWithTime()
      {
         return false;
      }

      @Override public boolean scaleFeedbackWeightWithGain()
      {
         return false;
      }

      @Override public boolean useFeedbackRate()
      {
         return false;
      }

      @Override public boolean allowStepAdjustment()
      {
         return true;
      }

      @Override public boolean useAngularMomentum()
      {
         return false;
      }

      @Override public boolean useFootstepRate()
      {
         return false;
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
      public double getFootstepSolutionResolution()
      {
         return resolution;
      }
      @Override
      public double getMinimumTimeRemaining()
      {
         return 0.001;
      }

      @Override public double getAdjustmentDeadband()
      {
         return deadbandSize;
      }
   };
}
