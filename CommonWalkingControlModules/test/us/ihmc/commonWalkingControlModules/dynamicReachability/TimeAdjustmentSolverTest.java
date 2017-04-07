package us.ihmc.commonWalkingControlModules.dynamicReachability;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.exceptions.NoConvergenceException;

import static org.junit.Assert.*;

public class TimeAdjustmentSolverTest
{
   private static final double epsilon = 0.00001;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @ContinuousIntegrationTest(estimatedDuration = 2.0)
   @Test(timeout = 2000)
   public void testWithoutHigherSteps()
   {
      int maximumNumberOfSteps = 3;
      YoVariableRegistry registry = new YoVariableRegistry("test");

      TimeAdjustmentSolver solver = new TimeAdjustmentSolver(maximumNumberOfSteps, false, registry);

      solver.setNumberOfFootstepsToConsider(maximumNumberOfSteps);
      solver.setNumberOfFootstepsRegistered(1);
      solver.reshape();

      FrameVector currentInitialTransferGradient = new FrameVector(worldFrame, -0.0005, -0.0005, 0.0);
      FrameVector currentEndTransferGradient = new FrameVector(worldFrame, -0.0005, -0.0005, 0.0);
      FrameVector currentInitialSwingGradient = new FrameVector(worldFrame, -0.01, -0.01, 0.0);
      FrameVector currentEndSwingGradient = new FrameVector(worldFrame, 0.015, 0.015, 0.0);
      FrameVector nextInitialTransferGradient = new FrameVector(worldFrame, -0.1, -0.1, 0.0);
      FrameVector nextEndTransferGradient = new FrameVector(worldFrame, -0.005, -0.005, 0.0);

      double currentTransferDuration = 1.0;
      double currentTransferAlpha = 0.5;

      double currentSwingDuration = 1.2;
      double currentSwingAlpha = 0.5;

      double nextTransferDuration = 0.8;
      double nextTransferAlpha = 0.5;

      double desiredAdjustment = 0.05;

      solver.setCurrentInitialTransferGradient(currentInitialTransferGradient);
      solver.setCurrentEndTransferGradient(currentEndTransferGradient);
      solver.setCurrentInitialSwingGradient(currentInitialSwingGradient);
      solver.setCurrentEndSwingGradient(currentEndSwingGradient);
      solver.setNextInitialTransferGradient(nextInitialTransferGradient);
      solver.setNextEndTransferGradient(nextEndTransferGradient);

      solver.setCurrentTransferDuration(currentTransferDuration, currentTransferAlpha);
      solver.setCurrentSwingDuration(currentSwingDuration, currentSwingAlpha);
      solver.setNextTransferDuration(nextTransferDuration, nextTransferAlpha);

      solver.setDesiredParallelAdjustment(desiredAdjustment);

      boolean caughtError = false;
      try
      {
         solver.compute();
      }
      catch (NoConvergenceException e)
      {
         caughtError = true;
      }

      assertFalse(caughtError);

      double currentInitialTransferAdjustment = solver.getCurrentInitialTransferAdjustment();
      double currentEndTransferAdjustment = solver.getCurrentEndTransferAdjustment();
      double currentInitialSwingAdjustment = solver.getCurrentInitialSwingAdjustment();
      double currentEndSwingAdjustment = solver.getCurrentEndSwingAdjustment();
      double nextInitialTransferAdjustment = solver.getNextInitialTransferAdjustment();
      double nextEndTransferAdjustment = solver.getNextEndTransferAdjustment();

      // check direction of adjustments
      assertTrue(currentInitialTransferAdjustment < 0.0);
      assertTrue(currentEndTransferAdjustment < 0.0);
      assertTrue(currentInitialSwingAdjustment < 0.0);
      assertTrue(currentEndSwingAdjustment > 0.0);
      assertTrue(nextInitialTransferAdjustment < 0.0);

      // current transfer should be approximately the same, as the gradients are the same
      assertEquals(currentInitialTransferAdjustment, currentEndTransferAdjustment, epsilon);

      /** Be careful on some of these tests - the size of some is a function of the weights **/

      // next initial transfer should be by far the greatest.
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(currentInitialTransferAdjustment));
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(currentEndTransferAdjustment));
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(currentInitialSwingAdjustment));
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(currentEndSwingAdjustment));
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(nextEndTransferAdjustment));

      // end swing adjustment should be second largest
      assertTrue(Math.abs(currentEndSwingAdjustment) > Math.abs(currentInitialTransferAdjustment));
      assertTrue(Math.abs(currentEndSwingAdjustment) > Math.abs(currentEndTransferAdjustment));
      assertTrue(Math.abs(currentEndSwingAdjustment) > Math.abs(currentInitialSwingAdjustment));
      assertTrue(Math.abs(currentEndSwingAdjustment) > Math.abs(nextEndTransferAdjustment));

      // initial swing adjustment should be third largest
      assertTrue(Math.abs(currentInitialSwingAdjustment) > Math.abs(currentInitialTransferAdjustment));
      assertTrue(Math.abs(currentInitialSwingAdjustment) > Math.abs(currentEndTransferAdjustment));
      assertTrue(Math.abs(currentInitialSwingAdjustment) > Math.abs(nextEndTransferAdjustment));

      caughtError = false;
      try
      {
         solver.getHigherSwingAdjustment(0);
      }
      catch(RuntimeException e)
      {
         caughtError = true;
      }

      // should have gotten an out of bounds exception
      assertTrue(caughtError);

      caughtError = false;
      try
      {
         solver.getHigherTransferAdjustment(0);
      }
      catch(RuntimeException e)
      {
         caughtError = true;
      }

      // should have gotten an out of bounds exception
      assertTrue(caughtError);
   }

   /**
    * We only have only told it to consider 3 steps, so none of the higher steps should be allowed.
    */
   @ContinuousIntegrationTest(estimatedDuration = 2.0)
   @Test(timeout = 2000)
   public void testWithHigherStepsButOutOfBounds()
   {
      int maximumNumberOfSteps = 3;
      YoVariableRegistry registry = new YoVariableRegistry("test");

      TimeAdjustmentSolver solver = new TimeAdjustmentSolver(maximumNumberOfSteps, true, registry);

      solver.setNumberOfFootstepsToConsider(maximumNumberOfSteps);
      solver.setNumberOfFootstepsRegistered(2);
      solver.reshape();

      FrameVector currentInitialTransferGradient = new FrameVector(worldFrame, -0.0005, -0.0005, 0.0);
      FrameVector currentEndTransferGradient = new FrameVector(worldFrame, -0.0005, -0.0005, 0.0);
      FrameVector currentInitialSwingGradient = new FrameVector(worldFrame, -0.01, -0.01, 0.0);
      FrameVector currentEndSwingGradient = new FrameVector(worldFrame, 0.015, 0.015, 0.0);
      FrameVector nextInitialTransferGradient = new FrameVector(worldFrame, -0.1, -0.1, 0.0);
      FrameVector nextEndTransferGradient = new FrameVector(worldFrame, -0.005, -0.005, 0.0);

      double currentTransferDuration = 1.0;
      double currentTransferAlpha = 0.5;

      double currentSwingDuration = 1.2;
      double currentSwingAlpha = 0.5;

      double nextTransferDuration = 0.8;
      double nextTransferAlpha = 0.5;

      double desiredAdjustment = 0.05;

      solver.setCurrentInitialTransferGradient(currentInitialTransferGradient);
      solver.setCurrentEndTransferGradient(currentEndTransferGradient);
      solver.setCurrentInitialSwingGradient(currentInitialSwingGradient);
      solver.setCurrentEndSwingGradient(currentEndSwingGradient);
      solver.setNextInitialTransferGradient(nextInitialTransferGradient);
      solver.setNextEndTransferGradient(nextEndTransferGradient);

      solver.setCurrentTransferDuration(currentTransferDuration, currentTransferAlpha);
      solver.setCurrentSwingDuration(currentSwingDuration, currentSwingAlpha);
      solver.setNextTransferDuration(nextTransferDuration, nextTransferAlpha);

      solver.setDesiredParallelAdjustment(desiredAdjustment);

      boolean caughtError = false;
      try
      {
         solver.compute();
      }
      catch (NoConvergenceException e)
      {
         caughtError = true;
      }

      assertFalse(caughtError);

      double currentInitialTransferAdjustment = solver.getCurrentInitialTransferAdjustment();
      double currentEndTransferAdjustment = solver.getCurrentEndTransferAdjustment();
      double currentInitialSwingAdjustment = solver.getCurrentInitialSwingAdjustment();
      double currentEndSwingAdjustment = solver.getCurrentEndSwingAdjustment();
      double nextInitialTransferAdjustment = solver.getNextInitialTransferAdjustment();
      double nextEndTransferAdjustment = solver.getNextEndTransferAdjustment();

      // check direction of adjustments
      assertTrue(currentInitialTransferAdjustment < 0.0);
      assertTrue(currentEndTransferAdjustment < 0.0);
      assertTrue(currentInitialSwingAdjustment < 0.0);
      assertTrue(currentEndSwingAdjustment > 0.0);
      assertTrue(nextInitialTransferAdjustment < 0.0);

      // current transfer should be approximately the same, as the gradients are the same
      assertEquals(currentInitialTransferAdjustment, currentEndTransferAdjustment, epsilon);

      /** Be careful on some of these tests - the size of some is a function of the weights **/

      // next initial transfer should be by far the greatest.
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(currentInitialTransferAdjustment));
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(currentEndTransferAdjustment));
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(currentInitialSwingAdjustment));
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(currentEndSwingAdjustment));
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(nextEndTransferAdjustment));

      // end swing adjustment should be second largest
      assertTrue(Math.abs(currentEndSwingAdjustment) > Math.abs(currentInitialTransferAdjustment));
      assertTrue(Math.abs(currentEndSwingAdjustment) > Math.abs(currentEndTransferAdjustment));
      assertTrue(Math.abs(currentEndSwingAdjustment) > Math.abs(currentInitialSwingAdjustment));
      assertTrue(Math.abs(currentEndSwingAdjustment) > Math.abs(nextEndTransferAdjustment));

      // initial swing adjustment should be third largest
      assertTrue(Math.abs(currentInitialSwingAdjustment) > Math.abs(currentInitialTransferAdjustment));
      assertTrue(Math.abs(currentInitialSwingAdjustment) > Math.abs(currentEndTransferAdjustment));
      assertTrue(Math.abs(currentInitialSwingAdjustment) > Math.abs(nextEndTransferAdjustment));

      caughtError = false;
      try
      {
         solver.getHigherSwingAdjustment(0);
      }
      catch(RuntimeException e)
      {
         caughtError = true;
      }

      // should have gotten an out of bounds exception
      assertTrue(caughtError);

      caughtError = false;
      try
      {
         solver.getHigherTransferAdjustment(0);
      }
      catch(RuntimeException e)
      {
         caughtError = true;
      }

      // should have gotten an out of bounds exception
      assertTrue(caughtError);
   }

   /**
    * We are not submitting higher gradients, so there should be zero adjustment
    */
   @ContinuousIntegrationTest(estimatedDuration = 2.0)
   @Test(timeout = 2000)
   public void testWithHigherStepsButNotSubmitted()
   {
      int maximumNumberOfSteps = 4;
      YoVariableRegistry registry = new YoVariableRegistry("test");

      TimeAdjustmentSolver solver = new TimeAdjustmentSolver(maximumNumberOfSteps, true, registry);

      solver.setNumberOfFootstepsToConsider(maximumNumberOfSteps);
      solver.setNumberOfFootstepsRegistered(2);
      solver.reshape();

      FrameVector currentInitialTransferGradient = new FrameVector(worldFrame, -0.0005, -0.0005, 0.0);
      FrameVector currentEndTransferGradient = new FrameVector(worldFrame, -0.0005, -0.0005, 0.0);
      FrameVector currentInitialSwingGradient = new FrameVector(worldFrame, -0.01, -0.01, 0.0);
      FrameVector currentEndSwingGradient = new FrameVector(worldFrame, 0.015, 0.015, 0.0);
      FrameVector nextInitialTransferGradient = new FrameVector(worldFrame, -0.1, -0.1, 0.0);
      FrameVector nextEndTransferGradient = new FrameVector(worldFrame, -0.005, -0.005, 0.0);

      double currentTransferDuration = 1.0;
      double currentTransferAlpha = 0.5;

      double currentSwingDuration = 1.2;
      double currentSwingAlpha = 0.5;

      double nextTransferDuration = 0.8;
      double nextTransferAlpha = 0.5;

      double desiredAdjustment = 0.05;

      solver.setCurrentInitialTransferGradient(currentInitialTransferGradient);
      solver.setCurrentEndTransferGradient(currentEndTransferGradient);
      solver.setCurrentInitialSwingGradient(currentInitialSwingGradient);
      solver.setCurrentEndSwingGradient(currentEndSwingGradient);
      solver.setNextInitialTransferGradient(nextInitialTransferGradient);
      solver.setNextEndTransferGradient(nextEndTransferGradient);

      solver.setCurrentTransferDuration(currentTransferDuration, currentTransferAlpha);
      solver.setCurrentSwingDuration(currentSwingDuration, currentSwingAlpha);
      solver.setNextTransferDuration(nextTransferDuration, nextTransferAlpha);

      solver.setDesiredParallelAdjustment(desiredAdjustment);

      boolean caughtError = false;
      try
      {
         solver.compute();
      }
      catch (NoConvergenceException e)
      {
         caughtError = true;
      }

      assertFalse(caughtError);

      double currentInitialTransferAdjustment = solver.getCurrentInitialTransferAdjustment();
      double currentEndTransferAdjustment = solver.getCurrentEndTransferAdjustment();
      double currentInitialSwingAdjustment = solver.getCurrentInitialSwingAdjustment();
      double currentEndSwingAdjustment = solver.getCurrentEndSwingAdjustment();
      double nextInitialTransferAdjustment = solver.getNextInitialTransferAdjustment();
      double nextEndTransferAdjustment = solver.getNextEndTransferAdjustment();

      // check direction of adjustments
      assertTrue(currentInitialTransferAdjustment < 0.0);
      assertTrue(currentEndTransferAdjustment < 0.0);
      assertTrue(currentInitialSwingAdjustment < 0.0);
      assertTrue(currentEndSwingAdjustment > 0.0);
      assertTrue(nextInitialTransferAdjustment < 0.0);

      // current transfer should be approximately the same, as the gradients are the same
      assertEquals(currentInitialTransferAdjustment, currentEndTransferAdjustment, epsilon);

      /** Be careful on some of these tests - the size of some is a function of the weights **/

      // next initial transfer should be by far the greatest.
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(currentInitialTransferAdjustment));
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(currentEndTransferAdjustment));
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(currentInitialSwingAdjustment));
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(currentEndSwingAdjustment));
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(nextEndTransferAdjustment));

      // end swing adjustment should be second largest
      assertTrue(Math.abs(currentEndSwingAdjustment) > Math.abs(currentInitialTransferAdjustment));
      assertTrue(Math.abs(currentEndSwingAdjustment) > Math.abs(currentEndTransferAdjustment));
      assertTrue(Math.abs(currentEndSwingAdjustment) > Math.abs(currentInitialSwingAdjustment));
      assertTrue(Math.abs(currentEndSwingAdjustment) > Math.abs(nextEndTransferAdjustment));

      // initial swing adjustment should be third largest
      assertTrue(Math.abs(currentInitialSwingAdjustment) > Math.abs(currentInitialTransferAdjustment));
      assertTrue(Math.abs(currentInitialSwingAdjustment) > Math.abs(currentEndTransferAdjustment));
      assertTrue(Math.abs(currentInitialSwingAdjustment) > Math.abs(nextEndTransferAdjustment));

      // should be able to get first higher swing adjustment
      caughtError = false;
      try { solver.getHigherSwingAdjustment(0); }
      catch(RuntimeException e)
      { caughtError = true; }
      assertFalse(caughtError);

      // should not be able to get first higher swing adjustment
      caughtError = false;
      try { solver.getHigherSwingAdjustment(1); }
      catch(RuntimeException e)
      { caughtError = true; }
      assertTrue(caughtError);

      // should be able to get first higher transfer adjustment
      caughtError = false;
      try { solver.getHigherTransferAdjustment(0); }
      catch(RuntimeException e)
      { caughtError = true; }
      assertFalse(caughtError);

      // should not be able to get first higher transfer adjustment
      caughtError = false;
      try { solver.getHigherTransferAdjustment(1); }
      catch(RuntimeException e)
      { caughtError = true; }
      assertTrue(caughtError);


      // since we haven't submitted a gradient, the higher adjustments should be zero
      assertEquals(0.0, solver.getHigherSwingAdjustment(0), epsilon);
      assertEquals(0.0, solver.getHigherTransferAdjustment(0), epsilon);
   }

   /**
    * The higher gradients are really small, so the adjustment should be really small.
    */
   @ContinuousIntegrationTest(estimatedDuration = 2.0)
   @Test(timeout = 2000)
   public void testWithHigherSteps()
   {
      int maximumNumberOfSteps = 4;
      YoVariableRegistry registry = new YoVariableRegistry("test");

      TimeAdjustmentSolver solver = new TimeAdjustmentSolver(maximumNumberOfSteps, true, registry);

      solver.setNumberOfFootstepsToConsider(maximumNumberOfSteps);
      solver.setNumberOfFootstepsRegistered(2);
      solver.reshape();

      FrameVector currentInitialTransferGradient = new FrameVector(worldFrame, -0.0005, -0.0005, 0.0);
      FrameVector currentEndTransferGradient = new FrameVector(worldFrame, -0.0005, -0.0005, 0.0);
      FrameVector currentInitialSwingGradient = new FrameVector(worldFrame, -0.01, -0.01, 0.0);
      FrameVector currentEndSwingGradient = new FrameVector(worldFrame, 0.015, 0.015, 0.0);
      FrameVector nextInitialTransferGradient = new FrameVector(worldFrame, -0.1, -0.1, 0.0);
      FrameVector nextEndTransferGradient = new FrameVector(worldFrame, -0.005, -0.005, 0.0);

      FrameVector higherTransferGradient = new FrameVector(worldFrame, -0.0001, -0.0001, 0.0);
      FrameVector higherSwingGradient = new FrameVector(worldFrame, -0.0001, -0.0001, 0.0);

      double currentTransferDuration = 1.0;
      double currentTransferAlpha = 0.5;

      double currentSwingDuration = 1.2;
      double currentSwingAlpha = 0.5;

      double nextTransferDuration = 0.8;
      double nextTransferAlpha = 0.5;

      double desiredAdjustment = 0.05;

      solver.setCurrentInitialTransferGradient(currentInitialTransferGradient);
      solver.setCurrentEndTransferGradient(currentEndTransferGradient);
      solver.setCurrentInitialSwingGradient(currentInitialSwingGradient);
      solver.setCurrentEndSwingGradient(currentEndSwingGradient);
      solver.setNextInitialTransferGradient(nextInitialTransferGradient);
      solver.setNextEndTransferGradient(nextEndTransferGradient);

      solver.setCurrentTransferDuration(currentTransferDuration, currentTransferAlpha);
      solver.setCurrentSwingDuration(currentSwingDuration, currentSwingAlpha);
      solver.setNextTransferDuration(nextTransferDuration, nextTransferAlpha);

      solver.setHigherSwingDuration(0, currentSwingDuration);
      solver.setHigherTransferDuration(0, currentTransferDuration);

      solver.setHigherSwingGradient(0, higherSwingGradient);
      solver.setHigherTransferGradient(0, higherTransferGradient);

      solver.setDesiredParallelAdjustment(desiredAdjustment);

      boolean caughtError = false;
      try
      {
         solver.compute();
      }
      catch (NoConvergenceException e)
      {
         caughtError = true;
      }

      assertFalse(caughtError);

      double currentInitialTransferAdjustment = solver.getCurrentInitialTransferAdjustment();
      double currentEndTransferAdjustment = solver.getCurrentEndTransferAdjustment();
      double currentInitialSwingAdjustment = solver.getCurrentInitialSwingAdjustment();
      double currentEndSwingAdjustment = solver.getCurrentEndSwingAdjustment();
      double nextInitialTransferAdjustment = solver.getNextInitialTransferAdjustment();
      double nextEndTransferAdjustment = solver.getNextEndTransferAdjustment();

      // check direction of adjustments
      assertTrue(currentInitialTransferAdjustment < 0.0);
      assertTrue(currentEndTransferAdjustment < 0.0);
      assertTrue(currentInitialSwingAdjustment < 0.0);
      assertTrue(currentEndSwingAdjustment > 0.0);
      assertTrue(nextInitialTransferAdjustment < 0.0);

      // current transfer should be approximately the same, as the gradients are the same
      assertEquals(currentInitialTransferAdjustment, currentEndTransferAdjustment, epsilon);

      /** Be careful on some of these tests - the size of some is a function of the weights **/

      // next initial transfer should be by far the greatest.
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(currentInitialTransferAdjustment));
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(currentEndTransferAdjustment));
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(currentInitialSwingAdjustment));
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(currentEndSwingAdjustment));
      assertTrue(Math.abs(nextInitialTransferAdjustment) > Math.abs(nextEndTransferAdjustment));

      // end swing adjustment should be second largest
      assertTrue(Math.abs(currentEndSwingAdjustment) > Math.abs(currentInitialTransferAdjustment));
      assertTrue(Math.abs(currentEndSwingAdjustment) > Math.abs(currentEndTransferAdjustment));
      assertTrue(Math.abs(currentEndSwingAdjustment) > Math.abs(currentInitialSwingAdjustment));
      assertTrue(Math.abs(currentEndSwingAdjustment) > Math.abs(nextEndTransferAdjustment));

      // initial swing adjustment should be third largest
      assertTrue(Math.abs(currentInitialSwingAdjustment) > Math.abs(currentInitialTransferAdjustment));
      assertTrue(Math.abs(currentInitialSwingAdjustment) > Math.abs(currentEndTransferAdjustment));
      assertTrue(Math.abs(currentInitialSwingAdjustment) > Math.abs(nextEndTransferAdjustment));

      // should be able to get first higher swing adjustment
      caughtError = false;
      try { solver.getHigherSwingAdjustment(0); }
      catch(RuntimeException e)
      { caughtError = true; }
      assertFalse(caughtError);

      // should not be able to get first higher swing adjustment
      caughtError = false;
      try { solver.getHigherSwingAdjustment(1); }
      catch(RuntimeException e)
      { caughtError = true; }
      assertTrue(caughtError);

      // should be able to get first higher transfer adjustment
      caughtError = false;
      try { solver.getHigherTransferAdjustment(0); }
      catch(RuntimeException e)
      { caughtError = true; }
      assertFalse(caughtError);

      // should not be able to get first higher transfer adjustment
      caughtError = false;
      try { solver.getHigherTransferAdjustment(1); }
      catch(RuntimeException e)
      { caughtError = true; }
      assertTrue(caughtError);


      // since we haven't submitted a gradient, the higher adjustments should be zero
      assertNotEquals(0.0, solver.getHigherSwingAdjustment(0), epsilon);
      assertNotEquals(0.0, solver.getHigherTransferAdjustment(0), epsilon);
   }
}