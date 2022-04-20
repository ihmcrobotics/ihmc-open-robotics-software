package us.ihmc.commonWalkingControlModules.captureRegion;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.capturePoint.TimeAdjustmentCalculator;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

import static us.ihmc.robotics.Assert.assertEquals;

public class TimeAdjustmentCalculatorTest
{
   private static boolean visualize = false;

   @BeforeEach
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   }

   @Test
   public void testEstimateTimeBetweenPoints()
   {
      TimeAdjustmentCalculator timeAdjustmentCalculator = new TimeAdjustmentCalculator();

      FramePoint2D desiredICP = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.5, 1.0);
      FramePoint2D desiredCMP = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.3, 0.7);
      double omega = 3.0;
      double timeRemaining = 0.7;
      FramePoint2D touchdownICP = new FramePoint2D();

      touchdownICP.sub(desiredICP, desiredCMP);
      touchdownICP.scale(Math.exp(omega * timeRemaining));
      touchdownICP.add(desiredCMP);

      double speedUpTimeExpected = 0.3;
      FramePoint2D actualICP = new FramePoint2D();
      actualICP.sub(desiredICP, desiredCMP);
      actualICP.scale(Math.exp(omega * speedUpTimeExpected));
      actualICP.add(desiredCMP);

      TimeAdjustmentCalculatorVisualizer visualizer = null;
      if (visualize)
      {
         visualizer = new TimeAdjustmentCalculatorVisualizer(null, null);
         visualizer.updateInputs(actualICP, desiredICP, touchdownICP, desiredCMP);
      }

      double speedUpTime = timeAdjustmentCalculator.estimateDeltaTimeBetweenDesiredICPAndActualICP(desiredICP, desiredCMP, touchdownICP, actualICP, omega);

      if (visualize)
      {
         visualizer.updateOutputs(timeAdjustmentCalculator.getProjectedICPEstimate(), speedUpTime);
      }
      assertEquals(speedUpTimeExpected, speedUpTime, 1e-7);

      speedUpTimeExpected = 0.8;
      actualICP = new FramePoint2D();
      actualICP.sub(desiredICP, desiredCMP);
      actualICP.scale(Math.exp(omega * speedUpTimeExpected));
      actualICP.add(desiredCMP);

      if (visualize)
      {
         visualizer.updateInputs(actualICP, desiredICP, touchdownICP, desiredCMP);
      }

      speedUpTime = timeAdjustmentCalculator.estimateDeltaTimeBetweenDesiredICPAndActualICP(desiredICP, desiredCMP, touchdownICP, actualICP, omega);

      if (visualize)
      {
         visualizer.updateOutputs(timeAdjustmentCalculator.getProjectedICPEstimate(), speedUpTime);
      }
      assertEquals(timeRemaining, speedUpTime, 1e-7);

      speedUpTimeExpected = -0.2;
      actualICP = new FramePoint2D();
      actualICP.sub(desiredICP, desiredCMP);
      actualICP.scale(Math.exp(omega * speedUpTimeExpected));
      actualICP.add(desiredCMP);

      if (visualize)
      {
         visualizer.updateInputs(actualICP, desiredICP, touchdownICP, desiredCMP);
      }

      speedUpTime = timeAdjustmentCalculator.estimateDeltaTimeBetweenDesiredICPAndActualICP(desiredICP, desiredCMP, touchdownICP, actualICP, omega);

      if (visualize)
      {
         visualizer.updateOutputs(timeAdjustmentCalculator.getProjectedICPEstimate(), speedUpTime);
      }
      assertEquals(speedUpTimeExpected, speedUpTime, 1e-7);

      for (double time = -0.4; time <= timeRemaining; time += 0.001)
      {
         actualICP = new FramePoint2D();
         actualICP.sub(desiredICP, desiredCMP);
         actualICP.scale(Math.exp(omega * time));
         actualICP.add(desiredCMP);

         if (visualize)
         {
            visualizer.updateInputs(actualICP, desiredICP, touchdownICP, desiredCMP);
         }

         speedUpTime = timeAdjustmentCalculator.estimateDeltaTimeBetweenDesiredICPAndActualICP(desiredICP, desiredCMP, touchdownICP, actualICP, omega);

         if (visualize)
         {
            visualizer.updateOutputs(timeAdjustmentCalculator.getProjectedICPEstimate(), speedUpTime);
         }
         assertEquals(time, speedUpTime, 1e-7);
      }

      if (visualize)
         ThreadTools.sleepForever();
   }
}
