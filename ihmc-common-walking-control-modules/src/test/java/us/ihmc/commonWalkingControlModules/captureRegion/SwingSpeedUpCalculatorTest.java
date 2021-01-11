package us.ihmc.commonWalkingControlModules.captureRegion;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.capturePoint.SwingSpeedUpCalculator;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

import static us.ihmc.robotics.Assert.assertEquals;

public class SwingSpeedUpCalculatorTest
{
   @Test
   public void testEstimateTimeBetweenPoints()
   {
      SwingSpeedUpCalculator swingSpeedUpCalculator = new SwingSpeedUpCalculator();

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

      double speedUpTime = swingSpeedUpCalculator.estimateDeltaTimeBetweenDesiredICPAndActualICP(desiredICP, desiredCMP, touchdownICP, actualICP, omega);

      assertEquals(speedUpTimeExpected, speedUpTime, 1e-7);

      speedUpTimeExpected = 0.8;
      actualICP = new FramePoint2D();
      actualICP.sub(desiredICP, desiredCMP);
      actualICP.scale(Math.exp(omega * speedUpTimeExpected));
      actualICP.add(desiredCMP);

      speedUpTime = swingSpeedUpCalculator.estimateDeltaTimeBetweenDesiredICPAndActualICP(desiredICP, desiredCMP, touchdownICP, actualICP, omega);

      assertEquals(timeRemaining, speedUpTime, 1e-7);

      speedUpTimeExpected = -0.2;
      actualICP = new FramePoint2D();
      actualICP.sub(desiredICP, desiredCMP);
      actualICP.scale(Math.exp(omega * speedUpTimeExpected));
      actualICP.add(desiredCMP);

      speedUpTime = swingSpeedUpCalculator.estimateDeltaTimeBetweenDesiredICPAndActualICP(desiredICP, desiredCMP, touchdownICP, actualICP, omega);

      assertEquals(speedUpTimeExpected, speedUpTime, 1e-7);

      for (double time = -0.4; time <= timeRemaining; time += 0.001)
      {
         actualICP = new FramePoint2D();
         actualICP.sub(desiredICP, desiredCMP);
         actualICP.scale(Math.exp(omega * time));
         actualICP.add(desiredCMP);

         speedUpTime = swingSpeedUpCalculator.estimateDeltaTimeBetweenDesiredICPAndActualICP(desiredICP, desiredCMP, touchdownICP, actualICP, omega);

         assertEquals(time, speedUpTime, 1e-7);
      }
   }
}
