package us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation;

import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.commons.MathTools.clamp;

public class SplitFractionToolsTest
{
   private static final double epsilon = 1e-8;
   private double defaultSplitFraction = 0.0;
   private double currentSplitFraction = 0.0;
   private double transferSplitFraction = 0.0;
   private double actualSplitFraction = 0.0;
   private double expectedForwardShift = 0.0;
   private double expectedBackwardShift = 0.0;

   public void calculateValues()
   {
      double percentShift = 0.0;
      if (transferSplitFraction > defaultSplitFraction)
      {
         percentShift = Math.abs(transferSplitFraction - defaultSplitFraction) / (1 - defaultSplitFraction);
      }
      else
      {
         percentShift = Math.abs(transferSplitFraction - defaultSplitFraction) / defaultSplitFraction;
      }

      actualSplitFraction = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultSplitFraction);
      double expectedForwardShiftedSplitFraction = percentShift * (1 - currentSplitFraction);
      double expectedBackwardShiftedSplitFraction = percentShift * currentSplitFraction;
      expectedForwardShift = currentSplitFraction + expectedForwardShiftedSplitFraction;
      expectedBackwardShift = currentSplitFraction - expectedBackwardShiftedSplitFraction;
   }

   @Test
   public void testEqual()
   {
      defaultSplitFraction = 0.5;
      currentSplitFraction = 0.5;
      transferSplitFraction = 0.5;
      calculateValues();
      assertEquals(expectedForwardShift, actualSplitFraction, epsilon);
   }

   @Test
   public void testForward()
   {
      defaultSplitFraction = 0.5;
      currentSplitFraction = 0.5;
      transferSplitFraction = 0.7;
      calculateValues();
      assertEquals(expectedForwardShift, actualSplitFraction, epsilon);
   }

   @Test
   public void testBackward()
   {
      defaultSplitFraction = 0.7;
      currentSplitFraction = 0.5;
      transferSplitFraction = 0.3;
      calculateValues();
      assertEquals(expectedBackwardShift, actualSplitFraction, epsilon);
   }

   @Test
   public void testNegative()
   {
      defaultSplitFraction = -0.7;
      currentSplitFraction = -0.5;
      transferSplitFraction = -0.3;
      defaultSplitFraction = clamp(defaultSplitFraction, 0, 1);
      currentSplitFraction = clamp(currentSplitFraction, 0, 1);
      transferSplitFraction = clamp(transferSplitFraction, 0, 1);
      calculateValues();
      assertEquals(expectedBackwardShift, actualSplitFraction, epsilon);
   }

   @Test
   public void testRandom()
   {
      Random random = new Random(66);
      defaultSplitFraction = random.nextDouble();
      currentSplitFraction = random.nextDouble();
      transferSplitFraction = random.nextDouble();
      System.out.printf("Values: %.2f, %.2f, %.2f\n", defaultSplitFraction, currentSplitFraction, transferSplitFraction);
      calculateValues();
      if (transferSplitFraction > defaultSplitFraction)
      {
         assertEquals(expectedForwardShift, actualSplitFraction, epsilon);
      }
      else
      {
         assertEquals(expectedBackwardShift, actualSplitFraction, epsilon);
      }
   }
   //   @Test
   //   public void testValuesStayInBounds()
   //   {
   //      double defaultSplitFraction = 0.5;
   //      double currentSplitFraction = 0.5;
   //      double transferSplitFraction = 0.5;
   //
   //      // check that if we don't tell it to move, it doesn't move.
   //      assertEquals(0.5, SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultSplitFraction), epsilon);
   //
   //      // the current value is the nominal value, so we should get what we are asking for.
   //      transferSplitFraction = 0.7;
   //      double expectedForwardShiftedSplitFraction = transferSplitFraction;
   //      double actualSplitFraction = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultSplitFraction);
   //      assertEquals(expectedForwardShiftedSplitFraction, actualSplitFraction, epsilon);
   //
   //      // let's move the current value back a little, so that we get less total shift.
   //      currentSplitFraction = 0.3;
   //      double ratioCurrentToDefault = (1.0 - currentSplitFraction) / (1.0 - defaultSplitFraction);
   //      double expectedShift = (transferSplitFraction - defaultSplitFraction) * ratioCurrentToDefault;
   //      expectedForwardShiftedSplitFraction = expectedShift + currentSplitFraction;
   //      actualSplitFraction = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultSplitFraction);
   //      assertEquals(expectedForwardShiftedSplitFraction, actualSplitFraction, epsilon);
   //      assertTrue(actualSplitFraction < transferSplitFraction);
   //
   //      // let's move the current value forward of the nominal value
   //      currentSplitFraction = 0.7;
   //      ratioCurrentToDefault = (1.0 - currentSplitFraction) / (1.0 - defaultSplitFraction);
   //      expectedShift = (transferSplitFraction - defaultSplitFraction) * ratioCurrentToDefault;
   //      expectedForwardShiftedSplitFraction = expectedShift + currentSplitFraction;
   //      actualSplitFraction = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultSplitFraction);
   //      assertEquals(expectedForwardShiftedSplitFraction, actualSplitFraction, epsilon);
   //      assertTrue(actualSplitFraction > transferSplitFraction);
   //
   //      // let's move the current value back a little, so that we get less total shift.
   //      transferSplitFraction = 0.3;
   //      currentSplitFraction = 0.5;
   //      double expectedBackwardShiftedSplitFraction = transferSplitFraction;
   //      actualSplitFraction = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultSplitFraction);
   //      assertEquals(expectedBackwardShiftedSplitFraction, actualSplitFraction, epsilon);
   //
   //      currentSplitFraction = 0.3;
   //      ratioCurrentToDefault = (defaultSplitFraction - currentSplitFraction) / defaultSplitFraction;
   //      expectedShift = -currentSplitFraction * ratioCurrentToDefault;
   //      expectedBackwardShiftedSplitFraction = expectedShift + currentSplitFraction;
   //      actualSplitFraction = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultSplitFraction);
   //      assertEquals(expectedBackwardShiftedSplitFraction, actualSplitFraction, epsilon);
   //      assertTrue(actualSplitFraction < transferSplitFraction);
   //
   //      // let's move the current value forward of the nominal value
   //      currentSplitFraction = 0.7;
   //      ratioCurrentToDefault = (defaultSplitFraction - currentSplitFraction) / defaultSplitFraction;
   //      expectedShift = -currentSplitFraction * ratioCurrentToDefault;
   //      expectedBackwardShiftedSplitFraction = expectedShift + currentSplitFraction;
   //      actualSplitFraction = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultSplitFraction);
   //      assertEquals(expectedBackwardShiftedSplitFraction, actualSplitFraction, epsilon);
   //      assertTrue(actualSplitFraction > transferSplitFraction);
   //
   //      // TODO tell it to try and do negative shifts
   //   }
}


