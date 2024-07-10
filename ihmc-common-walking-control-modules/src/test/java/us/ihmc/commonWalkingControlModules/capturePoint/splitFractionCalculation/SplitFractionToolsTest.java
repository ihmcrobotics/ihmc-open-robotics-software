package us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation;

import javafx.util.Pair;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class SplitFractionToolsTest
{
   private static final double epsilon = 1e-8;
   private double nominalShift = 0.0;
   private double currentSplitFraction = 0.0;
   private double desiredShift = 0.0;
   private double actualSplitFraction = 0.0;
   private double expectedForwardShift = 0.0;
   private double expectedBackwardShift = 0.0;

   public void calculateValues()
   {
      double percentShift = 0.0;
      if (desiredShift > nominalShift)
      {
         percentShift = Math.abs(desiredShift - nominalShift) / (1 - nominalShift);
      }
      else
      {
         percentShift = Math.abs(desiredShift - nominalShift) / nominalShift;
      }

      actualSplitFraction = SplitFractionTools.appendSplitFraction(desiredShift, currentSplitFraction, nominalShift);
      double expectedForwardShiftedSplitFraction = percentShift * (1 - currentSplitFraction);
      double expectedBackwardShiftedSplitFraction = percentShift * currentSplitFraction;
      expectedForwardShift = currentSplitFraction + expectedForwardShiftedSplitFraction;
      expectedBackwardShift = currentSplitFraction - expectedBackwardShiftedSplitFraction;
   }

   @Test
   public void testEqual()
   {
      nominalShift = 0.5;
      currentSplitFraction = 0.5;
      desiredShift = 0.5;
      calculateValues();
      assertEquals(expectedForwardShift, actualSplitFraction, epsilon);
   }

   @Test
   public void testForward()
   {
      nominalShift = 0.5;
      currentSplitFraction = 0.5;
      desiredShift = 0.7;
      calculateValues();
      assertEquals(expectedForwardShift, actualSplitFraction, epsilon);
   }

   @Test
   public void testBackward()
   {
      nominalShift = 0.7;
      currentSplitFraction = 0.5;
      desiredShift = 0.3;
      calculateValues();
      assertEquals(expectedBackwardShift, actualSplitFraction, epsilon);
   }

   @Test
   public void testNegative()
   {
      nominalShift = -0.7;
      currentSplitFraction = -0.5;
      desiredShift = -0.3;
      calculateValues();
      assertEquals(expectedBackwardShift, actualSplitFraction, epsilon);
   }

   @Test
   public void testRandom()
   {
      nominalShift = Math.random();
      currentSplitFraction = Math.random();
      desiredShift = Math.random();
      System.out.printf("Values: %.2f, %.2f, %.2f\n", nominalShift, currentSplitFraction, desiredShift);
      calculateValues();
      if (desiredShift > nominalShift)
      {
         assertEquals(expectedForwardShift, actualSplitFraction, epsilon);
      }
      else
      {
         assertEquals(expectedBackwardShift, actualSplitFraction, epsilon);
      }
   }
   @Test
   public void testValuesStayInBounds()
   {
      double defaultSplitFraction = 0.5;
      double currentSplitFraction = 0.5;
      double transferSplitFraction = 0.5;

      // check that if we don't tell it to move, it doesn't move.
      assertEquals(0.5, SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultSplitFraction), epsilon);

      // the current value is the nominal value, so we should get what we are asking for.
      transferSplitFraction = 0.7;
      double expectedForwardShiftedSplitFraction = transferSplitFraction;
      double actualSplitFraction = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultSplitFraction);
      assertEquals(expectedForwardShiftedSplitFraction, actualSplitFraction, epsilon);

      // let's move the current value back a little, so that we get less total shift.
      currentSplitFraction = 0.3;
      double ratioCurrentToDefault = (1.0 - currentSplitFraction) / (1.0 - defaultSplitFraction);
      double expectedShift = (transferSplitFraction - defaultSplitFraction) * ratioCurrentToDefault;
      expectedForwardShiftedSplitFraction = expectedShift + currentSplitFraction;
      actualSplitFraction = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultSplitFraction);
      assertEquals(expectedForwardShiftedSplitFraction, actualSplitFraction, epsilon);
      assertTrue(actualSplitFraction < transferSplitFraction);

      // let's move the current value forward of the nominal value
      currentSplitFraction = 0.7;
      ratioCurrentToDefault = (1.0 - currentSplitFraction) / (1.0 - defaultSplitFraction);
      expectedShift = (transferSplitFraction - defaultSplitFraction) * ratioCurrentToDefault;
      expectedForwardShiftedSplitFraction = expectedShift + currentSplitFraction;
      actualSplitFraction = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultSplitFraction);
      assertEquals(expectedForwardShiftedSplitFraction, actualSplitFraction, epsilon);
      assertTrue(actualSplitFraction > transferSplitFraction);

      // let's move the current value back a little, so that we get less total shift.
      transferSplitFraction = 0.3;
      currentSplitFraction = 0.5;
      double expectedBackwardShiftedSplitFraction = transferSplitFraction;
      actualSplitFraction = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultSplitFraction);
      assertEquals(expectedBackwardShiftedSplitFraction, actualSplitFraction, epsilon);

      currentSplitFraction = 0.3;
      ratioCurrentToDefault = (defaultSplitFraction - currentSplitFraction) / defaultSplitFraction;
      expectedShift = -currentSplitFraction * ratioCurrentToDefault;
      expectedBackwardShiftedSplitFraction = expectedShift + currentSplitFraction;
      actualSplitFraction = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultSplitFraction);
      assertEquals(expectedBackwardShiftedSplitFraction, actualSplitFraction, epsilon);
      assertTrue(actualSplitFraction < transferSplitFraction);

      // let's move the current value forward of the nominal value
      currentSplitFraction = 0.7;
      ratioCurrentToDefault = (defaultSplitFraction - currentSplitFraction) / defaultSplitFraction;
      expectedShift = -currentSplitFraction * ratioCurrentToDefault;
      expectedBackwardShiftedSplitFraction = expectedShift + currentSplitFraction;
      actualSplitFraction = SplitFractionTools.appendSplitFraction(transferSplitFraction, currentSplitFraction, defaultSplitFraction);
      assertEquals(expectedBackwardShiftedSplitFraction, actualSplitFraction, epsilon);
      assertTrue(actualSplitFraction > transferSplitFraction);

      // TODO tell it to try and do negative shifts
   }
}


