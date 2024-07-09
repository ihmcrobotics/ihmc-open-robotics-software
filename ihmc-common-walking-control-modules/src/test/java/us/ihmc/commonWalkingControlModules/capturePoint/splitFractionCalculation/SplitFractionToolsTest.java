package us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation;

import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class SplitFractionToolsTest
{
   private static final double epsilon = 1e-8;
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
