package us.ihmc.footstepPlanning.icp;

import org.junit.jupiter.api.Test;

import static us.ihmc.robotics.Assert.assertEquals;

public class SplitFractionToolsTest
{
   @Test
   public void testTwoShifts()
   {
      double nominalSplitFraction = 0.5;
      double currentSplitFraction = 0.5;
      double positionShiftForward = 0.75;
      double areaShiftBackward = 0.25;
      double areaShiftForward = 0.75;

      double positionShifted = SplitFractionTools.appendSplitFraction(positionShiftForward, currentSplitFraction, nominalSplitFraction);
      assertEquals(positionShiftForward, positionShifted, 1e-5);

      double shiftedBackward = SplitFractionTools.appendSplitFraction(areaShiftBackward, positionShifted, nominalSplitFraction);
      double shiftedForward = SplitFractionTools.appendSplitFraction(areaShiftForward, positionShifted, nominalSplitFraction);

      assertEquals(0.375, shiftedBackward, 1e-5);
      assertEquals(0.875, shiftedForward, 1e-5);
   }
}
