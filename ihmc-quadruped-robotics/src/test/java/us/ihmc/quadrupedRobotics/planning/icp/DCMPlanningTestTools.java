package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.quadrupedBasics.gait.TimeInterval;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import static junit.framework.TestCase.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class DCMPlanningTestTools
{
   public static void assertQuadrupedContactPhasesEqual(QuadrupedContactPhase expected, QuadrupedContactPhase actual, double epsilon)
   {
      assertTimeIntervalsEqual(expected.getTimeInterval(), actual.getTimeInterval(), epsilon);

      assertEquals(expected.getFeetInContact().size(), actual.getFeetInContact().size(), epsilon);
      for (RobotQuadrant robotQuadrant : expected.getFeetInContact())
      {
         assertTrue(actual.getFeetInContact().contains(robotQuadrant));
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expected.getSolePosition(robotQuadrant), actual.getSolePosition(robotQuadrant), epsilon);
      }
   }

   public static void assertTimeIntervalsEqual(TimeInterval expected, TimeInterval actual, double epsilon)
   {
      assertEquals(expected.getStartTime(), actual.getStartTime(), epsilon);
      assertEquals(expected.getEndTime(), actual.getEndTime(), epsilon);
   }
}
