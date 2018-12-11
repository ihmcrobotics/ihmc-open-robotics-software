package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.quadrupedBasics.gait.TimeInterval;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.List;

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

   public static void assertQuadrupedStepTransitionsEqual(QuadrupedStepTransition expected, QuadrupedStepTransition actual, double epsilon)
   {
      assertEquals(expected.getTransitionTime(), actual.getTransitionTime(), epsilon);
      assertEquals(expected.getNumberOfFeetInTransition(), actual.getNumberOfFeetInTransition());

      for (int i = 0; i < expected.getNumberOfFeetInTransition(); i++)
      {
         RobotQuadrant quadrant = expected.getTransitionQuadrant(i);
         assertTrue(actual.getTransitionQuadrants().contains(quadrant));
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expected.getTransitionPosition(quadrant), actual.getTransitionPosition(quadrant), epsilon);
         assertEquals(expected.getTransitionType(i), actual.getTransitionType(i));
      }
   }

   public static void assertQuadrupedStepTransitionsListEqual(List<QuadrupedStepTransition> expected, List<QuadrupedStepTransition> actual, double epsilon)
   {
      assertEquals(expected.size(), actual.size());

      for (int i = 0; i < expected.size(); i++)
         assertQuadrupedStepTransitionsEqual(expected.get(i), actual.get(i), epsilon);
   }


   public static void assertTimeIntervalsEqual(TimeInterval expected, TimeInterval actual, double epsilon)
   {
      assertEquals(expected.getStartTime(), actual.getStartTime(), epsilon);
      assertEquals(expected.getEndTime(), actual.getEndTime(), epsilon);
   }
}
