package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeInterval;

import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

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

   static QuadrantDependentList<MovingReferenceFrame> createSimpleSoleFrames(double nominalLength, double nominalWidth)
   {
      QuadrantDependentList<MovingReferenceFrame> soleFrames = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         TranslationMovingReferenceFrame referenceFrame = new TranslationMovingReferenceFrame("footFrame", ReferenceFrame.getWorldFrame());
         double xTranslation = robotQuadrant.getEnd().negateIfHindEnd(nominalLength / 2.0);
         double yTranslation = robotQuadrant.getSide().negateIfRightSide(nominalWidth / 2.0);
         Vector3D translation = new Vector3D(xTranslation, yTranslation, 0.0);
         referenceFrame.updateTranslation(translation);
         soleFrames.put(robotQuadrant, referenceFrame);
      }

      return soleFrames;
   }
}
