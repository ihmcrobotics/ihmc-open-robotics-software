package us.ihmc.quadrupedRobotics.planning.icp;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.gait.TimeInterval;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;
import java.util.List;

import static junit.framework.TestCase.assertTrue;
import static org.junit.jupiter.api.Assertions.assertEquals;

public class QuadrupedContactSequenceUpdaterTest
{
   private static final double epsilon = 1e-8;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOneSimpleStep()
   {
      double stepLength = 0.2;
      double nominalLength = 1.0;
      double nominalWidth = 0.5;

      QuadrantDependentList<ReferenceFrame> soleFrames = createSimpleSoleFrames(nominalLength, nominalWidth);
      QuadrupedContactSequenceUpdater contactSequenceUpdater = new QuadrupedContactSequenceUpdater(soleFrames, 4, 20);

      List<RobotQuadrant> currentFeetInContact = new ArrayList<>();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         currentFeetInContact.add(quadrant);

      QuadrupedTimedStep step = new QuadrupedTimedStep();
      step.setGoalPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), nominalLength / 2.0 + stepLength, nominalWidth / 2.0, 0.0));
      step.setRobotQuadrant(RobotQuadrant.FRONT_LEFT);
      step.setTimeInterval(new TimeInterval(0.5, 1.0));

      List<QuadrupedTimedStep> stepList = new ArrayList<>();
      stepList.add(step);

      contactSequenceUpdater.update(stepList, currentFeetInContact, 0.0);

      List<QuadrupedContactPhase> contactSequence = contactSequenceUpdater.getContactSequence();

      assertEquals(3, contactSequence.size());

      // test first contact state
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(0).getContactState());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           contactSequence.get(0).getSolePosition(RobotQuadrant.FRONT_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           contactSequence.get(0).getSolePosition(RobotQuadrant.FRONT_RIGHT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           contactSequence.get(0).getSolePosition(RobotQuadrant.HIND_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           contactSequence.get(0).getSolePosition(RobotQuadrant.HIND_RIGHT), epsilon);
      assertEquals(4, contactSequence.get(0).getFeetInContact().size());
      assertTrue(contactSequence.get(0).getFeetInContact().contains(RobotQuadrant.FRONT_LEFT));
      assertTrue(contactSequence.get(0).getFeetInContact().contains(RobotQuadrant.FRONT_RIGHT));
      assertTrue(contactSequence.get(0).getFeetInContact().contains(RobotQuadrant.HIND_LEFT));
      assertTrue(contactSequence.get(0).getFeetInContact().contains(RobotQuadrant.HIND_RIGHT));
      assertEquals(0.0, contactSequence.get(0).getTimeInterval().getStartTime(), epsilon);
      assertEquals(0.5, contactSequence.get(0).getTimeInterval().getEndTime(), epsilon);

      // test second contact state
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(1).getContactState());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           contactSequence.get(1).getSolePosition(RobotQuadrant.FRONT_RIGHT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           contactSequence.get(1).getSolePosition(RobotQuadrant.HIND_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           contactSequence.get(1).getSolePosition(RobotQuadrant.HIND_RIGHT), epsilon);
      assertEquals(3, contactSequence.get(1).getFeetInContact().size());
      assertTrue(contactSequence.get(1).getFeetInContact().contains(RobotQuadrant.FRONT_RIGHT));
      assertTrue(contactSequence.get(1).getFeetInContact().contains(RobotQuadrant.HIND_LEFT));
      assertTrue(contactSequence.get(1).getFeetInContact().contains(RobotQuadrant.HIND_RIGHT));
      assertEquals(0.5, contactSequence.get(1).getTimeInterval().getStartTime(), epsilon);
      assertEquals(1.0, contactSequence.get(1).getTimeInterval().getEndTime(), epsilon);

      // test final contact state
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(2).getContactState());
      assertEquals(4, contactSequence.get(2).getFeetInContact().size());
      assertTrue(contactSequence.get(2).getFeetInContact().contains(RobotQuadrant.FRONT_LEFT));
      assertTrue(contactSequence.get(2).getFeetInContact().contains(RobotQuadrant.FRONT_RIGHT));
      assertTrue(contactSequence.get(2).getFeetInContact().contains(RobotQuadrant.HIND_LEFT));
      assertTrue(contactSequence.get(2).getFeetInContact().contains(RobotQuadrant.HIND_RIGHT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0 + stepLength, nominalWidth / 2.0, 0.0),
                                                           contactSequence.get(2).getSolePosition(RobotQuadrant.FRONT_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           contactSequence.get(2).getSolePosition(RobotQuadrant.FRONT_RIGHT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           contactSequence.get(2).getSolePosition(RobotQuadrant.HIND_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           contactSequence.get(2).getSolePosition(RobotQuadrant.HIND_RIGHT), epsilon);
      assertEquals(1.0, contactSequence.get(2).getTimeInterval().getStartTime(), epsilon);
      assertEquals(1.0 + QuadrupedContactSequenceUpdater.finalTransferDuration, contactSequence.get(2).getTimeInterval().getEndTime(), epsilon);
   }

   private static QuadrantDependentList<ReferenceFrame> createSimpleSoleFrames(double nominalLength, double nominalWidth)
   {
      QuadrantDependentList<ReferenceFrame> soleFrames = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         TranslationReferenceFrame referenceFrame = new TranslationReferenceFrame("footFrame", ReferenceFrame.getWorldFrame());
         double xTranslation = robotQuadrant.getEnd().negateIfHindEnd(nominalLength / 2.0);
         double yTranslation = robotQuadrant.getSide().negateIfRightSide(nominalWidth / 2.0);
         Vector3D translation = new Vector3D(xTranslation, yTranslation, 0.0);
         referenceFrame.updateTranslation(translation);
         soleFrames.put(robotQuadrant, referenceFrame);
      }

      return soleFrames;
   }
}
