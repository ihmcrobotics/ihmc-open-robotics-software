package us.ihmc.quadrupedRobotics.planning.icp;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static us.ihmc.robotics.Assert.assertEquals;

public class QuadrupedContactSequenceUpdaterTest
{
   private static final double epsilon = 1e-8;

   @Test
   public void testNoSteps()
   {
      double nominalLength = 1.0;
      double nominalWidth = 0.5;

      QuadrantDependentList<MovingReferenceFrame> soleFrames = DCMPlanningTestTools.createSimpleSoleFrames(nominalLength, nominalWidth);
      QuadrupedContactSequenceUpdater contactSequenceUpdater = new QuadrupedContactSequenceUpdater(soleFrames, 4, 20, new YoVariableRegistry("registry"), null);

      List<RobotQuadrant> currentFeetInContact = new ArrayList<>();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         currentFeetInContact.add(quadrant);

      List<QuadrupedTimedStep> stepList = new ArrayList<>();

      contactSequenceUpdater.update(stepList, currentFeetInContact, 0.0);

      List<QuadrupedContactPhase> contactSequence = contactSequenceUpdater.getContactSequence();

      assertEquals(1, contactSequence.size());

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
      assertEquals(Double.POSITIVE_INFINITY, contactSequence.get(0).getTimeInterval().getEndTime(), epsilon);
   }

   @Test
   public void testOneSimpleStep()
   {
      double stepLength = 0.2;
      double nominalLength = 1.0;
      double nominalWidth = 0.5;

      QuadrantDependentList<MovingReferenceFrame> soleFrames = DCMPlanningTestTools.createSimpleSoleFrames(nominalLength, nominalWidth);
      QuadrupedContactSequenceUpdater contactSequenceUpdater = new QuadrupedContactSequenceUpdater(soleFrames, 4, 20, new YoVariableRegistry("registry"), null);

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
      assertEquals(Double.POSITIVE_INFINITY, contactSequence.get(2).getTimeInterval().getEndTime(), epsilon);

      double timeInPhase = 0.25;
      contactSequenceUpdater.update(stepList, currentFeetInContact, timeInPhase);

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
      assertEquals(Double.POSITIVE_INFINITY, contactSequence.get(2).getTimeInterval().getEndTime(), epsilon);

   }

   @Test
   public void testFourStepsInFutureUsingCrawl()
   {
      double stepLength = 0.2;
      double nominalLength = 1.0;
      double nominalWidth = 0.5;

      double currentTime = 13.7;

      QuadrantDependentList<MovingReferenceFrame> soleFrames = DCMPlanningTestTools.createSimpleSoleFrames(nominalLength, nominalWidth);
      QuadrupedContactSequenceUpdater contactSequenceUpdater = new QuadrupedContactSequenceUpdater(soleFrames, 4, 20, new YoVariableRegistry("registry"), null);

      List<RobotQuadrant> currentFeetInContact = new ArrayList<>();
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         currentFeetInContact.add(quadrant);

      QuadrupedTimedStep step1 = new QuadrupedTimedStep();
      step1.setGoalPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), nominalLength / 2.0 + stepLength, nominalWidth / 2.0, 0.0));
      step1.setRobotQuadrant(RobotQuadrant.FRONT_LEFT);
      step1.setTimeInterval(new TimeInterval(currentTime + 0.5, currentTime + 1.0));

      QuadrupedTimedStep step2 = new QuadrupedTimedStep();
      step2.setGoalPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), -nominalLength / 2.0 + 1.25 * stepLength, -nominalWidth / 2.0, 0.0));
      step2.setRobotQuadrant(RobotQuadrant.HIND_RIGHT);
      step2.setTimeInterval(new TimeInterval(currentTime + 0.75, currentTime + 1.25));

      QuadrupedTimedStep step3 = new QuadrupedTimedStep();
      step3.setGoalPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), nominalLength / 2.0 + 1.5 * stepLength, -nominalWidth / 2.0, 0.0));
      step3.setRobotQuadrant(RobotQuadrant.FRONT_RIGHT);
      step3.setTimeInterval(new TimeInterval(currentTime + 1.1, currentTime + 1.6));

      QuadrupedTimedStep step4 = new QuadrupedTimedStep();
      step4.setGoalPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), -nominalLength / 2.0 + 1.75 * stepLength, nominalWidth / 2.0, 0.0));
      step4.setRobotQuadrant(RobotQuadrant.HIND_LEFT);
      step4.setTimeInterval(new TimeInterval(currentTime + 1.35, currentTime + 1.85));

      List<QuadrupedTimedStep> stepList = new ArrayList<>();
      stepList.add(step1);
      stepList.add(step2);
      stepList.add(step3);
      stepList.add(step4);

      contactSequenceUpdater.update(stepList, currentFeetInContact, currentTime);

      List<QuadrupedContactPhase> contactSequence = contactSequenceUpdater.getContactSequence();

      assertEquals(9, contactSequence.size());

      // test first contact state
      QuadrupedContactPhase firstContactPhase = contactSequence.get(0);
      assertEquals(ContactState.IN_CONTACT, firstContactPhase.getContactState());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           firstContactPhase.getSolePosition(RobotQuadrant.FRONT_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           firstContactPhase.getSolePosition(RobotQuadrant.FRONT_RIGHT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           firstContactPhase.getSolePosition(RobotQuadrant.HIND_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           firstContactPhase.getSolePosition(RobotQuadrant.HIND_RIGHT), epsilon);
      assertEquals(4, firstContactPhase.getFeetInContact().size());
      assertTrue(firstContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_LEFT));
      assertTrue(firstContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_RIGHT));
      assertTrue(firstContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_LEFT));
      assertTrue(firstContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_RIGHT));
      assertEquals(currentTime, firstContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(currentTime + 0.5, firstContactPhase.getTimeInterval().getEndTime(), epsilon);

      // test second contact state
      QuadrupedContactPhase secondContactPhase = contactSequence.get(1);
      assertEquals(ContactState.IN_CONTACT, secondContactPhase.getContactState());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           secondContactPhase.getSolePosition(RobotQuadrant.FRONT_RIGHT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           secondContactPhase.getSolePosition(RobotQuadrant.HIND_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           secondContactPhase.getSolePosition(RobotQuadrant.HIND_RIGHT), epsilon);
      assertEquals(3, secondContactPhase.getFeetInContact().size());
      assertTrue(secondContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_RIGHT));
      assertTrue(secondContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_LEFT));
      assertTrue(secondContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_RIGHT));
      assertEquals(currentTime + 0.5, secondContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(currentTime + 0.75, secondContactPhase.getTimeInterval().getEndTime(), epsilon);

      QuadrupedContactPhase thirdContactPhase = contactSequence.get(2);
      assertEquals(ContactState.IN_CONTACT, thirdContactPhase.getContactState());
      assertEquals(2, thirdContactPhase.getFeetInContact().size());
      assertTrue(thirdContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_RIGHT));
      assertTrue(thirdContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_LEFT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           thirdContactPhase.getSolePosition(RobotQuadrant.FRONT_RIGHT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           thirdContactPhase.getSolePosition(RobotQuadrant.HIND_LEFT), epsilon);
      assertEquals(currentTime + 0.75, thirdContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(currentTime + 1.0, thirdContactPhase.getTimeInterval().getEndTime(), epsilon);

      QuadrupedContactPhase fourthContactPhase = contactSequence.get(3);
      assertEquals(ContactState.IN_CONTACT, fourthContactPhase.getContactState());
      assertEquals(3, fourthContactPhase.getFeetInContact().size());
      assertTrue(fourthContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_RIGHT));
      assertTrue(fourthContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_LEFT));
      assertTrue(fourthContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_LEFT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0 + stepLength, nominalWidth / 2.0, 0.0),
                                                           fourthContactPhase.getSolePosition(RobotQuadrant.FRONT_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           fourthContactPhase.getSolePosition(RobotQuadrant.FRONT_RIGHT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           fourthContactPhase.getSolePosition(RobotQuadrant.HIND_LEFT), epsilon);
      assertEquals(currentTime + 1.0, fourthContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(currentTime + 1.1, fourthContactPhase.getTimeInterval().getEndTime(), epsilon);

      QuadrupedContactPhase fifthContactPhase = contactSequence.get(4);
      assertEquals(ContactState.IN_CONTACT, fifthContactPhase.getContactState());
      assertEquals(2, fifthContactPhase.getFeetInContact().size());
      assertTrue(fifthContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_LEFT));
      assertTrue(fifthContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_LEFT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0 + stepLength, nominalWidth / 2.0, 0.0),
                                                           fifthContactPhase.getSolePosition(RobotQuadrant.FRONT_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           fifthContactPhase.getSolePosition(RobotQuadrant.HIND_LEFT), epsilon);

      assertEquals(currentTime + 1.1, fifthContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(currentTime + 1.25, fifthContactPhase.getTimeInterval().getEndTime(), epsilon);

      QuadrupedContactPhase sixthContactPhase = contactSequence.get(5);
      assertEquals(ContactState.IN_CONTACT, sixthContactPhase.getContactState());
      assertEquals(3, sixthContactPhase.getFeetInContact().size());
      assertTrue(sixthContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_LEFT));
      assertTrue(sixthContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_RIGHT));
      assertTrue(sixthContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_LEFT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0 + stepLength, nominalWidth / 2.0, 0.0),
                                                           sixthContactPhase.getSolePosition(RobotQuadrant.FRONT_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           sixthContactPhase.getSolePosition(RobotQuadrant.HIND_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0 + 1.25 * stepLength, -nominalWidth / 2.0, 0.0),
                                                           sixthContactPhase.getSolePosition(RobotQuadrant.HIND_RIGHT), epsilon);
      assertEquals(currentTime + 1.25, sixthContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(currentTime + 1.35, sixthContactPhase.getTimeInterval().getEndTime(), epsilon);

      QuadrupedContactPhase seventhContactPhase = contactSequence.get(6);
      assertEquals(ContactState.IN_CONTACT, seventhContactPhase.getContactState());
      assertEquals(2, seventhContactPhase.getFeetInContact().size());
      assertTrue(seventhContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_LEFT));
      assertTrue(seventhContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_RIGHT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0 + stepLength, nominalWidth / 2.0, 0.0),
                                                           seventhContactPhase.getSolePosition(RobotQuadrant.FRONT_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0 + 1.25 * stepLength, -nominalWidth / 2.0, 0.0),
                                                           seventhContactPhase.getSolePosition(RobotQuadrant.HIND_RIGHT), epsilon);
      assertEquals(currentTime +1.35, seventhContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(currentTime + 1.6, seventhContactPhase.getTimeInterval().getEndTime(), epsilon);

      QuadrupedContactPhase eighthContactPhase = contactSequence.get(7);
      assertEquals(ContactState.IN_CONTACT, eighthContactPhase.getContactState());
      assertEquals(3, eighthContactPhase.getFeetInContact().size());
      assertTrue(eighthContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_LEFT));
      assertTrue(eighthContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_RIGHT));
      assertTrue(eighthContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_RIGHT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0 + stepLength, nominalWidth / 2.0, 0.0),
                                                           eighthContactPhase.getSolePosition(RobotQuadrant.FRONT_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0 + 1.5 * stepLength, -nominalWidth / 2.0, 0.0),
                                                           eighthContactPhase.getSolePosition(RobotQuadrant.FRONT_RIGHT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0 + 1.25 * stepLength, -nominalWidth / 2.0, 0.0),
                                                           eighthContactPhase.getSolePosition(RobotQuadrant.HIND_RIGHT), epsilon);
      assertEquals(currentTime + 1.6, eighthContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(currentTime + 1.85, eighthContactPhase.getTimeInterval().getEndTime(), epsilon);

      QuadrupedContactPhase ninthContactPhase = contactSequence.get(8);
      assertEquals(ContactState.IN_CONTACT, ninthContactPhase.getContactState());
      assertEquals(4, ninthContactPhase.getFeetInContact().size());
      assertTrue(ninthContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_LEFT));
      assertTrue(ninthContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_RIGHT));
      assertTrue(ninthContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_LEFT));
      assertTrue(ninthContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_RIGHT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0 + stepLength, nominalWidth / 2.0, 0.0),
                                                           ninthContactPhase.getSolePosition(RobotQuadrant.FRONT_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0 + 1.5 * stepLength, -nominalWidth / 2.0, 0.0),
                                                           ninthContactPhase.getSolePosition(RobotQuadrant.FRONT_RIGHT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0 + 1.25 * stepLength, -nominalWidth / 2.0, 0.0),
                                                           ninthContactPhase.getSolePosition(RobotQuadrant.HIND_RIGHT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0 + 1.75 * stepLength, nominalWidth / 2.0, 0.0),
                                                           ninthContactPhase.getSolePosition(RobotQuadrant.HIND_LEFT), epsilon);
      assertEquals(currentTime + 1.85, ninthContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(Double.POSITIVE_INFINITY, ninthContactPhase.getTimeInterval().getEndTime(), epsilon);

      double timeInPhase = 0.25;
      contactSequenceUpdater.update(stepList, currentFeetInContact, currentTime + timeInPhase);

      firstContactPhase = contactSequence.get(0);
      assertEquals(ContactState.IN_CONTACT, firstContactPhase.getContactState());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           firstContactPhase.getSolePosition(RobotQuadrant.FRONT_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           firstContactPhase.getSolePosition(RobotQuadrant.FRONT_RIGHT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           firstContactPhase.getSolePosition(RobotQuadrant.HIND_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           firstContactPhase.getSolePosition(RobotQuadrant.HIND_RIGHT), epsilon);
      assertEquals(4, firstContactPhase.getFeetInContact().size());
      assertTrue(firstContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_LEFT));
      assertTrue(firstContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_RIGHT));
      assertTrue(firstContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_LEFT));
      assertTrue(firstContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_RIGHT));
      assertEquals(currentTime, firstContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(currentTime + 0.5, firstContactPhase.getTimeInterval().getEndTime(), epsilon);

      // test second contact state
      secondContactPhase = contactSequence.get(1);
      assertEquals(ContactState.IN_CONTACT, secondContactPhase.getContactState());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           secondContactPhase.getSolePosition(RobotQuadrant.FRONT_RIGHT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           secondContactPhase.getSolePosition(RobotQuadrant.HIND_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           secondContactPhase.getSolePosition(RobotQuadrant.HIND_RIGHT), epsilon);
      assertEquals(3, secondContactPhase.getFeetInContact().size());
      assertTrue(secondContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_RIGHT));
      assertTrue(secondContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_LEFT));
      assertTrue(secondContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_RIGHT));
      assertEquals(currentTime + 0.5, secondContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(currentTime + 0.75, secondContactPhase.getTimeInterval().getEndTime(), epsilon);

      thirdContactPhase = contactSequence.get(2);
      assertEquals(ContactState.IN_CONTACT, thirdContactPhase.getContactState());
      assertEquals(2, thirdContactPhase.getFeetInContact().size());
      assertTrue(thirdContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_RIGHT));
      assertTrue(thirdContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_LEFT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           thirdContactPhase.getSolePosition(RobotQuadrant.FRONT_RIGHT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           thirdContactPhase.getSolePosition(RobotQuadrant.HIND_LEFT), epsilon);
      assertEquals(currentTime + 0.75, thirdContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(currentTime + 1.0, thirdContactPhase.getTimeInterval().getEndTime(), epsilon);

      fourthContactPhase = contactSequence.get(3);
      assertEquals(ContactState.IN_CONTACT, fourthContactPhase.getContactState());
      assertEquals(3, fourthContactPhase.getFeetInContact().size());
      assertTrue(fourthContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_RIGHT));
      assertTrue(fourthContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_LEFT));
      assertTrue(fourthContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_LEFT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0 + stepLength, nominalWidth / 2.0, 0.0),
                                                           fourthContactPhase.getSolePosition(RobotQuadrant.FRONT_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0, -nominalWidth / 2.0, 0.0),
                                                           fourthContactPhase.getSolePosition(RobotQuadrant.FRONT_RIGHT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           fourthContactPhase.getSolePosition(RobotQuadrant.HIND_LEFT), epsilon);
      assertEquals(currentTime + 1.0, fourthContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(currentTime + 1.1, fourthContactPhase.getTimeInterval().getEndTime(), epsilon);

      fifthContactPhase = contactSequence.get(4);
      assertEquals(ContactState.IN_CONTACT, fifthContactPhase.getContactState());
      assertEquals(2, fifthContactPhase.getFeetInContact().size());
      assertTrue(fifthContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_LEFT));
      assertTrue(fifthContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_LEFT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0 + stepLength, nominalWidth / 2.0, 0.0),
                                                           fifthContactPhase.getSolePosition(RobotQuadrant.FRONT_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           fifthContactPhase.getSolePosition(RobotQuadrant.HIND_LEFT), epsilon);

      assertEquals(currentTime + 1.1, fifthContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(currentTime + 1.25, fifthContactPhase.getTimeInterval().getEndTime(), epsilon);

      sixthContactPhase = contactSequence.get(5);
      assertEquals(ContactState.IN_CONTACT, sixthContactPhase.getContactState());
      assertEquals(3, sixthContactPhase.getFeetInContact().size());
      assertTrue(sixthContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_LEFT));
      assertTrue(sixthContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_RIGHT));
      assertTrue(sixthContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_LEFT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0 + stepLength, nominalWidth / 2.0, 0.0),
                                                           sixthContactPhase.getSolePosition(RobotQuadrant.FRONT_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0, nominalWidth / 2.0, 0.0),
                                                           sixthContactPhase.getSolePosition(RobotQuadrant.HIND_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0 + 1.25 * stepLength, -nominalWidth / 2.0, 0.0),
                                                           sixthContactPhase.getSolePosition(RobotQuadrant.HIND_RIGHT), epsilon);
      assertEquals(currentTime + 1.25, sixthContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(currentTime + 1.35, sixthContactPhase.getTimeInterval().getEndTime(), epsilon);

      seventhContactPhase = contactSequence.get(6);
      assertEquals(ContactState.IN_CONTACT, seventhContactPhase.getContactState());
      assertEquals(2, seventhContactPhase.getFeetInContact().size());
      assertTrue(seventhContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_LEFT));
      assertTrue(seventhContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_RIGHT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0 + stepLength, nominalWidth / 2.0, 0.0),
                                                           seventhContactPhase.getSolePosition(RobotQuadrant.FRONT_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0 + 1.25 * stepLength, -nominalWidth / 2.0, 0.0),
                                                           seventhContactPhase.getSolePosition(RobotQuadrant.HIND_RIGHT), epsilon);
      assertEquals(currentTime + 1.35, seventhContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(currentTime + 1.6, seventhContactPhase.getTimeInterval().getEndTime(), epsilon);

      eighthContactPhase = contactSequence.get(7);
      assertEquals(ContactState.IN_CONTACT, eighthContactPhase.getContactState());
      assertEquals(3, eighthContactPhase.getFeetInContact().size());
      assertTrue(eighthContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_LEFT));
      assertTrue(eighthContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_RIGHT));
      assertTrue(eighthContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_RIGHT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0 + stepLength, nominalWidth / 2.0, 0.0),
                                                           eighthContactPhase.getSolePosition(RobotQuadrant.FRONT_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0 + 1.5 * stepLength, -nominalWidth / 2.0, 0.0),
                                                           eighthContactPhase.getSolePosition(RobotQuadrant.FRONT_RIGHT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0 + 1.25 * stepLength, -nominalWidth / 2.0, 0.0),
                                                           eighthContactPhase.getSolePosition(RobotQuadrant.HIND_RIGHT), epsilon);
      assertEquals(currentTime + 1.6, eighthContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(currentTime + 1.85, eighthContactPhase.getTimeInterval().getEndTime(), epsilon);

      ninthContactPhase = contactSequence.get(8);
      assertEquals(ContactState.IN_CONTACT, ninthContactPhase.getContactState());
      assertEquals(4, ninthContactPhase.getFeetInContact().size());
      assertTrue(ninthContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_LEFT));
      assertTrue(ninthContactPhase.getFeetInContact().contains(RobotQuadrant.FRONT_RIGHT));
      assertTrue(ninthContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_LEFT));
      assertTrue(ninthContactPhase.getFeetInContact().contains(RobotQuadrant.HIND_RIGHT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0 + stepLength, nominalWidth / 2.0, 0.0),
                                                           ninthContactPhase.getSolePosition(RobotQuadrant.FRONT_LEFT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(nominalLength / 2.0 + 1.5 * stepLength, -nominalWidth / 2.0, 0.0),
                                                           ninthContactPhase.getSolePosition(RobotQuadrant.FRONT_RIGHT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0 + 1.25 * stepLength, -nominalWidth / 2.0, 0.0),
                                                           ninthContactPhase.getSolePosition(RobotQuadrant.HIND_RIGHT), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(-nominalLength / 2.0 + 1.75 * stepLength, nominalWidth / 2.0, 0.0),
                                                           ninthContactPhase.getSolePosition(RobotQuadrant.HIND_LEFT), epsilon);
      assertEquals(currentTime + 1.85, ninthContactPhase.getTimeInterval().getStartTime(), epsilon);
      assertEquals(Double.POSITIVE_INFINITY, ninthContactPhase.getTimeInterval().getEndTime(), epsilon);
   }

}
