package us.ihmc.commonWalkingControlModules.capturePoint.comBasedPlanner;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.TimeInterval;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public class BipedContactSequenceUpdaterTest
{
   private static final double epsilon = 1e-8;

   private static SideDependentList<MovingReferenceFrame> createSimpleSoleFrames(double nominalWidth)
   {
      SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         TranslationMovingReferenceFrame referenceFrame = new TranslationMovingReferenceFrame("footFrame", ReferenceFrame.getWorldFrame());
         double yTranslation = robotSide.negateIfRightSide(nominalWidth / 2.0);
         Vector3D translation = new Vector3D(0.0, yTranslation, 0.0);
         referenceFrame.updateTranslation(translation);
         soleFrames.put(robotSide, referenceFrame);
      }

      return soleFrames;
   }

   @Test
   public void testNoSteps()
   {
      double nominalWidth = 0.5;

      SideDependentList<MovingReferenceFrame> soleFrames = createSimpleSoleFrames(nominalWidth);
      BipedContactSequenceUpdater contactSequenceUpdater = new BipedContactSequenceUpdater(soleFrames, null, null);

      List<RobotSide> currentFeetInContact = new ArrayList<>();
      for (RobotSide quadrant : RobotSide.values)
         currentFeetInContact.add(quadrant);

      List<BipedTimedStep> stepList = new ArrayList<>();

      contactSequenceUpdater.update(stepList, currentFeetInContact, 0.0);

      List<SimpleBipedContactPhase> contactSequence = contactSequenceUpdater.getContactSequence();

      assertEquals(1, contactSequence.size());

      // test first contact state
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(0).getContactState());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(), contactSequence.get(0).getCopStartPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(new Point3D(), contactSequence.get(0).getCopEndPosition(), epsilon);
      assertEquals(2, contactSequence.get(0).getFeetInContact().size());
      assertTrue(contactSequence.get(0).getFeetInContact().contains(RobotSide.LEFT));
      assertTrue(contactSequence.get(0).getFeetInContact().contains(RobotSide.RIGHT));
      assertEquals(0.0, contactSequence.get(0).getTimeInterval().getStartTime(), epsilon);
      assertEquals(Double.POSITIVE_INFINITY, contactSequence.get(0).getTimeInterval().getEndTime(), epsilon);
   }

   @Test
   public void testOneSimpleStep()
   {
      double stepLength = 0.2;
      double nominalWidth = 0.5;

      SideDependentList<MovingReferenceFrame> soleFrames = createSimpleSoleFrames(nominalWidth);
      BipedContactSequenceUpdater contactSequenceUpdater = new BipedContactSequenceUpdater(soleFrames, null, null);

      List<RobotSide> currentFeetInContact = new ArrayList<>();
      for (RobotSide quadrant : RobotSide.values)
         currentFeetInContact.add(quadrant);

      BipedTimedStep step = new BipedTimedStep();
      Point3D leftFootFirstStep = new Point3D(stepLength, nominalWidth / 2.0, 0.0);

      step.setGoalPose(new FramePoint3D(ReferenceFrame.getWorldFrame(), leftFootFirstStep), new FrameQuaternion());
      step.setRobotSide(RobotSide.LEFT);
      step.setTimeInterval(new TimeInterval(0.5, 1.0));

      List<BipedTimedStep> stepList = new ArrayList<>();
      stepList.add(step);

      contactSequenceUpdater.update(stepList, currentFeetInContact, 0.0);

      List<SimpleBipedContactPhase> contactSequence = contactSequenceUpdater.getContactSequence();

      assertEquals(3, contactSequence.size());

      Point3D centerOfFirstStance = new Point3D();
      Point3D rightFootFirstStance = new Point3D(0.0, -nominalWidth / 2.0, 0.0);
      Point3D finalStance = new Point3D();
      finalStance.interpolate(leftFootFirstStep, rightFootFirstStance, 0.5);
      // test first contact state, initial contact, center of feet
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(0).getContactState());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(centerOfFirstStance, contactSequence.get(0).getCopStartPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(rightFootFirstStance, contactSequence.get(0).getCopEndPosition(), epsilon);
      assertEquals(2, contactSequence.get(0).getFeetInContact().size());
      assertTrue(contactSequence.get(0).getFeetInContact().contains(RobotSide.LEFT));
      assertTrue(contactSequence.get(0).getFeetInContact().contains(RobotSide.RIGHT));
      assertEquals(0.0, contactSequence.get(0).getTimeInterval().getStartTime(), epsilon);
      assertEquals(0.5, contactSequence.get(0).getTimeInterval().getEndTime(), epsilon);

      // test second contact state, right foot stance
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(1).getContactState());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(rightFootFirstStance, contactSequence.get(1).getCopStartPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(rightFootFirstStance, contactSequence.get(1).getCopEndPosition(), epsilon);
      assertEquals(1, contactSequence.get(1).getFeetInContact().size());
      assertTrue(contactSequence.get(1).getFeetInContact().contains(RobotSide.RIGHT));
      assertEquals(0.5, contactSequence.get(1).getTimeInterval().getStartTime(), epsilon);
      assertEquals(1.0, contactSequence.get(1).getTimeInterval().getEndTime(), epsilon);

      // test final contact state
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(2).getContactState());
      assertEquals(2, contactSequence.get(2).getFeetInContact().size());
      assertTrue(contactSequence.get(2).getFeetInContact().contains(RobotSide.LEFT));
      assertTrue(contactSequence.get(2).getFeetInContact().contains(RobotSide.RIGHT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(rightFootFirstStance, contactSequence.get(2).getCopStartPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(finalStance, contactSequence.get(2).getCopEndPosition(), epsilon);
      assertEquals(1.0, contactSequence.get(2).getTimeInterval().getStartTime(), epsilon);
      assertEquals(Double.POSITIVE_INFINITY, contactSequence.get(2).getTimeInterval().getEndTime(), epsilon);



      double timeInPhase = 0.25;
      contactSequenceUpdater.update(stepList, currentFeetInContact, timeInPhase);

      assertEquals(3, contactSequence.size());

      // test first contact state
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(0).getContactState());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(centerOfFirstStance, contactSequence.get(0).getCopStartPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(rightFootFirstStance, contactSequence.get(0).getCopEndPosition(), epsilon);
      assertEquals(2, contactSequence.get(0).getFeetInContact().size());
      assertTrue(contactSequence.get(0).getFeetInContact().contains(RobotSide.LEFT));
      assertTrue(contactSequence.get(0).getFeetInContact().contains(RobotSide.RIGHT));
      assertEquals(0.0 - timeInPhase, contactSequence.get(0).getTimeInterval().getStartTime(), epsilon);
      assertEquals(0.5 - timeInPhase, contactSequence.get(0).getTimeInterval().getEndTime(), epsilon);

      // test second contact state
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(1).getContactState());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(rightFootFirstStance, contactSequence.get(1).getCopStartPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(rightFootFirstStance, contactSequence.get(1).getCopEndPosition(), epsilon);
      assertEquals(1, contactSequence.get(1).getFeetInContact().size());
      assertTrue(contactSequence.get(1).getFeetInContact().contains(RobotSide.RIGHT));
      assertEquals(0.5 - timeInPhase, contactSequence.get(1).getTimeInterval().getStartTime(), epsilon);
      assertEquals(1.0 - timeInPhase, contactSequence.get(1).getTimeInterval().getEndTime(), epsilon);

      // test final contact state
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(2).getContactState());
      assertEquals(2, contactSequence.get(2).getFeetInContact().size());
      assertTrue(contactSequence.get(2).getFeetInContact().contains(RobotSide.LEFT));
      assertTrue(contactSequence.get(2).getFeetInContact().contains(RobotSide.RIGHT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(rightFootFirstStance, contactSequence.get(2).getCopStartPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(finalStance, contactSequence.get(2).getCopEndPosition(), epsilon);
      assertEquals(1.0 - timeInPhase, contactSequence.get(2).getTimeInterval().getStartTime(), epsilon);
      assertEquals(Double.POSITIVE_INFINITY, contactSequence.get(2).getTimeInterval().getEndTime(), epsilon);
   }


   @Test
   public void testTwoSimpleSteps()
   {
      double stepLength = 0.2;
      double nominalWidth = 0.5;

      SideDependentList<MovingReferenceFrame> soleFrames = createSimpleSoleFrames(nominalWidth);
      BipedContactSequenceUpdater contactSequenceUpdater = new BipedContactSequenceUpdater(soleFrames, null, null);

      List<RobotSide> currentFeetInContact = new ArrayList<>();
      for (RobotSide quadrant : RobotSide.values)
         currentFeetInContact.add(quadrant);

      Point3D leftFootFirstStep = new Point3D(stepLength, nominalWidth / 2.0, 0.0);

      BipedTimedStep step1 = new BipedTimedStep();
      step1.setGoalPose(new FramePoint3D(ReferenceFrame.getWorldFrame(), leftFootFirstStep), new FrameQuaternion());
      step1.setRobotSide(RobotSide.LEFT);
      step1.setTimeInterval(new TimeInterval(0.5, 1.0));

      Point3D rightFootFirstStep = new Point3D(stepLength, -nominalWidth / 2.0, 0.0);

      BipedTimedStep step2 = new BipedTimedStep();
      step2.setGoalPose(new FramePoint3D(ReferenceFrame.getWorldFrame(), rightFootFirstStep), new FrameQuaternion());
      step2.setRobotSide(RobotSide.RIGHT);
      step2.setTimeInterval(new TimeInterval(1.5, 2.0));

      List<BipedTimedStep> stepList = new ArrayList<>();
      stepList.add(step1);
      stepList.add(step2);

      contactSequenceUpdater.update(stepList, currentFeetInContact, 0.0);

      List<SimpleBipedContactPhase> contactSequence = contactSequenceUpdater.getContactSequence();

      assertEquals(5, contactSequence.size());

      Point3D rightFootFirstStance = new Point3D(0.0, -nominalWidth / 2.0, 0.0);
      Point3D leftFootFirstStance = new Point3D(0.0, nominalWidth / 2.0, 0.0);
      Point3D leftFootSecondStance = new Point3D(stepLength, nominalWidth / 2.0, 0.0);
      Point3D rightFootSecondStance = new Point3D(stepLength, -nominalWidth / 2.0, 0.0);
      Point3D initialStance = new Point3D();
      Point3D finalStance = new Point3D();
      initialStance.interpolate(leftFootFirstStance, rightFootFirstStance, 0.5);
      finalStance.interpolate(leftFootSecondStance, rightFootSecondStance, 0.5);

      // test first contact state, initial contact, center of feet
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(0).getContactState());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialStance, contactSequence.get(0).getCopStartPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(rightFootFirstStance, contactSequence.get(0).getCopEndPosition(), epsilon);
      assertEquals(2, contactSequence.get(0).getFeetInContact().size());
      assertTrue(contactSequence.get(0).getFeetInContact().contains(RobotSide.LEFT));
      assertTrue(contactSequence.get(0).getFeetInContact().contains(RobotSide.RIGHT));
      assertEquals(0.0, contactSequence.get(0).getTimeInterval().getStartTime(), epsilon);
      assertEquals(0.5, contactSequence.get(0).getTimeInterval().getEndTime(), epsilon);

      // test second contact state, right foot stance
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(1).getContactState());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(rightFootFirstStance, contactSequence.get(1).getCopStartPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(rightFootFirstStance, contactSequence.get(1).getCopEndPosition(), epsilon);
      assertEquals(1, contactSequence.get(1).getFeetInContact().size());
      assertTrue(contactSequence.get(1).getFeetInContact().contains(RobotSide.RIGHT));
      assertEquals(0.5, contactSequence.get(1).getTimeInterval().getStartTime(), epsilon);
      assertEquals(1.0, contactSequence.get(1).getTimeInterval().getEndTime(), epsilon);

      // test third contact state, transfer
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(2).getContactState());
      assertEquals(2, contactSequence.get(2).getFeetInContact().size());
      assertTrue(contactSequence.get(2).getFeetInContact().contains(RobotSide.LEFT));
      assertTrue(contactSequence.get(2).getFeetInContact().contains(RobotSide.RIGHT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(rightFootFirstStance, contactSequence.get(2).getCopStartPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(leftFootSecondStance, contactSequence.get(2).getCopEndPosition(), epsilon);
      assertEquals(1.0, contactSequence.get(2).getTimeInterval().getStartTime(), epsilon);
      assertEquals(1.5, contactSequence.get(2).getTimeInterval().getEndTime(), epsilon);

      // test fourth contact state, left foot stance
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(3).getContactState());
      assertEquals(1, contactSequence.get(3).getFeetInContact().size());
      assertTrue(contactSequence.get(3).getFeetInContact().contains(RobotSide.LEFT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(leftFootSecondStance, contactSequence.get(3).getCopStartPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(leftFootSecondStance, contactSequence.get(3).getCopEndPosition(), epsilon);
      assertEquals(1.5, contactSequence.get(3).getTimeInterval().getStartTime(), epsilon);
      assertEquals(2.0, contactSequence.get(3).getTimeInterval().getEndTime(), epsilon);

      // test fourth contact state, final transfer
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(4).getContactState());
      assertEquals(2, contactSequence.get(4).getFeetInContact().size());
      assertTrue(contactSequence.get(4).getFeetInContact().contains(RobotSide.LEFT));
      assertTrue(contactSequence.get(4).getFeetInContact().contains(RobotSide.RIGHT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(leftFootSecondStance, contactSequence.get(4).getCopStartPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(finalStance, contactSequence.get(4).getCopEndPosition(), epsilon);
      assertEquals(2.0, contactSequence.get(4).getTimeInterval().getStartTime(), epsilon);
      assertEquals(Double.POSITIVE_INFINITY, contactSequence.get(4).getTimeInterval().getEndTime(), epsilon);
   }

   @Test
   public void testTwoSimpleStepsWithFlight()
   {
      double stepLength = 0.2;
      double nominalWidth = 0.5;

      SideDependentList<MovingReferenceFrame> soleFrames = createSimpleSoleFrames(nominalWidth);
      BipedContactSequenceUpdater contactSequenceUpdater = new BipedContactSequenceUpdater(soleFrames, null, null);

      List<RobotSide> currentFeetInContact = new ArrayList<>();
      for (RobotSide quadrant : RobotSide.values)
         currentFeetInContact.add(quadrant);

      Point3D leftFootFirstStep = new Point3D(stepLength, nominalWidth / 2.0, 0.0);

      BipedTimedStep step1 = new BipedTimedStep();
      step1.setGoalPose(new FramePoint3D(ReferenceFrame.getWorldFrame(), leftFootFirstStep), new FrameQuaternion());
      step1.setRobotSide(RobotSide.LEFT);
      step1.setTimeInterval(new TimeInterval(0.5, 1.0));

      Point3D rightFootFirstStep = new Point3D(stepLength, -nominalWidth / 2.0, 0.0);

      BipedTimedStep step2 = new BipedTimedStep();
      step2.setGoalPose(new FramePoint3D(ReferenceFrame.getWorldFrame(), rightFootFirstStep), new FrameQuaternion());
      step2.setRobotSide(RobotSide.RIGHT);
      step2.setTimeInterval(new TimeInterval(0.8, 1.3));

      List<BipedTimedStep> stepList = new ArrayList<>();
      stepList.add(step1);
      stepList.add(step2);

      contactSequenceUpdater.update(stepList, currentFeetInContact, 0.0);

      List<SimpleBipedContactPhase> contactSequence = contactSequenceUpdater.getContactSequence();

      assertEquals(5, contactSequence.size());

      Point3D rightFootFirstStance = new Point3D(0.0, -nominalWidth / 2.0, 0.0);
      Point3D leftFootFirstStance = new Point3D(0.0, nominalWidth / 2.0, 0.0);
      Point3D leftFootSecondStance = new Point3D(stepLength, nominalWidth / 2.0, 0.0);
      Point3D rightFootSecondStance = new Point3D(stepLength, -nominalWidth / 2.0, 0.0);
      Point3D initialStance = new Point3D();
      Point3D finalStance = new Point3D();
      initialStance.interpolate(leftFootFirstStance, rightFootFirstStance, 0.5);
      finalStance.interpolate(leftFootSecondStance, rightFootSecondStance, 0.5);

      // test first contact state, initial contact, center of feet
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(0).getContactState());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialStance, contactSequence.get(0).getCopStartPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(rightFootFirstStance, contactSequence.get(0).getCopEndPosition(), epsilon);
      assertEquals(2, contactSequence.get(0).getFeetInContact().size());
      assertTrue(contactSequence.get(0).getFeetInContact().contains(RobotSide.LEFT));
      assertTrue(contactSequence.get(0).getFeetInContact().contains(RobotSide.RIGHT));
      assertEquals(0.0, contactSequence.get(0).getTimeInterval().getStartTime(), epsilon);
      assertEquals(0.5, contactSequence.get(0).getTimeInterval().getEndTime(), epsilon);

      // test second contact state, right foot stance
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(1).getContactState());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(rightFootFirstStance, contactSequence.get(1).getCopStartPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(rightFootFirstStance, contactSequence.get(1).getCopEndPosition(), epsilon);
      assertEquals(1, contactSequence.get(1).getFeetInContact().size());
      assertTrue(contactSequence.get(1).getFeetInContact().contains(RobotSide.RIGHT));
      assertEquals(0.5, contactSequence.get(1).getTimeInterval().getStartTime(), epsilon);
      assertEquals(0.8, contactSequence.get(1).getTimeInterval().getEndTime(), epsilon);

      // test third contact state, flight
      assertEquals(ContactState.FLIGHT, contactSequence.get(2).getContactState());
      assertEquals(0, contactSequence.get(2).getFeetInContact().size());
//      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(leftFootSecondStance, contactSequence.get(2).getCopStartPosition(), epsilon);
//      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(leftFootSecondStance, contactSequence.get(2).getCopEndPosition(), epsilon);
      assertEquals(0.8, contactSequence.get(2).getTimeInterval().getStartTime(), epsilon);
      assertEquals(1.0, contactSequence.get(2).getTimeInterval().getEndTime(), epsilon);

      // test fourth contact state, left foot stance
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(3).getContactState());
      assertEquals(1, contactSequence.get(3).getFeetInContact().size());
      assertTrue(contactSequence.get(3).getFeetInContact().contains(RobotSide.LEFT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(leftFootSecondStance, contactSequence.get(3).getCopStartPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(leftFootSecondStance, contactSequence.get(3).getCopEndPosition(), epsilon);
      assertEquals(1.0, contactSequence.get(3).getTimeInterval().getStartTime(), epsilon);
      assertEquals(1.3, contactSequence.get(3).getTimeInterval().getEndTime(), epsilon);

      // test fourth contact state, final transfer
      assertEquals(ContactState.IN_CONTACT, contactSequence.get(4).getContactState());
      assertEquals(2, contactSequence.get(4).getFeetInContact().size());
      assertTrue(contactSequence.get(4).getFeetInContact().contains(RobotSide.LEFT));
      assertTrue(contactSequence.get(4).getFeetInContact().contains(RobotSide.RIGHT));
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(leftFootSecondStance, contactSequence.get(4).getCopStartPosition(), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(finalStance, contactSequence.get(4).getCopEndPosition(), epsilon);
      assertEquals(1.3, contactSequence.get(4).getTimeInterval().getStartTime(), epsilon);
      assertEquals(Double.POSITIVE_INFINITY, contactSequence.get(4).getTimeInterval().getEndTime(), epsilon);
   }
}
