package us.ihmc.quadrupedRobotics.planning.icp;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeInterval;

import java.util.*;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public class QuadrupedContactSequenceToolsTest
{
   private static final double epsilon = 1e-8;

   @Test
   public void testTrimPastContactSequences()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 10; iter++)
      {
         double currentTime = RandomNumbers.nextDouble(random, 5.0, 10.0);
         double nominalLength = RandomNumbers.nextDouble(random, 0.5, 2.0);
         double nominalWidth = RandomNumbers.nextDouble(random, 0.15, 1.0);

         int phasesToKeep = 4;

         RecyclingArrayList<QuadrupedContactPhase> contactSequence = getRandomContactSequence(random, currentTime, nominalWidth, nominalLength);
         RecyclingArrayList<QuadrupedContactPhase> contactSequenceExpected = new RecyclingArrayList<>(QuadrupedContactPhase::new);

         for (int i = 0; i < contactSequence.size(); i++)
         {
            contactSequenceExpected.add().set(contactSequence.get(i));
         }

         List<RobotQuadrant> feetInContact = getRandomFeetInContact(random);
         QuadrantDependentList<FramePoint3D> solePositions = getRandomSolePositions(random, nominalWidth, nominalLength);

         QuadrupedContactSequenceTools.setDebug(true);
         QuadrupedContactSequenceTools.trimPastContactSequences(contactSequence, currentTime, feetInContact, solePositions);

         int sequenceIndex = 0;
         while (sequenceIndex < contactSequenceExpected.size())
         {
            if (contactSequenceExpected.get(sequenceIndex).getTimeInterval().getStartTime() >= currentTime ||
                  contactSequenceExpected.get(sequenceIndex).getTimeInterval().getEndTime() < currentTime)
               contactSequenceExpected.remove(sequenceIndex);
            else
               sequenceIndex++;
         }

         // make sure all the start times are ascending
         double startTime = Double.NEGATIVE_INFINITY;
         for (QuadrupedContactPhase contactPhase : contactSequenceExpected)
         {
            double nextStartTime = contactPhase.getTimeInterval().getStartTime();
            assertTrue(nextStartTime > startTime);
            startTime = nextStartTime;
         }

         while (contactSequenceExpected.size() > phasesToKeep + 1)
            contactSequenceExpected.remove(0);

         if (contactSequenceExpected.isEmpty())
         {
            QuadrupedContactPhase contactPhase = contactSequenceExpected.add();
            contactPhase.getTimeInterval().setInterval(currentTime, currentTime);
            contactPhase.setFeetInContact(feetInContact);
            contactPhase.setSolePositions(solePositions);

            contactPhase.update();
         }
         else
         {
            QuadrupedContactPhase lastContactPhase = contactSequenceExpected.getLast();
            if (QuadrupedContactSequenceTools.isEqualContactState(lastContactPhase.getFeetInContact(), feetInContact))
            {
               // extend current contact phase
               QuadrupedContactPhase contactPhase = lastContactPhase;
               contactPhase.setSolePositions(solePositions);
            }
            else
            {
               // end previous contact phase
               lastContactPhase.getTimeInterval().setEndTime(currentTime);
               contactSequenceExpected.remove(0);

               QuadrupedContactPhase contactPhase = contactSequenceExpected.add();
               contactPhase.getTimeInterval().setInterval(currentTime, currentTime);
               contactPhase.setFeetInContact(feetInContact);
               contactPhase.setSolePositions(solePositions);

               contactPhase.update();
            }
         }

         assertEquals("iter " + iter + " failed.", contactSequenceExpected.size(), contactSequence.size());
         for (int i = 0; i < contactSequenceExpected.size(); i++)
         {
            DCMPlanningTestTools.assertQuadrupedContactPhasesEqual("iter " + iter + " failed.", contactSequenceExpected.get(i), contactSequence.get(i), epsilon);
         }
      }
   }

   @Test
   public void testComputeStepTransitionsFromStepSequence()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1; iter++)
      {
         double currentTime = RandomNumbers.nextDouble(random, 5.0, 10.0);
         double nominalLength = RandomNumbers.nextDouble(random, 0.5, 2.0);
         double nominalWidth = RandomNumbers.nextDouble(random, 0.15, 1.0);

         RecyclingArrayList<QuadrupedStepTransition> stepTransitions = new RecyclingArrayList<>(QuadrupedStepTransition::new);

         RecyclingArrayList<QuadrupedTimedStep> timedSteps = getRandomSteps(random, currentTime, nominalWidth, nominalLength);

         QuadrupedContactSequenceTools.computeStepTransitionsFromStepSequence(stepTransitions, currentTime, timedSteps);

         RecyclingArrayList<QuadrupedStepTransition> stepTransitionsExpected = new RecyclingArrayList<>(QuadrupedStepTransition::new);

         timedSteps.sort(Comparator.comparingDouble(timedStep -> timedStep.getTimeInterval().getStartTime()));

         for (int i = 0; i < timedSteps.size(); i++)
         {
            QuadrupedTimedStep step = timedSteps.get(i);

            if (step.getTimeInterval().getStartTime() >= currentTime)
            {
               QuadrupedStepTransition stepTransition = stepTransitionsExpected.add();

               stepTransition.setTransitionTime(step.getTimeInterval().getStartTime());
               stepTransition.addTransition(QuadrupedStepTransitionType.LIFT_OFF, step.getRobotQuadrant(), step.getGoalPosition());
            }

            if (step.getTimeInterval().getEndTime() >= currentTime)
            {
               QuadrupedStepTransition stepTransition = stepTransitionsExpected.add();

               stepTransition.setTransitionTime(step.getTimeInterval().getEndTime());
               stepTransition.addTransition(QuadrupedStepTransitionType.TOUCH_DOWN, step.getRobotQuadrant(), step.getGoalPosition());
            }
         }

         stepTransitionsExpected.sort(Comparator.comparingDouble(QuadrupedStepTransition::getTransitionTime));

         DCMPlanningTestTools.assertQuadrupedStepTransitionsListEqual(stepTransitionsExpected, stepTransitions, epsilon);
      }
   }

   private RecyclingArrayList<QuadrupedContactPhase> getRandomContactSequence(Random random, double currentTime, double nominalWidth, double nominalLength)
   {
      int sequenceSize = RandomNumbers.nextInt(random, 2, 50);
      RecyclingArrayList<QuadrupedContactPhase> contactSequence = new RecyclingArrayList<>(QuadrupedContactPhase::new);

      double startTime = RandomNumbers.nextDouble(random, -100.0, currentTime);

      for (int i = 0; i < sequenceSize; i++)
      {
         double duration = RandomNumbers.nextDouble(random, 0.01, 1.5);
         QuadrupedContactPhase phase = contactSequence.add();
         packRandomContactPhase(phase, random, startTime, duration, nominalWidth, nominalLength);

         startTime = phase.getTimeInterval().getEndTime();
      }

      return contactSequence;
   }

   private void packRandomContactPhase(QuadrupedContactPhase phase, Random random, double startTime, double duration, double nominalWidth,
                                       double nominalLength)
   {
      double endTime = startTime + duration;
      phase.setTimeInterval(new TimeInterval(startTime, endTime));

      List<RobotQuadrant> feetInContact = getRandomFeetInContact(random);

      QuadrantDependentList<FramePoint3D> solePositions = getRandomSolePositions(random, nominalWidth, nominalLength);

      phase.setFeetInContact(feetInContact);
      phase.setSolePositions(solePositions);
   }

   private RecyclingArrayList<QuadrupedTimedStep> getRandomSteps(Random random, double currentTime, double nominalWidth, double nominalLength)
   {
      int sequenceSize = RandomNumbers.nextInt(random, 2, 50);
      RecyclingArrayList<QuadrupedTimedStep> timedSteps = new RecyclingArrayList<>(QuadrupedTimedStep::new);

      double startTime = RandomNumbers.nextDouble(random, currentTime, 10.0);

      for (int i = 0; i < sequenceSize; i++)
      {
         double duration = RandomNumbers.nextDouble(random, 0.01, 1.5);
         QuadrupedTimedStep step = timedSteps.add();
         packRandomStep(step, random, startTime, duration, nominalWidth, nominalLength);

         startTime = step.getTimeInterval().getStartTime() + RandomNumbers.nextDouble(random, 2.0);
      }

      return timedSteps;
   }

   private void packRandomStep(QuadrupedTimedStep stepToPack, Random random, double startTime, double duration, double nominalWidth, double nominalLength)
   {
      RobotQuadrant quadrant = RobotQuadrant.values[RandomNumbers.nextInt(random, 0, 3)];
      FramePoint3D stepLocation = new FramePoint3D(ReferenceFrame.getWorldFrame(), quadrant.getEnd().negateIfHindEnd(nominalLength / 2.0),
                                                   quadrant.getSide().negateIfRightSide(nominalWidth / 2.0), 0.0);

      stepToPack.getTimeInterval().setInterval(startTime, startTime + duration);
      stepToPack.setGoalPosition(stepLocation);
      stepToPack.setRobotQuadrant(quadrant);
   }

   private List<RobotQuadrant> getRandomFeetInContact(Random random)
   {
      int numberOfFeetInContact = RandomNumbers.nextInt(random, 0, 4);

      Set<RobotQuadrant> feet = new HashSet<>();
      while (feet.size() < numberOfFeetInContact)
         feet.add(RobotQuadrant.values[RandomNumbers.nextInt(random, 0, 3)]);
      List<RobotQuadrant> feetInContact = new ArrayList<>();
      feetInContact.addAll(feet);

      return feetInContact;
   }

   private QuadrantDependentList<FramePoint3D> getRandomSolePositions(Random random, double nominalWidth, double nominalLength)
   {
      QuadrantDependentList<FramePoint3D> solePositions = new QuadrantDependentList<FramePoint3D>();
      for (RobotQuadrant foot : RobotQuadrant.values)
      {
         FramePoint3D point = new FramePoint3D(ReferenceFrame.getWorldFrame(), foot.getEnd().negateIfHindEnd(nominalLength / 2.0),
                                               foot.getSide().negateIfRightSide(nominalWidth / 2.0), 0.0);
         point.add(EuclidCoreRandomTools.nextPoint3D(random, 0.4));
         solePositions.put(foot, point);
      }

      return solePositions;
   }

}
