package us.ihmc.quadrupedRobotics.planning.icp;

import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.quadrupedBasics.gait.TimeInterval;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.tools.lists.ListSorter;

import java.util.*;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

public class QuadrupedContactSequenceToolsTest
{
   private static final double epsilon = 1e-8;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTrimPastContactSequences()
   {
      Random random = new Random(1738L);

      for (int iter = 0; iter < 1; iter++)
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

         QuadrupedContactSequenceTools.trimPastContactSequences(contactSequence, currentTime, feetInContact, solePositions, phasesToKeep);

         int sequenceIndex = 0;
         while (sequenceIndex < contactSequenceExpected.size())
         {
            if (contactSequenceExpected.get(sequenceIndex).getTimeInterval().getStartTime() > currentTime)
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

         while (contactSequenceExpected.size() > phasesToKeep)
            contactSequenceExpected.remove(0);

         // TODO finish the current phase

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
            contactPhase.getTimeInterval().setStartTime(currentTime);
            contactPhase.setFeetInContact(feetInContact);
            contactPhase.setSolePositions(solePositions);

            contactPhase.update();
         }


         assertEquals(contactSequenceExpected.size(), contactSequence.size());
         for (int i = 0; i < contactSequenceExpected.size(); i++)
         {
            DCMPlanningTestTools.assertQuadrupedContactPhasesEqual(contactSequenceExpected.get(i), contactSequence.get(i), epsilon);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeStepTransitionsFromStepSequence()
   {
      fail();
   }
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsContactSequenceEqual()
   {
      fail();
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

         startTime = phase.getTimeInterval().getStartTime();
      }

      return contactSequence;
   }

   private void packRandomContactPhase(QuadrupedContactPhase phase, Random random, double lastStartTime, double duration, double nominalWidth,
                                       double nominalLength)
   {
      double phaseDifference = RandomNumbers.nextDouble(random, 0.0, 0.5);
      double startTime = lastStartTime + phaseDifference;
      double endTime = startTime + duration;
      phase.setTimeInterval(new TimeInterval(startTime, endTime));

      List<RobotQuadrant> feetInContact = getRandomFeetInContact(random);

      QuadrantDependentList<FramePoint3D> solePositions = getRandomSolePositions(random, nominalWidth, nominalLength);

      phase.setFeetInContact(feetInContact);
      phase.setSolePositions(solePositions);
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
