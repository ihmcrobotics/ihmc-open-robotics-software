package us.ihmc.quadrupedRobotics.planning.icp;

import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.quadrupedBasics.gait.TimeInterval;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class CoMTrajectoryPlannerTest
{
   private static final double epsilon = 1e-6;
   @Test
   public void testOneSimpleMovingSegmentInContact()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);
      double gravityZ = 9.81;
      double nominalHeight = 0.7;

      List<QuadrupedContactPhase> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(contactSequence, omega, gravityZ, nominalHeight, registry);

      QuadrupedContactPhase firstContact = new QuadrupedContactPhase();
      QuadrupedContactPhase secondContact = new QuadrupedContactPhase();

      firstContact.setTimeInterval(new TimeInterval(0.0, 1.0));
      firstContact.setCopPosition(new FramePoint3D());
      secondContact.setTimeInterval(new TimeInterval(1.0, 2.0));
      secondContact.setCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.25, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);

      FramePoint3D comPosition = new FramePoint3D();
      comPosition.setZ(nominalHeight);
      planner.setCurrentCoMPosition(comPosition);

      planner.solveForTrajectory();
      planner.compute(0.0);

      FramePoint3D finalICP = new FramePoint3D(secondContact.getCopPosition());
      finalICP.addZ(nominalHeight);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comPosition, planner.getDesiredCoMPosition(), epsilon);

      planner.compute(1.0);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(finalICP, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3D initialDCM = new FramePoint3D();
      double interpolation = Math.exp(-omega.getDoubleValue());
      initialDCM.interpolate(firstContact.getCopPosition(), secondContact.getCopPosition(), interpolation);
      initialDCM.addZ(nominalHeight);

      planner.compute(0.0);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialDCM, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3D initialVRP = new FramePoint3D();
      initialVRP.set(firstContact.getCopPosition());
      initialVRP.addZ(nominalHeight);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, 1.0);
         FramePoint3D expectedDCM = new FramePoint3D();
         double exponential = Math.exp(omega.getDoubleValue() * time);
         expectedDCM.interpolate(initialVRP, initialDCM, exponential);

         planner.compute(time);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("time : " + time, expectedDCM, planner.getDesiredDCMPosition(), epsilon);
      }
   }

   @Test
   public void testOneMovingSegmentInContact()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);
      double gravityZ = 9.81;
      double nominalHeight = 0.7;

      List<QuadrupedContactPhase> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(contactSequence, omega, gravityZ, nominalHeight, registry);

      QuadrupedContactPhase firstContact = new QuadrupedContactPhase();
      QuadrupedContactPhase secondContact = new QuadrupedContactPhase();

      firstContact.setTimeInterval(new TimeInterval(0.0, 1.0));
      firstContact.setCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.1, 0.15, 0.0));
      secondContact.setTimeInterval(new TimeInterval(1.0, 2.0));
      secondContact.setCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.25, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);

      FramePoint3D comPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.1, 0.15, nominalHeight);
      planner.setCurrentCoMPosition(comPosition);

      planner.solveForTrajectory();
      planner.compute(0.0);

      FramePoint3D finalICP = new FramePoint3D(secondContact.getCopPosition());
      finalICP.addZ(nominalHeight);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comPosition, planner.getDesiredCoMPosition(), epsilon);

      planner.compute(1.0);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(finalICP, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3D initialDCM = new FramePoint3D();
      double interpolation = Math.exp(-omega.getDoubleValue());
      initialDCM.interpolate(firstContact.getCopPosition(), secondContact.getCopPosition(), interpolation);
      initialDCM.addZ(nominalHeight);

      planner.compute(0.0);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialDCM, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3D initialVRP = new FramePoint3D();
      initialVRP.set(firstContact.getCopPosition());
      initialVRP.addZ(nominalHeight);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, 1.0);
         FramePoint3D expectedDCM = new FramePoint3D();
         double exponential = Math.exp(omega.getDoubleValue() * time);
         expectedDCM.interpolate(initialVRP, initialDCM, exponential);

         planner.compute(time);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("time : " + time, expectedDCM, planner.getDesiredDCMPosition(), epsilon);
      }
   }

   @Test
   public void testTwoMovingSegmentsInContact()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);
      double gravityZ = 9.81;
      double nominalHeight = 0.7;

      List<QuadrupedContactPhase> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(contactSequence, omega, gravityZ, nominalHeight, registry);

      QuadrupedContactPhase firstContact = new QuadrupedContactPhase();
      QuadrupedContactPhase secondContact = new QuadrupedContactPhase();
      QuadrupedContactPhase thirdContact= new QuadrupedContactPhase();

      firstContact.setTimeInterval(new TimeInterval(0.0, 1.0));
      firstContact.setCopPosition(new FramePoint3D());
      secondContact.setTimeInterval(new TimeInterval(1.0, 2.0));
      secondContact.setCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.5, 0.0));
      thirdContact.setTimeInterval(new TimeInterval(2.0, 3.0));
      thirdContact.setCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);
      contactSequence.add(thirdContact);

      FramePoint3D comPosition = new FramePoint3D();
      comPosition.setZ(nominalHeight);
      planner.setCurrentCoMPosition(comPosition);
      planner.solveForTrajectory();
      planner.compute(0.0);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comPosition, planner.getDesiredCoMPosition(), epsilon);


      FramePoint3D secondDCM = new FramePoint3D();
      FramePoint3D initialDCM = new FramePoint3D();
      double interpolation = Math.exp(-omega.getDoubleValue());
      secondDCM.interpolate(secondContact.getCopPosition(), thirdContact.getCopPosition(), interpolation);
      initialDCM.interpolate(firstContact.getCopPosition(), secondDCM, interpolation);
      secondDCM.addZ(nominalHeight);
      initialDCM.addZ(nominalHeight);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialDCM, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3D initialVRP = new FramePoint3D(firstContact.getCopPosition());
      initialVRP.addZ(nominalHeight);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, 1.0);
         FramePoint3D expectedDCM = new FramePoint3D();
         double exponential = Math.exp(omega.getDoubleValue() * time);
         expectedDCM.interpolate(initialVRP, initialDCM, exponential);

         planner.compute(time);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedDCM, planner.getDesiredDCMPosition(), epsilon);
      }
   }




   @Test
   public void testManyMovingSegmentsInContact3D()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);
      double gravityZ = 9.81;
      double nominalHeight = 0.7;

      List<QuadrupedContactPhase> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(contactSequence, omega, gravityZ, nominalHeight, registry);

      Random random = new Random(1738L);

      for (int iter = 0; iter < 10; iter++)
      {
         contactSequence.clear();

         double initialTime = 0.0;
         FramePoint3DReadOnly startCoPPosition = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame(), 10.0);
         int numberOfContacts = RandomNumbers.nextInt(random, 2, 10);

         double currentStartTime = initialTime;

         double segmentDuration = RandomNumbers.nextDouble(random, 0.0, 5.0);

         // handle initial phase
         QuadrupedContactPhase contactPhase = new QuadrupedContactPhase();
         contactPhase.setTimeInterval(new TimeInterval(currentStartTime, segmentDuration + currentStartTime));
         contactPhase.setCopPosition(startCoPPosition);

         contactSequence.add(contactPhase);

         currentStartTime += segmentDuration;

         // add more phases
         FramePoint3D currentCoPPosition = new FramePoint3D(startCoPPosition);
         for (int contactIndex = 1; contactIndex < numberOfContacts; contactIndex++)
         {
            segmentDuration = RandomNumbers.nextDouble(random, 0.0, 5.0);
            currentCoPPosition.add(EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame(), new Vector3D(1.0, 1.0, 0.0)));

            contactPhase = new QuadrupedContactPhase();
            contactPhase.setTimeInterval(new TimeInterval(currentStartTime, segmentDuration + currentStartTime));
            contactPhase.setCopPosition(currentCoPPosition);

            contactSequence.add(contactPhase);

            currentStartTime += segmentDuration;
         }



         FramePoint3D initialCoMPosition = new FramePoint3D(startCoPPosition);
         initialCoMPosition.addZ(nominalHeight);

         planner.setCurrentCoMPosition(initialCoMPosition);
         planner.solveForTrajectory();
         planner.compute(0.0);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("iter = " + iter, initialCoMPosition, planner.getDesiredCoMPosition(), epsilon);

         FramePoint3DReadOnly desiredInitialDCM = planner.getDesiredDCMPosition();

         FramePoint3D finalDCM = new FramePoint3D(contactSequence.get(numberOfContacts - 1).getCopPosition());
         finalDCM.addZ(nominalHeight);
         FramePoint3D currentDCM = new FramePoint3D(finalDCM);
         for (int index = numberOfContacts - 1; index > 0; index--)
         {
            FramePoint3D vrp = new FramePoint3D(contactSequence.get(index - 1).getCopPosition());
            vrp.addZ(nominalHeight);
            double interpolation = Math.exp(-omega.getDoubleValue() * contactSequence.get(index).getTimeInterval().getDuration());

            currentDCM.interpolate(vrp, 1.0 - interpolation);
         }

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("iter = " + iter, currentDCM, desiredInitialDCM, epsilon);

         FramePoint3D initialVRP = new FramePoint3D(contactSequence.get(0).getCopPosition());
         initialVRP.addZ(nominalHeight);

         for (int i = 0; i < 100; i++)
         {
            double time = RandomNumbers.nextDouble(random, 0.0, contactSequence.get(0).getTimeInterval().getDuration());
            FramePoint3D expectedDCM = new FramePoint3D();
            double exponential = Math.exp(omega.getDoubleValue() * time);
            expectedDCM.interpolate(initialVRP, currentDCM, exponential);

            planner.compute(time);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedDCM, planner.getDesiredDCMPosition(), epsilon);
         }
      }
   }



   @Test
   public void testTwoMovingSegmentsOneFlight()
   {
      YoVariableRegistry registry = new YoVariableRegistry("test");
      YoDouble omega = new YoDouble("omega", registry);
      omega.set(3.0);
      double gravityZ = 9.81;
      double nominalHeight = 0.7;

      List<QuadrupedContactPhase> contactSequence = new ArrayList<>();
      CoMTrajectoryPlanner planner = new CoMTrajectoryPlanner(contactSequence, omega, gravityZ, nominalHeight, registry);

      QuadrupedContactPhase firstContact = new QuadrupedContactPhase();
      QuadrupedContactPhase secondContact = new QuadrupedContactPhase();
      QuadrupedContactPhase thirdContact = new QuadrupedContactPhase();

      firstContact.setTimeInterval(new TimeInterval(0.0, 1.0));
      firstContact.setCopPosition(new FramePoint3D());
      secondContact.setTimeInterval(new TimeInterval(1.0, 1.25));
      secondContact.setContactState(ContactState.NO_CONTACT);
      thirdContact.setTimeInterval(new TimeInterval(1.25, 2.25));
      thirdContact.setCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);
      contactSequence.add(thirdContact);

      FramePoint3D comPosition = new FramePoint3D();
      comPosition.addZ(nominalHeight);
      planner.setCurrentCoMPosition(comPosition);
      planner.solveForTrajectory();
      planner.compute(0.0);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Desired CoM is invalid.", comPosition, planner.getDesiredCoMPosition(), epsilon);


      FramePoint3D firstVRP = new FramePoint3D(firstContact.getCopPosition());
      FramePoint3D secondVRP = new FramePoint3D(secondContact.getCopPosition());
      FramePoint3D thirdVRP = new FramePoint3D(thirdContact.getCopPosition());
      firstVRP.addZ(nominalHeight);
      secondVRP.addZ(nominalHeight);
      thirdVRP.addZ(nominalHeight);


      FramePoint3D initialDCM = new FramePoint3D(planner.getDesiredDCMPosition());

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, contactSequence.get(0).getTimeInterval().getDuration());
         FramePoint3D expectedDCM = new FramePoint3D();
         double exponential = Math.exp(omega.getDoubleValue() * time);
         expectedDCM.interpolate(firstVRP, initialDCM, exponential);

         planner.compute(time);

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("i = " + i, expectedDCM, planner.getDesiredDCMPosition(), epsilon);
      }
   }
}
