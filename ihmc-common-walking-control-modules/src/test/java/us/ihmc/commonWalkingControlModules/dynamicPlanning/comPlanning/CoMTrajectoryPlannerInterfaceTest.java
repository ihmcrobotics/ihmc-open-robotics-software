package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public abstract class CoMTrajectoryPlannerInterfaceTest
{
   private static final double epsilon = 1e-4;

   protected YoRegistry registry;
   protected YoDouble omega;
   protected List<ContactStateProvider> contactSequence;
   protected double gravityZ;
   protected double nominalHeight;

   protected abstract CoMTrajectoryPlannerInterface createComTrajectoryPlanner();

   @BeforeEach
   private void setupTest()
   {
      registry = new YoRegistry("testJacobian");
      omega = new YoDouble("omega", registry);
      omega.set(3.0);
      gravityZ = 9.81;
      nominalHeight = gravityZ / MathTools.square(omega.getDoubleValue());
      contactSequence = new ArrayList<>();
   }

   @AfterEach
   private void tearDown()
   {
      registry = null;
      omega = null;
      gravityZ = Double.NaN;
      nominalHeight = Double.NaN;
      contactSequence = null;
   }

   @Test
   public void testNoSteps()
   {
      CoMTrajectoryPlannerInterface planner = createComTrajectoryPlanner();

      SettableContactStateProvider firstContact = new SettableContactStateProvider();

      firstContact.setTimeInterval(new TimeInterval(0.0, Double.POSITIVE_INFINITY));
      firstContact.setStartCopPosition(new FramePoint3D());
      firstContact.setEndCopPosition(new FramePoint3D());

      contactSequence.add(firstContact);

      FramePoint3D comPosition = new FramePoint3D();
      FrameVector3D comVelocity = new FrameVector3D();
      comPosition.setZ(nominalHeight);
      planner.setInitialCenterOfMassState(comPosition, comVelocity);

      planner.solveForTrajectory(contactSequence);
      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      FramePoint3D desiredDCM = new FramePoint3D(firstContact.getCopStartPosition());
      desiredDCM.addZ(nominalHeight);
      FrameVector3D desiredDCMVelocity = new FrameVector3D();

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredDCMPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredVRPPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredCoMPosition(), epsilon);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredDCMVelocity, planner.getDesiredDCMVelocity(), epsilon);

      planner.compute(1.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredDCMPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredVRPPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredCoMPosition(), epsilon);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredDCMVelocity, planner.getDesiredDCMVelocity(), epsilon);

      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredDCMPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredVRPPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredCoMPosition(), epsilon);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredDCMVelocity, planner.getDesiredDCMVelocity(), epsilon);

      planner.compute(10.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredDCMPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredVRPPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredCoMPosition(), epsilon);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredDCMVelocity, planner.getDesiredDCMVelocity(), epsilon);

      planner.compute(100.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredDCMPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredVRPPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredCoMPosition(), epsilon);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredDCMVelocity, planner.getDesiredDCMVelocity(), epsilon);

      planner.compute(1000.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredDCMPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredVRPPosition(), epsilon);
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredCoMPosition(), epsilon);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredDCMVelocity, planner.getDesiredDCMVelocity(), epsilon);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, 5000.0);
         planner.compute(time);
         checkPlannerDynamics(planner, omega.getDoubleValue());

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredDCMPosition(), epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredVRPPosition(), epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(desiredDCM, planner.getDesiredCoMPosition(), epsilon);
         EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals(desiredDCMVelocity, planner.getDesiredDCMVelocity(), epsilon);
      }
   }

   @Test
   public void testOneSimpleMovingSegmentInContact()
   {
      CoMTrajectoryPlannerInterface planner = createComTrajectoryPlanner();

      SettableContactStateProvider firstContact = new SettableContactStateProvider();
      SettableContactStateProvider secondContact = new SettableContactStateProvider();

      double duration = 1.0;

      firstContact.setTimeInterval(new TimeInterval(0.0, duration));
      firstContact.setStartCopPosition(new FramePoint3D());
      firstContact.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.25, 0.0));
      secondContact.setTimeInterval(new TimeInterval(1.0, 2.0));
      secondContact.setStartCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.25, 0.0));
      secondContact.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.25, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);

      FramePoint3D comPosition = new FramePoint3D();
      FrameVector3D comVelocity = new FrameVector3D();
      comPosition.setZ(nominalHeight);
      planner.setInitialCenterOfMassState(comPosition, comVelocity);

      planner.solveForTrajectory(contactSequence);
      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      FramePoint3D finalDCM = new FramePoint3D(secondContact.getCopStartPosition());
      finalDCM.addZ(nominalHeight);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comPosition, planner.getDesiredCoMPosition(), epsilon);

      planner.compute(duration);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(finalDCM, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3D initialDCM = new FramePoint3D();
      CapturePointTools.computeCapturePointPosition(comPosition, comVelocity, omega.getDoubleValue(), initialDCM);
      DCMTrajectoryTestTools
            .computeDCMUsingLinearVRP(omega.getDoubleValue(), -duration, -duration, secondContact.getCopStartPosition(), secondContact.getCopStartPosition(),
                                      firstContact.getCopStartPosition(), initialDCM);
      initialDCM.addZ(nominalHeight);

      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialDCM, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3D initialVRP = new FramePoint3D();
      FramePoint3D finalVRP = new FramePoint3D();
      initialVRP.set(firstContact.getCopStartPosition());
      finalVRP.set(firstContact.getCopEndPosition());
      initialVRP.addZ(nominalHeight);
      finalVRP.addZ(nominalHeight);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, 1.0);
         FramePoint3D expectedDCM = new FramePoint3D();
         DCMTrajectoryTestTools.computeDCMUsingLinearVRP(omega.getDoubleValue(), time, duration, initialDCM, initialVRP, finalVRP, expectedDCM);

         planner.compute(time);
         checkPlannerDynamics(planner, omega.getDoubleValue());

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("time : " + time, expectedDCM, planner.getDesiredDCMPosition(), epsilon);
      }
   }

   @Test
   public void testOneMovingSegmentInContact()
   {
      CoMTrajectoryPlannerInterface planner = createComTrajectoryPlanner();

      SettableContactStateProvider firstContact = new SettableContactStateProvider();
      SettableContactStateProvider secondContact = new SettableContactStateProvider();

      firstContact.setTimeInterval(new TimeInterval(0.0, 1.0));
      firstContact.setStartCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.1, 0.15, 0.0));
      firstContact.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.25, 0.0));
      secondContact.setTimeInterval(new TimeInterval(1.0, 2.0));
      secondContact.setStartCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.25, 0.0));
      secondContact.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.25, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);

      FramePoint3D comPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), -0.1, 0.15, nominalHeight);
      FrameVector3D comVelocity = new FrameVector3D();
      planner.setInitialCenterOfMassState(comPosition, comVelocity);

      planner.solveForTrajectory(contactSequence);
      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      FramePoint3D finalICP = new FramePoint3D(secondContact.getCopStartPosition());
      finalICP.addZ(nominalHeight);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comPosition, planner.getDesiredCoMPosition(), epsilon);

      planner.compute(1.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(finalICP, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3DReadOnly initialDCM = recursivelyComputeInitialDCMLinear(contactSequence, nominalHeight, omega.getDoubleValue());

      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialDCM, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3D initialVRP = new FramePoint3D();
      initialVRP.set(firstContact.getCopStartPosition());
      initialVRP.addZ(nominalHeight);

      FramePoint3D finalVRP = new FramePoint3D();
      finalVRP.set(firstContact.getCopEndPosition());
      finalVRP.addZ(nominalHeight);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, 1.0);
         FramePoint3D expectedDCM = new FramePoint3D();
         DCMTrajectoryTestTools
               .computeDCMUsingLinearVRP(omega.getDoubleValue(), time, contactSequence.get(0).getTimeInterval().getDuration(), initialDCM, initialVRP, finalVRP,
                                         expectedDCM);

         planner.compute(time);
         checkPlannerDynamics(planner, omega.getDoubleValue());

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("time : " + time, expectedDCM, planner.getDesiredDCMPosition(), epsilon);
      }
   }

   @Test
   public void testTwoMovingSegmentsInContact()
   {
      CoMTrajectoryPlannerInterface planner = createComTrajectoryPlanner();

      SettableContactStateProvider firstContact = new SettableContactStateProvider();
      SettableContactStateProvider secondContact = new SettableContactStateProvider();
      SettableContactStateProvider thirdContact = new SettableContactStateProvider();

      firstContact.setTimeInterval(new TimeInterval(0.0, 0.75));
      firstContact.setStartCopPosition(new FramePoint3D());
      firstContact.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.5, 0.0));
      secondContact.setTimeInterval(new TimeInterval(0.75, 1.9));
      secondContact.setStartCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.0, 0.5, 0.0));
      secondContact.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0));
      thirdContact.setTimeInterval(new TimeInterval(1.9, 3.0));
      thirdContact.setStartCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0));
      thirdContact.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);
      contactSequence.add(thirdContact);

      FramePoint3D comPosition = new FramePoint3D();
      FrameVector3D comVelocity = new FrameVector3D();
      comPosition.setZ(nominalHeight);
      planner.setInitialCenterOfMassState(comPosition, comVelocity);

      planner.solveForTrajectory(contactSequence);
      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comPosition, planner.getDesiredCoMPosition(), epsilon);

      FramePoint3DReadOnly initialDCM = recursivelyComputeInitialDCMLinear(contactSequence, nominalHeight, omega.getDoubleValue());

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialDCM, planner.getDesiredDCMPosition(), epsilon);

      FramePoint3D initialVRP = new FramePoint3D(firstContact.getCopStartPosition());
      FramePoint3D finalVRP = new FramePoint3D(firstContact.getCopEndPosition());
      initialVRP.addZ(nominalHeight);
      finalVRP.addZ(nominalHeight);

      /*
      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, 1.0);
         FramePoint3D expectedDCM = new FramePoint3D();
         DCMTrajectoryTestTools
               .computeDCMUsingLinearVRP(omega.getDoubleValue(), time, contactSequence.get(0).getTimeInterval().getDuration(), initialDCM, initialVRP, finalVRP,
                                         expectedDCM);

         planner.compute(time);
         checkPlannerDynamics(planner, omega.getDoubleValue());

         EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedDCM, planner.getDesiredDCMPosition(), 5e-2);
      }

       */
   }

   private FramePoint3DReadOnly recursivelyComputeInitialDCMLinear(List<ContactStateProvider> contactPhases, double nominalHeight, double omega)
   {
      int numberOfPhases = contactPhases.size();
      FramePoint3D lastDCM = new FramePoint3D(contactPhases.get(numberOfPhases - 1).getCopStartPosition());
      lastDCM.addZ(nominalHeight);
      FramePoint3D initialDCM = new FramePoint3D(lastDCM);
      for (int i = numberOfPhases - 2; i >= 0; i--)
      {
         FramePoint3D endDCM = new FramePoint3D(initialDCM);
         FramePoint3D initialVRP = new FramePoint3D(contactPhases.get(i).getCopStartPosition());
         FramePoint3D endVRP = new FramePoint3D(contactPhases.get(i).getCopEndPosition());
         initialVRP.addZ(nominalHeight);
         endVRP.addZ(nominalHeight);

         double duration = contactPhases.get(i).getTimeInterval().getDuration();

         DCMTrajectoryTestTools.computeDCMUsingLinearVRP(omega, -duration, -duration, endDCM, endVRP, initialVRP, initialDCM);
      }

      return initialDCM;
   }

   @Test
   public void testManyMovingSegmentsInContact3D()
   {
      CoMTrajectoryPlannerInterface planner = createComTrajectoryPlanner();

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
         SettableContactStateProvider contactPhase = new SettableContactStateProvider();
         contactPhase.setTimeInterval(new TimeInterval(currentStartTime, segmentDuration + currentStartTime));
         contactPhase.setStartCopPosition(startCoPPosition);

         FramePoint3D currentCoPPosition = new FramePoint3D(startCoPPosition);
         currentCoPPosition.add(EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame(), new Vector3D(1.0, 1.0, 0.0)));

         contactPhase.setEndCopPosition(currentCoPPosition);

         contactSequence.add(contactPhase);

         currentStartTime += segmentDuration;

         // add more phases
         for (int contactIndex = 1; contactIndex < numberOfContacts; contactIndex++)
         {
            segmentDuration = RandomNumbers.nextDouble(random, 0.0, 5.0);

            contactPhase = new SettableContactStateProvider();
            contactPhase.setTimeInterval(new TimeInterval(currentStartTime, segmentDuration + currentStartTime));
            contactPhase.setStartCopPosition(contactSequence.get(contactIndex - 1).getCopEndPosition());

            currentCoPPosition.add(EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame(), new Vector3D(1.0, 1.0, 0.0)));

            contactPhase.setEndCopPosition(currentCoPPosition);

            contactSequence.add(contactPhase);

            currentStartTime += segmentDuration;
         }

         FramePoint3D comPosition = new FramePoint3D();
         FrameVector3D comVelocity = new FrameVector3D();
         comPosition.setZ(nominalHeight);
         planner.setInitialCenterOfMassState(comPosition, comVelocity);

         planner.solveForTrajectory(contactSequence);
         planner.compute(0.0);
         checkPlannerDynamics(planner, omega.getDoubleValue());

         EuclidCoreTestTools
               .assertPoint3DGeometricallyEquals("iter = " + iter + ", Initial CoM is wrong.", comPosition, planner.getDesiredCoMPosition(), epsilon);

         FramePoint3DReadOnly desiredInitialDCM = planner.getDesiredDCMPosition();

         FramePoint3DReadOnly initialDCM = recursivelyComputeInitialDCMLinear(contactSequence, nominalHeight, omega.getDoubleValue());
         EuclidCoreTestTools.assertPoint3DGeometricallyEquals("iter = " + iter + ", Initial DCM is wrong.", initialDCM, desiredInitialDCM,
                                                              initialDCM.distance(new Point3D()) * epsilon);

         FramePoint3D initialVRP = new FramePoint3D(contactSequence.get(0).getCopStartPosition());
         FramePoint3D finalVRP = new FramePoint3D(contactSequence.get(0).getCopEndPosition());
         initialVRP.addZ(nominalHeight);
         finalVRP.addZ(nominalHeight);

         double duration = contactSequence.get(0).getTimeInterval().getDuration();
         for (int i = 0; i < 100; i++)
         {
            double time = RandomNumbers.nextDouble(random, 0.0, duration);

            FramePoint3D expectedDCM = new FramePoint3D();
            DCMTrajectoryTestTools.computeDCMUsingLinearVRP(omega.getDoubleValue(), time, duration, initialDCM, initialVRP, finalVRP, expectedDCM);

            planner.compute(time);
            checkPlannerDynamics(planner, omega.getDoubleValue());

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals("time : " + time, expectedDCM, planner.getDesiredDCMPosition(), 100.0 * epsilon);
         }
      }
   }

   @Test
   public void testTwoMovingSegmentsOneFlight()
   {
      CoMTrajectoryPlannerInterface planner = createComTrajectoryPlanner();

      SettableContactStateProvider firstContact = new SettableContactStateProvider();
      SettableContactStateProvider secondContact = new SettableContactStateProvider();
      SettableContactStateProvider thirdContact = new SettableContactStateProvider();

      firstContact.setTimeInterval(new TimeInterval(0.0, 1.0));
      firstContact.setStartCopPosition(new FramePoint3D());
      firstContact.setEndCopPosition(new FramePoint3D());
      secondContact.setTimeInterval(new TimeInterval(1.0, 1.25));
      secondContact.setContactState(ContactState.FLIGHT);
      thirdContact.setTimeInterval(new TimeInterval(1.25, 2.25));
      thirdContact.setStartCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0));
      thirdContact.setEndCopPosition(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0));

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);
      contactSequence.add(thirdContact);

      FramePoint3D comPosition = new FramePoint3D();
      FrameVector3D comVelocity = new FrameVector3D();
      comPosition.setZ(nominalHeight);
      planner.setInitialCenterOfMassState(comPosition, comVelocity);

      planner.solveForTrajectory(contactSequence);
      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Desired CoM is invalid.", comPosition, planner.getDesiredCoMPosition(), epsilon);

      FramePoint3D initialDCM = new FramePoint3D(planner.getDesiredDCMPosition());
      FramePoint3D firstVRP = new FramePoint3D(planner.getDesiredVRPPosition());

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, contactSequence.get(0).getTimeInterval().getDuration());
         FramePoint3D expectedDCM = new FramePoint3D();
         double exponential = Math.exp(omega.getDoubleValue() * time);
         expectedDCM.interpolate(firstVRP, initialDCM, exponential);

         planner.compute(time);
         checkPlannerDynamics(planner, omega.getDoubleValue());

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals("i = " + i, firstVRP, planner.getDesiredVRPPosition(), epsilon);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals("i = " + i, expectedDCM, planner.getDesiredDCMPosition(), epsilon);
      }
   }

   @Test
   public void testStartingInFlight()
   {
      CoMTrajectoryPlannerInterface planner = createComTrajectoryPlanner();

      SettableContactStateProvider firstContact = new SettableContactStateProvider();
      SettableContactStateProvider secondContact = new SettableContactStateProvider();

      double flightDuration = 0.25;
      FramePoint3D contactPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.0, 0.0, 0.0);
      firstContact.setTimeInterval(new TimeInterval(0.0, flightDuration));
      firstContact.setContactState(ContactState.FLIGHT);
      secondContact.setTimeInterval(new TimeInterval(flightDuration, 1.25));
      secondContact.setStartCopPosition(contactPosition);
      secondContact.setEndCopPosition(contactPosition);

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);

      FramePoint3D comPosition = new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.05, 0.05, nominalHeight + 0.05);
      FrameVector3D comVelocity = new FrameVector3D();
      planner.setInitialCenterOfMassState(comPosition, comVelocity);

      planner.solveForTrajectory(contactSequence);
      planner.compute(0.0);
      checkPlannerDynamics(planner, omega.getDoubleValue());

      FrameVector3DReadOnly acceleration = planner.getDesiredCoMAcceleration();

      assertEquals(0.0, acceleration.getX(), epsilon);
      assertEquals(0.0, acceleration.getY(), epsilon);
      assertEquals(-gravityZ, acceleration.getZ(), epsilon);

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals("Desired CoM is invalid.", comPosition, planner.getDesiredCoMPosition(), epsilon);

      FramePoint3D secondVRP = new FramePoint3D(firstContact.getCopStartPosition());
      FramePoint3D thirdVRP = new FramePoint3D(secondContact.getCopStartPosition());
      secondVRP.addZ(nominalHeight);
      thirdVRP.addZ(nominalHeight);

      Random random = new Random(1738L);
      for (int i = 0; i < 100; i++)
      {
         double time = RandomNumbers.nextDouble(random, 0.0, contactSequence.get(0).getTimeInterval().getDuration());
         planner.compute(time);
         checkPlannerDynamics(planner, omega.getDoubleValue());
      }

      FramePoint3D preTouchdownCoMPosition = new FramePoint3D();
      FramePoint3D preTouchdownDCMPosition = new FramePoint3D();
      FramePoint3D preTouchdownVRPPosition = new FramePoint3D();
      FramePoint3D preTouchdownCMPPosition = new FramePoint3D();
      FrameVector3D preTouchdownCoMVelocity = new FrameVector3D();
      FrameVector3D preTouchdownCoMAcceleration = new FrameVector3D();
      FrameVector3D preTouchdownDCMVelocity = new FrameVector3D();
      FramePoint3D postTouchdownCoMPosition = new FramePoint3D();
      FramePoint3D postTouchdownDCMPosition = new FramePoint3D();
      FramePoint3D postTouchdownVRPPosition = new FramePoint3D();
      FramePoint3D postTouchdownCMPPosition = new FramePoint3D();
      FrameVector3D postTouchdownCoMVelocity = new FrameVector3D();
      FrameVector3D postTouchdownCoMAcceleration = new FrameVector3D();
      FrameVector3D postTouchdownDCMVelocity = new FrameVector3D();
      planner.compute(0, flightDuration, preTouchdownCoMPosition, preTouchdownCoMVelocity, preTouchdownCoMAcceleration, preTouchdownDCMPosition,
                      preTouchdownDCMVelocity, preTouchdownVRPPosition, preTouchdownCMPPosition);
      planner.compute(1, 0.0, postTouchdownCoMPosition, postTouchdownCoMVelocity, postTouchdownCoMAcceleration, postTouchdownDCMPosition,
                      postTouchdownDCMVelocity, postTouchdownVRPPosition, postTouchdownCMPPosition);
   }



   protected static void checkPlannerDynamics(CoMTrajectoryPlannerInterface planner, double omega)
   {
      FramePoint3D constructedDCMPosition = new FramePoint3D();
      constructedDCMPosition.scaleAdd(1.0 / omega, planner.getDesiredCoMVelocity(), planner.getDesiredCoMPosition());

      FrameVector3D constructedDCMVelocity = new FrameVector3D();
      constructedDCMVelocity.set(constructedDCMPosition);
      constructedDCMVelocity.sub(planner.getDesiredVRPPosition());
      constructedDCMVelocity.scale(omega);

      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals("dcm dynamics are wrong.", constructedDCMPosition, planner.getDesiredDCMPosition(), epsilon);
      EuclidFrameTestTools.assertFrameVector3DGeometricallyEquals("vrp dynamics are wrong.", constructedDCMVelocity, planner.getDesiredDCMVelocity(), epsilon);

   }
}
