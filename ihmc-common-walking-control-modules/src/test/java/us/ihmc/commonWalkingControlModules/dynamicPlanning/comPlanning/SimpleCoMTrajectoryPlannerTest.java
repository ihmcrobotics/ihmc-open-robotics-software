package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class SimpleCoMTrajectoryPlannerTest
{
   private static final double comEpsilon = 1e-4;
   private static final double vrpEpsilon = 1e-5;
   private static final double dcmEpsilon = 5e-4;
   private static final double epsilon = 1e-6;

   private static final double omega  = 3.0;
   private static final double nominalHeight = 1.0;
   private static final double integrationDt = 1e-8;
   private static final double checkDT = 1e-3;
   private static final int ticksToCheck = (int) (checkDT / integrationDt);

   @Test
   public void testIntegration()
   {
      SimpleCoMTrajectoryPlanner planner = new SimpleCoMTrajectoryPlanner(() -> omega);

      List<SettableContactStateProvider> contactSequence = new ArrayList<>();

      SettableContactStateProvider firstContact = new SettableContactStateProvider();
      SettableContactStateProvider secondContact = new SettableContactStateProvider();
      SettableContactStateProvider thirdContact = new SettableContactStateProvider();

      FramePoint2D vrpStart = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
      FramePoint2D vrpMiddle = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.6, 0.75);
      FramePoint2D vrpMiddle2 = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.79, 0.88);
      FramePoint2D vrpEnd = new FramePoint2D(ReferenceFrame.getWorldFrame(), 1.0, 0.5);

      FramePoint3D startVRP = new FramePoint3D();
      FramePoint3D middleVRP = new FramePoint3D();
      FramePoint3D middleVRP2 = new FramePoint3D();
      FramePoint3D endVRP = new FramePoint3D();
      startVRP.set(vrpStart, nominalHeight);
      middleVRP.set(vrpMiddle, nominalHeight);
      middleVRP2.set(vrpMiddle2, nominalHeight);
      endVRP.set(vrpEnd, nominalHeight);

      double finalTime1 = 1.5;
      double finalTime2 = 3.1;
      double finalTime3 = 3.97;

      firstContact.getTimeInterval().setInterval(0.0, finalTime1);
      firstContact.setStartCopPosition(vrpStart);
      firstContact.setEndCopPosition(vrpMiddle);

      secondContact.getTimeInterval().setInterval(finalTime1, finalTime2);
      secondContact.setStartCopPosition(vrpMiddle);
      secondContact.setEndCopPosition(vrpMiddle2);

      thirdContact.getTimeInterval().setInterval(finalTime2, finalTime3);
      thirdContact.setStartCopPosition(vrpMiddle2);
      thirdContact.setEndCopPosition(vrpEnd);

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);
      contactSequence.add(thirdContact);

      planner.setNominalCoMHeight(nominalHeight);
      planner.solveForTrajectory(contactSequence);

      FramePoint3D initialCoMPosition = new FramePoint3D();
      planner.compute(0.0);
      FrameVector3D initialCoMVelocity = new FrameVector3D(planner.getDesiredCoMVelocity());

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, planner.getDesiredCoMPosition(), comEpsilon);

      FramePoint3D expectedCoMPosition = new FramePoint3D(initialCoMPosition);
      FrameVector3D expectedCoMVelocity = new FrameVector3D(initialCoMVelocity);
      FramePoint3D expectedDCMPosition = new FramePoint3D();
      expectedDCMPosition.scaleAdd(1.0 / omega, expectedCoMVelocity, expectedCoMPosition);

      FramePoint3D expectedVRP = new FramePoint3D();
      FramePoint3D expectedCMP = new FramePoint3D();
      FrameVector3D expectedCoMAcceleration = new FrameVector3D();
      FrameVector3D expectedDCMVelocity = new FrameVector3D();

      int tickCounter = ticksToCheck;
      double time = 0.0;
      for (; time <= finalTime1; time += integrationDt, tickCounter++)
      {
         expectedVRP.interpolate(startVRP, middleVRP, time / finalTime1);
         expectedCMP.set(expectedVRP);
         expectedCMP.subZ(nominalHeight);

         expectedCoMAcceleration.sub(expectedCoMPosition, expectedVRP);
         expectedCoMAcceleration.scale(omega * omega);

         expectedDCMVelocity.sub(expectedDCMPosition, expectedVRP);
         expectedDCMVelocity.scale(omega);

         if (tickCounter >= ticksToCheck)
         {
            String errorMessage = "Failed at time " + time;
            planner.compute(time);

            FramePoint3D constructedDCMPosition = new FramePoint3D();
            FrameVector3D constructedDCMVelocity = new FrameVector3D();
            constructedDCMPosition.scaleAdd(1.0 / omega, expectedCoMVelocity, expectedCoMPosition);
            constructedDCMVelocity.scaleAdd(1.0 / omega, expectedCoMAcceleration, expectedCoMVelocity);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(errorMessage, expectedCoMPosition, planner.getDesiredCoMPosition(), comEpsilon);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(errorMessage, expectedCoMVelocity, planner.getDesiredCoMVelocity(), comEpsilon);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(errorMessage, expectedCoMAcceleration, planner.getDesiredCoMAcceleration(), comEpsilon);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(errorMessage, constructedDCMPosition, planner.getDesiredDCMPosition(), comEpsilon);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(errorMessage, constructedDCMVelocity, planner.getDesiredDCMVelocity(), comEpsilon);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(errorMessage, expectedDCMPosition, planner.getDesiredDCMPosition(), dcmEpsilon);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(errorMessage, expectedCMP, planner.getDesiredECMPPosition(), vrpEpsilon);
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(errorMessage, expectedVRP, planner.getDesiredVRPPosition(), vrpEpsilon);

            // reset the integrator
            expectedCoMPosition.set(planner.getDesiredCoMPosition());
            expectedCoMVelocity.set(planner.getDesiredCoMVelocity());
            expectedDCMPosition.set(planner.getDesiredDCMPosition());

            tickCounter = 0;
         }

         expectedCoMPosition.scaleAdd(0.5 * integrationDt, expectedCoMVelocity, expectedCoMPosition);
         expectedCoMVelocity.scaleAdd(integrationDt, expectedCoMAcceleration, expectedCoMVelocity);
         expectedCoMPosition.scaleAdd(0.5 * integrationDt, expectedCoMVelocity, expectedCoMPosition);

         expectedDCMPosition.scaleAdd(integrationDt, expectedDCMVelocity, expectedDCMPosition);
      }


      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedCoMPosition, planner.comCornerPoints.get(1), comEpsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedDCMPosition, planner.dcmCornerPoints.get(1), dcmEpsilon);

      // reset to help with numerical integration issues
      expectedCoMPosition.set(planner.getDesiredCoMPosition());
      expectedCoMVelocity.set(planner.getDesiredCoMVelocity());

      for (; time <= finalTime2; time += integrationDt, tickCounter++)
      {
         double alpha = (time - finalTime1) / (finalTime2 - finalTime1);
         expectedVRP.interpolate(middleVRP, middleVRP2, alpha);

         expectedCMP.set(expectedVRP);
         expectedCMP.subZ(nominalHeight);

         expectedCoMAcceleration.sub(expectedCoMPosition, expectedVRP);
         expectedCoMAcceleration.scale(omega * omega);

         expectedDCMVelocity.sub(expectedDCMPosition, expectedVRP);
         expectedDCMVelocity.scale(omega);

         if (tickCounter >= ticksToCheck)
         {
            String errorMessage = "Failed at time " + time;
            planner.compute(1, time - finalTime1);

            FramePoint3D constructedDCMPosition = new FramePoint3D();
            FrameVector3D constructedDCMVelocity = new FrameVector3D();
            constructedDCMPosition.scaleAdd(1.0 / omega, expectedCoMVelocity, expectedCoMPosition);
            constructedDCMVelocity.scaleAdd(1.0 / omega, expectedCoMAcceleration, expectedCoMVelocity);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(errorMessage, expectedCMP, planner.getDesiredECMPPosition(), vrpEpsilon);
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(errorMessage, expectedVRP, planner.getDesiredVRPPosition(), vrpEpsilon);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(errorMessage, expectedCoMPosition, planner.getDesiredCoMPosition(), comEpsilon);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(errorMessage, expectedCoMVelocity, planner.getDesiredCoMVelocity(), comEpsilon);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(errorMessage, expectedCoMAcceleration, planner.getDesiredCoMAcceleration(), comEpsilon);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(errorMessage, constructedDCMPosition, planner.getDesiredDCMPosition(), comEpsilon);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(errorMessage, constructedDCMVelocity, planner.getDesiredDCMVelocity(), comEpsilon);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(errorMessage, expectedDCMPosition, planner.getDesiredDCMPosition(), dcmEpsilon);

            // reset the integrator
            expectedCoMPosition.set(planner.getDesiredCoMPosition());
            expectedCoMVelocity.set(planner.getDesiredCoMVelocity());
            expectedDCMPosition.set(planner.getDesiredDCMPosition());

            tickCounter = 0;
         }

         expectedCoMPosition.scaleAdd(0.5 * integrationDt, expectedCoMVelocity, expectedCoMPosition);
         expectedCoMVelocity.scaleAdd(integrationDt, expectedCoMAcceleration, expectedCoMVelocity);
         expectedCoMPosition.scaleAdd(0.5 * integrationDt, expectedCoMVelocity, expectedCoMPosition);

         expectedDCMPosition.scaleAdd(integrationDt, expectedDCMVelocity, expectedDCMPosition);
      }

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedCoMPosition, planner.comCornerPoints.get(2), comEpsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedDCMPosition, planner.dcmCornerPoints.get(2), dcmEpsilon);

      // reset to help with numerical integration issues
      expectedCoMPosition.set(planner.getDesiredCoMPosition());
      expectedCoMVelocity.set(planner.getDesiredCoMVelocity());

      tickCounter = 0;

      for (; time <= finalTime3; time += integrationDt, tickCounter++)
      {
         double localTime = time - finalTime2;
         double alpha = localTime / (finalTime3 - finalTime2);
         expectedVRP.interpolate(middleVRP2, endVRP, alpha);

         expectedCMP.set(expectedVRP);
         expectedCMP.subZ(nominalHeight);

         expectedCoMAcceleration.sub(expectedCoMPosition, expectedVRP);
         expectedCoMAcceleration.scale(omega * omega);

         expectedDCMVelocity.sub(expectedDCMPosition, expectedVRP);
         expectedDCMVelocity.scale(omega);

         if (tickCounter >= ticksToCheck)
         {
            String errorMessage = "Failed at time " + time + ", local time " + localTime;
            planner.compute(2, localTime);

            FramePoint3D constructedDCMPosition = new FramePoint3D();
            FrameVector3D constructedDCMVelocity = new FrameVector3D();
            constructedDCMPosition.scaleAdd(1.0 / omega, expectedCoMVelocity, expectedCoMPosition);
            constructedDCMVelocity.scaleAdd(1.0 / omega, expectedCoMAcceleration, expectedCoMVelocity);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(errorMessage, expectedCMP, planner.getDesiredECMPPosition(), vrpEpsilon);
            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(errorMessage, expectedVRP, planner.getDesiredVRPPosition(), vrpEpsilon);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(errorMessage, expectedCoMPosition, planner.getDesiredCoMPosition(), comEpsilon);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(errorMessage, expectedCoMVelocity, planner.getDesiredCoMVelocity(), comEpsilon);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(errorMessage, expectedCoMAcceleration, planner.getDesiredCoMAcceleration(), comEpsilon);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(errorMessage, constructedDCMPosition, planner.getDesiredDCMPosition(), comEpsilon);
            EuclidCoreTestTools.assertVector3DGeometricallyEquals(errorMessage, constructedDCMVelocity, planner.getDesiredDCMVelocity(), comEpsilon);

            EuclidCoreTestTools.assertPoint3DGeometricallyEquals(errorMessage, expectedDCMPosition, planner.getDesiredDCMPosition(), dcmEpsilon);

            // reset the integrator
            expectedCoMPosition.set(planner.getDesiredCoMPosition());
            expectedCoMVelocity.set(planner.getDesiredCoMVelocity());
            expectedDCMPosition.set(planner.getDesiredDCMPosition());

            tickCounter = 0;
         }

         expectedCoMVelocity.scaleAdd(integrationDt, expectedCoMAcceleration, expectedCoMVelocity);
         expectedCoMPosition.scaleAdd(integrationDt, expectedCoMVelocity, expectedCoMPosition);

         expectedDCMPosition.scaleAdd(integrationDt, expectedDCMVelocity, expectedDCMPosition);
      }


      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedCoMPosition, planner.comCornerPoints.get(3), comEpsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(expectedDCMPosition, planner.dcmCornerPoints.get(3), dcmEpsilon);
   }

   @Test
   public void testCornerPoints()
   {
      SimpleCoMTrajectoryPlanner planner = new SimpleCoMTrajectoryPlanner(() -> omega);

      List<SettableContactStateProvider> contactSequence = new ArrayList<>();

      SettableContactStateProvider firstContact = new SettableContactStateProvider();
      SettableContactStateProvider secondContact = new SettableContactStateProvider();
      SettableContactStateProvider thirdContact = new SettableContactStateProvider();

      FramePoint2D vrpStart = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.0, 0.0);
      FramePoint2D vrpMiddle = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.6, 0.75);
      FramePoint2D vrpMiddle2 = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.79, 0.88);
      FramePoint2D vrpEnd = new FramePoint2D(ReferenceFrame.getWorldFrame(), 1.0, 0.5);

      FramePoint3D startVRP = new FramePoint3D();
      FramePoint3D middleVRP = new FramePoint3D();
      FramePoint3D middleVRP2 = new FramePoint3D();
      FramePoint3D endVRP = new FramePoint3D();
      startVRP.set(vrpStart, nominalHeight);
      middleVRP.set(vrpMiddle, nominalHeight);
      middleVRP2.set(vrpMiddle2, nominalHeight);
      endVRP.set(vrpEnd, nominalHeight);

      double finalTime1 = 1.5;
      double finalTime2 = 3.1;
      double finalTime3 = 3.97;

      firstContact.getTimeInterval().setInterval(0.0, finalTime1);
      firstContact.setStartCopPosition(vrpStart);
      firstContact.setEndCopPosition(vrpMiddle);

      secondContact.getTimeInterval().setInterval(finalTime1, finalTime2);
      secondContact.setStartCopPosition(vrpMiddle);
      secondContact.setEndCopPosition(vrpMiddle2);

      thirdContact.getTimeInterval().setInterval(finalTime2, finalTime3);
      thirdContact.setStartCopPosition(vrpMiddle2);
      thirdContact.setEndCopPosition(vrpEnd);

      contactSequence.add(firstContact);
      contactSequence.add(secondContact);
      contactSequence.add(thirdContact);

      planner.setNominalCoMHeight(nominalHeight);
      planner.solveForTrajectory(contactSequence);

      FramePoint3D initialCoMPosition = new FramePoint3D();
      planner.compute(0.0);
      FrameVector3D initialCoMVelocity = new FrameVector3D(planner.getDesiredCoMVelocity());

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(initialCoMPosition, planner.getDesiredCoMPosition(), epsilon);

      FramePoint3D expectedCoMPosition = new FramePoint3D(initialCoMPosition);
      FrameVector3D expectedCoMVelocity = new FrameVector3D(initialCoMVelocity);
      FramePoint3D expectedDCMPosition = new FramePoint3D();
      expectedDCMPosition.scaleAdd(1.0 / omega, expectedCoMVelocity, expectedCoMPosition);

      FramePoint3D dcmCornerPoint = new FramePoint3D(expectedDCMPosition);
      FramePoint3D comCornerPoint = new FramePoint3D(initialCoMPosition);
      FramePoint3D nextDcmCornerPoint = new FramePoint3D(expectedDCMPosition);
      FramePoint3D nextCoMCornerPoint = new FramePoint3D(expectedDCMPosition);

      assertEquals(4, planner.dcmCornerPoints.size());
      assertEquals(4, planner.comCornerPoints.size());

      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(dcmCornerPoint, planner.dcmCornerPoints.get(0), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(comCornerPoint, planner.comCornerPoints.get(0), epsilon);
      CenterOfMassDynamicsTools.computeDesiredDCMPositionForwardTime(omega, finalTime1, finalTime1, dcmCornerPoint, startVRP, middleVRP, nextDcmCornerPoint);
      CenterOfMassDynamicsTools.computeDesiredCoMPositionForwardTime(omega, finalTime1, finalTime1, comCornerPoint, dcmCornerPoint, startVRP, middleVRP, nextCoMCornerPoint);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(nextDcmCornerPoint, planner.dcmCornerPoints.get(1), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(nextCoMCornerPoint, planner.comCornerPoints.get(1), epsilon);
      dcmCornerPoint.set(nextDcmCornerPoint);
      comCornerPoint.set(nextCoMCornerPoint);
      CenterOfMassDynamicsTools.computeDesiredDCMPositionForwardTime(omega, finalTime2 - finalTime1, finalTime2 - finalTime1, dcmCornerPoint, middleVRP, middleVRP2, nextDcmCornerPoint);
      CenterOfMassDynamicsTools.computeDesiredCoMPositionForwardTime(omega, finalTime2 - finalTime1, finalTime2 - finalTime1, comCornerPoint, dcmCornerPoint, middleVRP, middleVRP2, nextCoMCornerPoint);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(nextDcmCornerPoint, planner.dcmCornerPoints.get(2), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(nextCoMCornerPoint, planner.comCornerPoints.get(2), epsilon);
      dcmCornerPoint.set(nextDcmCornerPoint);
      comCornerPoint.set(nextCoMCornerPoint);
      CenterOfMassDynamicsTools.computeDesiredDCMPositionForwardTime(omega, finalTime3 - finalTime2, finalTime3 - finalTime2, dcmCornerPoint, middleVRP2, endVRP, nextDcmCornerPoint);
      CenterOfMassDynamicsTools.computeDesiredCoMPositionForwardTime(omega, finalTime3 - finalTime2, finalTime3 - finalTime2, comCornerPoint, dcmCornerPoint, middleVRP2, endVRP, nextCoMCornerPoint);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(nextDcmCornerPoint, planner.dcmCornerPoints.get(3), epsilon);
      EuclidCoreTestTools.assertPoint3DGeometricallyEquals(nextCoMCornerPoint, planner.comCornerPoints.get(3), epsilon);
   }

}
