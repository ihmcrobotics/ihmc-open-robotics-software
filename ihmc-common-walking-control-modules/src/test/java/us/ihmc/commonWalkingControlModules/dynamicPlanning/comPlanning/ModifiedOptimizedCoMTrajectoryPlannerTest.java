package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

import static us.ihmc.robotics.Assert.assertTrue;

public class ModifiedOptimizedCoMTrajectoryPlannerTest
{
   @Test
   public void testSimpleOneStepThreeSegments()
   {
      double firstDuration = 0.75;
      double secondDuration = 1.1;
      double thirdDuration = 0.4;
      double omega = 3.0;
      double gravity = 9.81;
      double nominalHeight = omega * omega / gravity;

      FramePoint3D initialVRP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.1, 0.75, 0.98);
      FramePoint3D finalVRP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.4, 0.92, 1.05);
      FramePoint3D initialCMP = new FramePoint3D(initialVRP);
      FramePoint3D finalCMP = new FramePoint3D(finalVRP);
      initialCMP.subZ(nominalHeight);
      finalCMP.subZ(nominalHeight);

      FramePoint3D startCoM = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.2, 0.8, 1.01);


      ModifiedOptimizedCoMTrajectoryPlanner comPlanner = new ModifiedOptimizedCoMTrajectoryPlanner(gravity, nominalHeight, new YoRegistry("test"));
      comPlanner.setMaintainInitialCoMVelocityContinuity(false);
      comPlanner.setJerkMinimizationWeight(0.0);
      comPlanner.setAccelerationMinimizationWeight(0.0);

      List<ContactStateProvider> contactSequence = new ArrayList<>();
      SettableContactStateProvider contact0 = new SettableContactStateProvider();
      SettableContactStateProvider contact1 = new SettableContactStateProvider();
      SettableContactStateProvider contact2 = new SettableContactStateProvider();

      contact0.setStartECMPPosition(initialCMP);
      contact0.setEndECMPPosition(initialCMP);
      contact0.getTimeInterval().setInterval(0.0, firstDuration);
      contact0.setLinearECMPVelocity();

      contact1.setStartECMPPosition(initialCMP);
      contact1.setEndECMPPosition(finalCMP);
      contact1.getTimeInterval().setInterval(firstDuration, firstDuration + secondDuration);
      contact1.setLinearECMPVelocity();

      contact2.setStartECMPPosition(finalCMP);
      contact2.setEndECMPPosition(finalCMP);
      contact2.getTimeInterval().setInterval(firstDuration + secondDuration, firstDuration + secondDuration + thirdDuration);
      contact2.setLinearECMPVelocity();

      contactSequence.add(contact0);
      contactSequence.add(contact1);
      contactSequence.add(contact2);

      comPlanner.setInitialCenterOfMassState(startCoM, new FrameVector3D());
      comPlanner.solveForTrajectory(contactSequence);


      comPlanner.compute(0, 0.0);
      FramePoint3D com00 = new FramePoint3D(comPlanner.getDesiredCoMPosition());
      FramePoint3D vrp00 = new FramePoint3D(comPlanner.getDesiredVRPPosition());

      comPlanner.compute(0, firstDuration);
      FramePoint3D com01 = new FramePoint3D(comPlanner.getDesiredCoMPosition());
      FramePoint3D vrp01 = new FramePoint3D(comPlanner.getDesiredVRPPosition());

      comPlanner.compute(1, 0.0);
      FramePoint3D com10 = new FramePoint3D(comPlanner.getDesiredCoMPosition());
      FramePoint3D vrp10 = new FramePoint3D(comPlanner.getDesiredVRPPosition());

      comPlanner.compute(1, secondDuration);
      FramePoint3D com11 = new FramePoint3D(comPlanner.getDesiredCoMPosition());
      FramePoint3D vrp11 = new FramePoint3D(comPlanner.getDesiredVRPPosition());

      comPlanner.compute(2, 0.0);
      FramePoint3D com20 = new FramePoint3D(comPlanner.getDesiredCoMPosition());
      FramePoint3D vrp20 = new FramePoint3D(comPlanner.getDesiredVRPPosition());

      comPlanner.compute(2, thirdDuration);
      FramePoint3D vrp21 = new FramePoint3D(comPlanner.getDesiredVRPPosition());


      EuclidFrameTestTools.assertGeometricallyEquals(startCoM, com00, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(com01, com10, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(com11, com20, 1e-4);

      EuclidFrameTestTools.assertGeometricallyEquals(initialVRP, vrp00, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(initialVRP, vrp01, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(initialVRP, vrp10, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(finalVRP, vrp11, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(finalVRP, vrp20, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(finalVRP, vrp21, 1e-4);
   }

   @Test
   public void testOneJumpThreeSegments()
   {
      double firstDuration = 0.75;
      double secondDuration = 1.1;
      double thirdDuration = 0.4;
      double omega = 3.0;
      double gravity = 9.81;
      double nominalHeight = omega * omega / gravity;

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, -gravity);

      FramePoint3D initialVRP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.1, 0.75, 0.98);
      FramePoint3D finalVRP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.4, 0.92, 1.05);
      FramePoint3D initialCMP = new FramePoint3D(initialVRP);
      FramePoint3D finalCMP = new FramePoint3D(finalVRP);
      initialCMP.subZ(nominalHeight);
      finalCMP.subZ(nominalHeight);

      FramePoint3D startCoM = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.2, 0.8, 1.01);


      ModifiedOptimizedCoMTrajectoryPlanner comPlanner = new ModifiedOptimizedCoMTrajectoryPlanner(gravity, nominalHeight, new YoRegistry("test"));
      comPlanner.setMaintainInitialCoMVelocityContinuity(false);
      comPlanner.setJerkMinimizationWeight(0.0);
      comPlanner.setAccelerationMinimizationWeight(0.0);

      List<ContactStateProvider> contactSequence = new ArrayList<>();
      SettableContactStateProvider contact0 = new SettableContactStateProvider();
      SettableContactStateProvider contact1 = new SettableContactStateProvider();
      SettableContactStateProvider contact2 = new SettableContactStateProvider();

      contact0.setStartECMPPosition(initialCMP);
      contact0.setEndECMPPosition(initialCMP);
      contact0.getTimeInterval().setInterval(0.0, firstDuration);
      contact0.setLinearECMPVelocity();

      contact1.setContactState(ContactState.FLIGHT);
      contact1.getTimeInterval().setInterval(firstDuration, firstDuration + secondDuration);

      contact2.setStartECMPPosition(finalCMP);
      contact2.setEndECMPPosition(finalCMP);
      contact2.getTimeInterval().setInterval(firstDuration + secondDuration, firstDuration + secondDuration + thirdDuration);
      contact2.setLinearECMPVelocity();

      contactSequence.add(contact0);
      contactSequence.add(contact1);
      contactSequence.add(contact2);

      comPlanner.setInitialCenterOfMassState(startCoM, new FrameVector3D());
      comPlanner.solveForTrajectory(contactSequence);


      comPlanner.compute(0, 0.0);
      FramePoint3D com00 = new FramePoint3D(comPlanner.getDesiredCoMPosition());
      FramePoint3D vrp00 = new FramePoint3D(comPlanner.getDesiredVRPPosition());
      FrameVector3D vrpVelocity00 = new FrameVector3D(comPlanner.getDesiredVRPVelocity());

      comPlanner.compute(0, firstDuration);
      FramePoint3D com01 = new FramePoint3D(comPlanner.getDesiredCoMPosition());
      FramePoint3D vrp01 = new FramePoint3D(comPlanner.getDesiredVRPPosition());
      FrameVector3D vrpVelocity01 = new FrameVector3D(comPlanner.getDesiredVRPVelocity());

      comPlanner.compute(1, 0.0);
      FramePoint3D com10 = new FramePoint3D(comPlanner.getDesiredCoMPosition());
      FrameVector3D comAcceleration10 = new FrameVector3D(comPlanner.getDesiredCoMAcceleration());

      comPlanner.compute(1, secondDuration);
      FramePoint3D com11 = new FramePoint3D(comPlanner.getDesiredCoMPosition());
      FrameVector3D comAcceleration11 = new FrameVector3D(comPlanner.getDesiredCoMAcceleration());

      comPlanner.compute(2, 0.0);
      FramePoint3D com20 = new FramePoint3D(comPlanner.getDesiredCoMPosition());
      FramePoint3D vrp20 = new FramePoint3D(comPlanner.getDesiredVRPPosition());
      FrameVector3D vrpVelocity20 = new FrameVector3D(comPlanner.getDesiredVRPVelocity());

      comPlanner.compute(2, thirdDuration);
      FramePoint3D vrp21 = new FramePoint3D(comPlanner.getDesiredVRPPosition());
      FramePoint3D dcm21 = new FramePoint3D(comPlanner.getDesiredDCMPosition());
      FrameVector3D vrpVelocity21 = new FrameVector3D(comPlanner.getDesiredVRPVelocity());


      FrameVector3D zeroVector = new FrameVector3D();
      EuclidFrameTestTools.assertGeometricallyEquals(startCoM, com00, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(com01, com10, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(com11, com20, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(gravityVector, comAcceleration10, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(gravityVector, comAcceleration11, 1e-4);

      assertTrue(initialVRP.distanceXY(vrp00) < 1e-4);
      assertTrue(initialVRP.distanceXY(vrp01) < 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(zeroVector, vrpVelocity00, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(zeroVector, vrpVelocity01, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(finalVRP, vrp20, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(finalVRP, vrp21, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(finalVRP, dcm21, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(zeroVector, vrpVelocity20, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(zeroVector, vrpVelocity21, 1e-4);
   }

   @Test
   public void testSimpleOneStepTwoSegments()
   {
      double firstDuration = 0.75;
      double secondDuration = 1.1;
      double omega = 3.0;
      double gravity = 9.81;
      double nominalHeight = omega * omega / gravity;

      FramePoint3D initialVRP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.1, 0.75, 0.98);
      FramePoint3D finalVRP = new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.4, 0.92, 1.05);
      FramePoint3D startCoM = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.2, 0.8, 1.01);

      FramePoint3D initialCMP = new FramePoint3D(initialVRP);
      FramePoint3D finalCMP = new FramePoint3D(finalVRP);
      initialCMP.subZ(nominalHeight);
      finalCMP.subZ(nominalHeight);

      ModifiedOptimizedCoMTrajectoryPlanner comPlanner = new ModifiedOptimizedCoMTrajectoryPlanner(gravity, nominalHeight, new YoRegistry("test"));
      comPlanner.setMaintainInitialCoMVelocityContinuity(false);
      comPlanner.setJerkMinimizationWeight(0.0);
      comPlanner.setAccelerationMinimizationWeight(0.0);

      List<ContactStateProvider> contactSequence = new ArrayList<>();
      SettableContactStateProvider contact0 = new SettableContactStateProvider();
      SettableContactStateProvider contact1 = new SettableContactStateProvider();

      contact0.setStartECMPPosition(initialCMP);
      contact0.setEndECMPPosition(finalCMP);
      contact0.getTimeInterval().setInterval(0.0, firstDuration);
      contact0.setLinearECMPVelocity();

      contact1.setStartECMPPosition(finalCMP);
      contact1.setEndECMPPosition(finalCMP);
      contact1.getTimeInterval().setInterval(firstDuration, firstDuration + secondDuration);
      contact1.setLinearECMPVelocity();

      contactSequence.add(contact0);
      contactSequence.add(contact1);

      comPlanner.setInitialCenterOfMassState(startCoM, new FrameVector3D());
      comPlanner.solveForTrajectory(contactSequence);

      comPlanner.compute(0, 0.0);
      FramePoint3D com00 = new FramePoint3D(comPlanner.getDesiredCoMPosition());
      FramePoint3D vrp00 = new FramePoint3D(comPlanner.getDesiredVRPPosition());

      comPlanner.compute(0, firstDuration);
      FramePoint3D com01 = new FramePoint3D(comPlanner.getDesiredCoMPosition());
      FramePoint3D vrp01 = new FramePoint3D(comPlanner.getDesiredVRPPosition());

      comPlanner.compute(1, 0.0);
      FramePoint3D com10 = new FramePoint3D(comPlanner.getDesiredCoMPosition());
      FramePoint3D vrp10 = new FramePoint3D(comPlanner.getDesiredVRPPosition());

      comPlanner.compute(1, secondDuration);
      FramePoint3D vrp11 = new FramePoint3D(comPlanner.getDesiredVRPPosition());
      FramePoint3D finalDCM = new FramePoint3D(comPlanner.getDesiredVRPPosition());

      EuclidFrameTestTools.assertGeometricallyEquals(startCoM, com00, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(com01, com10, 1e-4);

      EuclidFrameTestTools.assertGeometricallyEquals(initialVRP, vrp00, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(finalVRP, vrp01, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(finalVRP, vrp10, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(finalVRP, vrp11, 1e-4);
      EuclidFrameTestTools.assertGeometricallyEquals(finalVRP, finalDCM, 1e-4);
   }

   @Test
   public void testSimpleInPlaceOneSegment()
   {
      double firstDuration = 0.75;
      double omega = 3.0;
      double gravity = 9.81;
      double nominalHeight = omega * omega / gravity;

      FramePoint3D vrp = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.1, 0.75, 0.98);
      FramePoint3D startCoM = new FramePoint3D(ReferenceFrame.getWorldFrame(), 1.2, 0.8, 1.01);

      FrameVector3D zeroVelocity = new FrameVector3D();

      ModifiedOptimizedCoMTrajectoryPlanner comPlanner = new ModifiedOptimizedCoMTrajectoryPlanner(gravity, nominalHeight, new YoRegistry("test"));
      comPlanner.setMaintainInitialCoMVelocityContinuity(false);
      comPlanner.setJerkMinimizationWeight(0.0);
      comPlanner.setAccelerationMinimizationWeight(0.0);

      List<ContactStateProvider> contactSequence = new ArrayList<>();
      SettableContactStateProvider contact0 = new SettableContactStateProvider();

      contact0.setStartECMPPosition(vrp);
      contact0.setEndECMPPosition(vrp);
      contact0.getTimeInterval().setInterval(0.0, firstDuration);
      contact0.setLinearECMPVelocity();

      contactSequence.add(contact0);

      comPlanner.setInitialCenterOfMassState(startCoM, new FrameVector3D());
      comPlanner.solveForTrajectory(contactSequence);

      // assert the constraints hold
      comPlanner.compute(0, 0.0);
      FramePoint3D com00 = new FramePoint3D(comPlanner.getDesiredCoMPosition());
      FramePoint3D vrp00 = new FramePoint3D(comPlanner.getDesiredVRPPosition());

      comPlanner.compute(0, firstDuration);
      FramePoint3D finalDCM = new FramePoint3D(comPlanner.getDesiredDCMPosition());
      FramePoint3D vrp01 = new FramePoint3D(comPlanner.getDesiredVRPPosition());

      EuclidFrameTestTools.assertGeometricallyEquals(startCoM, com00, 1e-4);

      assertTrue(vrp.distanceXY(vrp00) < 1e-4);
      assertTrue(vrp.distanceXY(vrp01) < 1e-4);
      assertTrue(vrp.distanceXY(finalDCM) < 1e-4);
   }
}
