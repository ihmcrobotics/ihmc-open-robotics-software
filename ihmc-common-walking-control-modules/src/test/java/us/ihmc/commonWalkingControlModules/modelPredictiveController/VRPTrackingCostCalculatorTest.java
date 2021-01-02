package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.VRPTrackingCommand;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public class VRPTrackingCostCalculatorTest
{
   @Test
   public void testOneBasisVector()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double omega2 = omega * omega;
      double mu = 0.8;
      double dt = 1e-3;

      int numberOfBases = 1;

      ContactPlaneHelper contactPlaneHelper1 = new ContactPlaneHelper(4, numberOfBases, new ZeroConeRotationCalculator());

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(numberOfBases);
      VRPTrackingCostCalculator costCalculator = new VRPTrackingCostCalculator(indexHandler, gravityZ);

      FramePose3D contactPose1 = new FramePose3D();
      contactPose1.getPosition().set(0.0, 0.0, 0.0);

      FramePoint3D startVRP = new FramePoint3D();
      FramePoint3D endVRP = new FramePoint3D();
      startVRP.set(-0.05, -0.02, 0.9);
      endVRP.set(0.05, 0.02, 1.0);

      ConvexPolygon2D contactPolygon = new ConvexPolygon2D();
      contactPolygon.addVertex(0.0, 0.0);
      contactPolygon.update();

      contactPlaneHelper1.computeBasisVectors(contactPolygon, contactPose1, mu);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 1);

      double duration = 0.7;

      VRPTrackingCommand command = new VRPTrackingCommand();
      command.setSegmentDuration(duration);
      command.setSegmentNumber(0);
      command.setOmega(omega);
      command.setWeight(1.0);
      command.addContactPlaneHelper(contactPlaneHelper1);
      command.setStartVRP(startVRP);
      command.setEndVRP(endVRP);

      int problemSize = 6 + 4;
      DMatrixRMaj costHessian = new DMatrixRMaj(problemSize, problemSize);
      DMatrixRMaj costGradient = new DMatrixRMaj(problemSize, 1);

      costCalculator.calculateVRPTrackingObjective(costHessian, costGradient, command);

      DMatrixRMaj costHessianExpected = new DMatrixRMaj(problemSize, problemSize);
      DMatrixRMaj costGradientExpected = new DMatrixRMaj(problemSize, 1);


      costHessianExpected.set(0, 0, 1.0 / 3.0 * Math.pow(duration, 3.0));
      costHessianExpected.set(0, 1, 0.5 * duration * duration);
      costHessianExpected.set(1, 0, 0.5 * duration * duration);
      costHessianExpected.set(1, 1, duration);

      costHessianExpected.set(2, 2, 1.0 / 3.0 * Math.pow(duration, 3.0));
      costHessianExpected.set(2, 3, 0.5 * duration * duration);
      costHessianExpected.set(3, 2, 0.5 * duration * duration);
      costHessianExpected.set(3, 3, duration);

      costHessianExpected.set(4, 4, 1.0 / 3.0 * Math.pow(duration, 3.0));
      costHessianExpected.set(4, 5, 0.5 * duration * duration);
      costHessianExpected.set(5, 4, 0.5 * duration * duration);
      costHessianExpected.set(5, 5, duration);


      FrameVector3DReadOnly basisVector = contactPlaneHelper1.getContactPointHelper(0).getBasisVector(0);

      double c0a0 = 0.0;
      double c0a1 = 0.0;
      double c0a2 = basisVector.getX() * (1.0 / 5.0 * Math.pow(duration, 5.0) - 2.0 / omega2 * Math.pow(duration, 3.0));
      double c0a3 = basisVector.getX() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 1.0 / omega2 * Math.pow(duration, 2.0));

      double c1a0 = 0.0;
      double c1a1 = 0.0;
      double c1a2 = basisVector.getX() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 3.0 / omega2 * Math.pow(duration, 2.0));
      double c1a3 = basisVector.getX() * (1.0 / 3.0 * Math.pow(duration, 3.0) - 2.0 / omega2 * duration);

      double c2a0 = 0.0;
      double c2a1 = 0.0;
      double c2a2 = basisVector.getY() * (1.0 / 5.0 * Math.pow(duration, 5.0) - 2.0 / omega2 * Math.pow(duration, 3.0));
      double c2a3 = basisVector.getY() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 1.0 / omega2 * Math.pow(duration, 2.0));

      double c3a0 = 0.0;
      double c3a1 = 0.0;
      double c3a2 = basisVector.getY() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 3.0 / omega2 * Math.pow(duration, 2.0));
      double c3a3 = basisVector.getY() * (1.0 / 3.0 * Math.pow(duration, 3.0) - 2.0 / omega2 * duration);


      double c4a0 = 0.0;
      double c4a1 = 0.0;
      double c4a2 = basisVector.getZ() * (1.0 / 5.0 * Math.pow(duration, 5.0) - 2.0 / omega2 * Math.pow(duration, 3.0));
      double c4a3 = basisVector.getZ() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 1.0 / omega2 * Math.pow(duration, 2.0));

      double c5a0 = 0.0;
      double c5a1 = 0.0;
      double c5a2 = basisVector.getZ() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 3.0 / omega2 * Math.pow(duration, 2.0));
      double c5a3 = basisVector.getZ() * (1.0 / 3.0 * Math.pow(duration, 3.0) - 2.0 / omega2 * duration);


      double scalar = basisVector.dot(basisVector);
      double a2a2 =  (1.0 / 7.0 * Math.pow(duration, 7.0) - 12.0 / 5.0 / omega2 * Math.pow(duration, 5.0) + 12.0 / omega2 / omega2 * Math.pow(duration, 3.0));
      double a2a3 =  (1.0 / 6.0 * Math.pow(duration, 6.0) - 2.0 / omega2 * Math.pow(duration, 4.0) + 6.0 / omega2 / omega2 * Math.pow(duration, 2.0));
      double a3a3 =  (1.0 / 5.0 * Math.pow(duration, 5.0) - 4.0 / (3.0 * omega2) * Math.pow(duration, 3.0) + 4.0 / omega2 / omega2 * Math.pow(duration, 1.0));

      double ga2 = 1.0 / 12.0 * Math.pow(duration, 6.0) - 1.0 / omega2 * Math.pow(duration, 4.0) + 3.0 / (omega2 * omega2) * duration * duration;
      double ga3 = Math.pow(duration, 5.0) / 10.0 - 2.0 / 3.0 / omega2 * Math.pow(duration, 3.0) + 2.0 / (omega2 * omega2) * duration;

      costHessianExpected.set(0, 6, c0a0);
      costHessianExpected.set(0, 7, c0a1);
      costHessianExpected.set(0, 8, c0a2);
      costHessianExpected.set(0, 9, c0a3);
      costHessianExpected.set(6, 0, c0a0);
      costHessianExpected.set(7, 0, c0a1);
      costHessianExpected.set(8, 0, c0a2);
      costHessianExpected.set(9, 0, c0a3);

      costHessianExpected.set(1, 6, c1a0);
      costHessianExpected.set(1, 7, c1a1);
      costHessianExpected.set(1, 8, c1a2);
      costHessianExpected.set(1, 9, c1a3);
      costHessianExpected.set(6, 1, c1a0);
      costHessianExpected.set(7, 1, c1a1);
      costHessianExpected.set(8, 1, c1a2);
      costHessianExpected.set(9, 1, c1a3);

      costHessianExpected.set(2, 6, c2a0);
      costHessianExpected.set(2, 7, c2a1);
      costHessianExpected.set(2, 8, c2a2);
      costHessianExpected.set(2, 9, c2a3);
      costHessianExpected.set(6, 2, c2a0);
      costHessianExpected.set(7, 2, c2a1);
      costHessianExpected.set(8, 2, c2a2);
      costHessianExpected.set(9, 2, c2a3);

      costHessianExpected.set(3, 6, c3a0);
      costHessianExpected.set(3, 7, c3a1);
      costHessianExpected.set(3, 8, c3a2);
      costHessianExpected.set(3, 9, c3a3);
      costHessianExpected.set(6, 3, c3a0);
      costHessianExpected.set(7, 3, c3a1);
      costHessianExpected.set(8, 3, c3a2);
      costHessianExpected.set(9, 3, c3a3);

      costHessianExpected.set(4, 6, c4a0);
      costHessianExpected.set(4, 7, c4a1);
      costHessianExpected.set(4, 8, c4a2);
      costHessianExpected.set(4, 9, c4a3);
      costHessianExpected.set(6, 4, c4a0);
      costHessianExpected.set(7, 4, c4a1);
      costHessianExpected.set(8, 4, c4a2);
      costHessianExpected.set(9, 4, c4a3);

      costHessianExpected.set(5, 6, c5a0);
      costHessianExpected.set(5, 7, c5a1);
      costHessianExpected.set(5, 8, c5a2);
      costHessianExpected.set(5, 9, c5a3);
      costHessianExpected.set(6, 5, c5a0);
      costHessianExpected.set(7, 5, c5a1);
      costHessianExpected.set(8, 5, c5a2);
      costHessianExpected.set(9, 5, c5a3);

      costHessianExpected.set(8, 8, scalar * a2a2);
      costHessianExpected.set(8, 9, scalar * a2a3);
      costHessianExpected.set(9, 8, scalar * a2a3);
      costHessianExpected.set(9, 9, scalar * a3a3);

      EjmlUnitTests.assertEquals(costHessianExpected, costHessian, 1e-5);

      FrameVector3D vrpChange = new FrameVector3D();
      vrpChange.sub(endVRP, startVRP);

      double changeDotBasis = vrpChange.dot(basisVector);
      double startDotBasis = new FrameVector3D(startVRP).dot(basisVector);

      double deltaC0 = vrpChange.getX() * 1.0 / (3.0 * duration) * Math.pow(duration, 3.0) + 0.5 * startVRP.getX() * Math.pow(duration, 2.0);
      double deltaC1 = vrpChange.getX() * 1.0 / (2.0 * duration) * Math.pow(duration, 2.0) + startVRP.getX() * duration;

      double deltaC2 = vrpChange.getY() * 1.0 / (3.0 * duration) * Math.pow(duration, 3.0) + 0.5 * startVRP.getY() * Math.pow(duration, 2.0);
      double deltaC3 = vrpChange.getY() * 1.0 / (2.0 * duration) * Math.pow(duration, 2.0) + startVRP.getY() * duration;

      double deltaC4 = vrpChange.getZ() * 1.0 / (3.0 * duration) * Math.pow(duration, 3.0) + 0.5 * startVRP.getZ() * Math.pow(duration, 2.0);
      double deltaC5 = vrpChange.getZ() * 1.0 / (2.0 * duration) * Math.pow(duration, 2.0) + startVRP.getZ() * duration;

      double gc4 = (0.5 / 4.0 * Math.pow(duration, 4.0) - duration * duration / (2.0 * omega2)) * gravityZ;
      double gc5 = (0.5 / 3.0 * Math.pow(duration, 3.0) - duration / omega2) * gravityZ;

      double a2Change = changeDotBasis * (Math.pow(duration, 5.0) / (duration * 5.0) - 2.0 / (omega2 * duration) * Math.pow(duration, 3.0));
      double a2Start = startDotBasis * (Math.pow(duration, 4.0) / 4.0 - 3.0 / omega2 * Math.pow(duration, 2.0));

      double a3Change = changeDotBasis * (Math.pow(duration, 4.0) / (4.0 * duration) - Math.pow(duration, 2.0) / (omega2 * duration));
      double a3Start = startDotBasis * (Math.pow(duration, 3.0) / 3.0 - 2.0 / omega2 * duration);

      costGradientExpected.set(0, 0, -deltaC0);
      costGradientExpected.set(1, 0, -deltaC1);
      costGradientExpected.set(2, 0, -deltaC2);
      costGradientExpected.set(3, 0, -deltaC3);
      costGradientExpected.set(4, 0, gc4 - deltaC4);
      costGradientExpected.set(5, 0, gc5 - deltaC5);
      costGradientExpected.set(8, 0, -a2Change - a2Start + gravityZ * basisVector.getZ() * ga2);
      costGradientExpected.set(9, 0, -a3Change - a3Start + gravityZ * basisVector.getZ() * ga3) ;

      EjmlUnitTests.assertEquals(costGradientExpected, costGradient, 1e-5);
   }

   @Test
   public void testTwoBasisVectors()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double omega2 = omega * omega;
      double mu = 0.8;
      double dt = 1e-3;

      int numberOfBases = 2;

      ContactPlaneHelper contactPlaneHelper1 = new ContactPlaneHelper(4, numberOfBases, new ZeroConeRotationCalculator());

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(numberOfBases);
      VRPTrackingCostCalculator costCalculator = new VRPTrackingCostCalculator(indexHandler, gravityZ);

      FramePose3D contactPose1 = new FramePose3D();
      contactPose1.getPosition().set(0.0, 0.0, 0.0);

      FramePoint3D startVRP = new FramePoint3D();
      FramePoint3D endVRP = new FramePoint3D();
      startVRP.set(-0.05, -0.02, 0.9);
      endVRP.set(0.05, 0.02, 1.0);

      ConvexPolygon2D contactPolygon = new ConvexPolygon2D();
      contactPolygon.addVertex(0.0, 0.0);
      contactPolygon.update();

      contactPlaneHelper1.computeBasisVectors(contactPolygon, contactPose1, mu);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 1);

      double duration = 0.7;

      VRPTrackingCommand command = new VRPTrackingCommand();
      command.setSegmentDuration(duration);
      command.setSegmentNumber(0);
      command.setOmega(omega);
      command.setWeight(1.0);
      command.addContactPlaneHelper(contactPlaneHelper1);
      command.setStartVRP(startVRP);
      command.setEndVRP(endVRP);

      int problemSize = 6 + 2 * 4;
      DMatrixRMaj costHessian = new DMatrixRMaj(problemSize, problemSize);
      DMatrixRMaj costGradient = new DMatrixRMaj(problemSize, 1);

      costCalculator.calculateVRPTrackingObjective(costHessian, costGradient, command);

      DMatrixRMaj costHessianExpected = new DMatrixRMaj(problemSize, problemSize);
      DMatrixRMaj costGradientExpected = new DMatrixRMaj(problemSize, 1);


      costHessianExpected.set(0, 0, 1.0 / 3.0 * Math.pow(duration, 3.0));
      costHessianExpected.set(0, 1, 0.5 * duration * duration);
      costHessianExpected.set(1, 0, 0.5 * duration * duration);
      costHessianExpected.set(1, 1, duration);

      costHessianExpected.set(2, 2, 1.0 / 3.0 * Math.pow(duration, 3.0));
      costHessianExpected.set(2, 3, 0.5 * duration * duration);
      costHessianExpected.set(3, 2, 0.5 * duration * duration);
      costHessianExpected.set(3, 3, duration);

      costHessianExpected.set(4, 4, 1.0 / 3.0 * Math.pow(duration, 3.0));
      costHessianExpected.set(4, 5, 0.5 * duration * duration);
      costHessianExpected.set(5, 4, 0.5 * duration * duration);
      costHessianExpected.set(5, 5, duration);


      FrameVector3DReadOnly basisVector0 = contactPlaneHelper1.getContactPointHelper(0).getBasisVector(0);
      FrameVector3DReadOnly basisVector1 = contactPlaneHelper1.getContactPointHelper(0).getBasisVector(1);

      double c0a2 = basisVector0.getX() * (1.0 / 5.0 * Math.pow(duration, 5.0) - 2.0 / omega2 * Math.pow(duration, 3.0));
      double c0a3 = basisVector0.getX() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 1.0 / omega2 * Math.pow(duration, 2.0));

      double c0a6 = basisVector1.getX() * (1.0 / 5.0 * Math.pow(duration, 5.0) - 2.0 / omega2 * Math.pow(duration, 3.0));
      double c0a7 = basisVector1.getX() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 1.0 / omega2 * Math.pow(duration, 2.0));

      double c1a2 = basisVector0.getX() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 3.0 / omega2 * Math.pow(duration, 2.0));
      double c1a3 = basisVector0.getX() * (1.0 / 3.0 * Math.pow(duration, 3.0) - 2.0 / omega2 * duration);

      double c1a6 = basisVector1.getX() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 3.0 / omega2 * Math.pow(duration, 2.0));
      double c1a7 = basisVector1.getX() * (1.0 / 3.0 * Math.pow(duration, 3.0) - 2.0 / omega2 * duration);


      double c2a2 = basisVector0.getY() * (1.0 / 5.0 * Math.pow(duration, 5.0) - 2.0 / omega2 * Math.pow(duration, 3.0));
      double c2a3 = basisVector0.getY() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 1.0 / omega2 * Math.pow(duration, 2.0));

      double c3a2 = basisVector0.getY() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 3.0 / omega2 * Math.pow(duration, 2.0));
      double c3a3 = basisVector0.getY() * (1.0 / 3.0 * Math.pow(duration, 3.0) - 2.0 / omega2 * duration);

      double c2a6 = basisVector1.getY() * (1.0 / 5.0 * Math.pow(duration, 5.0) - 2.0 / omega2 * Math.pow(duration, 3.0));
      double c2a7 = basisVector1.getY() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 1.0 / omega2 * Math.pow(duration, 2.0));

      double c3a6 = basisVector1.getY() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 3.0 / omega2 * Math.pow(duration, 2.0));
      double c3a7 = basisVector1.getY() * (1.0 / 3.0 * Math.pow(duration, 3.0) - 2.0 / omega2 * duration);


      double c4a2 = basisVector0.getZ() * (1.0 / 5.0 * Math.pow(duration, 5.0) - 2.0 / omega2 * Math.pow(duration, 3.0));
      double c4a3 = basisVector0.getZ() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 1.0 / omega2 * Math.pow(duration, 2.0));

      double c5a2 = basisVector0.getZ() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 3.0 / omega2 * Math.pow(duration, 2.0));
      double c5a3 = basisVector0.getZ() * (1.0 / 3.0 * Math.pow(duration, 3.0) - 2.0 / omega2 * duration);

      double c4a6 = basisVector1.getZ() * (1.0 / 5.0 * Math.pow(duration, 5.0) - 2.0 / omega2 * Math.pow(duration, 3.0));
      double c4a7 = basisVector1.getZ() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 1.0 / omega2 * Math.pow(duration, 2.0));

      double c5a6 = basisVector1.getZ() * (1.0 / 4.0 * Math.pow(duration, 4.0) - 3.0 / omega2 * Math.pow(duration, 2.0));
      double c5a7 = basisVector1.getZ() * (1.0 / 3.0 * Math.pow(duration, 3.0) - 2.0 / omega2 * duration);


      double scalar00 = basisVector0.dot(basisVector0);
      double scalar01 = basisVector1.dot(basisVector0);
      double scalar11 = basisVector1.dot(basisVector1);

      double AA = (1.0 / 7.0 * Math.pow(duration, 7.0) - 12.0 / 5.0 / omega2 * Math.pow(duration, 5.0) + 12.0 / omega2 / omega2 * Math.pow(duration, 3.0));
      double AB = (1.0 / 6.0 * Math.pow(duration, 6.0) - 2.0 / omega2 * Math.pow(duration, 4.0) + 6.0 / omega2 / omega2 * Math.pow(duration, 2.0));
      double BB = (1.0 / 5.0 * Math.pow(duration, 5.0) - 4.0 / (3.0 * omega2) * Math.pow(duration, 3.0) + 4.0 / omega2 / omega2 * Math.pow(duration, 1.0));

      double ga2 = 1.0 / 12.0 * Math.pow(duration, 6.0) - 1.0 / omega2 * Math.pow(duration, 4.0) + 3.0 / (omega2 * omega2) * duration * duration;
      double ga3 = Math.pow(duration, 5.0) / 10.0 - 2.0 / 3.0 / omega2 * Math.pow(duration, 3.0) + 2.0 / (omega2 * omega2) * duration;

      double a2a2 = scalar00 * AA;
      double a2a3 = scalar00 * AB;
      double a2a4 = scalar01 * AA;
      double a2a5 = scalar01 * AB;

      double a3a2 = scalar00 * AB;
      double a3a3 = scalar00 * BB;
      double a3a4 = scalar01 * AB;
      double a3a5 = scalar01 * BB;

      double a4a2 = scalar01 * AA;
      double a4a3 = scalar01 * AB;
      double a4a4 = scalar11 * AA;
      double a4a5 = scalar11 * AB;

      double a5a2 = scalar01 * AB;
      double a5a3 = scalar01 * BB;
      double a5a4 = scalar11 * AB;
      double a5a5 = scalar11 * BB;

      costHessianExpected.set(0, 8, c0a2);
      costHessianExpected.set(0, 9, c0a3);
      costHessianExpected.set(0, 12, c0a6);
      costHessianExpected.set(0, 13, c0a7);
      costHessianExpected.set(8, 0, c0a2);
      costHessianExpected.set(9, 0, c0a3);
      costHessianExpected.set(12, 0, c0a6);
      costHessianExpected.set(13, 0, c0a7);

      costHessianExpected.set(1, 8, c1a2);
      costHessianExpected.set(1, 9, c1a3);
      costHessianExpected.set(1, 12, c1a6);
      costHessianExpected.set(1, 13, c1a7);
      costHessianExpected.set(8, 1, c1a2);
      costHessianExpected.set(9, 1, c1a3);
      costHessianExpected.set(12, 1, c1a6);
      costHessianExpected.set(13, 1, c1a7);

      costHessianExpected.set(2, 8, c2a2);
      costHessianExpected.set(2, 9, c2a3);
      costHessianExpected.set(2, 12, c2a6);
      costHessianExpected.set(2, 13, c2a7);
      costHessianExpected.set(8, 2, c2a2);
      costHessianExpected.set(9, 2, c2a3);
      costHessianExpected.set(12, 2, c2a6);
      costHessianExpected.set(13, 2, c2a7);

      costHessianExpected.set(3, 8, c3a2);
      costHessianExpected.set(3, 9, c3a3);
      costHessianExpected.set(3, 12, c3a6);
      costHessianExpected.set(3, 13, c3a7);
      costHessianExpected.set(8, 3, c3a2);
      costHessianExpected.set(9, 3, c3a3);
      costHessianExpected.set(12, 3, c3a6);
      costHessianExpected.set(13, 3, c3a7);

      costHessianExpected.set(4, 8, c4a2);
      costHessianExpected.set(4, 9, c4a3);
      costHessianExpected.set(4, 12, c4a6);
      costHessianExpected.set(4, 13, c4a7);
      costHessianExpected.set(8, 4, c4a2);
      costHessianExpected.set(9, 4, c4a3);
      costHessianExpected.set(12, 4, c4a6);
      costHessianExpected.set(13, 4, c4a7);

      costHessianExpected.set(5, 8, c5a2);
      costHessianExpected.set(5, 9, c5a3);
      costHessianExpected.set(5, 12, c5a6);
      costHessianExpected.set(5, 13, c5a7);
      costHessianExpected.set(8, 5, c5a2);
      costHessianExpected.set(9, 5, c5a3);
      costHessianExpected.set(12, 5, c5a6);
      costHessianExpected.set(13, 5, c5a7);

      costHessianExpected.set(8, 8, a2a2);
      costHessianExpected.set(8, 9, a2a3);
      costHessianExpected.set(8, 12, a2a4);
      costHessianExpected.set(8, 13, a2a5);

      costHessianExpected.set(9, 8, a3a2);
      costHessianExpected.set(9, 9, a3a3);
      costHessianExpected.set(9, 12, a3a4);
      costHessianExpected.set(9, 13, a3a5);

      costHessianExpected.set(12, 8, a4a2);
      costHessianExpected.set(12, 9, a4a3);
      costHessianExpected.set(12, 12, a4a4);
      costHessianExpected.set(12, 13, a4a5);

      costHessianExpected.set(13, 8, a5a2);
      costHessianExpected.set(13, 9, a5a3);
      costHessianExpected.set(13, 12, a5a4);
      costHessianExpected.set(13, 13, a5a5);

      EjmlUnitTests.assertEquals(costHessianExpected, costHessian, 1e-5);

      FrameVector3D vrpChange = new FrameVector3D();
      vrpChange.sub(endVRP, startVRP);

      double deltaC0 = vrpChange.getX() * 1.0 / (3.0 * duration) * Math.pow(duration, 3.0) + 0.5 * startVRP.getX() * Math.pow(duration, 2.0);
      double deltaC1 = vrpChange.getX() * 1.0 / (2.0 * duration) * Math.pow(duration, 2.0) + startVRP.getX() * duration;

      double deltaC2 = vrpChange.getY() * 1.0 / (3.0 * duration) * Math.pow(duration, 3.0) + 0.5 * startVRP.getY() * Math.pow(duration, 2.0);
      double deltaC3 = vrpChange.getY() * 1.0 / (2.0 * duration) * Math.pow(duration, 2.0) + startVRP.getY() * duration;

      double deltaC4 = vrpChange.getZ() * 1.0 / (3.0 * duration) * Math.pow(duration, 3.0) + 0.5 * startVRP.getZ() * Math.pow(duration, 2.0);
      double deltaC5 = vrpChange.getZ() * 1.0 / (2.0 * duration) * Math.pow(duration, 2.0) + startVRP.getZ() * duration;

      double gc4 = (0.5 / 4.0 * Math.pow(duration, 4.0) - duration * duration / (2.0 * omega2)) * gravityZ;
      double gc5 = (0.5 / 3.0 * Math.pow(duration, 3.0) - duration / omega2) * gravityZ;


      double changeDotBasis0 = vrpChange.dot(basisVector0);
      double changeDotBasis1 = vrpChange.dot(basisVector1);
      double startDotBasis0 = new FrameVector3D(startVRP).dot(basisVector0);
      double startDotBasis1 = new FrameVector3D(startVRP).dot(basisVector1);

      double a2Change = changeDotBasis0 * (Math.pow(duration, 5.0) / (duration * 5.0) - 2.0 / (omega2 * duration) * Math.pow(duration, 3.0));
      double a2Start = startDotBasis0 * (Math.pow(duration, 4.0) / 4.0 - 3.0 / omega2 * Math.pow(duration, 2.0));

      double a3Change = changeDotBasis0 * (Math.pow(duration, 4.0) / (4.0 * duration) - Math.pow(duration, 2.0) / (omega2 * duration));
      double a3Start = startDotBasis0 * (Math.pow(duration, 3.0) / 3.0 - 2.0 / omega2 * duration);

      double a6Change = a2Change / changeDotBasis0 * changeDotBasis1;
      double a6Start = a2Start / startDotBasis0 * startDotBasis1;

      double a7Change = a3Change / changeDotBasis0 * changeDotBasis1;
      double a7Start = a3Start / startDotBasis0 * startDotBasis1;

      costGradientExpected.set(0, 0, -deltaC0);
      costGradientExpected.set(1, 0, -deltaC1);
      costGradientExpected.set(2, 0, -deltaC2);
      costGradientExpected.set(3, 0, -deltaC3);
      costGradientExpected.set(4, 0, gc4 - deltaC4);
      costGradientExpected.set(5, 0, gc5 - deltaC5);

      costGradientExpected.set(8, 0, -a2Change - a2Start + gravityZ * basisVector0.getZ() * ga2);
      costGradientExpected.set(9, 0, -a3Change - a3Start + gravityZ * basisVector0.getZ() * ga3);
      costGradientExpected.set(12, 0, -a6Change - a6Start + gravityZ * basisVector1.getZ() * ga2);
      costGradientExpected.set(13, 0, -a7Change - a7Start + gravityZ * basisVector1.getZ() * ga3);

      EjmlUnitTests.assertEquals(costGradientExpected, costGradient, 1e-5);
   }
}
