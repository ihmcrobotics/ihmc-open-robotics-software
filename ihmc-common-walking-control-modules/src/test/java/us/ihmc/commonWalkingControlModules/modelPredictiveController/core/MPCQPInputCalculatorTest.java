package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ConstraintType;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPoint;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.core.MPCQPInputCalculator.sufficientlyLargeValue;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public class MPCQPInputCalculatorTest
{
   @Test
   public void testCoMObjectiveOneSegment()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);
      MPCContactPlane contactPlaneHelper = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      FramePose3D contactPose = new FramePose3D();

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      MPCQPInputCalculator inputCalculator = new MPCQPInputCalculator(indexHandler, gravityZ);

      indexHandler.initialize(contactProviders);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;
      QPInputTypeA comPositionQPInput = new QPInputTypeA(rhoCoefficients + comCoefficients);
      QPInputTypeA comVelocityQPInput = new QPInputTypeA(rhoCoefficients + comCoefficients);

      Random random = new Random(1738L);

      for (double time = 0.0; time < 2.0; time += 0.01)
      {
         helper.reshape(contact.getNumberOfContactPointsInPlane(0));
         helper.computeMatrices(time, omega);

         contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);

         rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-5, 1e-7, mu);

         DMatrixRMaj rhoMagnitudesJacobian = new DMatrixRMaj(rhoSize, rhoCoefficients);
         DMatrixRMaj rhoRatesJacobian = new DMatrixRMaj(rhoSize, rhoCoefficients);

         double c0 = Math.exp(omega * time);
         double c1 = Math.exp(-omega * time);
         double c2 = time * time * time;
         double c3 = time * time;

         double c0Dot = omega * Math.exp(omega * time);
         double c1Dot = -omega * Math.exp(-omega * time);
         double c2Dot = 3 * time * time;
         double c3Dot = 2 * time;

         for (int i = 0; i < 16; i++)
         {
            rhoMagnitudesJacobian.set(i, 4 * i, c0);
            rhoMagnitudesJacobian.set(i, 4 * i + 1, c1);
            rhoMagnitudesJacobian.set(i, 4 * i + 2, c2);
            rhoMagnitudesJacobian.set(i, 4 * i + 3, c3);

            rhoRatesJacobian.set(i, 4 * i, c0Dot);
            rhoRatesJacobian.set(i, 4 * i + 1, c1Dot);
            rhoRatesJacobian.set(i, 4 * i + 2, c2Dot);
            rhoRatesJacobian.set(i, 4 * i + 3, c3Dot);
         }

         DMatrixRMaj comPositionJacobian = new DMatrixRMaj(3, rhoCoefficients + comCoefficients);
         DMatrixRMaj comVelocityJacobian = new DMatrixRMaj(3, rhoCoefficients + comCoefficients);
         DMatrixRMaj rhoToPositionJacobian = new DMatrixRMaj(3, rhoCoefficients);
         DMatrixRMaj rhoToVelocityJacobian = new DMatrixRMaj(3, rhoCoefficients);
         CommonOps_DDRM.mult(rhoHelper.getLinearJacobianInWorldFrame(), rhoMagnitudesJacobian, rhoToPositionJacobian);
         CommonOps_DDRM.mult(rhoHelper.getLinearJacobianInWorldFrame(), rhoRatesJacobian, rhoToVelocityJacobian);

         MatrixTools.setMatrixBlock(comPositionJacobian, 0, 6, rhoToPositionJacobian, 0, 0, 3, rhoCoefficients, 1.0);
         MatrixTools.setMatrixBlock(comVelocityJacobian, 0, 6, rhoToVelocityJacobian, 0, 0, 3, rhoCoefficients, 1.0);
         CoMCoefficientJacobianCalculator.calculateCoMJacobian(0, time, comPositionJacobian, 0, 1.0);
         CoMCoefficientJacobianCalculator.calculateCoMJacobian(0, time, comVelocityJacobian, 1, 1.0);

         FramePoint3D comPositionObjective = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D comVelocityObjective = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         CoMPositionCommand comPositionCommand = new CoMPositionCommand();
         comPositionCommand.addContactPlaneHelper(contactPlaneHelper);
         comPositionCommand.setOmega(omega);
         comPositionCommand.setSegmentNumber(0);
         comPositionCommand.setTimeOfObjective(time);
         comPositionCommand.setObjective(comPositionObjective);

         CoMVelocityCommand comVelocityCommand = new CoMVelocityCommand();
         comVelocityCommand.addContactPlaneHelper(contactPlaneHelper);
         comVelocityCommand.setOmega(omega);
         comVelocityCommand.setSegmentNumber(0);
         comVelocityCommand.setTimeOfObjective(time);
         comVelocityCommand.setObjective(comVelocityObjective);

         inputCalculator.calculateValueObjective(comPositionQPInput, comPositionCommand);
         inputCalculator.calculateValueObjective(comVelocityQPInput, comVelocityCommand);

         EjmlUnitTests.assertEquals(comPositionJacobian, comPositionQPInput.getTaskJacobian(), 1e-4);
         EjmlUnitTests.assertEquals(comVelocityJacobian, comVelocityQPInput.getTaskJacobian(), 1e-4);

         DMatrixRMaj expectedPositionObjective = new DMatrixRMaj(3, 1);
         DMatrixRMaj expectedVelocityObjective = new DMatrixRMaj(3, 1);

         comPositionObjective.scaleAdd(-0.5 * time * time, gravityVector, comPositionObjective);
         comPositionObjective.get(expectedPositionObjective);

         comVelocityObjective.scaleAdd(-time, gravityVector, comVelocityObjective);
         comVelocityObjective.get(expectedVelocityObjective);

         EjmlUnitTests.assertEquals(expectedPositionObjective, comPositionQPInput.getTaskObjective(), 1e-4);
         EjmlUnitTests.assertEquals(expectedVelocityObjective, comVelocityQPInput.getTaskObjective(), 1e-4);
      }
   }


   @Test
   public void testCoMObjectiveTwoSegment()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);
      MPCContactPlane contactPlaneHelper = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      FramePose3D contactPose = new FramePose3D();
      contactPose.getPosition().set(0.3, 0.7, 0.0);

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      MPCQPInputCalculator inputCalculator = new MPCQPInputCalculator(indexHandler, gravityZ);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 2);

      int rhoSize = 16;
      QPInputTypeA comPositionQPInput = new QPInputTypeA(indexHandler.getTotalProblemSize());
      QPInputTypeA comVelocityQPInput = new QPInputTypeA(indexHandler.getTotalProblemSize());

      Random random = new Random(1738L);

      for (double time = 0.0; time < 2.0; time += 0.01)
      {
         helper.reshape(contactPolygon.getNumberOfVertices());
         helper.computeMatrices(time, omega);

         rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-5, 1e-7, mu);

         contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);

         DMatrixRMaj rhoMagnitudesJacobian = MPCTestHelper.getCoMPositionJacobian(time, omega, rhoHelper);
         DMatrixRMaj rhoRatesJacobian = MPCTestHelper.getCoMVelocityJacobian(time, omega, rhoHelper);

         DMatrixRMaj comPositionJacobian = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());
         DMatrixRMaj comVelocityJacobian = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());

//         MatrixTools.setMatrixBlock(comPositionJacobian, 0, 0, rhoMagnitudesJacobian, 0, 0, 3, 6 + 4 * rhoSize, 1.0);
//         MatrixTools.setMatrixBlock(comVelocityJacobian, 0, 0, rhoRatesJacobian, 0, 0, 3, 6 + 4 * rhoSize, 1.0);
         MatrixTools.setMatrixBlock(comPositionJacobian, 0, indexHandler.getComCoefficientStartIndex(1), rhoMagnitudesJacobian, 0, 0, 3, 6 + 4 * rhoSize, 1.0);
         MatrixTools.setMatrixBlock(comVelocityJacobian, 0, indexHandler.getComCoefficientStartIndex(1), rhoRatesJacobian, 0, 0, 3, 6 + 4 * rhoSize, 1.0);

         FramePoint3D comPositionObjective = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D comVelocityObjective = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         CoMPositionCommand comPositionCommand = new CoMPositionCommand();
         comPositionCommand.addContactPlaneHelper(contactPlaneHelper);
         comPositionCommand.setOmega(omega);
         comPositionCommand.setSegmentNumber(1);
         comPositionCommand.setTimeOfObjective(time);
         comPositionCommand.setObjective(comPositionObjective);

         CoMVelocityCommand comVelocityCommand = new CoMVelocityCommand();
         comVelocityCommand.addContactPlaneHelper(contactPlaneHelper);
         comVelocityCommand.setOmega(omega);
         comVelocityCommand.setSegmentNumber(1);
         comVelocityCommand.setTimeOfObjective(time);
         comVelocityCommand.setObjective(comVelocityObjective);

         inputCalculator.calculateValueObjective(comPositionQPInput, comPositionCommand);
         inputCalculator.calculateValueObjective(comVelocityQPInput, comVelocityCommand);

         EjmlUnitTests.assertEquals(comPositionJacobian, comPositionQPInput.getTaskJacobian(), 1e-4);
         EjmlUnitTests.assertEquals(comVelocityJacobian, comVelocityQPInput.getTaskJacobian(), 1e-4);

         DMatrixRMaj expectedPositionObjective = new DMatrixRMaj(3, 1);
         DMatrixRMaj expectedVelocityObjective = new DMatrixRMaj(3, 1);

         comPositionObjective.scaleAdd(-0.5 * time * time, gravityVector, comPositionObjective);
         comPositionObjective.get(expectedPositionObjective);

         comVelocityObjective.scaleAdd(-time, gravityVector, comVelocityObjective);
         comVelocityObjective.get(expectedVelocityObjective);

         EjmlUnitTests.assertEquals(expectedPositionObjective, comPositionQPInput.getTaskObjective(), 1e-4);
         EjmlUnitTests.assertEquals(expectedVelocityObjective, comVelocityQPInput.getTaskObjective(), 1e-4);
      }
   }

   

   @Test
   public void testDCMObjectiveOneSegment()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);
      MPCContactPlane contactPlaneHelper = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      FramePose3D contactPose = new FramePose3D();

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      MPCQPInputCalculator inputCalculator = new MPCQPInputCalculator(indexHandler, gravityZ);

      indexHandler.initialize(contactProviders);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;
      QPInputTypeA dcmPositionQPInput = new QPInputTypeA(rhoCoefficients + comCoefficients);
      QPInputTypeA dcmVelocityQPInput = new QPInputTypeA(rhoCoefficients + comCoefficients);

      Random random = new Random(1738L);

      for (double time = 0.0; time < 2.0; time += 0.01)
      {
         helper.reshape(contact.getNumberOfContactPointsInPlane(0));
         helper.computeMatrices(time, omega);

         rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-5, 1e-7, mu);

         contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);

         DMatrixRMaj rhoMagnitudesJacobian = new DMatrixRMaj(rhoSize, rhoCoefficients);
         DMatrixRMaj rhoRatesJacobian = new DMatrixRMaj(rhoSize, rhoCoefficients);

         double c0 = Math.exp(omega * time);
         double c1 = Math.exp(-omega * time);
         double c2 = time * time * time;
         double c3 = time * time;

         double c0Dot = omega * Math.exp(omega * time);
         double c1Dot = -omega * Math.exp(-omega * time);
         double c2Dot = 3 * time * time;
         double c3Dot = 2 * time;

         double c0Ddot = omega * omega * Math.exp(omega * time);
         double c1Ddot = omega * omega * Math.exp(-omega * time);
         double c2Ddot = 6 * time;
         double c3Ddot = 2;

         for (int i = 0; i < 16; i++)
         {
            rhoMagnitudesJacobian.set(i, 4 * i, c0 + 1.0 / omega * c0Dot);
            rhoMagnitudesJacobian.set(i, 4 * i + 1, c1 + 1.0 / omega * c1Dot);
            rhoMagnitudesJacobian.set(i, 4 * i + 2, c2 + 1.0 / omega * c2Dot);
            rhoMagnitudesJacobian.set(i, 4 * i + 3, c3 + 1.0 / omega * c3Dot);

            rhoRatesJacobian.set(i, 4 * i, c0Dot + 1.0 / omega * c0Ddot);
            rhoRatesJacobian.set(i, 4 * i + 1, c1Dot + 1.0 / omega * c1Ddot);
            rhoRatesJacobian.set(i, 4 * i + 2, c2Dot + 1.0 / omega * c2Ddot);
            rhoRatesJacobian.set(i, 4 * i + 3, c3Dot + 1.0 / omega * c3Ddot);
         }

         DMatrixRMaj dcmPositionJacobian = new DMatrixRMaj(3, rhoCoefficients + comCoefficients);
         DMatrixRMaj dcmVelocityJacobian = new DMatrixRMaj(3, rhoCoefficients + comCoefficients);
         DMatrixRMaj rhoToPositionJacobian = new DMatrixRMaj(3, rhoCoefficients);
         DMatrixRMaj rhoToVelocityJacobian = new DMatrixRMaj(3, rhoCoefficients);
         CommonOps_DDRM.mult(rhoHelper.getLinearJacobianInWorldFrame(), rhoMagnitudesJacobian, rhoToPositionJacobian);
         CommonOps_DDRM.mult(rhoHelper.getLinearJacobianInWorldFrame(), rhoRatesJacobian, rhoToVelocityJacobian);

         MatrixTools.setMatrixBlock(dcmPositionJacobian, 0, 6, rhoToPositionJacobian, 0, 0, 3, rhoCoefficients, 1.0);
         MatrixTools.setMatrixBlock(dcmVelocityJacobian, 0, 6, rhoToVelocityJacobian, 0, 0, 3, rhoCoefficients, 1.0);
         CoMCoefficientJacobianCalculator.calculateDCMJacobian(0, omega, time, dcmPositionJacobian, 0, 1.0);
         CoMCoefficientJacobianCalculator.calculateDCMJacobian(0, omega, time, dcmVelocityJacobian, 1, 1.0);

         FramePoint3D dcmPositionObjective = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D dcmVelocityObjective = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         DCMPositionCommand dcmPositionCommand = new DCMPositionCommand();
         dcmPositionCommand.addContactPlaneHelper(contactPlaneHelper);
         dcmPositionCommand.setOmega(omega);
         dcmPositionCommand.setSegmentNumber(0);
         dcmPositionCommand.setTimeOfObjective(time);
         dcmPositionCommand.setObjective(dcmPositionObjective);

         DCMVelocityCommand dcmVelocityCommand = new DCMVelocityCommand();
         dcmVelocityCommand.addContactPlaneHelper(contactPlaneHelper);
         dcmVelocityCommand.setOmega(omega);
         dcmVelocityCommand.setSegmentNumber(0);
         dcmVelocityCommand.setTimeOfObjective(time);
         dcmVelocityCommand.setObjective(dcmVelocityObjective);

         inputCalculator.calculateValueObjective(dcmPositionQPInput, dcmPositionCommand);
         inputCalculator.calculateValueObjective(dcmVelocityQPInput, dcmVelocityCommand);

         EjmlUnitTests.assertEquals(dcmPositionJacobian, dcmPositionQPInput.getTaskJacobian(), 1e-4);
         EjmlUnitTests.assertEquals(dcmVelocityJacobian, dcmVelocityQPInput.getTaskJacobian(), 1e-4);

         DMatrixRMaj expectedPositionObjective = new DMatrixRMaj(3, 1);
         DMatrixRMaj expectedVelocityObjective = new DMatrixRMaj(3, 1);

         dcmPositionObjective.scaleAdd(-0.5 * time * time, gravityVector, dcmPositionObjective);
         dcmPositionObjective.scaleAdd(-time / omega, gravityVector, dcmPositionObjective);
         dcmPositionObjective.get(expectedPositionObjective);

         dcmVelocityObjective.scaleAdd(-time, gravityVector, dcmVelocityObjective);
         dcmVelocityObjective.scaleAdd(-1.0 / omega, gravityVector, dcmVelocityObjective);
         dcmVelocityObjective.get(expectedVelocityObjective);

         EjmlUnitTests.assertEquals(expectedPositionObjective, dcmPositionQPInput.getTaskObjective(), 1e-4);
         EjmlUnitTests.assertEquals(expectedVelocityObjective, dcmVelocityQPInput.getTaskObjective(), 1e-4);
      }
   }

   @Test
   public void testVRPObjectiveOneSegment()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double mass = 1.5;
      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);
      MPCContactPlane contactPlaneHelper = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      FramePose3D contactPose = new FramePose3D();

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartECMPPosition(new FramePoint3D());
      contact.setEndECMPPosition(new FramePoint3D());

      contactProviders.add(contact);

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      MPCQPInputCalculator inputCalculator = new MPCQPInputCalculator(indexHandler, gravityZ);

      indexHandler.initialize(contactProviders);

      int rhoSize = 16;
      int rhoCoefficients = 4 * rhoSize;
      int comCoefficients = 6;
      QPInputTypeA vrpPositionQPInput = new QPInputTypeA(rhoCoefficients + comCoefficients);
      QPInputTypeA vrpVelocityQPInput = new QPInputTypeA(rhoCoefficients + comCoefficients);

      Random random = new Random(1738L);

      for (double time = 0.0; time < 2.0; time += 0.01)
      {
         helper.reshape(contact.getNumberOfContactPointsInPlane(0));
         helper.computeMatrices(time, omega);

         rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-5, 1e-7, mu);

         contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);

         DMatrixRMaj rhoMagnitudesJacobian = new DMatrixRMaj(rhoSize, rhoCoefficients);
         DMatrixRMaj rhoRatesJacobian = new DMatrixRMaj(rhoSize, rhoCoefficients);

         double c0 = Math.exp(omega * time);
         double c1 = Math.exp(-omega * time);
         double c2 = time * time * time;
         double c3 = time * time;

         double c0Dot = omega * Math.exp(omega * time);
         double c1Dot = -omega * Math.exp(-omega * time);
         double c2Dot = 3 * time * time;
         double c3Dot = 2 * time;

         double c0Ddot = omega * omega * Math.exp(omega * time);
         double c1Ddot = omega * omega * Math.exp(-omega * time);
         double c2Ddot = 6 * time;
         double c3Ddot = 2;

         double c0Dddot = omega * omega * omega * Math.exp(omega * time);
         double c1Dddot = -omega * omega * omega * Math.exp(-omega * time);
         double c2Dddot = 6;
         double c3Dddot = 0;

         double omega2 = omega * omega;
         for (int i = 0; i < 16; i++)
         {
            rhoMagnitudesJacobian.set(i, 4 * i, c0 - 1.0 / omega2 * c0Ddot);
            rhoMagnitudesJacobian.set(i, 4 * i + 1, c1 - 1.0 / omega2 * c1Ddot);
            rhoMagnitudesJacobian.set(i, 4 * i + 2, c2 - 1.0 / omega2 * c2Ddot);
            rhoMagnitudesJacobian.set(i, 4 * i + 3, c3 - 1.0 / omega2 * c3Ddot);

            rhoRatesJacobian.set(i, 4 * i, c0Dot - 1.0 / omega2 * c0Dddot);
            rhoRatesJacobian.set(i, 4 * i + 1, c1Dot - 1.0 / omega2 * c1Dddot);
            rhoRatesJacobian.set(i, 4 * i + 2, c2Dot - 1.0 / omega2 * c2Dddot);
            rhoRatesJacobian.set(i, 4 * i + 3, c3Dot - 1.0 / omega2 * c3Dddot);
         }

         DMatrixRMaj vrpPositionJacobian = new DMatrixRMaj(3, rhoCoefficients + comCoefficients);
         DMatrixRMaj vrpVelocityJacobian = new DMatrixRMaj(3, rhoCoefficients + comCoefficients);
         DMatrixRMaj rhoToPositionJacobian = new DMatrixRMaj(3, rhoCoefficients);
         DMatrixRMaj rhoToVelocityJacobian = new DMatrixRMaj(3, rhoCoefficients);
         CommonOps_DDRM.mult(rhoHelper.getLinearJacobianInWorldFrame(), rhoMagnitudesJacobian, rhoToPositionJacobian);
         CommonOps_DDRM.mult(rhoHelper.getLinearJacobianInWorldFrame(), rhoRatesJacobian, rhoToVelocityJacobian);

         MatrixTools.setMatrixBlock(vrpPositionJacobian, 0, 6, rhoToPositionJacobian, 0, 0, 3, rhoCoefficients, 1.0);
         MatrixTools.setMatrixBlock(vrpVelocityJacobian, 0, 6, rhoToVelocityJacobian, 0, 0, 3, rhoCoefficients, 1.0);
         CoMCoefficientJacobianCalculator.calculateVRPJacobian(0, omega, time, vrpPositionJacobian, 0, 1.0);
         CoMCoefficientJacobianCalculator.calculateVRPJacobian(0, omega, time, vrpVelocityJacobian, 1, 1.0);

         FramePoint3D vrpPositionObjective = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3D vrpVelocityObjective = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         VRPPositionCommand vrpPositionCommand = new VRPPositionCommand();
         vrpPositionCommand.addContactPlaneHelper(contactPlaneHelper);
         vrpPositionCommand.setOmega(omega);
         vrpPositionCommand.setSegmentNumber(0);
         vrpPositionCommand.setTimeOfObjective(time);
         vrpPositionCommand.setObjective(vrpPositionObjective);

         VRPVelocityCommand vrpVelocityCommand = new VRPVelocityCommand();
         vrpVelocityCommand.addContactPlaneHelper(contactPlaneHelper);
         vrpVelocityCommand.setOmega(omega);
         vrpVelocityCommand.setSegmentNumber(0);
         vrpVelocityCommand.setTimeOfObjective(time);
         vrpVelocityCommand.setObjective(vrpVelocityObjective);

         inputCalculator.calculateValueObjective(vrpPositionQPInput, vrpPositionCommand);
         inputCalculator.calculateValueObjective(vrpVelocityQPInput, vrpVelocityCommand);

         EjmlUnitTests.assertEquals(vrpPositionJacobian, vrpPositionQPInput.getTaskJacobian(), 1e-4);
         EjmlUnitTests.assertEquals(vrpVelocityJacobian, vrpVelocityQPInput.getTaskJacobian(), 1e-4);

         DMatrixRMaj expectedPositionObjective = new DMatrixRMaj(3, 1);
         DMatrixRMaj expectedVelocityObjective = new DMatrixRMaj(3, 1);

         vrpPositionObjective.scaleAdd(-0.5 * time * time, gravityVector, vrpPositionObjective);
         vrpPositionObjective.scaleAdd(1.0 / (omega * omega), gravityVector, vrpPositionObjective); // FIXME
         vrpPositionObjective.get(expectedPositionObjective);

         vrpVelocityObjective.scaleAdd(-time, gravityVector, vrpVelocityObjective);
         vrpVelocityObjective.get(expectedVelocityObjective);

         EjmlUnitTests.assertEquals(expectedPositionObjective, vrpPositionQPInput.getTaskObjective(), 1e-4);
         EjmlUnitTests.assertEquals(expectedVelocityObjective, vrpVelocityQPInput.getTaskObjective(), 1e-4);
      }
   }

   @Test
   public void testCalculateCompactCoMValueObjective()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);
      MPCContactPlane contactPlaneHelper = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D contactPose = new FramePose3D();
      contactPose.getPosition().set(0.3, 0.7, 0.0);

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      MPCQPInputCalculator inputCalculator = new MPCQPInputCalculator(indexHandler, gravityZ);
      LinearMPCQPSolver qpSolver1 = new LinearMPCQPSolver(indexHandler, 0.01, gravityZ, new YoRegistry("testRegistry1"));
      LinearMPCQPSolver qpSolver2 = new LinearMPCQPSolver(indexHandler, 0.01, gravityZ, new YoRegistry("testRegistry2"));

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 2);

      QPInputTypeA comPositionQPInput = new QPInputTypeA(indexHandler.getTotalProblemSize());
      QPInputTypeA comVelocityQPInput = new QPInputTypeA(indexHandler.getTotalProblemSize());
      QPInputTypeA comPositionQPInputCompact = new QPInputTypeA(indexHandler.getTotalProblemSize());
      QPInputTypeA comVelocityQPInputCompact = new QPInputTypeA(indexHandler.getTotalProblemSize());

      Random random = new Random(1738L);

      double dt = 0.05;
      for (ConstraintType constraintType : ConstraintType.values())
      {
         for (double time = 0.0; time < 2.0; time += dt)
         {
            helper.reshape(contactPolygon.getNumberOfVertices());
            helper.computeMatrices(time, omega);

            rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-5, 1e-7, mu);

            contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);

            FramePoint3D comPositionObjective = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
            FrameVector3D comVelocityObjective = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

            CoMPositionCommand comPositionCommand = new CoMPositionCommand();
            comPositionCommand.addContactPlaneHelper(contactPlaneHelper);
            comPositionCommand.setOmega(omega);
            comPositionCommand.setSegmentNumber(1);
            comPositionCommand.setTimeOfObjective(time);
            comPositionCommand.setObjective(comPositionObjective);
            comPositionCommand.setConstraintType(constraintType);

            CoMVelocityCommand comVelocityCommand = new CoMVelocityCommand();
            comVelocityCommand.addContactPlaneHelper(contactPlaneHelper);
            comVelocityCommand.setOmega(omega);
            comVelocityCommand.setSegmentNumber(1);
            comVelocityCommand.setTimeOfObjective(time);
            comVelocityCommand.setObjective(comVelocityObjective);
            comVelocityCommand.setConstraintType(constraintType);

            inputCalculator.calculateValueObjective(comPositionQPInput, comPositionCommand);
            inputCalculator.calculateValueObjective(comVelocityQPInput, comVelocityCommand);

            qpSolver1.initialize();
            qpSolver2.initialize();

            qpSolver1.addInput(comPositionQPInput);
            qpSolver1.addInput(comVelocityQPInput);

            int positionOffset = inputCalculator.calculateCompactValueObjective(comPositionQPInputCompact, comPositionCommand);
            int velocityOffset = inputCalculator.calculateCompactValueObjective(comVelocityQPInputCompact, comVelocityCommand);
            qpSolver2.addInput(comPositionQPInputCompact, positionOffset);
            qpSolver2.addInput(comVelocityQPInputCompact, velocityOffset);

            EjmlUnitTests.assertEquals(qpSolver1.solverInput_H, qpSolver2.solverInput_H, 1e-5);
            EjmlUnitTests.assertEquals(qpSolver1.solverInput_f, qpSolver2.solverInput_f, 1e-5);

            EjmlUnitTests.assertEquals(qpSolver1.solverInput_Aeq, qpSolver2.solverInput_Aeq, 1e-5);
            EjmlUnitTests.assertEquals(qpSolver1.solverInput_beq, qpSolver2.solverInput_beq, 1e-5);

            EjmlUnitTests.assertEquals(qpSolver1.solverInput_Ain, qpSolver2.solverInput_Ain, 1e-5);
            EjmlUnitTests.assertEquals(qpSolver1.solverInput_bin, qpSolver2.solverInput_bin, 1e-5);
         }
      }
   }

   @Test
   public void testCalculateCompactDCMValueObjective()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);
      MPCContactPlane contactPlaneHelper = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D contactPose = new FramePose3D();
      contactPose.getPosition().set(0.3, 0.7, 0.0);

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      MPCQPInputCalculator inputCalculator = new MPCQPInputCalculator(indexHandler, gravityZ);
      LinearMPCQPSolver qpSolver1 = new LinearMPCQPSolver(indexHandler, 0.01, gravityZ, new YoRegistry("testRegistry1"));
      LinearMPCQPSolver qpSolver2 = new LinearMPCQPSolver(indexHandler, 0.01, gravityZ, new YoRegistry("testRegistry2"));

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 2);

      QPInputTypeA dcmPositionQPInput = new QPInputTypeA(indexHandler.getTotalProblemSize());
      QPInputTypeA dcmVelocityQPInput = new QPInputTypeA(indexHandler.getTotalProblemSize());
      QPInputTypeA dcmPositionQPInputCompact = new QPInputTypeA(indexHandler.getTotalProblemSize());
      QPInputTypeA dcmVelocityQPInputCompact = new QPInputTypeA(indexHandler.getTotalProblemSize());

      Random random = new Random(1738L);

      for (ConstraintType constraintType : ConstraintType.values())
      {
         for (double time = 0.0; time < 2.0; time += 0.05)
         {
            helper.reshape(contactPolygon.getNumberOfVertices());
            helper.computeMatrices(time, omega);

            rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-5, 1e-7, mu);

            contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);

            FramePoint3D comPositionObjective = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
            FrameVector3D comVelocityObjective = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

            DCMPositionCommand dcmPositionCommand = new DCMPositionCommand();
            dcmPositionCommand.addContactPlaneHelper(contactPlaneHelper);
            dcmPositionCommand.setOmega(omega);
            dcmPositionCommand.setSegmentNumber(1);
            dcmPositionCommand.setTimeOfObjective(time);
            dcmPositionCommand.setObjective(comPositionObjective);
            dcmPositionCommand.setConstraintType(constraintType);

            DCMVelocityCommand dcmVelocityCommand = new DCMVelocityCommand();
            dcmVelocityCommand.addContactPlaneHelper(contactPlaneHelper);
            dcmVelocityCommand.setOmega(omega);
            dcmVelocityCommand.setSegmentNumber(1);
            dcmVelocityCommand.setTimeOfObjective(time);
            dcmVelocityCommand.setObjective(comVelocityObjective);
            dcmVelocityCommand.setConstraintType(constraintType);

            inputCalculator.calculateValueObjective(dcmPositionQPInput, dcmPositionCommand);
            inputCalculator.calculateValueObjective(dcmVelocityQPInput, dcmVelocityCommand);

            qpSolver1.initialize();
            qpSolver2.initialize();

            qpSolver1.addInput(dcmPositionQPInput);
            qpSolver1.addInput(dcmVelocityQPInput);

            int positionOffset = inputCalculator.calculateCompactValueObjective(dcmPositionQPInputCompact, dcmPositionCommand);
            int velocityOffset = inputCalculator.calculateCompactValueObjective(dcmVelocityQPInputCompact, dcmVelocityCommand);
            qpSolver2.addInput(dcmPositionQPInputCompact, positionOffset);
            qpSolver2.addInput(dcmVelocityQPInputCompact, velocityOffset);

            EjmlUnitTests.assertEquals(qpSolver1.solverInput_H, qpSolver2.solverInput_H, 1e-5);
            EjmlUnitTests.assertEquals(qpSolver1.solverInput_f, qpSolver2.solverInput_f, 1e-5);

            EjmlUnitTests.assertEquals(qpSolver1.solverInput_Aeq, qpSolver2.solverInput_Aeq, 1e-5);
            EjmlUnitTests.assertEquals(qpSolver1.solverInput_beq, qpSolver2.solverInput_beq, 1e-5);

            EjmlUnitTests.assertEquals(qpSolver1.solverInput_Ain, qpSolver2.solverInput_Ain, 1e-5);
            EjmlUnitTests.assertEquals(qpSolver1.solverInput_bin, qpSolver2.solverInput_bin, 1e-5);
         }
      }
   }

   @Test
   public void testCalculateCompactVRPValueObjective()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);
      MPCContactPlane contactPlaneHelper = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FramePose3D contactPose = new FramePose3D();
      contactPose.getPosition().set(0.3, 0.7, 0.0);

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(4);
      MPCQPInputCalculator inputCalculator = new MPCQPInputCalculator(indexHandler, gravityZ);
      LinearMPCQPSolver qpSolver1 = new LinearMPCQPSolver(indexHandler, 0.01, gravityZ, new YoRegistry("testRegistry1"));
      LinearMPCQPSolver qpSolver2 = new LinearMPCQPSolver(indexHandler, 0.01, gravityZ, new YoRegistry("testRegistry2"));

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 2);

      QPInputTypeA vrpPositionQPInput = new QPInputTypeA(indexHandler.getTotalProblemSize());
      QPInputTypeA vrpVelocityQPInput = new QPInputTypeA(indexHandler.getTotalProblemSize());
      QPInputTypeA vrpPositionQPInputCompact = new QPInputTypeA(indexHandler.getTotalProblemSize());
      QPInputTypeA vrpVelocityQPInputCompact = new QPInputTypeA(indexHandler.getTotalProblemSize());

      Random random = new Random(1738L);

      for (ConstraintType constraintType : ConstraintType.values())
      {
         for (double time = 0.0; time < 2.0; time += 0.05)
         {
            helper.reshape(contactPolygon.getNumberOfVertices());
            helper.computeMatrices(time, omega);

            rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-5, 1e-7, mu);

            contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);

            FramePoint3D comPositionObjective = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
            FrameVector3D comVelocityObjective = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

            VRPPositionCommand vrpPositionCommand = new VRPPositionCommand();
            vrpPositionCommand.addContactPlaneHelper(contactPlaneHelper);
            vrpPositionCommand.setOmega(omega);
            vrpPositionCommand.setSegmentNumber(1);
            vrpPositionCommand.setTimeOfObjective(time);
            vrpPositionCommand.setObjective(comPositionObjective);
            vrpPositionCommand.setConstraintType(constraintType);

            VRPVelocityCommand vrpVelocityCommand = new VRPVelocityCommand();
            vrpVelocityCommand.addContactPlaneHelper(contactPlaneHelper);
            vrpVelocityCommand.setOmega(omega);
            vrpVelocityCommand.setSegmentNumber(1);
            vrpVelocityCommand.setTimeOfObjective(time);
            vrpVelocityCommand.setObjective(comVelocityObjective);
            vrpVelocityCommand.setConstraintType(constraintType);

            inputCalculator.calculateValueObjective(vrpPositionQPInput, vrpPositionCommand);
            inputCalculator.calculateValueObjective(vrpVelocityQPInput, vrpVelocityCommand);

            qpSolver1.initialize();
            qpSolver2.initialize();

            qpSolver1.addInput(vrpPositionQPInput);
            qpSolver1.addInput(vrpVelocityQPInput);

            int positionOffset = inputCalculator.calculateCompactValueObjective(vrpPositionQPInputCompact, vrpPositionCommand);
            int velocityOffset = inputCalculator.calculateCompactValueObjective(vrpVelocityQPInputCompact, vrpVelocityCommand);
            qpSolver2.addInput(vrpPositionQPInputCompact, positionOffset);
            qpSolver2.addInput(vrpVelocityQPInputCompact, velocityOffset);

            EjmlUnitTests.assertEquals(qpSolver1.solverInput_H, qpSolver2.solverInput_H, 1e-5);
            EjmlUnitTests.assertEquals(qpSolver1.solverInput_f, qpSolver2.solverInput_f, 1e-5);

            EjmlUnitTests.assertEquals(qpSolver1.solverInput_Aeq, qpSolver2.solverInput_Aeq, 1e-5);
            EjmlUnitTests.assertEquals(qpSolver1.solverInput_beq, qpSolver2.solverInput_beq, 1e-5);

            EjmlUnitTests.assertEquals(qpSolver1.solverInput_Ain, qpSolver2.solverInput_Ain, 1e-5);
            EjmlUnitTests.assertEquals(qpSolver1.solverInput_bin, qpSolver2.solverInput_bin, 1e-5);
         }
      }
   }

   @Test
   public void testRhoMinimizationCommand()
   {
      int numberOfBasisVectorsPerContactPoint = 4;
      MPCContactPlane contactPlane = new MPCContactPlane(4, numberOfBasisVectorsPerContactPoint, new ZeroConeRotationCalculator());
      double mu = 0.8;

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();
      contactPlane.computeBasisVectors(contactPolygon, new FramePose3D(), mu);

      double omega = 3.0;
      double duration = 0.7;
      double goalValueForBasis = 0.2;

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(numberOfBasisVectorsPerContactPoint);
      indexHandler.initialize((i) -> 4, 1);

      RhoTrackingCommand rhoTrackingCommand = new RhoTrackingCommand();
      rhoTrackingCommand.setSegmentDuration(duration);
      rhoTrackingCommand.setOmega(omega);
      rhoTrackingCommand.setSegmentNumber(0);
      rhoTrackingCommand.setWeight(100.0);
      rhoTrackingCommand.setObjectiveValue(goalValueForBasis);
      rhoTrackingCommand.addContactPlaneHelper(contactPlane);

      LinearMPCQPSolver solver = new LinearMPCQPSolver(indexHandler, 1e-3, -9.81, new YoRegistry("registry"));

      solver.initialize();
      solver.setRhoCoefficientRegularizationWeight(0.0);
      solver.addValueRegularization();
      solver.submitRhoTrackingCommand(rhoTrackingCommand);
      assertTrue(solver.solve());

      contactPlane.computeContactForceCoefficientMatrix(solver.getSolution(), indexHandler.getRhoCoefficientStartIndex(0));

      for (double time = 0; time <= duration; time += 0.001)
      {
         contactPlane.computeContactForce(omega, time);

         double omega2 = omega * omega;
         double exponential = Math.min(Math.exp(omega * time), sufficientlyLargeValue);
         double a0 = omega2 * exponential;
         double a1 = omega2 / exponential;
         double a2 = 6.0 * time;
         double a3 = 2.0;

         for (int i = 0; i < contactPlane.getRhoSize(); i++)
         {
            DMatrixRMaj basisCoefficients = contactPlane.getBasisCoefficients(i);

            double rhoValue = a0 * basisCoefficients.get(0, 0);
            rhoValue += a1 * basisCoefficients.get(0, 1);
            rhoValue += a2 * basisCoefficients.get(0, 2);
            rhoValue += a3 * basisCoefficients.get(0, 3);

            assertEquals(goalValueForBasis, rhoValue, 1e-3);
            assertEquals(goalValueForBasis, contactPlane.getBasisMagnitude(i).length(), 1e-3);
         }
      }
   }

   @Test
   public void testRhoMinCommand()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double dt = 1e-3;

      int bases = 3;
      MPCContactPlane contactPlaneHelper1 = new MPCContactPlane(4, bases, new ZeroConeRotationCalculator());

      LinearMPCIndexHandler indexHandler = new LinearMPCIndexHandler(bases);
      LinearMPCQPSolver solver = new LinearMPCQPSolver(indexHandler, dt, gravityZ, new YoRegistry("test"));

      FramePose3D contactPose = new FramePose3D();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      contactPlaneHelper1.computeBasisVectors(contactPolygon, contactPose, mu);

      FramePoint3D startPosition = new FramePoint3D(contactPose.getPosition());
      startPosition.setZ(0.75);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 1);

      double firstSegmentDuration = 0.7;
      double minRho = 0.001;
      double maxGs = 2.5;
      double maxForce = maxGs * Math.abs(gravityZ);
      double maxRho = maxForce / (contactPolygon.getNumberOfVertices() * bases) / mu;

      RhoBoundCommand rhoMinBoundsSegment1 = new RhoBoundCommand();
      rhoMinBoundsSegment1.setOmega(omega);
      rhoMinBoundsSegment1.setSegmentDuration(firstSegmentDuration);
      rhoMinBoundsSegment1.setSegmentNumber(0);
      rhoMinBoundsSegment1.addContactPlane(contactPlaneHelper1, minRho);
      rhoMinBoundsSegment1.setConstraintType(ConstraintType.GEQ_INEQUALITY);

      double regularization = 1e-5;
      solver.setMaxNumberOfIterations(1000);
      solver.initialize();

      solver.submitMPCCommand(rhoMinBoundsSegment1);

      solver.setComCoefficientRegularizationWeight(regularization);
      solver.setRhoCoefficientRegularizationWeight(regularization);

      assertTrue(solver.solve());

      contactPlaneHelper1.computeContactForceCoefficientMatrix(solver.getSolution(), indexHandler.getRhoCoefficientStartIndex(0));

      double epsilon = 1e-2;

      double timeStep = 0.05;
      for (double time = 0.0; time <= (firstSegmentDuration + timeStep / 10.0); time += timeStep)
      {
         contactPlaneHelper1.computeContactForce(omega, time);
         for (int contactPointIdx = 0; contactPointIdx < contactPlaneHelper1.getNumberOfContactPoints(); contactPointIdx++)
         {
            MPCContactPoint contactPoint = contactPlaneHelper1.getContactPointHelper(contactPointIdx);
            for (int rhoIdx = 0; rhoIdx < contactPoint.getRhoSize(); rhoIdx++)
            {
               FrameVector3DReadOnly basis = contactPoint.getBasisMagnitude(rhoIdx);
               double force = basis.length();
               String minMessage = "Force 1 is " + force + " at time " + time + ", and is expected to be above " + minRho;
               String maxMessage = "Force 1 is " + force + " at time " + time + ", and is expected to be below " + maxRho;
               boolean minGood = force>= minRho - epsilon;
               boolean maxGood = force <= maxRho + epsilon;
               if (time == 0.0 || MathTools.epsilonEquals(time, firstSegmentDuration, timeStep / 10.0))
               {
                  assertTrue(minMessage, minGood);
                  //                  assertTrue(maxMessage, maxGood);
               }
               else
               {
                  if (!minGood)
                     LogTools.info(minMessage);
                  //                  if (!maxGood)
                  //                     LogTools.info(maxMessage);
               }
            }
         }
      }
   }
}
