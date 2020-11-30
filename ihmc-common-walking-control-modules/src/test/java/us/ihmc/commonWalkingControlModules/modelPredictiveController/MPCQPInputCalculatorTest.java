package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.QPInputTypeA;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.matrixlib.MatrixTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

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
      ContactPlaneHelper contactPlaneHelper = new ContactPlaneHelper(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      FramePose3D contactPose = new FramePose3D();

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartCopPosition(new FramePoint3D());
      contact.setEndCopPosition(new FramePoint3D());

      contactProviders.add(contact);

      MPCIndexHandler indexHandler = new MPCIndexHandler(4);
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
         contactPlaneHelper.computeJacobians(time, omega);

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
      ContactPlaneHelper contactPlaneHelper = new ContactPlaneHelper(4, 4, new ZeroConeRotationCalculator());

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      FramePose3D contactPose = new FramePose3D();
      contactPose.getPosition().set(0.3, 0.7, 0.0);

      MPCIndexHandler indexHandler = new MPCIndexHandler(4);
      MPCQPInputCalculator inputCalculator = new MPCQPInputCalculator(indexHandler, gravityZ);

      indexHandler.initialize(i -> contactPolygon.getNumberOfVertices(), 2);

      int rhoSize = 16;
      int rhoCoefficients = 2 * 4 * rhoSize;
      int comCoefficients = 2 * 6;
      QPInputTypeA comPositionQPInput = new QPInputTypeA(rhoCoefficients + comCoefficients);
      QPInputTypeA comVelocityQPInput = new QPInputTypeA(rhoCoefficients + comCoefficients);

      Random random = new Random(1738L);

      for (double time = 0.0; time < 2.0; time += 0.01)
      {
         helper.reshape(contactPolygon.getNumberOfVertices());
         helper.computeMatrices(time, omega);

         rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-5, 1e-7, mu);

         contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);
         contactPlaneHelper.computeJacobians(time, omega);

         DMatrixRMaj rhoMagnitudesJacobian = MPCTestHelper.getCoMPositionJacobian(time, omega, rhoHelper);
         DMatrixRMaj rhoRatesJacobian = MPCTestHelper.getCoMVelocityJacobian(time, omega, rhoHelper);


         DMatrixRMaj comPositionJacobian = new DMatrixRMaj(3, rhoCoefficients + comCoefficients);
         DMatrixRMaj comVelocityJacobian = new DMatrixRMaj(3, rhoCoefficients + comCoefficients);

         MatrixTools.setMatrixBlock(comPositionJacobian, 0, 6, rhoMagnitudesJacobian, 0, 0, 3, 6, 1.0);
         MatrixTools.setMatrixBlock(comVelocityJacobian, 0, 6, rhoRatesJacobian, 0, 0, 3, 6, 1.0);
         MatrixTools.setMatrixBlock(comPositionJacobian, 0, 12 + 4 * rhoSize, rhoMagnitudesJacobian, 0, 6, 3, 4 * rhoSize, 1.0);
         MatrixTools.setMatrixBlock(comVelocityJacobian, 0, 12 + 4 * rhoSize, rhoRatesJacobian, 0, 6, 3, 4 * rhoSize, 1.0);

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
      ContactPlaneHelper contactPlaneHelper = new ContactPlaneHelper(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      FramePose3D contactPose = new FramePose3D();

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartCopPosition(new FramePoint3D());
      contact.setEndCopPosition(new FramePoint3D());

      contactProviders.add(contact);

      MPCIndexHandler indexHandler = new MPCIndexHandler(4);
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
         contactPlaneHelper.computeJacobians(time, omega);

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
      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);
      ContactPlaneHelper contactPlaneHelper = new ContactPlaneHelper(4, 4, new ZeroConeRotationCalculator());

      List<ContactPlaneProvider> contactProviders = new ArrayList<>();
      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      FramePose3D contactPose = new FramePose3D();

      ContactPlaneProvider contact = new ContactPlaneProvider();
      contact.getTimeInterval().setInterval(0.0, 1.0);
      contact.addContact(contactPose, contactPolygon);
      contact.setStartCopPosition(new FramePoint3D());
      contact.setEndCopPosition(new FramePoint3D());

      contactProviders.add(contact);

      MPCIndexHandler indexHandler = new MPCIndexHandler(4);
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
         contactPlaneHelper.computeJacobians(time, omega);

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
}
