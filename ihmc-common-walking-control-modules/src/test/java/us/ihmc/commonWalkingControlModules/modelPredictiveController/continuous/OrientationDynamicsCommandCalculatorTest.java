package us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.*;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationDynamicsCommand;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous.OrientationCoefficientJacobianCalculator;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.continuous.OrientationDynamicsCommandCalculator;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.matrixlib.NativeCommonOps;
import us.ihmc.mecano.spatial.SpatialInertia;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.fail;

public class OrientationDynamicsCommandCalculatorTest
{
   @Test
   public void testCompute()
   {
      double gravityZ = -9.81;
      double omega = 3.0;
      double mu = 0.8;
      double dt = 1e-3;
      double mass = 1.5;

      FrameVector3D gravityVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), 0.0, 0.0, gravityZ);

      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      CoefficientJacobianMatrixHelper helper = new CoefficientJacobianMatrixHelper(4, 4);
      ContactPlaneHelper contactPlaneHelper = new ContactPlaneHelper(4, 4, new ZeroConeRotationCalculator());

      FramePose3D contactPose = new FramePose3D();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-8, 1e-10, mu);
      contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);

      ContinuousMPCIndexHandler indexHandler = new ContinuousMPCIndexHandler(4);
      SpatialInertia spatialInertia = new SpatialInertia(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      spatialInertia.setMomentOfInertia(10.0, 10.0, 5.0);
      spatialInertia.setMass(mass);

      indexHandler.initialize((i) -> 4, 1);
      OrientationDynamicsCommandCalculator calculator = new OrientationDynamicsCommandCalculator(indexHandler, mass);

      Random random = new Random(1738L);
      for (int i = 0; i < 1000; i++)
      {
         Orientation3DReadOnly orientationEstimate = EuclidCoreRandomTools.nextOrientation3D(random);
         Vector3DReadOnly angularVelocityEstimate = EuclidCoreRandomTools.nextVector3D(random);
         Point3DReadOnly comPositionEstimate = EuclidCoreRandomTools.nextPoint3D(random);

         double time = RandomNumbers.nextDouble(random, 0.0, 3.0);

         OrientationDynamicsCommand command = new OrientationDynamicsCommand();
         command.setOrientationEstimate(orientationEstimate);
         command.setAngularVelocityEstimate(angularVelocityEstimate);
         command.setComPositionEstimate(comPositionEstimate);
         command.setTimeOfObjective(time);
         command.setOmega(omega);
         command.setWeight(10);
         command.setSegmentNumber(0);
         command.setBodyInertia(spatialInertia);
         command.addContactPlaneHelper(contactPlaneHelper);

         calculator.compute(command);

         double yaw = orientationEstimate.getYaw();
         double pitch = orientationEstimate.getPitch();
         double roll = orientationEstimate.getRoll();
         double yawRate = angularVelocityEstimate.getZ();
         double rollRate = angularVelocityEstimate.getX();
         double pitchRate = angularVelocityEstimate.getY();

         DMatrixRMaj rotationMatrixExpected = new DMatrixRMaj(3, 3);
         rotationMatrixExpected.set(0, 0, Math.cos(pitch) * Math.cos(yaw));
         rotationMatrixExpected.set(0, 1, -Math.sin(yaw));
         rotationMatrixExpected.set(1, 0, Math.cos(pitch) * Math.sin(yaw));
         rotationMatrixExpected.set(1, 1, Math.cos(yaw));
         rotationMatrixExpected.set(2, 2, 1);

         DMatrixRMaj rotationRateMatrixExpected = new DMatrixRMaj(3, 3);
         rotationRateMatrixExpected.set(0, 0, -pitchRate * Math.sin(pitch) * Math.cos(yaw) - yawRate * Math.cos(pitch) * Math.sin(yaw));
         rotationRateMatrixExpected.set(0, 1, -yawRate * Math.cos(yaw));
         rotationRateMatrixExpected.set(1, 0, -pitchRate * Math.sin(pitch) * Math.sin(yaw) + yawRate * Math.cos(pitch) * Math.cos(yaw));
         rotationRateMatrixExpected.set(1, 1, -yawRate * Math.sin(yaw));

         DMatrixRMaj rotationJacobianExpected = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());
         DMatrixRMaj rotationRateJacobianExpected = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());

         DMatrixRMaj angularVelocityJacobian = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());
         DMatrixRMaj angularAccelerationJacobian = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());

         int yawStart = indexHandler.getYawCoefficientsStartIndex(0);
         int pitchStart = indexHandler.getPitchCoefficientsStartIndex(0);
         int rollStart = indexHandler.getRollCoefficientsStartIndex(0);
         OrientationCoefficientJacobianCalculator.calculateAngularVelocityJacobian(yawStart, pitchStart, rollStart, omega, time, angularVelocityJacobian, 1.0);
         OrientationCoefficientJacobianCalculator.calculateAngularAccelerationJacobian(yawStart, pitchStart, rollStart, omega, time, angularAccelerationJacobian, 1.0);

         CommonOps_DDRM.mult(rotationMatrixExpected, angularAccelerationJacobian, rotationJacobianExpected );
         CommonOps_DDRM.mult(rotationRateMatrixExpected, angularVelocityJacobian, rotationRateJacobianExpected );

         DMatrixRMaj orientationJacobianExpected = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());
         CommonOps_DDRM.add(rotationJacobianExpected, rotationRateJacobianExpected, orientationJacobianExpected);

         EjmlUnitTests.assertEquals(rotationMatrixExpected, calculator.bodyVelocityToWorld, 1e-6);
         EjmlUnitTests.assertEquals(rotationRateMatrixExpected, calculator.bodyAccelerationToWorld, 1e-6);

         EjmlUnitTests.assertEquals(angularVelocityJacobian, calculator.getRotationRateJacobian(), 1e-6);
         EjmlUnitTests.assertEquals(angularAccelerationJacobian, calculator.getRotationAccelerationJacobian(), 1e-6);
         EjmlUnitTests.assertEquals(orientationJacobianExpected, calculator.getOrientationJacobian(), 1e-6);

         DMatrixRMaj inertiaInWorld = new DMatrixRMaj(3, 3);
         DMatrixRMaj inertia = new DMatrixRMaj(3, 3);
         DMatrixRMaj tempInertia = new DMatrixRMaj(3, 3);
         spatialInertia.getMomentOfInertia().get(inertia);
         CommonOps_DDRM.multTransB(inertia, rotationMatrixExpected, tempInertia);
         CommonOps_DDRM.mult(rotationMatrixExpected, tempInertia, inertiaInWorld);

         DMatrixRMaj inverseInertiaInWorld = new DMatrixRMaj(3, 3);
         NativeCommonOps.invert(inertiaInWorld, inverseInertiaInWorld);

         EjmlUnitTests.assertEquals(inertiaInWorld, calculator.inertiaInWorld, 1e-5);
         EjmlUnitTests.assertEquals(inverseInertiaInWorld, calculator.inertiaInWorldInverse, 1e-5);

         double a0 = omega * omega * Math.exp(omega * time);
         double a1 = omega * omega * Math.exp(-omega * time);
         double a2 = 6.0 * time;
         double a3 = 2.0;

         DMatrixRMaj torqueJacobian = new DMatrixRMaj(3, indexHandler.getRhoCoefficientsInSegment(0));
         for (int contactIdx = 0; contactIdx < contactPlaneHelper.getNumberOfContactPoints(); contactIdx++)
         {
            DMatrixRMaj forceJacobian = new DMatrixRMaj(3, indexHandler.getRhoCoefficientsInSegment(0));
            ContactPointHelper pointHelper = contactPlaneHelper.getContactPointHelper(contactIdx);
            DMatrixRMaj pointTorqueJacobian = new DMatrixRMaj(3, ContinuousMPCIndexHandler.coefficientsPerRho * pointHelper.getRhoSize());
            for (int rhoIdx = 0; rhoIdx < pointHelper.getRhoSize(); rhoIdx++)
            {
               int startIdx = ContinuousMPCIndexHandler.coefficientsPerRho * rhoIdx;
               FrameVector3DReadOnly basisVector = rhoHelper.getBasisVector(rhoIdx);
               basisVector.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

               pointTorqueJacobian.set(0, startIdx, rhoHelper.getBasisVector(rhoIdx).getX() * a0);
               pointTorqueJacobian.set(0, startIdx + 1, rhoHelper.getBasisVector(rhoIdx).getX() * a1);
               pointTorqueJacobian.set(0, startIdx + 2, rhoHelper.getBasisVector(rhoIdx).getX() * a2);
               pointTorqueJacobian.set(0, startIdx + 3, rhoHelper.getBasisVector(rhoIdx).getX() * a3);
               pointTorqueJacobian.set(1, startIdx, rhoHelper.getBasisVector(rhoIdx).getY() * a0);
               pointTorqueJacobian.set(1, startIdx + 1, rhoHelper.getBasisVector(rhoIdx).getY() * a1);
               pointTorqueJacobian.set(1, startIdx + 2, rhoHelper.getBasisVector(rhoIdx).getY() * a2);
               pointTorqueJacobian.set(1, startIdx + 3, rhoHelper.getBasisVector(rhoIdx).getY() * a3);
               pointTorqueJacobian.set(2, startIdx, rhoHelper.getBasisVector(rhoIdx).getZ() * a0);
               pointTorqueJacobian.set(2, startIdx + 1, rhoHelper.getBasisVector(rhoIdx).getZ() * a1);
               pointTorqueJacobian.set(2, startIdx + 2, rhoHelper.getBasisVector(rhoIdx).getZ() * a2);
               pointTorqueJacobian.set(2, startIdx + 3, rhoHelper.getBasisVector(rhoIdx).getZ() * a3);
            }

            EjmlUnitTests.assertEquals(pointTorqueJacobian, pointHelper.getLinearJacobian(2), 1e-5);

            int startIdx = ContinuousMPCIndexHandler.coefficientsPerRho * 4 * contactIdx;
            MatrixTools.setMatrixBlock(forceJacobian, 0, startIdx, pointTorqueJacobian, 0, 0, 3, pointTorqueJacobian.getNumCols(), 1.0);

            Vector3D vectorToPoint = new Vector3D();
            vectorToPoint.sub(pointHelper.getBasisVectorOrigin(), comPositionEstimate);

            DMatrixRMaj skewVector = new DMatrixRMaj(3, 3);
            skewVector.set(0, 1, -vectorToPoint.getZ());
            skewVector.set(0, 2, vectorToPoint.getY());
            skewVector.set(1, 0, vectorToPoint.getZ());
            skewVector.set(1, 2, -vectorToPoint.getX());
            skewVector.set(2, 0, -vectorToPoint.getY());
            skewVector.set(2, 1, vectorToPoint.getX());

            DMatrixRMaj altSkew = new DMatrixRMaj(3, 3);
            OrientationDynamicsCommandCalculator.convertToSkewSymmetric(vectorToPoint, altSkew);

            EjmlUnitTests.assertEquals(skewVector, altSkew, 1e-5);

            DMatrixRMaj tempForceJacobian = new DMatrixRMaj(3, indexHandler.getRhoCoefficientsInSegment(0));
            CommonOps_DDRM.mult(mass, skewVector, forceJacobian, tempForceJacobian);

            CommonOps_DDRM.multAdd(inverseInertiaInWorld, tempForceJacobian, torqueJacobian);
         }

         DMatrixRMaj fullTorqueJacobian = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());
         MatrixTools.setMatrixBlock(fullTorqueJacobian, 0, indexHandler.getRhoCoefficientStartIndex(0), torqueJacobian, 0, 0, 3, indexHandler.getRhoCoefficientsInSegment(0), 1.0);
         MatrixTestTools.assertMatrixEquals("i = " + i, fullTorqueJacobian, calculator.getTorqueJacobian(), 1e-5);

      }
   }

   @Test
   public void testTorqueJacobian()
   {
      double omega = 3.0;
      double mu = 0.8;
      double mass = 1.5;

      ContactStateMagnitudeToForceMatrixHelper rhoHelper = new ContactStateMagnitudeToForceMatrixHelper(4, 4, new ZeroConeRotationCalculator());
      ContactPlaneHelper contactPlaneHelper = new ContactPlaneHelper(4, 4, new ZeroConeRotationCalculator());

      FramePose3D contactPose = new FramePose3D();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      rhoHelper.computeMatrices(contactPolygon, contactPose, 1e-8, 1e-10, mu);
      contactPlaneHelper.computeBasisVectors(contactPolygon, contactPose, mu);

      ContinuousMPCIndexHandler indexHandler = new ContinuousMPCIndexHandler(4);
      SpatialInertia spatialInertia = new SpatialInertia(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      spatialInertia.setMomentOfInertia(10.0, 10.0, 5.0);
      spatialInertia.setMass(mass);

      indexHandler.initialize((i) -> 4, 1);
      OrientationDynamicsCommandCalculator calculator = new OrientationDynamicsCommandCalculator(indexHandler, mass);

      Random random = new Random(1738L);
      for (int i = 0; i < 1000; i++)
      {
         Orientation3DReadOnly orientationEstimate = EuclidCoreRandomTools.nextOrientation3D(random);
         Vector3DReadOnly angularVelocityEstimate = EuclidCoreRandomTools.nextVector3D(random);
         Point3DReadOnly comPositionEstimate = EuclidCoreRandomTools.nextPoint3D(random);

         double time = RandomNumbers.nextDouble(random, 0.0, 3.0);

         OrientationDynamicsCommand command = new OrientationDynamicsCommand();
         command.setOrientationEstimate(orientationEstimate);
         command.setAngularVelocityEstimate(angularVelocityEstimate);
         command.setComPositionEstimate(comPositionEstimate);
         command.setTimeOfObjective(time);
         command.setOmega(omega);
         command.setWeight(10);
         command.setSegmentNumber(0);
         command.setBodyInertia(spatialInertia);
         command.addContactPlaneHelper(contactPlaneHelper);

         calculator.compute(command);

         DMatrixRMaj rhoCoefficients = new DMatrixRMaj(indexHandler.getRhoCoefficientsInSegment(0), 1);
         DMatrixRMaj fullSolution = new DMatrixRMaj(indexHandler.getTotalProblemSize(), 1);

         MatrixTools.setMatrixBlock(fullSolution, indexHandler.getRhoCoefficientStartIndex(0), 0, rhoCoefficients, 0, 0, indexHandler.getRhoCoefficientsInSegment(0), 1, 1.0);

         DMatrixRMaj rhoValues = new DMatrixRMaj(contactPlaneHelper.getRhoSize(), 1);

         List<FrameVector3D> rhoForces = new ArrayList<>();

         FrameVector3D totalTorque = new FrameVector3D();

         double a0 = omega * omega * Math.exp(omega * time);
         double a1 = omega * omega * Math.exp(-omega * time);
         double a2 = 6.0 * time;
         double a3 = 2.0;

         DMatrixRMaj torqueJacobian = new DMatrixRMaj(3, indexHandler.getRhoCoefficientsInSegment(0));
         for (int contactIdx = 0; contactIdx < contactPlaneHelper.getNumberOfContactPoints(); contactIdx++)
         {
            ContactPointHelper pointHelper = contactPlaneHelper.getContactPointHelper(contactIdx);
            for (int rhoIdx = 0; rhoIdx < pointHelper.getRhoSize(); rhoIdx++)
            {
               int startIdx = ContinuousMPCIndexHandler.coefficientsPerRho * (contactIdx * 4 + rhoIdx);
               FrameVector3DReadOnly basisVector = rhoHelper.getBasisVector(rhoIdx);
               basisVector.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

               double rhoValue = a0 * rhoCoefficients.get(startIdx, 0);
               rhoValue += a1 * rhoCoefficients.get(startIdx + 1, 0);
               rhoValue += a2 * rhoCoefficients.get(startIdx + 2, 0);
               rhoValue += a3 * rhoCoefficients.get(startIdx + 3, 0);

               rhoValues.set(contactIdx * 4 + rhoIdx, 0, rhoValue);

               FrameVector3D rhoForce = new FrameVector3D(basisVector);
               rhoForce.changeFrame(ReferenceFrame.getWorldFrame());
               rhoForce.normalize();
               rhoForce.scale(rhoValue);

               rhoForces.add(rhoForce);

               FrameVector3D vectorToForce = new FrameVector3D();
               vectorToForce.sub(pointHelper.getBasisVectorOrigin(), comPositionEstimate);

               FrameVector3D rhoTorque = new FrameVector3D();
               rhoTorque.cross(vectorToForce, rhoForce);

               totalTorque.add(rhoTorque);
            }
         }

         DMatrixRMaj angularAccelerationExpected = new DMatrixRMaj(3, 1);
         DMatrixRMaj angularAccelerationFromJacobian = new DMatrixRMaj(3, 1);
         DMatrixRMaj totalTorqueVector = new DMatrixRMaj(3, 1);
         totalTorque.get(totalTorqueVector);

         CommonOps_DDRM.mult(calculator.inertiaInWorldInverse, totalTorqueVector, angularAccelerationExpected);
         CommonOps_DDRM.mult(calculator.getTorqueJacobian(), fullSolution, angularAccelerationFromJacobian);

         MatrixTestTools.assertMatrixEquals("i = " + i, angularAccelerationExpected, angularAccelerationFromJacobian, 1e-5);

      }
   }
}
