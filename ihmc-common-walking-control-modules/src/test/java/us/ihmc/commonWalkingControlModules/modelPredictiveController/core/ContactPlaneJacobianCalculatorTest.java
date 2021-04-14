package us.ihmc.commonWalkingControlModules.modelPredictiveController.core;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.ioHandling.MPCContactPlane;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

public class ContactPlaneJacobianCalculatorTest
{
   @Test
   public void testComputePointMagnitudeJacobian()
   {
      double omega = 3.0;
      double mu = 0.8;

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      FramePose3D contactPose = new FramePose3D();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      DMatrixRMaj jacobian = new DMatrixRMaj(contactPolygon.getNumberOfVertices() * 3, 4 * 4 * contactPolygon.getNumberOfVertices());

      double time = 0.75;
      ContactPlaneJacobianCalculator.computeContactPointMagnitudeJacobian(1.0, time, omega, 0, 0, contactPlane, jacobian);

      DMatrixRMaj jacobianExpected = new DMatrixRMaj(jacobian);
      jacobianExpected.zero();

      int rowStart = 0;
      int colStart = 0;
      double firstCoefficient = Math.exp(omega * time);
      double secondCoefficient = Math.exp(-omega * time);
      double thirdCoefficient = MathTools.pow(time, 3);
      double fourthCoefficient = MathTools.pow(time, 2);
      for (int contactPointIndex = 0; contactPointIndex < contactPolygon.getNumberOfVertices(); contactPointIndex++)
      {
         for (int basisVectorIndex = 0; basisVectorIndex < contactPlane.getContactPointHelper(contactPointIndex).getRhoSize(); basisVectorIndex++)
         {
            Vector3DReadOnly basis = contactPlane.getContactPointHelper(contactPointIndex).getBasisVector(basisVectorIndex);

            jacobianExpected.set(rowStart, colStart, firstCoefficient * basis.getX());
            jacobianExpected.set(rowStart + 1, colStart, firstCoefficient * basis.getY());
            jacobianExpected.set(rowStart + 2, colStart, firstCoefficient * basis.getZ());

            jacobianExpected.set(rowStart, colStart + 1, secondCoefficient * basis.getX());
            jacobianExpected.set(rowStart + 1, colStart + 1, secondCoefficient * basis.getY());
            jacobianExpected.set(rowStart + 2, colStart + 1, secondCoefficient * basis.getZ());

            jacobianExpected.set(rowStart, colStart + 2, thirdCoefficient * basis.getX());
            jacobianExpected.set(rowStart + 1, colStart + 2, thirdCoefficient * basis.getY());
            jacobianExpected.set(rowStart + 2, colStart + 2, thirdCoefficient * basis.getZ());

            jacobianExpected.set(rowStart, colStart + 3, fourthCoefficient * basis.getX());
            jacobianExpected.set(rowStart + 1, colStart + 3, fourthCoefficient * basis.getY());
            jacobianExpected.set(rowStart + 2, colStart + 3, fourthCoefficient * basis.getZ());
            colStart += 4;
         }
         rowStart += 3;
      }

      MatrixTestTools.assertMatrixEquals(jacobianExpected, jacobian, 1e-5);
   }

   @Test
   public void testComputeContactPointAccelerationJacobian()
   {
      double omega = 3.0;
      double mu = 0.8;

      MPCContactPlane contactPlane = new MPCContactPlane(4, 4, new ZeroConeRotationCalculator());

      FramePose3D contactPose = new FramePose3D();

      ConvexPolygon2DReadOnly contactPolygon = MPCTestHelper.createDefaultContact();

      contactPlane.computeBasisVectors(contactPolygon, contactPose, mu);

      DMatrixRMaj jacobian = new DMatrixRMaj(contactPolygon.getNumberOfVertices() * 3, 4 * 4 * contactPolygon.getNumberOfVertices());

      double time = 0.75;
      ContactPlaneJacobianCalculator.computeContactPointAccelerationJacobian(1.0, time, omega, 0, 0, contactPlane, jacobian);

      DMatrixRMaj jacobianExpected = new DMatrixRMaj(jacobian);
      jacobianExpected.zero();

      int rowStart = 0;
      int colStart = 0;
      double firstCoefficient = omega * omega * Math.exp(omega * time);
      double secondCoefficient = omega * omega * Math.exp(-omega * time);
      double thirdCoefficient = 6.0 * time;
      double fourthCoefficient = 2.0;
      for (int contactPointIndex = 0; contactPointIndex < contactPolygon.getNumberOfVertices(); contactPointIndex++)
      {
         for (int basisVectorIndex = 0; basisVectorIndex < contactPlane.getContactPointHelper(contactPointIndex).getRhoSize(); basisVectorIndex++)
         {
            Vector3DReadOnly basis = contactPlane.getContactPointHelper(contactPointIndex).getBasisVector(basisVectorIndex);

            jacobianExpected.set(rowStart, colStart, firstCoefficient * basis.getX());
            jacobianExpected.set(rowStart + 1, colStart, firstCoefficient * basis.getY());
            jacobianExpected.set(rowStart + 2, colStart, firstCoefficient * basis.getZ());

            jacobianExpected.set(rowStart, colStart + 1, secondCoefficient * basis.getX());
            jacobianExpected.set(rowStart + 1, colStart + 1, secondCoefficient * basis.getY());
            jacobianExpected.set(rowStart + 2, colStart + 1, secondCoefficient * basis.getZ());

            jacobianExpected.set(rowStart, colStart + 2, thirdCoefficient * basis.getX());
            jacobianExpected.set(rowStart + 1, colStart + 2, thirdCoefficient * basis.getY());
            jacobianExpected.set(rowStart + 2, colStart + 2, thirdCoefficient * basis.getZ());

            jacobianExpected.set(rowStart, colStart + 3, fourthCoefficient * basis.getX());
            jacobianExpected.set(rowStart + 1, colStart + 3, fourthCoefficient * basis.getY());
            jacobianExpected.set(rowStart + 2, colStart + 3, fourthCoefficient * basis.getZ());
            colStart += 4;
         }
         rowStart += 3;
      }

      MatrixTestTools.assertMatrixEquals(jacobianExpected, jacobian, 1e-5);

      Random random = new Random(1738L);
      for (int iter = 0; iter < 500; iter++)
      {
         time = RandomNumbers.nextDouble(random, 0.0, 5.0);
         int numberOfTrajectoryCoefficients = contactPlane.getCoefficientSize();
         DMatrixRMaj trajectoryCoefficients = new DMatrixRMaj(numberOfTrajectoryCoefficients, 1);
         trajectoryCoefficients.setData(RandomNumbers.nextDoubleArray(random, numberOfTrajectoryCoefficients, 10.0));

         contactPlane.computeContactForceCoefficientMatrix(trajectoryCoefficients,0);
         contactPlane.computeContactForce(omega, time);

         ContactPlaneJacobianCalculator.computeContactPointAccelerationJacobian(1.0, time, omega, 0, 0, contactPlane, jacobian);

         DMatrixRMaj constructedVector = new DMatrixRMaj(3 * contactPlane.getNumberOfContactPoints(), 1);
         DMatrixRMaj expectedVector = new DMatrixRMaj(3 * contactPlane.getNumberOfContactPoints(), 1);
         for (int i = 0; i < contactPlane.getNumberOfContactPoints(); i++)
            contactPlane.getContactPointHelper(i).getContactAcceleration().get(3 * i, expectedVector);

         CommonOps_DDRM.mult(jacobian, trajectoryCoefficients, constructedVector);

         MatrixTestTools.assertMatrixEquals(expectedVector, constructedVector, 1e-6);
      }
   }
}
