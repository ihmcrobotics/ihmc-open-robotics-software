package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.EjmlUnitTests;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.commands.OrientationDynamicsCommand;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.ZeroConeRotationCalculator;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.sql.Ref;
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

      MPCIndexHandler indexHandler = new MPCIndexHandler(4);
      SpatialInertia inertia = new SpatialInertia(ReferenceFrame.getWorldFrame(), ReferenceFrame.getWorldFrame());
      inertia.setMomentOfInertia(10.0, 10.0, 5.0);
      inertia.setMass(mass);

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
         command.setBodyInertia(inertia);

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

         int orientationStart = indexHandler.getOrientationCoefficientsStartIndex(0);
         OrientationCoefficientJacobianCalculator.calculateAngularVelocityJacobian(orientationStart, time, angularVelocityJacobian, 1.0);
         OrientationCoefficientJacobianCalculator.calculateAngularAccelerationJacobian(orientationStart, time, angularAccelerationJacobian, 1.0);

         CommonOps_DDRM.mult(rotationMatrixExpected, angularAccelerationJacobian, rotationJacobianExpected );
         CommonOps_DDRM.mult(rotationRateMatrixExpected, angularVelocityJacobian, rotationRateJacobianExpected );

         DMatrixRMaj orientationJacobianExpected = new DMatrixRMaj(3, indexHandler.getTotalProblemSize());
         CommonOps_DDRM.add(rotationJacobianExpected, rotationRateJacobianExpected, orientationJacobianExpected);

         EjmlUnitTests.assertEquals(rotationMatrixExpected, calculator.rotationMatrix, 1e-6);
         EjmlUnitTests.assertEquals(rotationRateMatrixExpected, calculator.rotationMatrixDot, 1e-6);

         EjmlUnitTests.assertEquals(angularVelocityJacobian, calculator.getRotationRateJacobian(), 1e-6);
         EjmlUnitTests.assertEquals(angularAccelerationJacobian, calculator.getRotationAccelerationJacobian(), 1e-6);
         EjmlUnitTests.assertEquals(orientationJacobianExpected, calculator.getOrientationJacobian(), 1e-6);

         fail("Need to add the angular torque thing still");

      }
   }
}
