package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.random.RandomGeometry;

public class ContinuousSimpleReactionDynamicsTest
{
   @Test
   public void testDynamics()
   {
      double mass = 10.0;
      double gravity = 9.81;

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      ContinuousSimpleReactionDynamics dynamics = new ContinuousSimpleReactionDynamics(mass, gravity);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);

      FrameVector3D desiredLinearAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame(), 2.5, 3.5, 4.5);
      FrameVector3D desiredAngularAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame(), 5.5, 6.5, 7.5);

      DMatrixRMaj currentState = new DMatrixRMaj(stateVectorSize / 2, 1);
      DMatrixRMaj currentControl = new DMatrixRMaj(controlVectorSize, 1);
      DMatrixRMaj constants = new DMatrixRMaj(constantVectorSize, 1);

      currentControl.set(x, 0, desiredLinearAcceleration.getX() * mass);
      currentControl.set(y, 0, desiredLinearAcceleration.getY() * mass);
      currentControl.set(z, 0, (desiredLinearAcceleration.getZ() + gravity) * mass);
      currentControl.set(thetaX, 0, desiredAngularAcceleration.getX() * inertia.getX());
      currentControl.set(thetaY, 0, desiredAngularAcceleration.getY() * inertia.getY());
      currentControl.set(thetaZ, 0, desiredAngularAcceleration.getZ() * inertia.getZ());

      DMatrixRMaj function = new DMatrixRMaj(currentState);
      DMatrixRMaj functionExpected = new DMatrixRMaj(currentState);

      dynamics.getDynamics(STANCE, currentState, currentControl, constants, function);

      functionExpected.set(x, desiredLinearAcceleration.getX());
      functionExpected.set(y, desiredLinearAcceleration.getY());
      functionExpected.set(z, desiredLinearAcceleration.getZ());
      functionExpected.set(thetaX, desiredAngularAcceleration.getX());
      functionExpected.set(thetaY, desiredAngularAcceleration.getY());
      functionExpected.set(thetaZ, desiredAngularAcceleration.getZ());

      MatrixTestTools.assertMatrixEquals(functionExpected, function, 1e-7);

      dynamics.getDynamics(FLIGHT, currentState, currentControl, constants, function);

      functionExpected.zero();
      functionExpected.set(z, 0, -gravity);
      MatrixTestTools.assertMatrixEquals(functionExpected, function, 1e-7);
   }

   @Test
   public void testDynamicsStateGradient()
   {
      double mass = 10.0;
      double gravity = 9.81;

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      ContinuousSimpleReactionDynamics dynamics = new ContinuousSimpleReactionDynamics(mass, gravity);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);

      Random random = new Random(1738L);
      DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DMatrixRMaj gradient = new DMatrixRMaj(stateVectorSize / 2, stateVectorSize);
      DMatrixRMaj gradientExpected = new DMatrixRMaj(stateVectorSize / 2, stateVectorSize);

      dynamics.getDynamicsStateGradient(STANCE, currentState, currentControl, constants, gradient);

      MatrixTestTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);

      dynamics.getDynamicsStateGradient(FLIGHT, currentState, currentControl, constants, gradient);

      MatrixTestTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }

   @Test
   public void testStateGradientNumericalDifferentiationStance()
   {
      double epsilon = 1e-9;
      double mass = 15.0;
      double gravityZ = 9.81;
      ContinuousSimpleReactionDynamics dynamics = new ContinuousSimpleReactionDynamics(mass, gravityZ);

      Random random = new Random(1738L);
      DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DMatrixRMaj gradient = new DMatrixRMaj(stateVectorSize / 2, stateVectorSize);
      DMatrixRMaj expectedGradient = new DMatrixRMaj(stateVectorSize / 2, stateVectorSize);

      DMatrixRMaj dynamicsUnmodified = new DMatrixRMaj(stateVectorSize / 2, 1);
      DMatrixRMaj dynamicsModified = new DMatrixRMaj(stateVectorSize / 2, 1);

      dynamics.getDynamics(SLIPState.STANCE, currentState, currentControl, constants, dynamicsUnmodified);

      dynamics.getDynamicsStateGradient(SLIPState.STANCE, currentState, currentControl, constants, gradient);

      for (int modifiedStateIndex = 0; modifiedStateIndex < stateVectorSize / 2; modifiedStateIndex++)
      {
         DMatrixRMaj currentStateModified = new DMatrixRMaj(currentState);
         currentStateModified.add(modifiedStateIndex, 0, epsilon);
         dynamics.getDynamics(SLIPState.STANCE, currentStateModified, currentControl, constants, dynamicsModified);

         for (int stateIndex = 0; stateIndex < stateVectorSize / 2; stateIndex++)
            expectedGradient.set(stateIndex, modifiedStateIndex, (dynamicsModified.get(stateIndex) - dynamicsUnmodified.get(stateIndex)) / epsilon);
      }

      MatrixTestTools.assertMatrixEquals(expectedGradient, gradient, 1e-2);
   }

   @Test
   public void testStateGradientNumericalDifferentiationFlight()
   {
      double epsilon = 1e-9;
      double mass = 15.0;
      double gravityZ = 9.81;
      ContinuousSimpleReactionDynamics dynamics = new ContinuousSimpleReactionDynamics(mass, gravityZ);

      Random random = new Random(1738L);
      DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DMatrixRMaj gradient = new DMatrixRMaj(stateVectorSize / 2, stateVectorSize);
      DMatrixRMaj expectedGradient = new DMatrixRMaj(stateVectorSize / 2, stateVectorSize);

      DMatrixRMaj dynamicsUnmodified = new DMatrixRMaj(stateVectorSize / 2, 1);
      DMatrixRMaj dynamicsModified = new DMatrixRMaj(stateVectorSize / 2, 1);

      dynamics.getDynamics(SLIPState.FLIGHT, currentState, currentControl, constants, dynamicsUnmodified);

      dynamics.getDynamicsStateGradient(SLIPState.FLIGHT, currentState, currentControl, constants, gradient);

      for (int modifiedStateIndex = 0; modifiedStateIndex < stateVectorSize / 2; modifiedStateIndex++)
      {
         DMatrixRMaj currentStateModified = new DMatrixRMaj(currentState);
         currentStateModified.add(modifiedStateIndex, 0, epsilon);
         dynamics.getDynamics(SLIPState.FLIGHT, currentStateModified, currentControl, constants, dynamicsModified);

         for (int stateIndex = 0; stateIndex < stateVectorSize / 2; stateIndex++)
            expectedGradient.set(stateIndex, modifiedStateIndex, (dynamicsModified.get(stateIndex) - dynamicsUnmodified.get(stateIndex)) / epsilon);
      }

      MatrixTestTools.assertMatrixEquals(expectedGradient, gradient, 1e-2);
   }

   @Test
   public void testControlGradientNumericalDifferentiationStance()
   {
      double epsilon = 1e-9;
      double mass = 15.0;
      double gravityZ = 9.81;
      ContinuousSimpleReactionDynamics dynamics = new ContinuousSimpleReactionDynamics(mass, gravityZ);

      Random random = new Random(1738L);
      DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DMatrixRMaj gradient = new DMatrixRMaj(stateVectorSize / 2, controlVectorSize);
      DMatrixRMaj expectedGradient = new DMatrixRMaj(stateVectorSize / 2, controlVectorSize);

      DMatrixRMaj dynamicsUnmodified = new DMatrixRMaj(stateVectorSize / 2, 1);
      DMatrixRMaj dynamicsModified = new DMatrixRMaj(stateVectorSize / 2, 1);

      dynamics.getDynamics(SLIPState.STANCE, currentState, currentControl, constants, dynamicsUnmodified);

      dynamics.getDynamicsControlGradient(SLIPState.STANCE, currentState, currentControl, constants, gradient);

      for (int modifiedStateIndex = 0; modifiedStateIndex < controlVectorSize; modifiedStateIndex++)
      {
         DMatrixRMaj currentControlModified = new DMatrixRMaj(currentControl);
         currentControlModified.add(modifiedStateIndex, 0, epsilon);
         dynamics.getDynamics(SLIPState.STANCE, currentState, currentControlModified, constants, dynamicsModified);

         for (int stateIndex = 0; stateIndex < stateVectorSize / 2; stateIndex++)
            expectedGradient.set(stateIndex, modifiedStateIndex, (dynamicsModified.get(stateIndex) - dynamicsUnmodified.get(stateIndex)) / epsilon);
      }

      MatrixTestTools.assertMatrixEquals(expectedGradient, gradient, 1e-2);
   }

   @Test
   public void testControlGradientNumericalDifferentiationFlight()
   {
      double epsilon = 1e-9;
      double mass = 15.0;
      double gravityZ = 9.81;
      ContinuousSimpleReactionDynamics dynamics = new ContinuousSimpleReactionDynamics(mass, gravityZ);

      Random random = new Random(1738L);
      DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DMatrixRMaj gradient = new DMatrixRMaj(stateVectorSize / 2, controlVectorSize);
      DMatrixRMaj expectedGradient = new DMatrixRMaj(stateVectorSize / 2, controlVectorSize);

      DMatrixRMaj dynamicsUnmodified = new DMatrixRMaj(stateVectorSize / 2, 1);
      DMatrixRMaj dynamicsModified = new DMatrixRMaj(stateVectorSize / 2, 1);

      dynamics.getDynamics(SLIPState.FLIGHT, currentState, currentControl, constants, dynamicsUnmodified);

      dynamics.getDynamicsControlGradient(SLIPState.FLIGHT, currentState, currentControl, constants, gradient);

      for (int modifiedStateIndex = 0; modifiedStateIndex < controlVectorSize; modifiedStateIndex++)
      {
         DMatrixRMaj currentControlModified = new DMatrixRMaj(currentControl);
         currentControlModified.add(modifiedStateIndex, 0, epsilon);
         dynamics.getDynamics(SLIPState.FLIGHT, currentState, currentControlModified, constants, dynamicsModified);

         for (int stateIndex = 0; stateIndex < stateVectorSize / 2; stateIndex++)
            expectedGradient.set(stateIndex, modifiedStateIndex, (dynamicsModified.get(stateIndex) - dynamicsUnmodified.get(stateIndex)) / epsilon);
      }

      MatrixTestTools.assertMatrixEquals(expectedGradient, gradient, 1e-2);
   }

   @Test
   public void testDynamicsControlGradient()
   {
      double mass = 10.0;
      double gravity = 9.81;

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      ContinuousSimpleReactionDynamics dynamics = new ContinuousSimpleReactionDynamics(mass, gravity);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);

      Random random = new Random(1738L);
      DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DMatrixRMaj gradient = new DMatrixRMaj(stateVectorSize / 2, controlVectorSize);
      DMatrixRMaj gradientExpected = new DMatrixRMaj(stateVectorSize / 2, controlVectorSize);

      dynamics.getDynamicsControlGradient(STANCE, currentState, currentControl, constants, gradient);

      gradientExpected.set(x, fx, 1.0 / mass);
      gradientExpected.set(y, fy, 1.0 / mass);
      gradientExpected.set(z, fz, 1.0 / mass);
      gradientExpected.set(thetaX, tauX, 1.0 / inertia.getX());
      gradientExpected.set(thetaY, tauY, 1.0 / inertia.getY());
      gradientExpected.set(thetaZ, tauZ, 1.0 / inertia.getZ());

      MatrixTestTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);

      dynamics.getDynamicsControlGradient(FLIGHT, currentState, currentControl, constants, gradient);

      gradientExpected.zero();
      MatrixTestTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }

   @Test
   public void testDynamicsStateGradientNumericalDifferentiationStance()
   {
      double mass = 11.0;
      double gravity = 9.81;
      double epsilon = 1e-7;

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      ContinuousSimpleReactionDynamics dynamics = new ContinuousSimpleReactionDynamics(mass, gravity);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);

      Random random = new Random(1738L);
      DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DMatrixRMaj gradient = new DMatrixRMaj(stateVectorSize / 2, stateVectorSize);
      DMatrixRMaj gradientExpected = new DMatrixRMaj(stateVectorSize / 2, stateVectorSize);

      dynamics.getDynamicsStateGradient(STANCE, currentState, currentControl, constants, gradient);

      DMatrixRMaj dynamicState = new DMatrixRMaj(stateVectorSize / 2, 1);
      dynamics.getDynamics(STANCE, currentState, currentControl, constants, dynamicState);

      DMatrixRMaj dynamicStateModified = new DMatrixRMaj(stateVectorSize / 2, 1);

      DMatrixRMaj currentStateModified = new DMatrixRMaj(currentState);
      currentStateModified.add(x, 0, epsilon);
      dynamics.getDynamics(STANCE, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, x, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(y, 0, epsilon);
      dynamics.getDynamics(STANCE, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, y, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(z, 0, epsilon);
      dynamics.getDynamics(STANCE, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, z, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(3, 0, epsilon);
      dynamics.getDynamics(STANCE, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, 3, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(4, 0, epsilon);
      dynamics.getDynamics(STANCE, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, 4, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(5, 0, epsilon);
      dynamics.getDynamics(STANCE, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, 5, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      MatrixTestTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }

   @Test
   public void testDynamicsStateGradientNumericalDifferentiationFlight()
   {
      double mass = 11.0;
      double gravity = 9.81;
      double epsilon = 1e-7;

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      ContinuousSimpleReactionDynamics dynamics = new ContinuousSimpleReactionDynamics(mass, gravity);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);

      Random random = new Random(1738L);
      DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DMatrixRMaj gradient = new DMatrixRMaj(stateVectorSize / 2, stateVectorSize);
      DMatrixRMaj gradientExpected = new DMatrixRMaj(stateVectorSize / 2, stateVectorSize);

      dynamics.getDynamicsStateGradient(FLIGHT, currentState, currentControl, constants, gradient);

      DMatrixRMaj dynamicState = new DMatrixRMaj(stateVectorSize / 2, 1);
      dynamics.getDynamics(FLIGHT, currentState, currentControl, constants, dynamicState);

      DMatrixRMaj dynamicStateModified = new DMatrixRMaj(stateVectorSize / 2, 1);

      DMatrixRMaj currentStateModified = new DMatrixRMaj(currentState);
      currentStateModified.add(x, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, x, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(y, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, y, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(z, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, z, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(3, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, 3, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(4, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, 4, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(5, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, 5, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      MatrixTestTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }

   @Test
   public void testDynamicsControlGradientNumericalDifferentiationStance()
   {
      double mass = 11.0;
      double gravity = 9.81;
      double epsilon = 1e-7;

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      ContinuousSimpleReactionDynamics dynamics = new ContinuousSimpleReactionDynamics(mass, gravity);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);

      Random random = new Random(1738L);
      DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DMatrixRMaj gradient = new DMatrixRMaj(stateVectorSize / 2, controlVectorSize);
      DMatrixRMaj gradientExpected = new DMatrixRMaj(stateVectorSize / 2, controlVectorSize);

      dynamics.getDynamicsControlGradient(STANCE, currentState, currentControl, constants, gradient);

      DMatrixRMaj dynamicState = new DMatrixRMaj(stateVectorSize / 2, 1);
      dynamics.getDynamics(STANCE, currentState, currentControl, constants, dynamicState);

      DMatrixRMaj dynamicStateModified = new DMatrixRMaj(stateVectorSize / 2, 1);

      DMatrixRMaj currentControlModified = new DMatrixRMaj(currentControl);
      currentControlModified.add(fx, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, fx, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(fy, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, fy, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(fz, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, fz, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauX, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, tauX, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauY, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, tauY, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauZ, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, tauZ, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(xF, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, xF, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(yF, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, yF, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(k, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, k, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      MatrixTestTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }

   @Test
   public void testDynamicsControlGradientNumericalDifferentiationFlight()
   {
      double mass = 11.0;
      double gravity = 9.81;
      double epsilon = 1e-7;

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      ContinuousSimpleReactionDynamics dynamics = new ContinuousSimpleReactionDynamics(mass, gravity);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);

      Random random = new Random(1738L);
      DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      DMatrixRMaj gradient = new DMatrixRMaj(stateVectorSize / 2, controlVectorSize);
      DMatrixRMaj gradientExpected = new DMatrixRMaj(stateVectorSize / 2, controlVectorSize);

      dynamics.getDynamicsControlGradient(FLIGHT, currentState, currentControl, constants, gradient);

      DMatrixRMaj dynamicState = new DMatrixRMaj(stateVectorSize / 2, 1);
      dynamics.getDynamics(FLIGHT, currentState, currentControl, constants, dynamicState);

      DMatrixRMaj dynamicStateModified = new DMatrixRMaj(stateVectorSize / 2, 1);

      DMatrixRMaj currentControlModified = new DMatrixRMaj(currentControl);
      currentControlModified.add(fx, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, fx, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(fy, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, fy, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(fz, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, fz, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauX, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, tauX, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauY, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, tauY, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauZ, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, tauZ, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(xF, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, xF, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(yF, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, yF, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(k, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps_DDRM.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, k, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      MatrixTestTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }
}
