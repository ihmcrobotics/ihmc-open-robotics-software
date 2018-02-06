package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.testing.JUnitTools;

import java.util.Random;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class ContinuousSimpleReactionDynamicsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
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

      DenseMatrix64F currentState = new DenseMatrix64F(stateVectorSize / 2, 1);
      DenseMatrix64F currentControl = new DenseMatrix64F(controlVectorSize, 1);
      DenseMatrix64F constants = new DenseMatrix64F(constantVectorSize, 1);

      currentControl.set(x, 0, desiredLinearAcceleration.getX() * mass);
      currentControl.set(y, 0, desiredLinearAcceleration.getY() * mass);
      currentControl.set(z, 0, (desiredLinearAcceleration.getZ() + gravity) * mass);
      currentControl.set(thetaX, 0, desiredAngularAcceleration.getX() * inertia.getX());
      currentControl.set(thetaY, 0, desiredAngularAcceleration.getY() * inertia.getY());
      currentControl.set(thetaZ, 0, desiredAngularAcceleration.getZ() * inertia.getZ());

      DenseMatrix64F function = new DenseMatrix64F(currentState);
      DenseMatrix64F functionExpected = new DenseMatrix64F(currentState);

      dynamics.getDynamics(STANCE, currentState, currentControl, constants, function);

      functionExpected.set(x, desiredLinearAcceleration.getX());
      functionExpected.set(y, desiredLinearAcceleration.getY());
      functionExpected.set(z, desiredLinearAcceleration.getZ());
      functionExpected.set(thetaX, desiredAngularAcceleration.getX());
      functionExpected.set(thetaY, desiredAngularAcceleration.getY());
      functionExpected.set(thetaZ, desiredAngularAcceleration.getZ());

      JUnitTools.assertMatrixEquals(functionExpected, function, 1e-7);

      dynamics.getDynamics(FLIGHT, currentState, currentControl, constants, function);

      functionExpected.zero();
      functionExpected.set(z, 0, -gravity);
      JUnitTools.assertMatrixEquals(functionExpected, function, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
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
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);
      DenseMatrix64F gradientExpected = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);

      dynamics.getDynamicsStateGradient(STANCE, currentState, currentControl, constants, gradient);

      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);

      dynamics.getDynamicsStateGradient(FLIGHT, currentState, currentControl, constants, gradient);

      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testStateGradientNumericalDifferentiationStance()
   {
      double epsilon = 1e-9;
      double mass = 15.0;
      double gravityZ = 9.81;
      ContinuousSimpleReactionDynamics dynamics = new ContinuousSimpleReactionDynamics(mass, gravityZ);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);
      DenseMatrix64F expectedGradient = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);

      DenseMatrix64F dynamicsUnmodified = new DenseMatrix64F(stateVectorSize / 2, 1);
      DenseMatrix64F dynamicsModified = new DenseMatrix64F(stateVectorSize / 2, 1);

      dynamics.getDynamics(SLIPState.STANCE, currentState, currentControl, constants, dynamicsUnmodified);

      dynamics.getDynamicsStateGradient(SLIPState.STANCE, currentState, currentControl, constants, gradient);

      for (int modifiedStateIndex = 0; modifiedStateIndex < stateVectorSize / 2; modifiedStateIndex++)
      {
         DenseMatrix64F currentStateModified = new DenseMatrix64F(currentState);
         currentStateModified.add(modifiedStateIndex, 0, epsilon);
         dynamics.getDynamics(SLIPState.STANCE, currentStateModified, currentControl, constants, dynamicsModified);

         for (int stateIndex = 0; stateIndex < stateVectorSize / 2; stateIndex++)
            expectedGradient.set(stateIndex, modifiedStateIndex, (dynamicsModified.get(stateIndex) - dynamicsUnmodified.get(stateIndex)) / epsilon);
      }

      JUnitTools.assertMatrixEquals(expectedGradient, gradient, 1e-2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testStateGradientNumericalDifferentiationFlight()
   {
      double epsilon = 1e-9;
      double mass = 15.0;
      double gravityZ = 9.81;
      ContinuousSimpleReactionDynamics dynamics = new ContinuousSimpleReactionDynamics(mass, gravityZ);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);
      DenseMatrix64F expectedGradient = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);

      DenseMatrix64F dynamicsUnmodified = new DenseMatrix64F(stateVectorSize / 2, 1);
      DenseMatrix64F dynamicsModified = new DenseMatrix64F(stateVectorSize / 2, 1);

      dynamics.getDynamics(SLIPState.FLIGHT, currentState, currentControl, constants, dynamicsUnmodified);

      dynamics.getDynamicsStateGradient(SLIPState.FLIGHT, currentState, currentControl, constants, gradient);

      for (int modifiedStateIndex = 0; modifiedStateIndex < stateVectorSize / 2; modifiedStateIndex++)
      {
         DenseMatrix64F currentStateModified = new DenseMatrix64F(currentState);
         currentStateModified.add(modifiedStateIndex, 0, epsilon);
         dynamics.getDynamics(SLIPState.FLIGHT, currentStateModified, currentControl, constants, dynamicsModified);

         for (int stateIndex = 0; stateIndex < stateVectorSize / 2; stateIndex++)
            expectedGradient.set(stateIndex, modifiedStateIndex, (dynamicsModified.get(stateIndex) - dynamicsUnmodified.get(stateIndex)) / epsilon);
      }

      JUnitTools.assertMatrixEquals(expectedGradient, gradient, 1e-2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testControlGradientNumericalDifferentiationStance()
   {
      double epsilon = 1e-9;
      double mass = 15.0;
      double gravityZ = 9.81;
      ContinuousSimpleReactionDynamics dynamics = new ContinuousSimpleReactionDynamics(mass, gravityZ);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);
      DenseMatrix64F expectedGradient = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);

      DenseMatrix64F dynamicsUnmodified = new DenseMatrix64F(stateVectorSize / 2, 1);
      DenseMatrix64F dynamicsModified = new DenseMatrix64F(stateVectorSize / 2, 1);

      dynamics.getDynamics(SLIPState.STANCE, currentState, currentControl, constants, dynamicsUnmodified);

      dynamics.getDynamicsControlGradient(SLIPState.STANCE, currentState, currentControl, constants, gradient);

      for (int modifiedStateIndex = 0; modifiedStateIndex < controlVectorSize; modifiedStateIndex++)
      {
         DenseMatrix64F currentControlModified = new DenseMatrix64F(currentControl);
         currentControlModified.add(modifiedStateIndex, 0, epsilon);
         dynamics.getDynamics(SLIPState.STANCE, currentState, currentControlModified, constants, dynamicsModified);

         for (int stateIndex = 0; stateIndex < stateVectorSize / 2; stateIndex++)
            expectedGradient.set(stateIndex, modifiedStateIndex, (dynamicsModified.get(stateIndex) - dynamicsUnmodified.get(stateIndex)) / epsilon);
      }

      JUnitTools.assertMatrixEquals(expectedGradient, gradient, 1e-2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testControlGradientNumericalDifferentiationFlight()
   {
      double epsilon = 1e-9;
      double mass = 15.0;
      double gravityZ = 9.81;
      ContinuousSimpleReactionDynamics dynamics = new ContinuousSimpleReactionDynamics(mass, gravityZ);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);
      DenseMatrix64F expectedGradient = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);

      DenseMatrix64F dynamicsUnmodified = new DenseMatrix64F(stateVectorSize / 2, 1);
      DenseMatrix64F dynamicsModified = new DenseMatrix64F(stateVectorSize / 2, 1);

      dynamics.getDynamics(SLIPState.FLIGHT, currentState, currentControl, constants, dynamicsUnmodified);

      dynamics.getDynamicsControlGradient(SLIPState.FLIGHT, currentState, currentControl, constants, gradient);

      for (int modifiedStateIndex = 0; modifiedStateIndex < controlVectorSize; modifiedStateIndex++)
      {
         DenseMatrix64F currentControlModified = new DenseMatrix64F(currentControl);
         currentControlModified.add(modifiedStateIndex, 0, epsilon);
         dynamics.getDynamics(SLIPState.FLIGHT, currentState, currentControlModified, constants, dynamicsModified);

         for (int stateIndex = 0; stateIndex < stateVectorSize / 2; stateIndex++)
            expectedGradient.set(stateIndex, modifiedStateIndex, (dynamicsModified.get(stateIndex) - dynamicsUnmodified.get(stateIndex)) / epsilon);
      }

      JUnitTools.assertMatrixEquals(expectedGradient, gradient, 1e-2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
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
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);
      DenseMatrix64F gradientExpected = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);

      dynamics.getDynamicsControlGradient(STANCE, currentState, currentControl, constants, gradient);

      gradientExpected.set(x, fx, 1.0 / mass);
      gradientExpected.set(y, fy, 1.0 / mass);
      gradientExpected.set(z, fz, 1.0 / mass);
      gradientExpected.set(thetaX, tauX, 1.0 / inertia.getX());
      gradientExpected.set(thetaY, tauY, 1.0 / inertia.getY());
      gradientExpected.set(thetaZ, tauZ, 1.0 / inertia.getZ());

      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);

      dynamics.getDynamicsControlGradient(FLIGHT, currentState, currentControl, constants, gradient);

      gradientExpected.zero();
      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
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
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);
      DenseMatrix64F gradientExpected = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);

      dynamics.getDynamicsStateGradient(STANCE, currentState, currentControl, constants, gradient);

      DenseMatrix64F dynamicState = new DenseMatrix64F(stateVectorSize / 2, 1);
      dynamics.getDynamics(STANCE, currentState, currentControl, constants, dynamicState);

      DenseMatrix64F dynamicStateModified = new DenseMatrix64F(stateVectorSize / 2, 1);

      DenseMatrix64F currentStateModified = new DenseMatrix64F(currentState);
      currentStateModified.add(x, 0, epsilon);
      dynamics.getDynamics(STANCE, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, x, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(y, 0, epsilon);
      dynamics.getDynamics(STANCE, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, y, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(z, 0, epsilon);
      dynamics.getDynamics(STANCE, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, z, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(3, 0, epsilon);
      dynamics.getDynamics(STANCE, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, 3, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(4, 0, epsilon);
      dynamics.getDynamics(STANCE, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, 4, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(5, 0, epsilon);
      dynamics.getDynamics(STANCE, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, 5, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
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
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);
      DenseMatrix64F gradientExpected = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);

      dynamics.getDynamicsStateGradient(FLIGHT, currentState, currentControl, constants, gradient);

      DenseMatrix64F dynamicState = new DenseMatrix64F(stateVectorSize / 2, 1);
      dynamics.getDynamics(FLIGHT, currentState, currentControl, constants, dynamicState);

      DenseMatrix64F dynamicStateModified = new DenseMatrix64F(stateVectorSize / 2, 1);

      DenseMatrix64F currentStateModified = new DenseMatrix64F(currentState);
      currentStateModified.add(x, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, x, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(y, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, y, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(z, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, z, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(3, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, 3, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(4, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, 4, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);

      currentStateModified.set(currentState);
      currentStateModified.add(5, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentStateModified, currentControl, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, 5, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
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
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);
      DenseMatrix64F gradientExpected = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);

      dynamics.getDynamicsControlGradient(STANCE, currentState, currentControl, constants, gradient);

      DenseMatrix64F dynamicState = new DenseMatrix64F(stateVectorSize / 2, 1);
      dynamics.getDynamics(STANCE, currentState, currentControl, constants, dynamicState);

      DenseMatrix64F dynamicStateModified = new DenseMatrix64F(stateVectorSize / 2, 1);

      DenseMatrix64F currentControlModified = new DenseMatrix64F(currentControl);
      currentControlModified.add(fx, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, fx, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(fy, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, fy, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(fz, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, fz, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauX, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, tauX, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauY, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, tauY, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauZ, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, tauZ, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(xF, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, xF, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(yF, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, yF, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(k, 0, epsilon);
      dynamics.getDynamics(STANCE, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, k, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
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
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);
      DenseMatrix64F gradientExpected = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);

      dynamics.getDynamicsControlGradient(FLIGHT, currentState, currentControl, constants, gradient);

      DenseMatrix64F dynamicState = new DenseMatrix64F(stateVectorSize / 2, 1);
      dynamics.getDynamics(FLIGHT, currentState, currentControl, constants, dynamicState);

      DenseMatrix64F dynamicStateModified = new DenseMatrix64F(stateVectorSize / 2, 1);

      DenseMatrix64F currentControlModified = new DenseMatrix64F(currentControl);
      currentControlModified.add(fx, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, fx, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(fy, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, fy, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(fz, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, fz, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauX, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, tauX, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauY, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, tauY, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(tauZ, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, tauZ, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(xF, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, xF, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(yF, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, yF, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      currentControlModified.set(currentControl);
      currentControlModified.add(k, 0, epsilon);
      dynamics.getDynamics(FLIGHT, currentState, currentControlModified, constants, dynamicStateModified);

      CommonOps.subtractEquals(dynamicStateModified, dynamicState);
      MatrixTools.setMatrixBlock(gradientExpected, 0, k, dynamicStateModified, 0, 0, stateVectorSize / 2, 1, 1.0 / epsilon);


      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }
}
