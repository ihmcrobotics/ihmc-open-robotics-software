package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.matrixlib.MatrixTestTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.random.RandomGeometry;

public class SimpleReactionDynamicsTest
{
   @Test
   public void testDynamics()
   {
      double mass = 10.0;
      double gravity = 9.81;
      double deltaT = 0.01;

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      SimpleReactionDynamics dynamics = new SimpleReactionDynamics(deltaT, mass, gravity);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);

      FrameVector3D desiredLinearAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame(), 2.5, 3.5, 4.5);
      FrameVector3D desiredAngularAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame(), 5.5, 6.5, 7.5);

      Random random = new Random(1738L);
      DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);
      constants.set(nominalLength, 0, RandomNumbers.nextDouble(random, 0.1, 10.0));

      currentControl.set(x, 0, desiredLinearAcceleration.getX() * mass);
      currentControl.set(y, 0, desiredLinearAcceleration.getY() * mass);
      currentControl.set(z, 0, (desiredLinearAcceleration.getZ() + gravity) * mass);
      currentControl.set(thetaX, 0, desiredAngularAcceleration.getX() * inertia.getX());
      currentControl.set(thetaY, 0, desiredAngularAcceleration.getY() * inertia.getY());
      currentControl.set(thetaZ, 0, desiredAngularAcceleration.getZ() * inertia.getZ());

      DMatrixRMaj nextState = new DMatrixRMaj(currentState);
      DMatrixRMaj nextStateExpected = new DMatrixRMaj(currentState);

      dynamics.getNextState(STANCE, currentState, currentControl, constants, nextState);

      nextStateExpected.set(x, currentState.get(x) + deltaT * currentState.get(xDot) + 0.5 * deltaT * deltaT * desiredLinearAcceleration.getX());
      nextStateExpected.set(y, currentState.get(y) + deltaT * currentState.get(yDot) + 0.5 * deltaT * deltaT * desiredLinearAcceleration.getY());
      nextStateExpected.set(z, currentState.get(z) + deltaT * currentState.get(zDot) + 0.5 * deltaT * deltaT * desiredLinearAcceleration.getZ());
      nextStateExpected.set(thetaX, currentState.get(thetaX) + deltaT * currentState.get(thetaXDot) + 0.5 * deltaT * deltaT * desiredAngularAcceleration.getX());
      nextStateExpected.set(thetaY, currentState.get(thetaY) + deltaT * currentState.get(thetaYDot) + 0.5 * deltaT * deltaT * desiredAngularAcceleration.getY());
      nextStateExpected.set(thetaZ, currentState.get(thetaZ) + deltaT * currentState.get(thetaZDot) + 0.5 * deltaT * deltaT * desiredAngularAcceleration.getZ());
      nextStateExpected.set(xDot, currentState.get(xDot) + deltaT * desiredLinearAcceleration.getX());
      nextStateExpected.set(yDot, currentState.get(yDot) + deltaT * desiredLinearAcceleration.getY());
      nextStateExpected.set(zDot, currentState.get(zDot) + deltaT * desiredLinearAcceleration.getZ());
      nextStateExpected.set(thetaXDot, currentState.get(thetaXDot) + deltaT * desiredAngularAcceleration.getX());
      nextStateExpected.set(thetaYDot, currentState.get(thetaYDot) + deltaT * desiredAngularAcceleration.getY());
      nextStateExpected.set(thetaZDot, currentState.get(thetaZDot) + deltaT * desiredAngularAcceleration.getZ());

      MatrixTestTools.assertMatrixEquals(nextStateExpected, nextState, 1e-7);

      dynamics.getNextState(FLIGHT, currentState, currentControl, constants, nextState);

      nextStateExpected.zero();

      nextStateExpected.set(x, currentState.get(x) + deltaT * currentState.get(xDot));
      nextStateExpected.set(y, currentState.get(y) + deltaT * currentState.get(yDot));
      nextStateExpected.set(z, currentState.get(z) + deltaT * currentState.get(zDot) - 0.5 * deltaT * deltaT * gravity);
      nextStateExpected.set(thetaX, currentState.get(thetaX) + deltaT * currentState.get(thetaXDot));
      nextStateExpected.set(thetaY, currentState.get(thetaY) + deltaT * currentState.get(thetaYDot));
      nextStateExpected.set(thetaZ, currentState.get(thetaZ) + deltaT * currentState.get(thetaZDot));
      nextStateExpected.set(xDot, currentState.get(xDot));
      nextStateExpected.set(yDot, currentState.get(yDot));
      nextStateExpected.set(zDot, currentState.get(zDot) - deltaT * gravity);
      nextStateExpected.set(thetaXDot, currentState.get(thetaXDot));
      nextStateExpected.set(thetaYDot, currentState.get(thetaYDot));
      nextStateExpected.set(thetaZDot, currentState.get(thetaZDot));

      MatrixTestTools.assertMatrixEquals(nextStateExpected, nextState, 1e-7);
   }

   @Test
   public void testDynamicsStateGradient()
   {
      double mass = 10.0;
      double gravity = 9.81;
      double deltaT = 0.01;

      ContinuousSimpleReactionDynamics continuousDynamics = new ContinuousSimpleReactionDynamics(mass, gravity);
      DMatrixRMaj continuousDynamicsStateGradient = new DMatrixRMaj(stateVectorSize / 2, stateVectorSize);

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      SimpleReactionDynamics dynamics = new SimpleReactionDynamics(deltaT, mass, gravity);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);

      Random random = new Random(1738L);
      DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);
      constants.set(nominalLength, 0, RandomNumbers.nextDouble(random, 0.1, 10.0));

      DMatrixRMaj gradient = new DMatrixRMaj(stateVectorSize, stateVectorSize);
      DMatrixRMaj gradientExpected = new DMatrixRMaj(stateVectorSize, stateVectorSize);

      continuousDynamics.getDynamicsStateGradient(STANCE, currentState, currentControl, constants, continuousDynamicsStateGradient);
      dynamics.getDynamicsStateGradient(STANCE, currentState, currentControl, constants, gradient);

      gradientExpected.zero();
      CommonOps_DDRM.setIdentity(gradientExpected);
      gradientExpected.set(x, xDot, deltaT);
      gradientExpected.set(y, yDot, deltaT);
      gradientExpected.set(z, zDot, deltaT);
      gradientExpected.set(thetaX, thetaXDot, deltaT);
      gradientExpected.set(thetaY, thetaYDot, deltaT);
      gradientExpected.set(thetaZ, thetaZDot, deltaT);

      MatrixTools.addMatrixBlock(gradientExpected, x, 0, continuousDynamicsStateGradient, x, 0, stateVectorSize / 2, stateVectorSize, 0.5 * deltaT * deltaT);
      MatrixTools.addMatrixBlock(gradientExpected, xDot, 0, continuousDynamicsStateGradient, x, 0, stateVectorSize / 2, stateVectorSize, deltaT);

      MatrixTestTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);

      continuousDynamics.getDynamicsStateGradient(FLIGHT, currentState, currentControl, constants, continuousDynamicsStateGradient);
      dynamics.getDynamicsStateGradient(FLIGHT, currentState, currentControl, constants, gradient);

      gradientExpected.zero();
      CommonOps_DDRM.setIdentity(gradientExpected);

      gradientExpected.set(x, xDot, deltaT);
      gradientExpected.set(y, yDot, deltaT);
      gradientExpected.set(z, zDot, deltaT);
      gradientExpected.set(thetaX, thetaXDot, deltaT);
      gradientExpected.set(thetaY, thetaYDot, deltaT);
      gradientExpected.set(thetaZ, thetaZDot, deltaT);

      MatrixTools.addMatrixBlock(gradientExpected, x, 0, continuousDynamicsStateGradient, x, 0, stateVectorSize / 2, stateVectorSize, 0.5 * deltaT * deltaT);
      MatrixTools.addMatrixBlock(gradientExpected, xDot, 0, continuousDynamicsStateGradient, x, 0, stateVectorSize / 2, stateVectorSize, deltaT);

      MatrixTestTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }

   @Test
   public void testDynamicsControlGradient()
   {
      double mass = 10.0;
      double gravity = 9.81;
      double deltaT = 0.01;

      ContinuousSimpleReactionDynamics continuousDynamics = new ContinuousSimpleReactionDynamics(mass, gravity);
      DMatrixRMaj continuousDynamicsControlGradient = new DMatrixRMaj(stateVectorSize / 2, controlVectorSize);

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      SimpleReactionDynamics dynamics = new SimpleReactionDynamics(deltaT, mass, gravity);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);

      Random random = new Random(1738L);
      DMatrixRMaj currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize, 1);
      DMatrixRMaj currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DMatrixRMaj constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);
      constants.set(nominalLength, 0, RandomNumbers.nextDouble(random, 0.1, 10.0));

      DMatrixRMaj gradient = new DMatrixRMaj(stateVectorSize, controlVectorSize);
      DMatrixRMaj gradientExpected = new DMatrixRMaj(stateVectorSize, controlVectorSize);

      continuousDynamics.getDynamicsControlGradient(STANCE, currentState, currentControl, constants, continuousDynamicsControlGradient);
      dynamics.getDynamicsControlGradient(STANCE, currentState, currentControl, constants, gradient);

      gradientExpected.zero();
      MatrixTools.addMatrixBlock(gradientExpected, x, 0, continuousDynamicsControlGradient, x, 0, stateVectorSize / 2, controlVectorSize, 0.5 * deltaT * deltaT);
      MatrixTools.addMatrixBlock(gradientExpected, xDot, 0, continuousDynamicsControlGradient, x, 0, stateVectorSize / 2, controlVectorSize, deltaT);

      MatrixTestTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);

      continuousDynamics.getDynamicsControlGradient(FLIGHT, currentState, currentControl, constants, continuousDynamicsControlGradient);
      dynamics.getDynamicsControlGradient(FLIGHT, currentState, currentControl, constants, gradient);

      gradientExpected.zero();
      MatrixTools.addMatrixBlock(gradientExpected, x, 0, continuousDynamicsControlGradient, x, 0, stateVectorSize / 2, controlVectorSize, 0.5 * deltaT * deltaT);
      MatrixTools.addMatrixBlock(gradientExpected, xDot, 0, continuousDynamicsControlGradient, x, 0, stateVectorSize / 2, controlVectorSize, deltaT);

      MatrixTestTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }
}
