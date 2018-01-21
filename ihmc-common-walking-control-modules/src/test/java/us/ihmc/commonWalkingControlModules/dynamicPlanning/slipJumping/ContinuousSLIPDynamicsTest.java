package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.testing.JUnitTools;

import java.util.Random;

import static us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping.SLIPState.*;

public class ContinuousSLIPDynamicsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testDynamics()
   {
      double mass = 10.0;
      double gravity = 9.81;

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      ContinuousSLIPDynamics dynamics = new ContinuousSLIPDynamics(mass, gravity);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);


      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);
      constants.set(nominalLength, 0, RandomNumbers.nextDouble(random, 0.1, 10.0));

      Vector3D relativePosition = new Vector3D();
      relativePosition.setX(currentState.get(x));
      relativePosition.setY(currentState.get(y));
      relativePosition.setZ(currentState.get(z));
      relativePosition.subX(currentControl.get(xF));
      relativePosition.subY(currentControl.get(yF));
      relativePosition.subZ(constants.get(zF));

      Vector3D reactionForce = new Vector3D();
      reactionForce.setX(currentControl.get(fx));
      reactionForce.setY(currentControl.get(fy));
      reactionForce.setZ(currentControl.get(fz));

      Vector3D reactionTorque = new Vector3D();
      reactionTorque.cross(relativePosition, reactionForce);

      double nominalPendulumLength = constants.get(nominalLength, 0);
      double pendulumLength = relativePosition.length();
      double stiffness = currentControl.get(k);
      double springForce = stiffness * (nominalPendulumLength - pendulumLength);

      Vector3D force = new Vector3D(relativePosition);
      force.normalize();
      force.scale(springForce);

      Vector3D linearAcceleration = new Vector3D(force);
      linearAcceleration.scale(1.0 / mass);
      linearAcceleration.subZ(gravity);

      DenseMatrix64F function = new DenseMatrix64F(currentState);
      DenseMatrix64F functionExpected = new DenseMatrix64F(currentState);

      dynamics.getDynamics(STANCE, currentState, currentControl, constants, function);

      functionExpected.set(x, linearAcceleration.getX());
      functionExpected.set(y, linearAcceleration.getY());
      functionExpected.set(z, linearAcceleration.getZ());
      functionExpected.set(thetaX, reactionTorque.getX() / inertia.getX());
      functionExpected.set(thetaY, reactionTorque.getY() / inertia.getY());
      functionExpected.set(thetaZ, reactionTorque.getZ() / inertia.getZ());

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
      double mass = 11.0;
      double gravity = 9.81;

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      ContinuousSLIPDynamics dynamics = new ContinuousSLIPDynamics(mass, gravity);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);
      DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);
      constants.set(nominalLength, 0, RandomNumbers.nextDouble(random, 0.1, 10.0));

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);
      DenseMatrix64F gradientExpected = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);

      dynamics.getDynamicsStateGradient(STANCE, currentState, currentControl, constants, gradient);

      double x_k = currentState.get(x, 0);
      double y_k = currentState.get(y, 0);
      double z_k = currentState.get(z, 0);
      double xF_k = currentControl.get(xF, 0);
      double yF_k = currentControl.get(yF, 0);
      double K = currentControl.get(k, 0);

      double zF_k = constants.get(zF, 0);
      double nominalPendulumLength = constants.get(nominalLength, 0);

      double length2 = (x_k - xF_k) * (x_k - xF_k) + (y_k - yF_k) * (y_k - yF_k) + (z_k - zF_k) * (z_k - zF_k);
      double length = Math.sqrt(length2);
      double outsideTerm = -K * nominalPendulumLength / Math.pow(length2, 1.5);

      double f1x = outsideTerm * (x_k - xF_k) * (x_k - xF_k) + K * nominalPendulumLength / length - K;
      double f1y = outsideTerm * (x_k - xF_k) * (y_k - yF_k);
      double f1z = outsideTerm * (x_k - xF_k) * (z_k - zF_k);

      double f2x = outsideTerm * (x_k - xF_k) * (y_k - yF_k);
      double f2y = outsideTerm * (y_k - yF_k) * (y_k - yF_k) + K * nominalPendulumLength / length - K;
      double f2z = outsideTerm * (y_k - yF_k) * (z_k - zF_k);

      double f3x = outsideTerm * (x_k - xF_k) * (z_k - zF_k);
      double f3y = outsideTerm * (y_k - yF_k) * (z_k - zF_k);
      double f3z = outsideTerm * (z_k - zF_k) * (z_k - zF_k) + K * nominalPendulumLength / length - K;

      double tau1x = 0.0;
      double tau1y = currentControl.get(fz);
      double tau1z = -currentControl.get(fy);

      double tau2x = -currentControl.get(fz);
      double tau2y = 0.0;
      double tau2z = currentControl.get(fx);

      double tau3x = currentControl.get(fy);
      double tau3y = -currentControl.get(fx);
      double tau3z = 0.0;

      gradientExpected.set(x, x, f1x);
      gradientExpected.set(x, y, f1y);
      gradientExpected.set(x, z, f1z);

      gradientExpected.set(y, x, f2x);
      gradientExpected.set(y, y, f2y);
      gradientExpected.set(y, z, f2z);

      gradientExpected.set(z, x, f3x);
      gradientExpected.set(z, y, f3y);
      gradientExpected.set(z, z, f3z);

      CommonOps.scale(1.0 / mass, gradientExpected);

      gradientExpected.set(tauX, x, 1.0 / inertia.getX() * tau1x);
      gradientExpected.set(tauX, y, 1.0 / inertia.getX() * tau1y);
      gradientExpected.set(tauX, z, 1.0 / inertia.getX() * tau1z);

      gradientExpected.set(tauY, x, 1.0 / inertia.getY() * tau2x);
      gradientExpected.set(tauY, y, 1.0 / inertia.getY() * tau2y);
      gradientExpected.set(tauY, z, 1.0 / inertia.getY() * tau2z);

      gradientExpected.set(tauZ, x, 1.0 / inertia.getZ() * tau3x);
      gradientExpected.set(tauZ, y, 1.0 / inertia.getZ() * tau3y);
      gradientExpected.set(tauZ, z, 1.0 / inertia.getZ() * tau3z);


      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);



      dynamics.getDynamicsStateGradient(FLIGHT, currentState, currentControl, constants, gradient);
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
      ContinuousSLIPDynamics dynamics = new ContinuousSLIPDynamics(mass, gravity);

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
      ContinuousSLIPDynamics dynamics = new ContinuousSLIPDynamics(mass, gravity);

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
   public void testDynamicsControlGradient()
   {
      double mass = 11.0;
      double gravity = 9.81;

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      ContinuousSLIPDynamics dynamics = new ContinuousSLIPDynamics(mass, gravity);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      DenseMatrix64F constants = RandomGeometry.nextDenseMatrix64F(random, constantVectorSize, 1);
      constants.set(nominalLength, 0, RandomNumbers.nextDouble(random, 0.1, 10.0));

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);
      DenseMatrix64F gradientExpected = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);

      dynamics.getDynamicsControlGradient(STANCE, currentState, currentControl, constants, gradient);

      double x_k = currentState.get(x, 0);
      double y_k = currentState.get(y, 0);
      double z_k = currentState.get(z, 0);
      double xF_k = currentControl.get(xF, 0);
      double yF_k = currentControl.get(yF, 0);
      double K = currentControl.get(k, 0);

      double zF_k = constants.get(zF, 0);
      double nominalPendulumLength = constants.get(nominalLength, 0);

      double fx_k = currentControl.get(fx, 0);
      double fy_k = currentControl.get(fy, 0);
      double fz_k = currentControl.get(fz, 0);

      double length2 = (x_k - xF_k) * (x_k - xF_k) + (y_k - yF_k) * (y_k - yF_k) + (z_k - zF_k) * (z_k - zF_k);
      double outsideTerm = K * nominalPendulumLength / Math.pow(length2, 1.5);

      double f1x = outsideTerm * (x_k - xF_k) * (x_k - xF_k) - K * nominalPendulumLength / Math.sqrt(length2) + K;
      double f1y = outsideTerm * (x_k - xF_k) * (y_k - yF_k);
      double f1k = (nominalPendulumLength / Math.sqrt(length2) - 1.0) * (x_k - xF_k);

      double f2x = outsideTerm * (x_k - xF_k) * (y_k - yF_k);
      double f2y = outsideTerm * (y_k - yF_k) * (y_k - yF_k) - K * nominalPendulumLength / Math.sqrt(length2) + K;
      double f2k = (nominalPendulumLength / Math.sqrt(length2) - 1.0) * (y_k - yF_k);

      double f3x = outsideTerm * (x_k - xF_k) * (z_k - zF_k);
      double f3y = outsideTerm * (y_k - yF_k) * (z_k - zF_k);
      double f3k = (nominalPendulumLength / Math.sqrt(length2) - 1.0) * (z_k - zF_k);

      gradientExpected.set(x, xF, f1x);
      gradientExpected.set(x, yF, f1y);
      gradientExpected.set(x, k, f1k);

      gradientExpected.set(y, xF, f2x);
      gradientExpected.set(y, yF, f2y);
      gradientExpected.set(y, k, f2k);

      gradientExpected.set(z, xF, f3x);
      gradientExpected.set(z, yF, f3y);
      gradientExpected.set(z, k, f3k);

      CommonOps.scale(1.0 / mass, gradientExpected);

      gradientExpected.set(tauX, fy, 1.0 / inertia.getX() * -(z_k - zF_k));
      gradientExpected.set(tauX, fz, 1.0 / inertia.getX() * (y_k - yF_k));
      gradientExpected.set(tauX, yF, 1.0 / inertia.getX() * -fz_k);

      gradientExpected.set(tauY, fx, 1.0 / inertia.getY() * (z_k - zF_k));
      gradientExpected.set(tauY, fz, 1.0 / inertia.getY() * (xF_k - x_k));
      gradientExpected.set(tauY, xF, 1.0 / inertia.getY() * fz_k);

      gradientExpected.set(tauZ, fx, 1.0 / inertia.getZ() * (yF_k - y_k));
      gradientExpected.set(tauZ, fy, 1.0 / inertia.getZ() * (x_k - xF_k));
      gradientExpected.set(tauZ, xF, 1.0 / inertia.getZ() * -fy_k);
      gradientExpected.set(tauZ, yF, 1.0 / inertia.getZ() * fx_k);


      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);



      dynamics.getDynamicsControlGradient(FLIGHT, currentState, currentControl, constants, gradient);
      gradientExpected.zero();
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
      ContinuousSLIPDynamics dynamics = new ContinuousSLIPDynamics(mass, gravity);

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
      ContinuousSLIPDynamics dynamics = new ContinuousSLIPDynamics(mass, gravity);

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
