package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Vector3D;
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
      double nominalLength = 1.0;

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      ContinuousSLIPDynamics dynamics = new ContinuousSLIPDynamics(mass, nominalLength, gravity);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);


      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      Vector3D relativePosition = new Vector3D();
      relativePosition.setX(currentState.get(x));
      relativePosition.setY(currentState.get(y));
      relativePosition.setZ(currentState.get(z));
      relativePosition.subX(currentControl.get(xF));
      relativePosition.subY(currentControl.get(yF));

      Vector3D reactionForce = new Vector3D();
      reactionForce.setX(currentControl.get(fx));
      reactionForce.setY(currentControl.get(fy));
      reactionForce.setZ(currentControl.get(fz));

      Vector3D reactionTorque = new Vector3D();
      reactionTorque.cross(relativePosition, reactionForce);

      double pendulumLength = relativePosition.length();
      double stiffness = currentControl.get(k);
      double springForce = stiffness * (nominalLength - pendulumLength);

      Vector3D force = new Vector3D(relativePosition);
      force.normalize();
      force.scale(springForce);

      Vector3D linearAcceleration = new Vector3D(force);
      linearAcceleration.scale(1.0 / mass);
      linearAcceleration.subZ(gravity);

      DenseMatrix64F function = new DenseMatrix64F(currentState);
      DenseMatrix64F functionExpected = new DenseMatrix64F(currentState);

      dynamics.getDynamics(STANCE, currentState, currentControl, function);

      functionExpected.set(x, linearAcceleration.getX());
      functionExpected.set(y, linearAcceleration.getY());
      functionExpected.set(z, linearAcceleration.getZ());
      functionExpected.set(thetaX, reactionTorque.getX() / inertia.getX());
      functionExpected.set(thetaY, reactionTorque.getY() / inertia.getY());
      functionExpected.set(thetaZ, reactionTorque.getZ() / inertia.getZ());

      JUnitTools.assertMatrixEquals(functionExpected, function, 1e-7);

      dynamics.getDynamics(FLIGHT, currentState, currentControl, function);

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
      double nominalLength = 1.2;

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      ContinuousSLIPDynamics dynamics = new ContinuousSLIPDynamics(mass, nominalLength, gravity);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);
      DenseMatrix64F gradientExpected = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);

      dynamics.getDynamicsStateGradient(STANCE, currentState, currentControl, gradient);

      double x_k = currentState.get(x, 0);
      double y_k = currentState.get(y, 0);
      double z_k = currentState.get(z, 0);
      double xF_k = currentControl.get(xF, 0);
      double yF_k = currentControl.get(yF, 0);
      double K = currentControl.get(k, 0);

      double length2 = (x_k - xF_k) * (x_k - xF_k) + (y_k - yF_k) * (y_k - yF_k) + z_k * z_k;
      double outsideTerm = -K * nominalLength / Math.pow(length2, 1.5);

      double f1x = outsideTerm * (x_k - xF_k) * (x_k - xF_k) + K * nominalLength / Math.sqrt(length2) - K;
      double f1y = outsideTerm * (x_k - xF_k) * (y_k - yF_k);
      double f1z = outsideTerm * (x_k - xF_k) * (z_k);

      double f2x = outsideTerm * (x_k - xF_k) * (y_k - yF_k);
      double f2y = outsideTerm * (y_k - yF_k) * (y_k - yF_k) + K * nominalLength / Math.sqrt(length2) - K;
      double f2z = outsideTerm * (y_k - yF_k) * (z_k);

      double f3x = outsideTerm * (x_k - xF_k) * (z_k);
      double f3y = outsideTerm * (y_k - yF_k) * (z_k);
      double f3z = outsideTerm * z_k * z_k + K * nominalLength / Math.sqrt(length2) - K;

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



      dynamics.getDynamicsStateGradient(FLIGHT, currentState, currentControl, gradient);
      gradientExpected.zero();
      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testDynamicsControlGradient()
   {
      double mass = 11.0;
      double gravity = 9.81;
      double nominalLength = 1.2;

      Vector3D boxSize = new Vector3D(0.3, 0.3, 0.5);
      ContinuousSLIPDynamics dynamics = new ContinuousSLIPDynamics(mass, nominalLength, gravity);

      Vector3D inertia = new Vector3D();
      inertia.setX(boxSize.getY() * boxSize.getY() + boxSize.getZ() * boxSize.getZ());
      inertia.setY(boxSize.getX() * boxSize.getX() + boxSize.getZ() * boxSize.getZ());
      inertia.setZ(boxSize.getY() * boxSize.getY() + boxSize.getX() * boxSize.getX());
      inertia.scale(mass / 12.0);

      Random random = new Random(1738L);
      DenseMatrix64F currentState = RandomGeometry.nextDenseMatrix64F(random, stateVectorSize / 2, 1);
      DenseMatrix64F currentControl = RandomGeometry.nextDenseMatrix64F(random, controlVectorSize, 1);

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);
      DenseMatrix64F gradientExpected = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);

      dynamics.getDynamicsControlGradient(STANCE, currentState, currentControl, gradient);

      double x_k = currentState.get(x, 0);
      double y_k = currentState.get(y, 0);
      double z_k = currentState.get(z, 0);
      double xF_k = currentControl.get(xF, 0);
      double yF_k = currentControl.get(yF, 0);
      double K = currentControl.get(k, 0);

      double length2 = (x_k - xF_k) * (x_k - xF_k) + (y_k - yF_k) * (y_k - yF_k) + z_k * z_k;
      double outsideTerm = K * nominalLength / Math.pow(length2, 1.5);

      double f1x = outsideTerm * (x_k - xF_k) * (x_k - xF_k) - K * nominalLength / Math.sqrt(length2) + K;
      double f1y = outsideTerm * (x_k - xF_k) * (y_k - yF_k);
      double f1k = (nominalLength / Math.sqrt(length2) - 1.0) * (x_k - xF_k);

      double f2x = outsideTerm * (x_k - xF_k) * (y_k - yF_k);
      double f2y = outsideTerm * (y_k - yF_k) * (y_k - yF_k) - K * nominalLength / Math.sqrt(length2) + K;
      double f2k = (nominalLength / Math.sqrt(length2) - 1.0) * (y_k - yF_k);

      double f3x = outsideTerm * (x_k - xF_k) * (z_k);
      double f3y = outsideTerm * (y_k - yF_k) * (z_k);
      double f3k = (nominalLength / Math.sqrt(length2) - 1.0) * (z_k);

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

      gradientExpected.set(tauX, fy, 1.0 / inertia.getX() * -z_k);
      gradientExpected.set(tauX, fz, 1.0 / inertia.getX() * (y_k - yF_k));

      gradientExpected.set(tauY, fx, 1.0 / inertia.getY() * (z_k));
      gradientExpected.set(tauY, fz, 1.0 / inertia.getY() * (xF_k - x_k));

      gradientExpected.set(tauZ, fx, 1.0 / inertia.getZ() * (yF_k - y_k));
      gradientExpected.set(tauZ, fy, 1.0 / inertia.getZ() * (x_k - xF_k));


      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);



      dynamics.getDynamicsControlGradient(FLIGHT, currentState, currentControl, gradient);
      gradientExpected.zero();
      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }
}
