package us.ihmc.commonWalkingControlModules.dynamicPlanning.slipJumping;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
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

      currentControl.set(x, 0, desiredLinearAcceleration.getX() * mass);
      currentControl.set(y, 0, desiredLinearAcceleration.getY() * mass);
      currentControl.set(z, 0, (desiredLinearAcceleration.getZ() + gravity) * mass);
      currentControl.set(thetaX, 0, desiredAngularAcceleration.getX() * inertia.getX());
      currentControl.set(thetaY, 0, desiredAngularAcceleration.getY() * inertia.getY());
      currentControl.set(thetaZ, 0, desiredAngularAcceleration.getZ() * inertia.getZ());

      DenseMatrix64F function = new DenseMatrix64F(currentState);
      DenseMatrix64F functionExpected = new DenseMatrix64F(currentState);

      dynamics.getDynamics(STANCE, currentState, currentControl, function);

      functionExpected.set(x, desiredLinearAcceleration.getX());
      functionExpected.set(y, desiredLinearAcceleration.getY());
      functionExpected.set(z, desiredLinearAcceleration.getZ());
      functionExpected.set(thetaX, desiredAngularAcceleration.getX());
      functionExpected.set(thetaY, desiredAngularAcceleration.getY());
      functionExpected.set(thetaZ, desiredAngularAcceleration.getZ());

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

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);
      DenseMatrix64F gradientExpected = new DenseMatrix64F(stateVectorSize / 2, stateVectorSize);

      dynamics.getDynamicsStateGradient(STANCE, currentState, currentControl, gradient);

      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);

      dynamics.getDynamicsStateGradient(FLIGHT, currentState, currentControl, gradient);

      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
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

      DenseMatrix64F gradient = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);
      DenseMatrix64F gradientExpected = new DenseMatrix64F(stateVectorSize / 2, controlVectorSize);

      dynamics.getDynamicsControlGradient(STANCE, currentState, currentControl, gradient);

      gradientExpected.set(x, fx, 1.0 / mass);
      gradientExpected.set(y, fy, 1.0 / mass);
      gradientExpected.set(z, fz, 1.0 / mass);
      gradientExpected.set(thetaX, tauX, 1.0 / inertia.getX());
      gradientExpected.set(thetaY, tauY, 1.0 / inertia.getY());
      gradientExpected.set(thetaZ, tauZ, 1.0 / inertia.getZ());

      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);

      dynamics.getDynamicsControlGradient(FLIGHT, currentState, currentControl, gradient);

      gradientExpected.zero();
      JUnitTools.assertMatrixEquals(gradientExpected, gradient, 1e-7);
   }
}
