package us.ihmc.robotics.screwTheory;

import java.util.Random;

import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * @author twan
 *         Date: 5/18/13
 */
public class ConvectiveTermCalculatorTest
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test()
   {
      Random random = new Random(12512352L);
      Vector3d[] jointAxes = new Vector3d[] {X, Y, Z, Y, Y, X};
      ScrewTestTools.RandomFloatingChain randomFloatingChain = new ScrewTestTools.RandomFloatingChain(random, jointAxes);
      randomFloatingChain.setRandomPositionsAndVelocities(random);
      // zero accelerations

      RigidBody elevator = randomFloatingChain.getElevator();
      GeometricJacobian jacobian = new GeometricJacobian(elevator, randomFloatingChain.getLeafBody(), randomFloatingChain.getLeafBody().getBodyFixedFrame());
      jacobian.compute();

      ConvectiveTermCalculator convectiveTermCalculator = new ConvectiveTermCalculator();
      SpatialAccelerationVector acceleration = new SpatialAccelerationVector();
      convectiveTermCalculator.computeJacobianDerivativeTerm(jacobian, acceleration);

      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      SpatialAccelerationCalculator spatialAccelerationCalculator = createSpatialAccelerationCalculator(twistCalculator, elevator);

      twistCalculator.compute();
      spatialAccelerationCalculator.compute();

      SpatialAccelerationVector accelerationBack = new SpatialAccelerationVector();
      spatialAccelerationCalculator.getRelativeAcceleration(accelerationBack, jacobian.getBase(), jacobian.getEndEffector());

      SpatialMotionVectorTest.assertSpatialMotionVectorEquals(accelerationBack, acceleration, 1e-12);
   }

   private static SpatialAccelerationCalculator createSpatialAccelerationCalculator(TwistCalculator twistCalculator, RigidBody elevator)
   {
      ReferenceFrame rootFrame = elevator.getBodyFixedFrame();
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootFrame, rootFrame, rootFrame);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, rootFrame, rootAcceleration, twistCalculator,
            true, true);

      return spatialAccelerationCalculator;
   }
}
