package us.ihmc.robotics.screwTheory;

import java.util.List;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.ScrewTestTools.RandomFloatingChain;

/**
 * @author twan Date: 5/18/13
 */
public class ConvectiveTermCalculatorTest
{
   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testWithoutJointAccelerations()
   {
      Random random = new Random(12512352L);
      Vector3D[] jointAxes = new Vector3D[] {X, Y, Z, Y, Y, X};
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
      ReferenceFrame rootFrame = elevator.getBodyFixedFrame();
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootFrame, rootFrame, rootFrame);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, rootFrame, rootAcceleration, twistCalculator,
                                                                                                      true, true);

      twistCalculator.compute();
      spatialAccelerationCalculator.compute();

      SpatialAccelerationVector accelerationBack = new SpatialAccelerationVector();
      spatialAccelerationCalculator.getRelativeAcceleration(jacobian.getBase(), jacobian.getEndEffector(), accelerationBack);

      SpatialMotionVectorTest.assertSpatialMotionVectorEquals(accelerationBack, acceleration, 1e-12);
   }

   @Test
   public void testWithJointAccelerations() throws Exception
   {
      Random random = new Random(345345L);

      int numberOfRevoluteJoints = 100;
      RandomFloatingChain floatingChain = new RandomFloatingChain(random, numberOfRevoluteJoints);
      SixDoFJoint floatingJoint = floatingChain.getRootJoint();
      List<RevoluteJoint> revoluteJoints = floatingChain.getRevoluteJoints();
      List<InverseDynamicsJoint> joints = floatingChain.getInverseDynamicsJoints();

      for (int i = 0; i < 1000; i++)
      {
         ScrewTestTools.setRandomPositionAndOrientation(floatingJoint, random);
         ScrewTestTools.setRandomVelocity(floatingJoint, random);
         ScrewTestTools.setRandomAcceleration(floatingJoint, random);

         ScrewTestTools.setRandomPositions(revoluteJoints, random, -10.0, 10.0);
         ScrewTestTools.setRandomVelocities(revoluteJoints, random, -1.0, 1.0);
         ScrewTestTools.setRandomAccelerations(revoluteJoints, random, -10.0, 10.0);

         RigidBody body = joints.get(0).getPredecessor();
         RigidBody rootBody = ScrewTools.getRootBody(body);
         SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootBody.getBodyFixedFrame(), worldFrame, rootBody.getBodyFixedFrame());
         TwistCalculator twistCalculator = new  TwistCalculator(worldFrame, body);
         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(body, worldFrame, rootAcceleration, twistCalculator , true, false, false);
         
         twistCalculator.compute();
         spatialAccelerationCalculator.compute();

         int randomEndEffectorIndex = random.nextInt(numberOfRevoluteJoints + 1);
         RigidBody randomEndEffector = joints.get(randomEndEffectorIndex).getSuccessor();
         RigidBody randomBase = joints.get(random.nextInt(randomEndEffectorIndex + 1)).getPredecessor();
         GeometricJacobian jacobian = new GeometricJacobian(randomBase, randomEndEffector, randomEndEffector.getBodyFixedFrame());
         jacobian.compute();
         SpatialAccelerationVector actualConvectiveTerm = new SpatialAccelerationVector();
         new ConvectiveTermCalculator().computeJacobianDerivativeTerm(jacobian, actualConvectiveTerm);

         SpatialAccelerationVector expectedConvectiveTerm = new SpatialAccelerationVector();
         spatialAccelerationCalculator.getRelativeAcceleration(randomBase, randomEndEffector, expectedConvectiveTerm);
         
         SpatialMotionVectorTest.assertSpatialMotionVectorEquals(expectedConvectiveTerm, actualConvectiveTerm, 1.0e-12);
      }
   }
}
