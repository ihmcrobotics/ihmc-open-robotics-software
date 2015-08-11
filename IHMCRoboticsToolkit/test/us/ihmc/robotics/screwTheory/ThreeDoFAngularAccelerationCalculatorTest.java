package us.ihmc.robotics.screwTheory;

import org.junit.Test;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.agileTesting.BambooAnnotations.EstimatedDuration;
import us.ihmc.tools.test.JUnitTools;

import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.Random;

public class ThreeDoFAngularAccelerationCalculatorTest
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void testForward()
   {
      Random random = new Random(1234L);
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>();
      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      Vector3d[] jointAxes = new Vector3d[] {X, Y, Z};
      ScrewTestTools.createRandomChainRobot("test", joints, elevator, jointAxes, random);

      RigidBody base = elevator;
      RigidBody endEffector = joints.get(joints.size() - 1).getSuccessor();
      FrameVector desiredAngularAcceleration = new FrameVector(ReferenceFrame.getWorldFrame(), random.nextDouble(), random.nextDouble(), random.nextDouble());
      assertConsistencyWithSpatialAccelerationCalculator(random, joints, elevator, base, endEffector, desiredAngularAcceleration);
   }

	@EstimatedDuration(duration = 0.0)
	@Test(timeout = 30000)
   public void testBackward()
   {
      Random random = new Random(1234L);
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>();
      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      Vector3d[] jointAxes = new Vector3d[] {X, Y, Z};
      ScrewTestTools.createRandomChainRobot("test", joints, elevator, jointAxes, random);

      RigidBody base = joints.get(joints.size() - 1).getSuccessor();
      RigidBody endEffector = elevator;
      FrameVector desiredAngularAcceleration = new FrameVector(ReferenceFrame.getWorldFrame(), random.nextDouble(), random.nextDouble(), random.nextDouble());
      assertConsistencyWithSpatialAccelerationCalculator(random, joints, elevator, base, endEffector, desiredAngularAcceleration);
   }

   private void assertConsistencyWithSpatialAccelerationCalculator(Random random, ArrayList<RevoluteJoint> joints, RigidBody elevator, RigidBody base,
           RigidBody endEffector, FrameVector desiredAngularAcceleration)
   {
      ThreeDoFAngularAccelerationCalculator controlModule = new ThreeDoFAngularAccelerationCalculator(base, endEffector);

      for (RevoluteJoint joint : joints)
      {
         joint.setQ(random.nextDouble());
         joint.setQd(random.nextDouble());
      }

      elevator.updateFramesRecursively();

      controlModule.compute(desiredAngularAcceleration);

      TwistCalculator twistCalculator = new TwistCalculator(elevator.getBodyFixedFrame(), elevator);
      twistCalculator.compute();

      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, twistCalculator, 0.0, true);
      spatialAccelerationCalculator.compute();

      SpatialAccelerationVector endEffectorAcceleration = new SpatialAccelerationVector();
      spatialAccelerationCalculator.packAccelerationOfBody(endEffectorAcceleration, endEffector);
      Twist endEffectorTwist = new Twist();
      twistCalculator.packTwistOfBody(endEffectorTwist, endEffector);

      SpatialAccelerationVector baseAcceleration = new SpatialAccelerationVector();
      spatialAccelerationCalculator.packAccelerationOfBody(baseAcceleration, base);
      Twist baseTwist = new Twist();
      twistCalculator.packTwistOfBody(baseTwist, base);

      endEffectorTwist.changeFrame(baseAcceleration.getExpressedInFrame());
      baseTwist.changeFrame(baseAcceleration.getExpressedInFrame());

      Twist twistOfCurrentWithRespectToNew = new Twist(baseTwist);
      twistOfCurrentWithRespectToNew.sub(endEffectorTwist);

      baseAcceleration.changeFrame(endEffectorAcceleration.getExpressedInFrame(), twistOfCurrentWithRespectToNew, baseTwist);
      endEffectorAcceleration.sub(baseAcceleration);

      FrameVector desiredAngularAccelerationBack = new FrameVector(endEffectorAcceleration.getExpressedInFrame());
      endEffectorAcceleration.packAngularPart(desiredAngularAccelerationBack);

      desiredAngularAccelerationBack.changeFrame(desiredAngularAcceleration.getReferenceFrame());
      JUnitTools.assertTuple3dEquals(desiredAngularAcceleration.getVector(), desiredAngularAccelerationBack.getVector(), 1e-12);
   }

}
