package us.ihmc.robotics.screwTheory;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CenterOfMassAccelerationCalculatorTest
{
   @Before
   public void setUp()
   {
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testOneRigidBody()
   {
      Random random = new Random(1779L);
      double mass = 1.0;
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      SixDoFJoint sixDoFJoint = new SixDoFJoint("sixDoF", elevator, elevatorFrame);
      ScrewTools.addRigidBody("body", sixDoFJoint, getRandomDiagonalMatrix(random), mass, new Vector3D());
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(elevatorFrame, elevatorFrame, elevatorFrame);
      TwistCalculator twistCalculator = new TwistCalculator(elevatorFrame, elevator);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, elevatorFrame, rootAcceleration,
                                                                       twistCalculator, true, false);
      CenterOfMassAccelerationCalculator comAccelerationCalculator = new CenterOfMassAccelerationCalculator(elevator, spatialAccelerationCalculator);

      ReferenceFrame frameAfterJoint = sixDoFJoint.getFrameAfterJoint();
      ReferenceFrame frameBeforeJoint = sixDoFJoint.getFrameBeforeJoint();
      SpatialAccelerationVector jointAcceleration = new SpatialAccelerationVector(frameAfterJoint, frameBeforeJoint, frameAfterJoint, getRandomVector(random),
                                                       new Vector3D());
      sixDoFJoint.setPosition(getRandomVector(random));
      sixDoFJoint.setRotation(random.nextDouble(), random.nextDouble(), random.nextDouble());
      sixDoFJoint.setAcceleration(jointAcceleration);
      elevator.updateFramesRecursively();

      RotationMatrix rotationMatrix = new RotationMatrix();
      sixDoFJoint.getRotation(rotationMatrix);

      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
      FrameVector comAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());
      comAccelerationCalculator.getCoMAcceleration(comAcceleration);

      Vector3D expected = jointAcceleration.getLinearPartCopy();
      rotationMatrix.transform(expected);
      EuclidCoreTestTools.assertTuple3DEquals(expected, comAcceleration.getVectorCopy(), 1e-5);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTwoSliderJointsZeroAcceleration()
   {
      Random random = new Random(1779L);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      Vector3D jointAxis = new Vector3D(1.0, 0.0, 0.0);
      PrismaticJoint j1 = ScrewTools.addPrismaticJoint("j1", elevator, new Vector3D(), jointAxis);
      RigidBody r1 = ScrewTools.addRigidBody("r1", j1, getRandomDiagonalMatrix(random), random.nextDouble(), getRandomVector(random));
      PrismaticJoint j2 = ScrewTools.addPrismaticJoint("j2", r1, new Vector3D(), jointAxis);
      RigidBody r2 = ScrewTools.addRigidBody("r2", j2, getRandomDiagonalMatrix(random), random.nextDouble(), getRandomVector(random));

      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(elevatorFrame, elevatorFrame, elevatorFrame);
      TwistCalculator twistCalculator = new TwistCalculator(elevatorFrame, elevator);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, elevatorFrame, rootAcceleration,
                                                                       twistCalculator, true, false);
      CenterOfMassAccelerationCalculator comAccelerationCalculator = new CenterOfMassAccelerationCalculator(elevator, spatialAccelerationCalculator);

      double qdd1 = random.nextDouble();
      double m1 = r1.getInertia().getMass();
      double m2 = r2.getInertia().getMass();
      double qdd2 = -(m1 + m2) * qdd1 / m2;

      j1.setQ(random.nextDouble());
      j2.setQ(random.nextDouble());
      j1.setQd(random.nextDouble());
      j2.setQd(random.nextDouble());
      j1.setQdd(qdd1);
      j2.setQdd(qdd2);
      elevator.updateFramesRecursively();

      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
      FrameVector comAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());
      comAccelerationCalculator.getCoMAcceleration(comAcceleration);

      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), comAcceleration.getVectorCopy(), 1e-5);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPendulumCentripetalAcceleration()
   {
      Random random = new Random(1779L);

      double mass = random.nextDouble();
      double qd = random.nextDouble();
      double length = 3.0;
      Vector3D comOffset = new Vector3D(0.0, 0.0, length);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      Vector3D jointAxis = new Vector3D(0.0, 1.0, 0.0);
      RevoluteJoint j1 = ScrewTools.addRevoluteJoint("j1", elevator, new Vector3D(), jointAxis);
      ScrewTools.addRigidBody("r1", j1, getRandomDiagonalMatrix(random), mass, comOffset);

      j1.setQ(random.nextDouble());
      j1.setQd(qd);
      j1.setQdd(0.0);
      elevator.updateFramesRecursively();

      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(elevatorFrame, elevatorFrame, elevatorFrame);
      TwistCalculator twistCalculator = new TwistCalculator(elevatorFrame, elevator);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, elevatorFrame, rootAcceleration,
                                                                       twistCalculator, true, false);
      CenterOfMassAccelerationCalculator comAccelerationCalculator = new CenterOfMassAccelerationCalculator(elevator, spatialAccelerationCalculator);

      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
      FrameVector comAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());
      comAccelerationCalculator.getCoMAcceleration(comAcceleration);

      assertEquals(length * qd * qd, comAcceleration.length(), 1e-5);
   }

   // Just tests whether it will crash or not for now

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTree()
   {
      Random random = new Random(1779L);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      Vector3D jointAxis = new Vector3D(1.0, 0.0, 0.0);
      PrismaticJoint j1 = ScrewTools.addPrismaticJoint("j1", elevator, new Vector3D(), jointAxis);
      ScrewTools.addRigidBody("r1", j1, getRandomDiagonalMatrix(random), random.nextDouble(), getRandomVector(random));
      PrismaticJoint j2 = ScrewTools.addPrismaticJoint("j2", elevator, new Vector3D(), jointAxis);
      ScrewTools.addRigidBody("r2", j2, getRandomDiagonalMatrix(random), random.nextDouble(), getRandomVector(random));

      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(elevatorFrame, elevatorFrame, elevatorFrame);
      TwistCalculator twistCalculator = new TwistCalculator(elevatorFrame, elevator);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, elevatorFrame, rootAcceleration,
                                                                       twistCalculator, true, false);
      CenterOfMassAccelerationCalculator comAccelerationCalculator = new CenterOfMassAccelerationCalculator(elevator, spatialAccelerationCalculator);

      twistCalculator.compute();
      spatialAccelerationCalculator.compute();
      FrameVector comAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());
      comAccelerationCalculator.getCoMAcceleration(comAcceleration);
   }

   private Vector3D getRandomVector(Random random)
   {
      return new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
   }

   private Matrix3D getRandomDiagonalMatrix(Random random)
   {
      Matrix3D ret = new Matrix3D();
      ret.setM00(random.nextDouble());
      ret.setM11(random.nextDouble());
      ret.setM22(random.nextDouble());

      return ret;
   }
}
