package us.ihmc.robotics.screwTheory;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.algorithms.SpatialAccelerationCalculator;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;

public class CenterOfMassAccelerationCalculatorTest
{
   @BeforeEach
   public void setUp()
   {
   }

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@Test
   public void testOneRigidBody()
   {
      Random random = new Random(1779L);
      double mass = 1.0;
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);
      ReferenceFrame elevatorFrame = elevator.getBodyFixedFrame();
      SixDoFJoint sixDoFJoint = new SixDoFJoint("sixDoF", elevator);
      new RigidBody("body", sixDoFJoint, getRandomDiagonalMatrix(random), mass, new Vector3D());
      SpatialAcceleration rootAcceleration = new SpatialAcceleration(elevatorFrame, worldFrame, elevatorFrame);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, ReferenceFrame.getWorldFrame());
      spatialAccelerationCalculator.setRootAcceleration(rootAcceleration);
      CenterOfMassAccelerationCalculator comAccelerationCalculator = new CenterOfMassAccelerationCalculator(elevator, spatialAccelerationCalculator);

      ReferenceFrame frameAfterJoint = sixDoFJoint.getFrameAfterJoint();
      ReferenceFrame frameBeforeJoint = sixDoFJoint.getFrameBeforeJoint();
      SpatialAcceleration jointAcceleration = new SpatialAcceleration(frameAfterJoint, frameBeforeJoint, frameAfterJoint, new Vector3D(),
                                                       getRandomVector(random));
      sixDoFJoint.setJointPosition(getRandomVector(random));
      sixDoFJoint.getJointPose().getOrientation().setYawPitchRoll(random.nextDouble(), random.nextDouble(), random.nextDouble());
      sixDoFJoint.setJointAcceleration(jointAcceleration);
      elevator.updateFramesRecursively();

      RotationMatrix rotationMatrix = new RotationMatrix();
      rotationMatrix.set(sixDoFJoint.getJointPose().getOrientation());

      spatialAccelerationCalculator.reset();
      FrameVector3D comAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame());
      comAccelerationCalculator.getCoMAcceleration(comAcceleration);

      Vector3D expected = new Vector3D(jointAcceleration.getLinearPart());
      rotationMatrix.transform(expected);
      EuclidCoreTestTools.assertTuple3DEquals(expected, comAcceleration, 1e-5);
   }

	@Test
   public void testTwoSliderJointsZeroAcceleration()
   {
      Random random = new Random(1779L);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);
      ReferenceFrame elevatorFrame = elevator.getBodyFixedFrame();
      Vector3D jointAxis = new Vector3D(1.0, 0.0, 0.0);
      PrismaticJoint j1 = new PrismaticJoint("j1", elevator, new Vector3D(), jointAxis);
      RigidBodyBasics r1 = new RigidBody("r1", j1, getRandomDiagonalMatrix(random), random.nextDouble(), getRandomVector(random));
      PrismaticJoint j2 = new PrismaticJoint("j2", r1, new Vector3D(), jointAxis);
      RigidBodyBasics r2 = new RigidBody("r2", j2, getRandomDiagonalMatrix(random), random.nextDouble(), getRandomVector(random));

      SpatialAcceleration rootAcceleration = new SpatialAcceleration(elevatorFrame, worldFrame, elevatorFrame);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, ReferenceFrame.getWorldFrame());
      spatialAccelerationCalculator.setRootAcceleration(rootAcceleration);
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

      spatialAccelerationCalculator.reset();
      FrameVector3D comAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame());
      comAccelerationCalculator.getCoMAcceleration(comAcceleration);

      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(), comAcceleration, 1e-5);
   }

	@Test
   public void testPendulumCentripetalAcceleration()
   {
      Random random = new Random(1779L);

      double mass = random.nextDouble();
      double qd = random.nextDouble();
      double length = 3.0;
      Vector3D comOffset = new Vector3D(0.0, 0.0, length);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);
      ReferenceFrame elevatorFrame = elevator.getBodyFixedFrame();
      Vector3D jointAxis = new Vector3D(0.0, 1.0, 0.0);
      RevoluteJoint j1 = new RevoluteJoint("j1", elevator, new Vector3D(), jointAxis);
      new RigidBody("r1", j1, getRandomDiagonalMatrix(random), mass, comOffset);

      j1.setQ(random.nextDouble());
      j1.setQd(qd);
      j1.setQdd(0.0);
      elevator.updateFramesRecursively();

      SpatialAcceleration rootAcceleration = new SpatialAcceleration(elevatorFrame, worldFrame, elevatorFrame);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, ReferenceFrame.getWorldFrame());
      spatialAccelerationCalculator.setRootAcceleration(rootAcceleration);
      CenterOfMassAccelerationCalculator comAccelerationCalculator = new CenterOfMassAccelerationCalculator(elevator, spatialAccelerationCalculator);

      spatialAccelerationCalculator.reset();
      FrameVector3D comAcceleration = new FrameVector3D(worldFrame);
      comAccelerationCalculator.getCoMAcceleration(comAcceleration);

      assertEquals(length * qd * qd, comAcceleration.length(), 1e-5);
   }

   // Just tests whether it will crash or not for now

	@Test
   public void testTree()
   {
      Random random = new Random(1779L);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);
      ReferenceFrame elevatorFrame = elevator.getBodyFixedFrame();
      Vector3D jointAxis = new Vector3D(1.0, 0.0, 0.0);
      PrismaticJoint j1 = new PrismaticJoint("j1", elevator, new Vector3D(), jointAxis);
      new RigidBody("r1", j1, getRandomDiagonalMatrix(random), random.nextDouble(), getRandomVector(random));
      PrismaticJoint j2 = new PrismaticJoint("j2", elevator, new Vector3D(), jointAxis);
      new RigidBody("r2", j2, getRandomDiagonalMatrix(random), random.nextDouble(), getRandomVector(random));

      SpatialAcceleration rootAcceleration = new SpatialAcceleration(elevatorFrame, worldFrame, elevatorFrame);
      SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(elevator, ReferenceFrame.getWorldFrame());
      spatialAccelerationCalculator.setRootAcceleration(rootAcceleration);
      CenterOfMassAccelerationCalculator comAccelerationCalculator = new CenterOfMassAccelerationCalculator(elevator, spatialAccelerationCalculator);

      spatialAccelerationCalculator.reset();
      FrameVector3D comAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame());
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
