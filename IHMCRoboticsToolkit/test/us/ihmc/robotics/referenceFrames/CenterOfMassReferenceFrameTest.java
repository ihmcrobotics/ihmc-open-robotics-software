package us.ihmc.robotics.referenceFrames;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.screwTheory.CenterOfMassCalculator;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.tools.testing.JUnitTools;

public class CenterOfMassReferenceFrameTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRandomChain()
   {
      Random random = new Random(124L);
      int nJoints = 10;
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>(nJoints);
      RigidBody elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      SixDoFJoint sixDoFJoint = new SixDoFJoint("sixDoF", elevator, elevator.getBodyFixedFrame());
      RigidBody floatingBody = ScrewTools.addRigidBody("floatingBody", sixDoFJoint, RandomTools.generateRandomDiagonalMatrix3d(random), random.nextDouble(),
                                  RandomTools.generateRandomVector(random));

      Vector3d[] jointAxes = new Vector3d[nJoints];
      for (int i = 0; i < nJoints; i++)
      {
         jointAxes[i] = RandomTools.generateRandomVector(random);
      }

      ScrewTestTools.createRandomChainRobot("test", joints, floatingBody, jointAxes, random);
      ReferenceFrame centerOfMassReferenceFrame = new CenterOfMassReferenceFrame("com", elevator.getBodyFixedFrame(), elevator);

      sixDoFJoint.setPosition(RandomTools.generateRandomVector(random));
      sixDoFJoint.setRotation(random.nextDouble(), random.nextDouble(), random.nextDouble());

      for (RevoluteJoint joint : joints)
      {
         joint.setQ(random.nextDouble());
      }

      elevator.updateFramesRecursively();
      centerOfMassReferenceFrame.update();

      CenterOfMassCalculator comCalculator = new CenterOfMassCalculator(elevator, elevator.getBodyFixedFrame());
      comCalculator.compute();
      FramePoint centerOfMass = new FramePoint(elevator.getBodyFixedFrame());
      comCalculator.getCenterOfMass(centerOfMass);

      FramePoint centerOfMassFromFrame = new FramePoint(centerOfMassReferenceFrame);
      centerOfMassFromFrame.changeFrame(elevator.getBodyFixedFrame());

      JUnitTools.assertTuple3dEquals(centerOfMass.getVectorCopy(), centerOfMassFromFrame.getVectorCopy(), 1e-12);

      Matrix3d rotation = new Matrix3d();
      RigidBodyTransform transform = centerOfMassReferenceFrame.getTransformToDesiredFrame(elevator.getBodyFixedFrame());
      transform.getRotation(rotation);
      Matrix3d idenitity = new Matrix3d();
      idenitity.setIdentity();
      JUnitTools.assertMatrix3dEquals("", idenitity, rotation, 1e-12);
   }
}
