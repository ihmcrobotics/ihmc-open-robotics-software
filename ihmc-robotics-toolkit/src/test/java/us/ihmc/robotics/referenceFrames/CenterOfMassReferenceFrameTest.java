package us.ihmc.robotics.referenceFrames;

import java.util.ArrayList;
import java.util.Random;

import org.junit.After;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.algorithms.CenterOfMassCalculator;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.robotics.random.RandomGeometry;

public class CenterOfMassReferenceFrameTest
{
   @After
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRandomChain()
   {
      Random random = new Random(124L);
      int nJoints = 10;
      ArrayList<RevoluteJoint> joints = new ArrayList<RevoluteJoint>(nJoints);
      RigidBodyBasics elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      SixDoFJoint sixDoFJoint = new SixDoFJoint("sixDoF", elevator);
      RigidBodyBasics floatingBody = new RigidBody("floatingBody", sixDoFJoint, RandomGeometry.nextDiagonalMatrix3D(random), random.nextDouble(), RandomGeometry.nextVector3D(random));

      Vector3D[] jointAxes = new Vector3D[nJoints];
      for (int i = 0; i < nJoints; i++)
      {
         jointAxes[i] = RandomGeometry.nextVector3D(random);
      }

      joints.addAll(MultiBodySystemRandomTools.nextRevoluteJointChain(random, "test", floatingBody, jointAxes));
      ReferenceFrame centerOfMassReferenceFrame = new CenterOfMassReferenceFrame("com", elevator.getBodyFixedFrame(), elevator);

      sixDoFJoint.setJointPosition(RandomGeometry.nextVector3D(random));
      sixDoFJoint.getJointPose().setOrientationYawPitchRoll(random.nextDouble(), random.nextDouble(), random.nextDouble());

      for (RevoluteJoint joint : joints)
      {
         joint.setQ(random.nextDouble());
      }

      elevator.updateFramesRecursively();
      centerOfMassReferenceFrame.update();

      CenterOfMassCalculator comCalculator = new CenterOfMassCalculator(elevator, elevator.getBodyFixedFrame());
      comCalculator.reset();
      FramePoint3D centerOfMass = new FramePoint3D(comCalculator.getCenterOfMass());

      FramePoint3D centerOfMassFromFrame = new FramePoint3D(centerOfMassReferenceFrame);
      centerOfMassFromFrame.changeFrame(elevator.getBodyFixedFrame());

      EuclidCoreTestTools.assertTuple3DEquals(centerOfMass, centerOfMassFromFrame, 1e-12);

      RotationMatrix rotation = new RotationMatrix();
      RigidBodyTransform transform = centerOfMassReferenceFrame.getTransformToDesiredFrame(elevator.getBodyFixedFrame());
      transform.getRotation(rotation);
      RotationMatrix idenitity = new RotationMatrix();
      idenitity.setIdentity();
      EuclidCoreTestTools.assertMatrix3DEquals("", idenitity, rotation, 1e-12);
   }
}
