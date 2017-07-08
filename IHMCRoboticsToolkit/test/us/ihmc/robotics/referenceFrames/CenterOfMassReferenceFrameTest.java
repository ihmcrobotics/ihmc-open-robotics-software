package us.ihmc.robotics.referenceFrames;

import java.util.ArrayList;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.screwTheory.CenterOfMassCalculator;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTestTools;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;

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
      SixDoFJoint sixDoFJoint = new SixDoFJoint("sixDoF", elevator);
      RigidBody floatingBody = ScrewTools.addRigidBody("floatingBody", sixDoFJoint, RandomGeometry.nextDiagonalMatrix3D(random), random.nextDouble(),
                                  RandomGeometry.nextVector3D(random));

      Vector3D[] jointAxes = new Vector3D[nJoints];
      for (int i = 0; i < nJoints; i++)
      {
         jointAxes[i] = RandomGeometry.nextVector3D(random);
      }

      ScrewTestTools.createRandomChainRobot("test", joints, floatingBody, jointAxes, random);
      ReferenceFrame centerOfMassReferenceFrame = new CenterOfMassReferenceFrame("com", elevator.getBodyFixedFrame(), elevator);

      sixDoFJoint.setPosition(RandomGeometry.nextVector3D(random));
      sixDoFJoint.setRotation(random.nextDouble(), random.nextDouble(), random.nextDouble());

      for (RevoluteJoint joint : joints)
      {
         joint.setQ(random.nextDouble());
      }

      elevator.updateFramesRecursively();
      centerOfMassReferenceFrame.update();

      CenterOfMassCalculator comCalculator = new CenterOfMassCalculator(elevator, elevator.getBodyFixedFrame());
      comCalculator.compute();
      FramePoint3D centerOfMass = new FramePoint3D(elevator.getBodyFixedFrame());
      comCalculator.getCenterOfMass(centerOfMass);

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
