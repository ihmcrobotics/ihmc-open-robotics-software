package us.ihmc.simulationconstructionset.utilities.screwTheory;

import java.util.LinkedHashMap;
import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.CentroidalMomentumMatrix;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.tools.testing.JUnitTools;

public class CentroidalMomentumMatrixSCSTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testTree()
   {
      Random random = new Random(167L);

      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      LinkedHashMap<RevoluteJoint, PinJoint> jointMap = new LinkedHashMap<RevoluteJoint, PinJoint>();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      double gravity = 0.0;

      int numberOfJoints = 3;
      InverseDynamicsCalculatorSCSTest.createRandomTreeRobotAndSetJointPositionsAndVelocities(robot, jointMap, worldFrame, elevator, numberOfJoints, gravity,
            true, true, random);
      robot.updateVelocities();

      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();
      CentroidalMomentumMatrix centroidalMomentumMatrix = new CentroidalMomentumMatrix(elevator, centerOfMassFrame);
      centroidalMomentumMatrix.compute();

      Momentum comMomentum = computeCoMMomentum(elevator, centerOfMassFrame, centroidalMomentumMatrix);

      Point3d comPoint = new Point3d();
      Vector3d linearMomentum = new Vector3d();
      Vector3d angularMomentum = new Vector3d();
      robot.computeCOMMomentum(comPoint, linearMomentum, angularMomentum);

      JUnitTools.assertTuple3dEquals(linearMomentum, comMomentum.getLinearPart(), 1e-12);
      JUnitTools.assertTuple3dEquals(angularMomentum, comMomentum.getAngularPart(), 1e-12);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testFloatingBody()
   {
      Random random = new Random(167L);

      double mass = random.nextDouble();
      Matrix3d momentOfInertia = RandomTools.generateRandomDiagonalMatrix3d(random);
      Vector3d comOffset = RandomTools.generateRandomVector(random);

      Robot robot = new Robot("robot");
      FloatingJoint rootJoint = new FloatingJoint("rootJoint", new Vector3d(), robot);
      Link link = new Link("link");
      link.setMass(mass);
      link.setMomentOfInertia(momentOfInertia);
      link.setComOffset(comOffset);
      rootJoint.setLink(link);
      robot.addRootJoint(rootJoint);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);
      SixDoFJoint sixDoFJoint = new SixDoFJoint("sixDoFJoint", elevator, elevatorFrame);
      ScrewTools.addRigidBody("rigidBody", sixDoFJoint, momentOfInertia, mass, comOffset);

      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      CentroidalMomentumMatrix centroidalMomentumMatrix = new CentroidalMomentumMatrix(elevator, centerOfMassFrame);

      int nTests = 10;
      for (int i = 0; i < nTests; i++)
      {
         Vector3d position = RandomTools.generateRandomVector(random);
         Matrix3d rotation = new Matrix3d();
         RotationTools.convertYawPitchRollToMatrix(random.nextDouble(), random.nextDouble(), random.nextDouble(), rotation);
         Vector3d linearVelocityInBody = RandomTools.generateRandomVector(random);
         Vector3d linearVelocityInWorld = new Vector3d(linearVelocityInBody);
         rotation.transform(linearVelocityInWorld);
         Vector3d angularVelocity = RandomTools.generateRandomVector(random);

         rootJoint.setPosition(position);
         rootJoint.setRotation(rotation);
         rootJoint.setVelocity(linearVelocityInWorld);
         rootJoint.setAngularVelocityInBody(angularVelocity);
         robot.updateVelocities();
         Point3d comPoint = new Point3d();
         Vector3d linearMomentum = new Vector3d();
         Vector3d angularMomentum = new Vector3d();
         robot.computeCOMMomentum(comPoint, linearMomentum, angularMomentum);

         sixDoFJoint.setPosition(position);
         sixDoFJoint.setRotation(rotation);
         Twist jointTwist = new Twist();
         sixDoFJoint.getJointTwist(jointTwist);
         jointTwist.setAngularPart(angularVelocity);
         jointTwist.setLinearPart(linearVelocityInBody);
         sixDoFJoint.setJointTwist(jointTwist);
         elevator.updateFramesRecursively();

         centerOfMassFrame.update();

         centroidalMomentumMatrix.compute();
         Momentum comMomentum = computeCoMMomentum(elevator, centerOfMassFrame, centroidalMomentumMatrix);

         JUnitTools.assertTuple3dEquals(linearMomentum, comMomentum.getLinearPart(), 1e-12);
         JUnitTools.assertTuple3dEquals(angularMomentum, comMomentum.getAngularPart(), 1e-12);
      }
   }

   public static Momentum computeCoMMomentum(RigidBody elevator, ReferenceFrame centerOfMassFrame, CentroidalMomentumMatrix centroidalMomentumMatrix)
   {
      DenseMatrix64F mat = centroidalMomentumMatrix.getMatrix();
//      InverseDynamicsJoint[] jointList = ScrewTools.computeJointsInOrder(elevator); //deprecated method
      InverseDynamicsJoint[] jointList = ScrewTools.computeSubtreeJoints(elevator);

      DenseMatrix64F jointVelocities = new DenseMatrix64F(ScrewTools.computeDegreesOfFreedom(jointList), 1);
      ScrewTools.getJointVelocitiesMatrix(jointList, jointVelocities);

      DenseMatrix64F comMomentumMatrix = MatrixTools.mult(mat, jointVelocities);

      Momentum comMomentum = new Momentum(centerOfMassFrame, comMomentumMatrix);

      return comMomentum;
   }
}
