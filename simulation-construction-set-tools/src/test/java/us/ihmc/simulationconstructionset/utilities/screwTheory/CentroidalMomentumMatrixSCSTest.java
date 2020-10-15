package us.ihmc.simulationconstructionset.utilities.screwTheory;

import java.util.LinkedHashMap;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.mecano.algorithms.CentroidalMomentumCalculator;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

public class CentroidalMomentumMatrixSCSTest
{
	@Test
   public void testTree()
   {
      Random random = new Random(167L);

      Robot robot = new Robot("robot");
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      LinkedHashMap<RevoluteJoint, PinJoint> jointMap = new LinkedHashMap<RevoluteJoint, PinJoint>();
      RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);
      double gravity = 0.0;

      int numberOfJoints = 3;
      InverseDynamicsCalculatorSCSTest.createRandomTreeRobotAndSetJointPositionsAndVelocities(robot, jointMap, worldFrame, elevator, numberOfJoints, gravity,
            true, true, random);
      robot.updateVelocities();

      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      centerOfMassFrame.update();
      CentroidalMomentumCalculator centroidalMomentumMatrix = new CentroidalMomentumCalculator(elevator, centerOfMassFrame);
      centroidalMomentumMatrix.reset();

      Momentum comMomentum = computeCoMMomentum(elevator, centerOfMassFrame, centroidalMomentumMatrix);

      Point3D comPoint = new Point3D();
      Vector3D linearMomentum = new Vector3D();
      Vector3D angularMomentum = new Vector3D();
      robot.computeCOMMomentum(comPoint, linearMomentum, angularMomentum);

      EuclidCoreTestTools.assertTuple3DEquals(linearMomentum, comMomentum.getLinearPart(), 1e-12);
      EuclidCoreTestTools.assertTuple3DEquals(angularMomentum, comMomentum.getAngularPart(), 1e-12);
   }

	@Test
   public void testFloatingBody()
   {
      Random random = new Random(167L);

      double mass = random.nextDouble();
      Matrix3D momentOfInertia = RandomGeometry.nextDiagonalMatrix3D(random);
      Vector3D comOffset = RandomGeometry.nextVector3D(random);

      Robot robot = new Robot("robot");
      FloatingJoint rootJoint = new FloatingJoint("rootJoint", new Vector3D(), robot);
      Link link = new Link("link");
      link.setMass(mass);
      link.setMomentOfInertia(momentOfInertia);
      link.setComOffset(comOffset);
      rootJoint.setLink(link);
      robot.addRootJoint(rootJoint);

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      RigidBodyBasics elevator = new RigidBody("elevator", worldFrame);
      SixDoFJoint sixDoFJoint = new SixDoFJoint("sixDoFJoint", elevator);
      new RigidBody("rigidBody", sixDoFJoint, momentOfInertia, mass, comOffset);

      CenterOfMassReferenceFrame centerOfMassFrame = new CenterOfMassReferenceFrame("com", worldFrame, elevator);
      CentroidalMomentumCalculator centroidalMomentumMatrix = new CentroidalMomentumCalculator(elevator, centerOfMassFrame);

      int nTests = 10;
      for (int i = 0; i < nTests; i++)
      {
         Vector3D position = RandomGeometry.nextVector3D(random);
         RotationMatrix rotation = new RotationMatrix();
         rotation.setYawPitchRoll(random.nextDouble(), random.nextDouble(), random.nextDouble());
         Vector3D linearVelocityInBody = RandomGeometry.nextVector3D(random);
         Vector3D linearVelocityInWorld = new Vector3D(linearVelocityInBody);
         rotation.transform(linearVelocityInWorld);
         Vector3D angularVelocity = RandomGeometry.nextVector3D(random);

         rootJoint.setPosition(position);
         rootJoint.setRotation(rotation);
         rootJoint.setVelocity(linearVelocityInWorld);
         rootJoint.setAngularVelocityInBody(angularVelocity);
         robot.updateVelocities();
         Point3D comPoint = new Point3D();
         Vector3D linearMomentum = new Vector3D();
         Vector3D angularMomentum = new Vector3D();
         robot.computeCOMMomentum(comPoint, linearMomentum, angularMomentum);

         sixDoFJoint.setJointPosition(position);
         sixDoFJoint.setJointOrientation(rotation);
         Twist jointTwist = new Twist();
         jointTwist.setIncludingFrame(sixDoFJoint.getJointTwist());
         jointTwist.getAngularPart().set(angularVelocity);
         jointTwist.getLinearPart().set(linearVelocityInBody);
         sixDoFJoint.setJointTwist(jointTwist);
         elevator.updateFramesRecursively();

         centerOfMassFrame.update();

         centroidalMomentumMatrix.reset();
         Momentum comMomentum = computeCoMMomentum(elevator, centerOfMassFrame, centroidalMomentumMatrix);

         EuclidCoreTestTools.assertTuple3DEquals(linearMomentum, comMomentum.getLinearPart(), 1e-12);
         EuclidCoreTestTools.assertTuple3DEquals(angularMomentum, comMomentum.getAngularPart(), 1e-12);
      }
   }

   public static Momentum computeCoMMomentum(RigidBodyBasics elevator, ReferenceFrame centerOfMassFrame, CentroidalMomentumCalculator centroidalMomentumMatrix)
   {
      DMatrixRMaj mat = centroidalMomentumMatrix.getCentroidalMomentumMatrix();
      JointBasics[] jointList = MultiBodySystemTools.collectSubtreeJoints(elevator);

      DMatrixRMaj jointVelocities = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(jointList), 1);
      MultiBodySystemTools.extractJointsState(jointList, JointStateType.VELOCITY, jointVelocities);

      DMatrixRMaj comMomentumMatrix = MatrixTools.mult(mat, jointVelocities);

      Momentum comMomentum = new Momentum(centerOfMassFrame, comMomentumMatrix);

      return comMomentum;
   }
}
