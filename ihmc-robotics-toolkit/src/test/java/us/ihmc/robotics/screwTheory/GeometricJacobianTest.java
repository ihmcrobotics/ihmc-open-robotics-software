package us.ihmc.robotics.screwTheory;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.List;
import java.util.Random;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.Joint;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoTestTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class GeometricJacobianTest
{
   private final Random random = new Random(101L);

   private GeometricJacobian bodyManipulatorJacobian;
   private GeometricJacobian spatialManipulatorJacobian;

   private PrismaticJoint joint1;
   private RevoluteJoint joint2;
   private PrismaticJoint joint3;



   @BeforeEach
   public void setUp() throws Exception
   {
      buildMechanismAndJacobians();

      // joint positions
      joint1.setQ(random.nextDouble());
      joint2.setQ(random.nextDouble());
      joint3.setQ(random.nextDouble());
   }

   @AfterEach
   public void tearDown() throws Exception
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testAgainstTwistCalculatorChainRobot() throws Exception
   {
      Random random = new Random(4324342L);

      int numberOfJoints = random.nextInt(100);

      List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getSuccessor());

      Twist expectedTwist = new Twist();
      Twist actualTwist = new Twist();

      for (int i = 0; i < 1000; i++)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, joints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, -10.0, 10.0, joints);
         rootBody.updateFramesRecursively();

         int randomEndEffectorIndex = random.nextInt(numberOfJoints);
         RigidBodyBasics randomEndEffector = joints.get(randomEndEffectorIndex).getSuccessor();
         GeometricJacobian rootJacobian = new GeometricJacobian(rootBody, randomEndEffector, randomEndEffector.getBodyFixedFrame());
         rootJacobian.compute();

         DMatrixRMaj jointVelocitiesMatrix = new DMatrixRMaj(rootJacobian.getNumberOfColumns(), 1);
         MultiBodySystemTools.extractJointsState(rootJacobian.getJointsInOrder(), JointStateType.VELOCITY, jointVelocitiesMatrix);

         randomEndEffector.getBodyFixedFrame().getTwistRelativeToOther(rootBody.getBodyFixedFrame(), expectedTwist);
         rootJacobian.getTwist(jointVelocitiesMatrix, actualTwist);

         MecanoTestTools.assertTwistEquals(expectedTwist, actualTwist, 1.0e-12);

         RigidBodyBasics randomBase = joints.get(random.nextInt(randomEndEffectorIndex + 1)).getPredecessor();
         GeometricJacobian jacobian = new GeometricJacobian(randomBase, randomEndEffector, randomEndEffector.getBodyFixedFrame());
         jacobian.compute();

         jointVelocitiesMatrix.reshape(jacobian.getNumberOfColumns(), 1);
         MultiBodySystemTools.extractJointsState(jacobian.getJointsInOrder(), JointStateType.VELOCITY, jointVelocitiesMatrix);

         randomEndEffector.getBodyFixedFrame().getTwistRelativeToOther(randomBase.getBodyFixedFrame(), expectedTwist);
         jacobian.getTwist(jointVelocitiesMatrix, actualTwist);

         MecanoTestTools.assertTwistEquals(expectedTwist, actualTwist, 1.0e-12);
      }
   }

   @Test
   public void testAgainstTwistCalculatorFloatingJointRobot() throws Exception
   {
      Random random = new Random(4324342L);

      int numberOfJoints = random.nextInt(100);

      RandomFloatingRevoluteJointChain floatingChain = new RandomFloatingRevoluteJointChain(random, numberOfJoints);
      SixDoFJoint floatingJoint = floatingChain.getRootJoint();
      List<RevoluteJoint> revoluteJoints = floatingChain.getRevoluteJoints();
      List<Joint> joints = floatingChain.getJoints();

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getSuccessor());

      Twist expectedTwist = new Twist();
      Twist actualTwist = new Twist();

      for (int i = 0; i < 1000; i++)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, floatingJoint);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, floatingJoint);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, revoluteJoints);
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, -10.0, 10.0, revoluteJoints);
         floatingChain.getElevator().updateFramesRecursively();

         int randomEndEffectorIndex = random.nextInt(numberOfJoints);
         RigidBodyBasics randomEndEffector = joints.get(randomEndEffectorIndex).getSuccessor();
         GeometricJacobian rootJacobian = new GeometricJacobian(rootBody, randomEndEffector, randomEndEffector.getBodyFixedFrame());
         rootJacobian.compute();

         DMatrixRMaj jointVelocitiesMatrix = new DMatrixRMaj(rootJacobian.getNumberOfColumns(), 1);
         MultiBodySystemTools.extractJointsState(rootJacobian.getJointsInOrder(), JointStateType.VELOCITY, jointVelocitiesMatrix);

         randomEndEffector.getBodyFixedFrame().getTwistRelativeToOther(rootBody.getBodyFixedFrame(), expectedTwist);
         rootJacobian.getTwist(jointVelocitiesMatrix, actualTwist);

         MecanoTestTools.assertTwistEquals(expectedTwist, actualTwist, 1.0e-12);

         RigidBodyBasics randomBase = joints.get(random.nextInt(randomEndEffectorIndex + 1)).getPredecessor();
         GeometricJacobian jacobian = new GeometricJacobian(randomBase, randomEndEffector, randomEndEffector.getBodyFixedFrame());
         jacobian.compute();

         jointVelocitiesMatrix.reshape(jacobian.getNumberOfColumns(), 1);
         MultiBodySystemTools.extractJointsState(jacobian.getJointsInOrder(), JointStateType.VELOCITY, jointVelocitiesMatrix);

         randomEndEffector.getBodyFixedFrame().getTwistRelativeToOther(randomBase.getBodyFixedFrame(), expectedTwist);
         jacobian.getTwist(jointVelocitiesMatrix, actualTwist);

         MecanoTestTools.assertTwistEquals(expectedTwist, actualTwist, 1.0e-12);
      }
   }

   /**
    * Tests computation of the Jacobian for a simple 2D manipulator, as shown in
    * Duindam - Port-based modeling and control for efficient bipedal robots, p.34
    * Jacobian was also computed manually using a Matlab script.
    */

	@Test
   public void testDuindamExample()
   {
      // compute
      joint1.getPredecessor().updateFramesRecursively();
      bodyManipulatorJacobian.compute();
      spatialManipulatorJacobian.compute();

      // set some joint velocities, and get the twist back
      double q1d = random.nextDouble();
      double q2d = random.nextDouble();
      double q3d = random.nextDouble();

      DMatrixRMaj qd = new DMatrixRMaj(3, 1);
      qd.set(0, 0, q1d);
      qd.set(1, 0, q2d);
      qd.set(2, 0, q3d);

      DMatrixRMaj twistMatrixFromBodyManipulatorJacobian = new DMatrixRMaj(Twist.SIZE, 1);
      CommonOps_DDRM.mult(bodyManipulatorJacobian.getJacobianMatrix(), qd, twistMatrixFromBodyManipulatorJacobian);
      Twist twistFromBodyManipulatorJacobian = new Twist(bodyManipulatorJacobian.getEndEffectorFrame(), bodyManipulatorJacobian.getBaseFrame(),
                                                  bodyManipulatorJacobian.getJacobianFrame(), twistMatrixFromBodyManipulatorJacobian);
      Vector3D omegaFromBodyManipulatorJacobian = new Vector3D(twistFromBodyManipulatorJacobian.getAngularPart());
      Vector3D vFromBodyManipulatorJacobian = new Vector3D(twistFromBodyManipulatorJacobian.getLinearPart());


      DMatrixRMaj twistMatrixFromSpatialManipulatorJacobian = new DMatrixRMaj(Twist.SIZE, 1);
      CommonOps_DDRM.mult(spatialManipulatorJacobian.getJacobianMatrix(), qd, twistMatrixFromSpatialManipulatorJacobian);

      Twist twistFromSpatialManipulatorJacobian = new Twist(spatialManipulatorJacobian.getEndEffectorFrame(), spatialManipulatorJacobian.getBaseFrame(),
                                                     spatialManipulatorJacobian.getJacobianFrame(), twistMatrixFromSpatialManipulatorJacobian);
      Vector3D omegaFromSpatialManipulatorJacobian = new Vector3D(twistFromSpatialManipulatorJacobian.getAngularPart());
      Vector3D vFromSpatialManipulatorJacobian = new Vector3D(twistFromSpatialManipulatorJacobian.getLinearPart());

      // compare to hand calculations
      double q1 = joint1.getQ();
      double q2 = joint2.getQ();
      double q3 = joint3.getQ();
      double epsilon = 1e-8;
      Vector3D omegaBodyByHand = new Vector3D(-q2d, 0.0, 0.0);
      Vector3D vBodyByHand = new Vector3D(0.0, -Math.cos(q2) * q1d + q3 * q2d, -Math.sin(q2) * q1d + q3d);

      Vector3D omegaBodyError = new Vector3D(omegaFromBodyManipulatorJacobian);
      omegaBodyError.sub(omegaBodyByHand);
      assertEquals(0.0, omegaBodyError.length(), epsilon);

      Vector3D vBodyError = new Vector3D(vFromBodyManipulatorJacobian);
      vBodyError.sub(vBodyByHand);
      assertEquals(0.0, vBodyError.length(), epsilon);

      Vector3D omegaSpatialByHand = new Vector3D(-q2d, 0.0, 0.0);
      Vector3D vSpatialByHand = new Vector3D(0.0, -q1d - 3.0 * q2d + Math.sin(q2) * q3d, -q1 * q2d + Math.cos(q2) * q3d);

      Vector3D omegaSpatialError = new Vector3D(omegaFromSpatialManipulatorJacobian);
      omegaSpatialError.sub(omegaSpatialByHand);
      assertEquals(0.0, omegaSpatialError.length(), epsilon);

      Vector3D vSpatialError = new Vector3D(vFromSpatialManipulatorJacobian);
      vSpatialError.sub(vSpatialByHand);
      assertEquals(0.0, vSpatialError.length(), epsilon);

      // rotate components of twistFromBodyManipulatorJacobian to base frame and compare to hand calculations again
      FrameVector3D omegaInBaseFrame = new FrameVector3D(twistFromBodyManipulatorJacobian.getAngularPart());
      omegaInBaseFrame.changeFrame(twistFromBodyManipulatorJacobian.getBaseFrame());
      FrameVector3D vInBaseFrame = new FrameVector3D(twistFromBodyManipulatorJacobian.getLinearPart());
      vInBaseFrame.changeFrame(twistFromBodyManipulatorJacobian.getBaseFrame());

      Vector3D omegaInBaseFrameByHand = new Vector3D(-q2d, 0.0, 0.0);
      Vector3D vInBaseFrameByHand = new Vector3D(0.0, -q1d + Math.cos(q2) * q3 * q2d + Math.sin(q2) * q3d, -Math.sin(q2) * q3 * q2d + Math.cos(q2) * q3d);

      omegaBodyError = new Vector3D(omegaInBaseFrame);
      omegaBodyError.sub(omegaInBaseFrameByHand);
      assertEquals(0.0, omegaBodyError.length(), epsilon);

      vBodyError = new Vector3D(vInBaseFrame);
      vBodyError.sub(vInBaseFrameByHand);
      assertEquals(0.0, vBodyError.length(), epsilon);
   }

   private void buildMechanismAndJacobians()
   {
      RigidBodyBasics base = new RigidBody("base", ReferenceFrame.getWorldFrame());
      joint1 = new PrismaticJoint("joint1", base, new Vector3D(), new Vector3D(0.0, -1.0, 0.0));
      RigidBodyBasics body1 = new RigidBody("body1", joint1, new Matrix3D(), 0.0, new Vector3D());
      joint2 = new RevoluteJoint("joint2", body1, new Vector3D(0.0, 0.0, 3.0), new Vector3D(-1.0, 0.0, 0.0));
      RigidBodyBasics body2 = new RigidBody("body2", joint2, new Matrix3D(), 0.0, new Vector3D());
      joint3 = new PrismaticJoint("joint4", body2, new Vector3D(), new Vector3D(0.0, 0.0, 1.0));
      RigidBodyBasics body3 = new RigidBody("body3", joint3, new Matrix3D(), 0.0, new Vector3D());

      // create Jacobians
      bodyManipulatorJacobian = new GeometricJacobian(base, body3, joint3.getFrameAfterJoint());
      spatialManipulatorJacobian = new GeometricJacobian(base, body3, joint1.getFrameBeforeJoint());
   }
}
