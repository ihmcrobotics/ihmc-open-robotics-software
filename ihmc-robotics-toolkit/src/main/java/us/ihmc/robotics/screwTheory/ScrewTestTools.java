package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.RandomMatrices;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.robotics.random.RandomGeometry;

public class ScrewTestTools
{
   public static void integrateVelocity(OneDoFJoint joint, double dt)
   {
      double oldQ = joint.getQ();
      double deltaQ = joint.getQd() * dt;
      double newQ = oldQ + deltaQ;
      joint.setQ(newQ);
   }

   public static void integrateVelocities(SixDoFJoint sixDoFJoint, double dt)
   {
      Twist twist = new Twist();
      twist.setIncludingFrame(sixDoFJoint.getJointTwist());

      RotationMatrix rotation = new RotationMatrix();
      rotation.set(sixDoFJoint.getJointPose().getOrientation());
      Vector3D position = new Vector3D();
      position.set(sixDoFJoint.getJointPose().getPosition());

      integrate(rotation, position, dt, twist);

      sixDoFJoint.setJointOrientation(rotation);
      sixDoFJoint.setJointPosition(position);
   }

   public static void integrate(RotationMatrix rotationToPack, Tuple3DBasics positionToPack, double dt, Twist twist)
   {
      twist.changeFrame(twist.getBodyFrame());
      Vector3D dPosition = new Vector3D();
      dPosition.set(twist.getLinearPart()); // velocity in body frame
      rotationToPack.transform(dPosition); // velocity in base frame
      dPosition.scale(dt); // translation in base frame
      positionToPack.add(dPosition);

      Vector3D axis = new Vector3D();
      axis.set(twist.getAngularPart());
      axis.scale(dt);
      double angle = axis.length();
      if (angle > 0.0)
         axis.normalize();
      else
         axis.set(1.0, 0.0, 0.0);
      AxisAngle axisAngle = new AxisAngle(axis, angle);
      RotationMatrix dRotation = new RotationMatrix();
      dRotation.set(axisAngle);
      rotationToPack.multiply(dRotation);
   }

   public static void integrateVelocities(Iterable<? extends OneDoFJoint> joints, double dt)
   {
      for (OneDoFJoint joint : joints)
      {
         integrateVelocity(joint, dt);
      }
   }

   public static void integrateAccelerations(SixDoFJoint sixDoFJoint, double dt)
   {
      SpatialAcceleration deltaTwist = new SpatialAcceleration();
      deltaTwist.setIncludingFrame(sixDoFJoint.getJointAcceleration());
      deltaTwist.scale(dt);

      Twist rootJointTwist = new Twist();
      rootJointTwist.setIncludingFrame(sixDoFJoint.getJointTwist());
      rootJointTwist.getAngularPart().add(deltaTwist.getAngularPart());
      rootJointTwist.getLinearPart().add(deltaTwist.getLinearPart());
      sixDoFJoint.setJointTwist(rootJointTwist);
   }

   public static void doubleIntegrateFromAcceleration(SixDoFJoint sixDoFJoint, double dt)
   {
      SpatialAcceleration deltaTwist = new SpatialAcceleration();
      deltaTwist.setIncludingFrame(sixDoFJoint.getJointAcceleration());
      deltaTwist.scale(dt);

      Twist rootJointTwist = new Twist();
      rootJointTwist.setIncludingFrame(sixDoFJoint.getJointTwist());
      rootJointTwist.getAngularPart().add(deltaTwist.getAngularPart());
      rootJointTwist.getLinearPart().add(deltaTwist.getLinearPart());
      sixDoFJoint.setJointTwist(rootJointTwist);

      Twist deltaConfiguration = new Twist();
      deltaConfiguration.setIncludingFrame(sixDoFJoint.getJointTwist());

      RotationMatrix rotation = new RotationMatrix();
      rotation.set(sixDoFJoint.getJointPose().getOrientation());
      Vector3D position = new Vector3D();
      position.set(sixDoFJoint.getJointPose().getPosition());

      deltaTwist.scale(0.5 * dt);
      deltaConfiguration.getAngularPart().add(deltaTwist.getAngularPart());
      deltaConfiguration.getLinearPart().add(deltaTwist.getLinearPart());

      integrate(rotation, position, dt, deltaConfiguration);

      sixDoFJoint.setJointOrientation(rotation);
      sixDoFJoint.setJointPosition(position);
   }

   public static void integrateAccelerations(Iterable<? extends OneDoFJoint> joints, double dt)
   {
      for (OneDoFJoint joint : joints)
      {
         joint.setQd(joint.getQd() + joint.getQdd() * dt);
      }
   }

   public static void setRandomVelocities(JointBasics[] joints, Random random)
   {
      for (JointBasics joint : joints)
      {
         DenseMatrix64F jointVelocity = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         RandomMatrices.setRandom(jointVelocity, random);
         joint.setJointVelocity(0, jointVelocity);
      }
   }

   public static class RandomFloatingChain
   {
      private final RigidBodyBasics elevator;
      private final SixDoFJoint rootJoint;
      private final List<RevoluteJoint> revoluteJoints;
      private final List<JointBasics> inverseDynamicsJoints = new ArrayList<>();

      public RandomFloatingChain(Random random, int numberOfRevoluteJoints)
      {
         this(random, RandomGeometry.nextVector3DArray(random, numberOfRevoluteJoints, 1.0));
      }

      public RandomFloatingChain(Random random, Vector3D[] jointAxes)
      {
         elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());

         rootJoint = new SixDoFJoint("rootJoint", elevator);
         RigidBodyBasics rootBody = MultiBodySystemRandomTools.nextRigidBody(random, "rootBody", rootJoint);

         revoluteJoints = new ArrayList<RevoluteJoint>();

         revoluteJoints.addAll(MultiBodySystemRandomTools.nextRevoluteJointChain(random, "test", rootBody, jointAxes));

         inverseDynamicsJoints.add(rootJoint);
         inverseDynamicsJoints.addAll(revoluteJoints);
      }

      public void setRandomPositionsAndVelocities(Random random)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, getRootJoint());
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, getRootJoint());
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, -Math.PI / 2.0, Math.PI / 2.0, getRevoluteJoints());
         MultiBodySystemRandomTools.nextState(random, JointStateType.VELOCITY, getRevoluteJoints());
         getElevator().updateFramesRecursively();
      }

      public void setRandomAccelerations(Random random)
      {
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, getRootJoint());
         MultiBodySystemRandomTools.nextState(random, JointStateType.ACCELERATION, getRevoluteJoints());
      }

      public RigidBodyBasics getElevator()
      {
         return elevator;
      }

      public SixDoFJoint getRootJoint()
      {
         return rootJoint;
      }

      public List<RevoluteJoint> getRevoluteJoints()
      {
         return revoluteJoints;
      }

      public List<JointBasics> getInverseDynamicsJoints()
      {
         return inverseDynamicsJoints;
      }

      public RigidBodyBasics getLeafBody()
      {
         int nRevoluteJoints = revoluteJoints.size();
         if (nRevoluteJoints > 0)
            return revoluteJoints.get(nRevoluteJoints - 1).getSuccessor();
         else
            return rootJoint.getSuccessor();
      }
   }
}
