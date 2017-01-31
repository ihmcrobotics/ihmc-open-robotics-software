package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.RandomMatrices;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Tuple3d;
import javax.vecmath.Vector3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class ScrewTestTools
{
   public static void createRandomChainRobot(String prefix, List<RevoluteJoint> joints, RigidBody rootBody, Vector3d[] jointAxes, Random random)
   {
      RigidBody currentIDBody = rootBody;
      for (int i = 0; i < jointAxes.length; i++)
      {
         RevoluteJoint currentIDJoint = addRandomRevoluteJoint(prefix + "joint" + i, jointAxes[i], random, currentIDBody);
         currentIDBody = addRandomRigidBody(prefix + "body" + i, random, currentIDJoint);
         joints.add(currentIDJoint);
      }

      rootBody.updateFramesRecursively();
   }

   public static void createRandomTreeRobot(List<RevoluteJoint> joints, RigidBody rootBody, int numberOfJoints, Random random)
   {
      ArrayList<RevoluteJoint> potentialInverseDynamicsParentJoints = new ArrayList<RevoluteJoint>();    // synchronized with potentialParentJoints

      for (int i = 0; i < numberOfJoints; i++)
      {
         RigidBody inverseDynamicsParentBody;
         if (potentialInverseDynamicsParentJoints.isEmpty())
         {
            inverseDynamicsParentBody = rootBody;
         }
         else
         {
            int parentIndex = random.nextInt(potentialInverseDynamicsParentJoints.size());
            RevoluteJoint inverseDynamicsParentJoint = potentialInverseDynamicsParentJoints.get(parentIndex);
            inverseDynamicsParentBody = inverseDynamicsParentJoint.getSuccessor();
         }

         RevoluteJoint currentJoint = addRandomRevoluteJoint("jointID" + i,  random, inverseDynamicsParentBody);
         addRandomRigidBody("bodyID" + i, random, currentJoint);

//         joints.add(currentJoint);
         potentialInverseDynamicsParentJoints.add(currentJoint);
      }

      InverseDynamicsJoint[] idJoints = ScrewTools.computeSubtreeJoints(rootBody);
      for(int i = 0;i<idJoints.length;i++)
      {
         if(idJoints[i] instanceof RevoluteJoint)
         {
            joints.add((RevoluteJoint) idJoints[i]);
         }
         else
         {
            throw new RuntimeException("Invalid joint type");
         }
      }
      rootBody.updateFramesRecursively();
   }

   public static RevoluteJoint addRandomRevoluteJoint(String name, Random random, RigidBody predecessor)
   {
      Vector3d jointAxis = new Vector3d(random.nextDouble(), random.nextDouble(), random.nextDouble());
      return addRandomRevoluteJoint(name, jointAxis, random, predecessor);
   }

   public static RevoluteJoint addRandomRevoluteJoint(String name, Vector3d jointAxis, Random random, RigidBody predecessor)
   {
      Vector3d jointOffset = RandomTools.generateRandomVector(random);
      jointAxis.normalize();
      RevoluteJoint ret = ScrewTools.addRevoluteJoint(name, predecessor, jointOffset, jointAxis);
      
      return ret;
   }

   public static RigidBody addRandomRigidBody(String name, Random random, InverseDynamicsJoint parentJoint)
   {
      Matrix3d momentOfInertia = RandomTools.generateRandomDiagonalMatrix3d(random);
      double mass = random.nextDouble();
      Vector3d comOffset = RandomTools.generateRandomVector(random);
      RigidBody ret = ScrewTools.addRigidBody(name, parentJoint, momentOfInertia, mass, comOffset);

      return ret;
   }

   public static void setRandomPositions(OneDoFJoint[] joints, Random random, double boundaryOne, double boundaryTwo)
   {
      for (OneDoFJoint joint : joints)
      {
         setRandomPosition(joint, random, boundaryOne, boundaryTwo);
      }
   }

   public static void setRandomPositionsWithinJointLimits(OneDoFJoint[] joints, Random random)
   {
      for (OneDoFJoint joint : joints)
      {
         setRandomPositionWithinJointLimits(joint, random);
      }
   }

   public static void setRandomPosition(OneDoFJoint joint, Random random, double boundaryOne, double boundaryTwo)
   {
      joint.setQ(RandomTools.generateRandomDouble(random, boundaryOne, boundaryTwo));
   }

   public static void setRandomPositionWithinJointLimits(OneDoFJoint joint, Random random)
   {
      setRandomPosition(joint, random, joint.getJointLimitLower(), joint.getJointLimitUpper());
   }

   public static void setRandomVelocities(OneDoFJoint[] joints, Random random)
   {
      for (OneDoFJoint joint : joints)
      {
         setRandomVelocity(joint, random);
      }
   }

   public static void setRandomVelocity(OneDoFJoint joint, Random random)
   {
      joint.setQd(random.nextDouble());
   }

   public static void setRandomVelocity(OneDoFJoint joint, Random random, double min, double max)
   {
      joint.setQd(RandomTools.generateRandomDouble(random, min, max));
   }

   public static void setRandomDesiredAccelerations(OneDoFJoint[] joints, Random random)
   {
      for (OneDoFJoint joint : joints)
      {
         joint.setQddDesired(random.nextDouble());
      }
   }

   public static void setRandomAccelerations(OneDoFJoint[] joints, Random random)
   {
      for (OneDoFJoint joint : joints)
      {
         joint.setQdd(random.nextDouble());
      }
   }

   public static void setRandomPositions(Iterable<? extends OneDoFJoint> joints, Random random)
   {
      for (OneDoFJoint joint : joints)
      {
         setRandomPosition(joint, random, -Math.PI/2.0, Math.PI/2.0);
      }
   }

   public static void setRandomPositions(Iterable<? extends OneDoFJoint> joints, Random random, double min, double max)
   {
      for (OneDoFJoint joint : joints)
      {
         setRandomPosition(joint, random, min, max);
      }
   }

   public static void setRandomVelocities(Iterable<? extends OneDoFJoint> joints, Random random)
   {
      for (OneDoFJoint joint : joints)
      {
         setRandomVelocity(joint, random);
      }
   }

   public static void setRandomVelocities(Iterable<? extends OneDoFJoint> joints, Random random, double min, double max)
   {
      for (OneDoFJoint joint : joints)
      {
         setRandomVelocity(joint, random, min, max);
      }
   }

   public static void setRandomDesiredAccelerations(Iterable<? extends OneDoFJoint> joints, Random random)
   {
      for (OneDoFJoint joint : joints)
      {
         joint.setQddDesired(random.nextDouble());
      }
   }

   public static void setRandomAccelerations(Iterable<? extends OneDoFJoint> joints, Random random)
   {
      for (OneDoFJoint joint : joints)
      {
         joint.setQdd(random.nextDouble());
      }
   }

   public static void setRandomTorques(Iterable<? extends OneDoFJoint> joints, Random random)
   {
      for (OneDoFJoint joint : joints)
      {
         joint.setTau(random.nextDouble());
      }
   }

   public static void setRandomTorques(Iterable<? extends OneDoFJoint> joints, Random random, double magnitude)
   {
      for (OneDoFJoint joint : joints)
      {
         joint.setTau(RandomTools.generateRandomDouble(random, magnitude));
      }
   }
   
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
      sixDoFJoint.getJointTwist(twist);

      Matrix3d rotation = new Matrix3d();
      sixDoFJoint.getRotation(rotation);
      Vector3d position = new Vector3d();
      sixDoFJoint.getTranslation(position);

      integrate(rotation, position, dt, twist);

      sixDoFJoint.setRotation(rotation);
      sixDoFJoint.setPosition(position);
   }

   public static void integrate(Matrix3d rotationToPack, Tuple3d positionToPack, double dt, Twist twist)
   {
      twist.changeFrame(twist.getBodyFrame());
      Vector3d dPosition = new Vector3d();
      twist.getLinearPart(dPosition);    // velocity in body frame
      rotationToPack.transform(dPosition);    // velocity in base frame
      dPosition.scale(dt);    // translation in base frame
      positionToPack.add(dPosition);

      Vector3d axis = new Vector3d();
      twist.getAngularPart(axis);
      axis.scale(dt);
      double angle = axis.length();
      if (angle > 0.0)
         axis.normalize();
      else
         axis.set(1.0, 0.0, 0.0);
      AxisAngle4d axisAngle = new AxisAngle4d(axis, angle);
      Matrix3d dRotation = new Matrix3d();
      dRotation.set(axisAngle);
      rotationToPack.mul(dRotation);
   }

   public static void integrateVelocities(Iterable<? extends OneDoFJoint> joints, double dt)
   {
      for (OneDoFJoint joint : joints)
      {
         integrateVelocity(joint, dt);
      }
   }

   public static void setRandomPositionAndOrientation(FloatingInverseDynamicsJoint rootJoint, Random random)
   {
      rootJoint.setPositionAndRotation(RigidBodyTransform.generateRandomTransform(random));
   }

   public static void setRandomVelocity(FloatingInverseDynamicsJoint rootJoint, Random random)
   {
      Twist jointTwist = new Twist();
      rootJoint.getJointTwist(jointTwist);
      jointTwist.setAngularPart(RandomTools.generateRandomVector(random));
      jointTwist.setLinearPart(RandomTools.generateRandomVector(random));
      rootJoint.setJointTwist(jointTwist);
   }

   public static void setRandomAcceleration(SixDoFJoint rootJoint, Random random)
   {
      SpatialAccelerationVector jointAcceleration = new SpatialAccelerationVector();
      rootJoint.getJointAcceleration(jointAcceleration);
      jointAcceleration.setAngularPart(RandomTools.generateRandomVector(random));
      jointAcceleration.setLinearPart(RandomTools.generateRandomVector(random));
      rootJoint.setAcceleration(jointAcceleration);
   }
   
   public static void copyDesiredAccelerationToActual(SixDoFJoint rootJoint)
   {
      SpatialAccelerationVector rootJointAcceleration = new SpatialAccelerationVector();
      rootJoint.getDesiredJointAcceleration(rootJointAcceleration);
      rootJoint.setAcceleration(rootJointAcceleration);
   }

   public static void copyDesiredAccelerationsToActual(Iterable<? extends OneDoFJoint> joints)
   {
      for (OneDoFJoint joint : joints)
      {
         joint.setQdd(joint.getQddDesired());
      }
   }

   public static void integrateAccelerations(SixDoFJoint rootJoint, double dt)
   {
      SpatialAccelerationVector deltaTwist = new SpatialAccelerationVector();
      rootJoint.getJointAcceleration(deltaTwist);
      deltaTwist.scale(dt);

      Twist rootJointTwist = new Twist();
      rootJoint.getJointTwist(rootJointTwist);
      rootJointTwist.angularPart.add(deltaTwist.angularPart);
      rootJointTwist.linearPart.add(deltaTwist.linearPart);
      rootJoint.setJointTwist(rootJointTwist);
   }

   public static void integrateAccelerations(Iterable<? extends OneDoFJoint> joints, double dt)
   {
      for (OneDoFJoint joint : joints)
      {
         joint.setQd(joint.getQd() + joint.getQdd() * dt);
      }
   }

   public static void setRandomVelocities(InverseDynamicsJoint[] joints, Random random)
   {
      for (InverseDynamicsJoint joint : joints)
      {
         DenseMatrix64F jointVelocity = new DenseMatrix64F(joint.getDegreesOfFreedom(), 1);
         RandomMatrices.setRandom(jointVelocity, random);
         joint.setVelocity(jointVelocity, 0);
      }
   }

   public static class RandomFloatingChain
   {
      private final RigidBody elevator;
      private final SixDoFJoint rootJoint;
      private final List<RevoluteJoint> revoluteJoints;

      public RandomFloatingChain(Random random, Vector3d[] jointAxes)
      {
         ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", ReferenceFrame.getWorldFrame(),
                                           new RigidBodyTransform());
         elevator = new RigidBody("elevator", elevatorFrame);

         rootJoint = new SixDoFJoint("rootJoint", elevator, elevatorFrame);
         RigidBody rootBody = ScrewTestTools.addRandomRigidBody("rootBody", random, rootJoint);

         revoluteJoints = new ArrayList<RevoluteJoint>();

         ScrewTestTools.createRandomChainRobot("test", revoluteJoints, rootBody, jointAxes, random);
      }

      public void setRandomPositionsAndVelocities(Random random)
      {
         ScrewTestTools.setRandomPositionAndOrientation(getRootJoint(), random);
         ScrewTestTools.setRandomVelocity(getRootJoint(), random);
         ScrewTestTools.setRandomPositions(getRevoluteJoints(), random);
         setRandomVelocities(getRevoluteJoints(), random);
         getElevator().updateFramesRecursively();
      }

      public void setRandomAccelerations(Random random)
      {
         ScrewTestTools.setRandomAcceleration(getRootJoint(), random);
         ScrewTestTools.setRandomAccelerations(getRevoluteJoints(), random);
      }

      public RigidBody getElevator()
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

      public RigidBody getLeafBody()
      {
         int nRevoluteJoints = revoluteJoints.size();
         if (nRevoluteJoints > 0)
            return revoluteJoints.get(nRevoluteJoints - 1).getSuccessor();
         else
            return rootJoint.getSuccessor();
      }
   }
}
