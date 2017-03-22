package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.RandomMatrices;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ScrewTestTools
{
   public static List<RevoluteJoint> createRandomChainRobot(int numberOfJoints, Random random)
   {
      return createRandomChainRobot("", numberOfJoints, random);
   }

   public static List<RevoluteJoint> createRandomChainRobot(String prefix, int numberOfJoints, Random random)
   {
      return createRandomChainRobot(prefix, RandomGeometry.nextVector3DArray(random, numberOfJoints, 1.0), random);
   }

   public static List<RevoluteJoint> createRandomChainRobot(String prefix, Vector3D[] jointAxes, Random random)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame rootBodyFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("rootBodyFrame", worldFrame, new RigidBodyTransform());
      RigidBody rootBody = new RigidBody(prefix + "RootBody", rootBodyFrame);
      return createRandomChainRobot(prefix, rootBody, jointAxes, random);
   }

   public static List<RevoluteJoint> createRandomChainRobot(String prefix, RigidBody rootBody, Vector3D[] jointAxes, Random random)
   {
      List<RevoluteJoint> revoluteJoints = new ArrayList<>();
      createRandomChainRobot(prefix, revoluteJoints, rootBody, jointAxes, random);
      return revoluteJoints;
   }

   public static void createRandomChainRobot(String prefix, List<RevoluteJoint> revoluteJointsToPack, RigidBody rootBody, Vector3D[] jointAxes, Random random)
   {
      RigidBody currentIDBody = rootBody;
      for (int i = 0; i < jointAxes.length; i++)
      {
         RevoluteJoint currentIDJoint = addRandomRevoluteJoint(prefix + "joint" + i, jointAxes[i], random, currentIDBody);
         currentIDBody = addRandomRigidBody(prefix + "body" + i, random, currentIDJoint);
         revoluteJointsToPack.add(currentIDJoint);
      }

      rootBody.updateFramesRecursively();
   }

   public static List<PrismaticJoint> createRandomChainRobotWithPrismaticJoints(int numberOfJoints, Random random)
   {
      return createRandomChainRobotWithPrismaticJoints("", numberOfJoints, random);
   }

   public static List<PrismaticJoint> createRandomChainRobotWithPrismaticJoints(String prefix, int numberOfJoints, Random random)
   {
      return createRandomChainRobotWithPrismaticJoints(prefix, RandomGeometry.nextVector3DArray(random, numberOfJoints, 1.0), random);
   }

   public static List<PrismaticJoint> createRandomChainRobotWithPrismaticJoints(String prefix, Vector3D[] jointAxes, Random random)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame rootBodyFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("rootBodyFrame", worldFrame, new RigidBodyTransform());
      RigidBody rootBody = new RigidBody(prefix + "RootBody", rootBodyFrame);
      return createRandomChainRobotWithPrismaticJoints(prefix, rootBody, jointAxes, random);
   }

   public static List<PrismaticJoint> createRandomChainRobotWithPrismaticJoints(String prefix, RigidBody rootBody, Vector3D[] jointAxes, Random random)
   {
      List<PrismaticJoint> prismaticJoints = new ArrayList<>();
      createRandomChainRobotWithPrismaticJoints(prefix, prismaticJoints, rootBody, jointAxes, random);
      return prismaticJoints;
   }

   public static void createRandomChainRobotWithPrismaticJoints(String prefix, List<PrismaticJoint> prismaticJointsToPack, RigidBody rootBody,
                                                                Vector3D[] jointAxes, Random random)
   {
      RigidBody predecessor = rootBody;

      for (int i = 0; i < jointAxes.length; i++)
      {
         PrismaticJoint joint = addRandomPrismaticJoint(prefix + "Joint" + i, jointAxes[i], random, predecessor);
         prismaticJointsToPack.add(joint);
         predecessor = ScrewTestTools.addRandomRigidBody(prefix + "Body" + i, random, joint);
      }
   }

   public static List<OneDoFJoint> createRandomChainRobotWithOneDoFJoints(int numberOfJoints, Random random)
   {
      return createRandomChainRobotWithOneDoFJoints("", numberOfJoints, random);
   }

   public static List<OneDoFJoint> createRandomChainRobotWithOneDoFJoints(String prefix, int numberOfJoints, Random random)
   {
      return createRandomChainRobotWithOneDoFJoints(prefix, RandomGeometry.nextVector3DArray(random, numberOfJoints, 1.0), random);
   }

   public static List<OneDoFJoint> createRandomChainRobotWithOneDoFJoints(String prefix, Vector3D[] jointAxes, Random random)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame rootBodyFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("rootBodyFrame", worldFrame, new RigidBodyTransform());
      RigidBody rootBody = new RigidBody(prefix + "RootBody", rootBodyFrame);
      return createRandomChainRobotWithOneDoFJoints(prefix, rootBody, jointAxes, random);
   }

   public static List<OneDoFJoint> createRandomChainRobotWithOneDoFJoints(String prefix, RigidBody rootBody, Vector3D[] jointAxes, Random random)
   {
      List<OneDoFJoint> oneDoFJoints = new ArrayList<>();
      createRandomChainRobotWithOneDoFJoints(prefix, oneDoFJoints, rootBody, jointAxes, random);
      return oneDoFJoints;
   }

   public static void createRandomChainRobotWithOneDoFJoints(String prefix, List<OneDoFJoint> oneDoFJointsToPack, RigidBody rootBody,
                                                                Vector3D[] jointAxes, Random random)
   {
      RigidBody predecessor = rootBody;

      for (int i = 0; i < jointAxes.length; i++)
      {
         OneDoFJoint joint;
         if (random.nextBoolean())
            joint = addRandomPrismaticJoint(prefix + "Joint" + i, jointAxes[i], random, predecessor);
         else
            joint = addRandomRevoluteJoint(prefix + "Joint" + i, jointAxes[i], random, predecessor);
         oneDoFJointsToPack.add(joint);
         predecessor = ScrewTestTools.addRandomRigidBody(prefix + "Body" + i, random, joint);
      }
   }

   public static List<RevoluteJoint> createRandomTreeRobot(int numberOfJoints, Random random)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame rootBodyFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("rootBodyFrame", worldFrame, new RigidBodyTransform());
      RigidBody rootBody = new RigidBody("RootBody", rootBodyFrame);
      return createRandomTreeRobot(rootBody, numberOfJoints, random);
   }

   public static List<RevoluteJoint> createRandomTreeRobot(RigidBody rootBody, int numberOfJoints, Random random)
   {
      List<RevoluteJoint> joints = new ArrayList<>();
      createRandomTreeRobot(joints, rootBody, numberOfJoints, random);
      return joints;
   }

   public static void createRandomTreeRobot(List<RevoluteJoint> revoluteJointsToPack, RigidBody rootBody, int numberOfJoints, Random random)
   {
      ArrayList<RevoluteJoint> potentialInverseDynamicsParentJoints = new ArrayList<RevoluteJoint>(); // synchronized with potentialParentJoints

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

         RevoluteJoint currentJoint = addRandomRevoluteJoint("jointID" + i, random, inverseDynamicsParentBody);
         addRandomRigidBody("bodyID" + i, random, currentJoint);

         //         joints.add(currentJoint);
         potentialInverseDynamicsParentJoints.add(currentJoint);
      }

      InverseDynamicsJoint[] idJoints = ScrewTools.computeSubtreeJoints(rootBody);
      for (int i = 0; i < idJoints.length; i++)
      {
         if (idJoints[i] instanceof RevoluteJoint)
         {
            revoluteJointsToPack.add((RevoluteJoint) idJoints[i]);
         }
         else
         {
            throw new RuntimeException("Invalid joint type");
         }
      }
      rootBody.updateFramesRecursively();
   }

   public static List<PrismaticJoint> createRandomTreeRobotWithPrismaticJoints(int numberOfJoints, Random random)
   {
      return createRandomTreeRobotWithPrismaticJoints("", numberOfJoints, random);
   }

   public static List<PrismaticJoint> createRandomTreeRobotWithPrismaticJoints(String prefix, int numberOfJoints, Random random)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame rootBodyFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("rootBodyFrame", worldFrame, new RigidBodyTransform());
      RigidBody rootBody = new RigidBody(prefix + "RootBody", rootBodyFrame);
      return createRandomTreeRobotWithPrismaticJoints(prefix, rootBody, numberOfJoints, random);
   }

   public static List<PrismaticJoint> createRandomTreeRobotWithPrismaticJoints(String prefix, RigidBody rootBody, int numberOfJoints, Random random)
   {
      List<PrismaticJoint> tempJoints = new ArrayList<>();

      RigidBody predecessor = rootBody;

      for (int i = 0; i < numberOfJoints; i++)
      {
         PrismaticJoint joint = addRandomPrismaticJoint(prefix + "Joint" + i, random, predecessor);
         ScrewTestTools.addRandomRigidBody(prefix + "Body" + i, random, joint);
         tempJoints.add(joint);
         predecessor = tempJoints.get(random.nextInt(tempJoints.size())).getSuccessor();
      }

      PrismaticJoint[] jointArray = ScrewTools.filterJoints(ScrewTools.computeSubtreeJoints(rootBody), PrismaticJoint.class);
      return Arrays.asList(jointArray);
   }

   public static List<OneDoFJoint> createRandomTreeRobotWithOneDoFJoints(int numberOfJoints, Random random)
   {
      return createRandomTreeRobotWithOneDoFJoints("", numberOfJoints, random);
   }

   public static List<OneDoFJoint> createRandomTreeRobotWithOneDoFJoints(String prefix, int numberOfJoints, Random random)
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      ReferenceFrame rootBodyFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("rootBodyFrame", worldFrame, new RigidBodyTransform());
      RigidBody rootBody = new RigidBody(prefix + "RootBody", rootBodyFrame);
      return createRandomTreeRobotWithOneDoFJoints(prefix, rootBody, numberOfJoints, random);
   }

   public static List<OneDoFJoint> createRandomTreeRobotWithOneDoFJoints(String prefix, RigidBody rootBody, int numberOfJoints, Random random)
   {
      List<OneDoFJoint> tempJoints = new ArrayList<>();

      RigidBody predecessor = rootBody;

      for (int i = 0; i < numberOfJoints; i++)
      {
         OneDoFJoint joint;
         if (random.nextBoolean())
            joint = addRandomPrismaticJoint(prefix + "Joint" + i, random, predecessor);
         else
            joint = addRandomRevoluteJoint(prefix + "Joint" + i, random, predecessor);
         ScrewTestTools.addRandomRigidBody(prefix + "Body" + i, random, joint);
         tempJoints.add(joint);
         predecessor = tempJoints.get(random.nextInt(tempJoints.size())).getSuccessor();
      }

      OneDoFJoint[] jointArray = ScrewTools.filterJoints(ScrewTools.computeSubtreeJoints(rootBody), OneDoFJoint.class);
      return Arrays.asList(jointArray);
   }

   public static RevoluteJoint addRandomRevoluteJoint(String name, Random random, RigidBody predecessor)
   {
      Vector3D jointAxis = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      return addRandomRevoluteJoint(name, jointAxis, random, predecessor);
   }

   public static RevoluteJoint addRandomRevoluteJoint(String name, Vector3D jointAxis, Random random, RigidBody predecessor)
   {
      Vector3D jointOffset = RandomGeometry.nextVector3D(random);
      jointAxis.normalize();
      RevoluteJoint ret = ScrewTools.addRevoluteJoint(name, predecessor, jointOffset, jointAxis);

      return ret;
   }

   public static PrismaticJoint addRandomPrismaticJoint(String name, Random random, RigidBody predecessor)
   {
      Vector3D jointAxis = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      return addRandomPrismaticJoint(name, jointAxis, random, predecessor);
   }

   public static PrismaticJoint addRandomPrismaticJoint(String name, Vector3D jointAxis, Random random, RigidBody predecessor)
   {
      Vector3D jointOffset = RandomGeometry.nextVector3D(random);
      jointAxis.normalize();
      PrismaticJoint ret = ScrewTools.addPrismaticJoint(name, predecessor, jointOffset, jointAxis);

      return ret;
   }

   public static RigidBody addRandomRigidBody(String name, Random random, InverseDynamicsJoint parentJoint)
   {
      Matrix3D momentOfInertia = RandomGeometry.nextDiagonalMatrix3D(random);
      double mass = random.nextDouble();
      Vector3D comOffset = RandomGeometry.nextVector3D(random);
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
      joint.setQ(RandomNumbers.nextDouble(random, boundaryOne, boundaryTwo));
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
      joint.setQd(RandomNumbers.nextDouble(random, min, max));
   }

   public static void setRandomAcceleration(OneDoFJoint joint, Random random, double min, double max)
   {
      joint.setQdd(RandomNumbers.nextDouble(random, min, max));
   }

   public static void setRandomDesiredAcceleration(OneDoFJoint joint, Random random, double min, double max)
   {
      joint.setQddDesired(RandomNumbers.nextDouble(random, min, max));
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
         setRandomPosition(joint, random, -Math.PI / 2.0, Math.PI / 2.0);
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

   public static void setRandomAccelerations(Iterable<? extends OneDoFJoint> joints, Random random, double min, double max)
   {
      for (OneDoFJoint joint : joints)
      {
         setRandomAcceleration(joint, random, min, max);
      }
   }

   public static void setRandomDesiredAccelerations(Iterable<? extends OneDoFJoint> joints, Random random, double min, double max)
   {
      for (OneDoFJoint joint : joints)
      {
         setRandomDesiredAcceleration(joint, random, min, max);
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
         joint.setTau(RandomNumbers.nextDouble(random, magnitude));
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

      RotationMatrix rotation = new RotationMatrix();
      sixDoFJoint.getRotation(rotation);
      Vector3D position = new Vector3D();
      sixDoFJoint.getTranslation(position);

      integrate(rotation, position, dt, twist);

      sixDoFJoint.setRotation(rotation);
      sixDoFJoint.setPosition(position);
   }

   public static void integrate(RotationMatrix rotationToPack, Tuple3DBasics positionToPack, double dt, Twist twist)
   {
      twist.changeFrame(twist.getBodyFrame());
      Vector3D dPosition = new Vector3D();
      twist.getLinearPart(dPosition); // velocity in body frame
      rotationToPack.transform(dPosition); // velocity in base frame
      dPosition.scale(dt); // translation in base frame
      positionToPack.add(dPosition);

      Vector3D axis = new Vector3D();
      twist.getAngularPart(axis);
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

   public static void setRandomPositionAndOrientation(FloatingInverseDynamicsJoint rootJoint, Random random)
   {
      rootJoint.setPositionAndRotation(EuclidCoreRandomTools.generateRandomRigidBodyTransform(random));
   }

   public static void setRandomVelocity(FloatingInverseDynamicsJoint rootJoint, Random random)
   {
      Twist jointTwist = new Twist();
      rootJoint.getJointTwist(jointTwist);
      jointTwist.setAngularPart(RandomGeometry.nextVector3D(random));
      jointTwist.setLinearPart(RandomGeometry.nextVector3D(random));
      rootJoint.setJointTwist(jointTwist);
   }

   public static void setRandomAcceleration(SixDoFJoint rootJoint, Random random)
   {
      SpatialAccelerationVector jointAcceleration = new SpatialAccelerationVector();
      rootJoint.getJointAcceleration(jointAcceleration);
      jointAcceleration.setAngularPart(RandomGeometry.nextVector3D(random));
      jointAcceleration.setLinearPart(RandomGeometry.nextVector3D(random));
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

   public static void integrateAccelerations(SixDoFJoint sixDoFJoint, double dt)
   {
      SpatialAccelerationVector deltaTwist = new SpatialAccelerationVector();
      sixDoFJoint.getJointAcceleration(deltaTwist);
      deltaTwist.scale(dt);

      Twist rootJointTwist = new Twist();
      sixDoFJoint.getJointTwist(rootJointTwist);
      rootJointTwist.angularPart.add(deltaTwist.angularPart);
      rootJointTwist.linearPart.add(deltaTwist.linearPart);
      sixDoFJoint.setJointTwist(rootJointTwist);
   }

   public static void doubleIntegrateAccelerations(SixDoFJoint sixDoFJoint, double dt)
   {
      SpatialAccelerationVector deltaTwist = new SpatialAccelerationVector();
      sixDoFJoint.getJointAcceleration(deltaTwist);
      deltaTwist.scale(dt);

      Twist rootJointTwist = new Twist();
      sixDoFJoint.getJointTwist(rootJointTwist);
      rootJointTwist.angularPart.add(deltaTwist.angularPart);
      rootJointTwist.linearPart.add(deltaTwist.linearPart);
      sixDoFJoint.setJointTwist(rootJointTwist);


      Twist deltaConfiguration = new Twist();
      sixDoFJoint.getJointTwist(deltaConfiguration);

      RotationMatrix rotation = new RotationMatrix();
      sixDoFJoint.getRotation(rotation);
      Vector3D position = new Vector3D();
      sixDoFJoint.getTranslation(position);

      deltaTwist.scale(0.5 * dt);
      deltaConfiguration.angularPart.add(deltaTwist.getAngularPart());
      deltaConfiguration.linearPart.add(deltaTwist.getLinearPart());

      integrate(rotation, position, dt, deltaConfiguration);

      sixDoFJoint.setRotation(rotation);
      sixDoFJoint.setPosition(position);
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
      private final List<InverseDynamicsJoint> inverseDynamicsJoints = new ArrayList<>();

      public RandomFloatingChain(Random random, int numberOfRevoluteJoints)
      {
         this(random, RandomGeometry.nextVector3DArray(random, numberOfRevoluteJoints, 1.0));
      }

      public RandomFloatingChain(Random random, Vector3D[] jointAxes)
      {
         ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", ReferenceFrame.getWorldFrame(),
                                                                                                     new RigidBodyTransform());
         elevator = new RigidBody("elevator", elevatorFrame);

         rootJoint = new SixDoFJoint("rootJoint", elevator, elevatorFrame);
         RigidBody rootBody = ScrewTestTools.addRandomRigidBody("rootBody", random, rootJoint);

         revoluteJoints = new ArrayList<RevoluteJoint>();

         ScrewTestTools.createRandomChainRobot("test", revoluteJoints, rootBody, jointAxes, random);

         inverseDynamicsJoints.add(rootJoint);
         inverseDynamicsJoints.addAll(revoluteJoints);
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

      public List<InverseDynamicsJoint> getInverseDynamicsJoints()
      {
         return inverseDynamicsJoints;
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
