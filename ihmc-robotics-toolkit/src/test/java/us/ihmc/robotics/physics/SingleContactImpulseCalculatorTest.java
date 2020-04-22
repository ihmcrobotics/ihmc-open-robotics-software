package us.ihmc.robotics.physics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Arrays;
import java.util.List;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.junit.jupiter.api.Test;
import org.opentest4j.AssertionFailedError;

import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.algorithms.SpatialAccelerationCalculator;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyAccelerationProvider;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.mecano.tools.MomentOfInertiaFactory;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class SingleContactImpulseCalculatorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 10000;
   private static final double EPSILON = 2.0e-12;
   private static final double POST_IMPULSE_VELOCITY_EPSILON = 5.0e-11;
   private static final double GAMMA = 1.0e-10;

   @Test
   public void testComputeContactPointVelocity()
   {
      Random random = new Random(4564);
      //      MultiBodySystemStateIntegrator integrator = new MultiBodySystemStateIntegrator();

      for (int i = 0; i < ITERATIONS; i++)
      {
         double gravity = EuclidCoreRandomTools.nextDouble(random, -20.0, -1.0);
         double dt = random.nextDouble();
         int numberOfJoints = random.nextInt(50) + 1;
         List<OneDoFJoint> joints = MultiBodySystemRandomTools.nextOneDoFJointChain(random, numberOfJoints);

         for (JointStateType state : JointStateType.values())
            MultiBodySystemRandomTools.nextState(random, state, joints);

         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(joints.get(0).getPredecessor());
         rootBody.updateFramesRecursively();

         SpatialAccelerationCalculator spatialAccelerationCalculator = new SpatialAccelerationCalculator(rootBody, worldFrame, false);
         spatialAccelerationCalculator.setGravitionalAcceleration(gravity);

         RigidBodyBasics contactingBody = joints.get(random.nextInt(numberOfJoints)).getSuccessor();
         FramePoint3D contactPoint = EuclidFrameRandomTools.nextFramePoint3D(random, contactingBody.getBodyFixedFrame());

         FrameVector3D actualLinearVelocity = new FrameVector3D();

         SingleContactImpulseCalculator.computeContactPointVelocity(dt,
                                                                    rootBody,
                                                                    contactingBody,
                                                                    spatialAccelerationCalculator,
                                                                    contactPoint,
                                                                    actualLinearVelocity);

         for (OneDoFJoint joint : joints)
         {
            joint.setQd(joint.getQd() + dt * joint.getQdd());
         }

         //         integrator.setIntegrationDT(dt);
         //         integrator.doubleIntegrateFromAcceleration(joints);
         rootBody.updateFramesRecursively();

         FrameVector3D expectedLinearVelocity = new FrameVector3D();
         contactingBody.getBodyFixedFrame().getTwistOfFrame().getLinearVelocityAt(contactPoint, expectedLinearVelocity);

         EuclidFrameTestTools.assertFrameTuple3DEquals(expectedLinearVelocity, actualLinearVelocity, EPSILON);
      }
   }

   @Test
   public void testFloatingBodyAgainstEnvironment() throws Throwable
   {
      Random random = new Random(57476);

      for (int i = 0; i < ITERATIONS; i++)
      {// Collision RobotA <=> Environment
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-6, 1.0e-2);
         Vector3DReadOnly gravity = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 5.0, 40.0));

         RigidBodyBasics singleBodyA = nextSingleFloatingRigidBody(random, "shoup");
         RigidBodyBasics rootBodyA = MultiBodySystemTools.getRootBody(singleBodyA);

         CollisionResult collisionResult = nextCollisionResult(random, singleBodyA);

         ForwardDynamicsCalculator forwardDynamicsCalculatorA = setupForwardDynamicsCalculator(gravity, singleBodyA);

         FrameVector3D contactLinearVelocityNoImpulse = predictContactVelocity(dt, collisionResult, forwardDynamicsCalculatorA, null);
         double normalVelocityMagnitudePreImpulse = contactLinearVelocityNoImpulse.dot(collisionResult.getCollisionAxisForA());

         SingleContactImpulseCalculator impulseCalculator = new SingleContactImpulseCalculator(worldFrame, rootBodyA, forwardDynamicsCalculatorA, null, null);
         impulseCalculator.setCollision(collisionResult);
         impulseCalculator.setTolerance(GAMMA);
         impulseCalculator.setContactParameters(new ContactParameters(0.7, 0.0, 0.0, 1.0));
         impulseCalculator.initialize(dt);
         impulseCalculator.updateInertia(null, null);
         impulseCalculator.computeImpulse(dt);
         impulseCalculator.finalizeImpulse();
         updateVelocities(impulseCalculator, dt);

         assertEquals(normalVelocityMagnitudePreImpulse < 0.0, impulseCalculator.isConstraintActive(), "Iteration " + i);

         assertContactResponseProperties("Iteration " + i, dt, contactLinearVelocityNoImpulse, impulseCalculator, EPSILON, POST_IMPULSE_VELOCITY_EPSILON);
      }
   }

   @Test
   public void testFloatingBodyAgainstFloatingBody() throws Throwable
   {
      Random random = new Random(57476);

      for (int i = 0; i < ITERATIONS; i++)
      {// Collision RobotA <=> Environment
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-6, 1.0e-2);
         Vector3DReadOnly gravity = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.0, 15.0));

         RigidBodyBasics singleBodyA = nextSingleFloatingRigidBody(random, "shoup");
         RigidBodyBasics singleBodyB = nextSingleFloatingRigidBody(random, "kolop");
         CollisionResult collisionResult = nextCollisionResult(random, singleBodyA, singleBodyB);
         RigidBodyBasics rootBodyA = collisionResult.getCollidableA().getRootBody();
         RigidBodyBasics rootBodyB = collisionResult.getCollidableB().getRootBody();

         ForwardDynamicsCalculator forwardDynamicsCalculatorA = setupForwardDynamicsCalculator(gravity, singleBodyA);
         ForwardDynamicsCalculator forwardDynamicsCalculatorB = setupForwardDynamicsCalculator(gravity, singleBodyB);

         FrameVector3D contactLinearVelocityNoImpulse = predictContactVelocity(dt, collisionResult, forwardDynamicsCalculatorA, forwardDynamicsCalculatorB);

         SingleContactImpulseCalculator impulseCalculator = new SingleContactImpulseCalculator(worldFrame,
                                                                                               rootBodyA,
                                                                                               forwardDynamicsCalculatorA,
                                                                                               rootBodyB,
                                                                                               forwardDynamicsCalculatorB);
         impulseCalculator.setCollision(collisionResult);
         impulseCalculator.setTolerance(GAMMA);
         impulseCalculator.setContactParameters(new ContactParameters(0.7, 0.0, 0.0, 1.0));
         impulseCalculator.initialize(dt);
         impulseCalculator.updateInertia(null, null);

         try
         {
            impulseCalculator.computeImpulse(dt);
         }
         catch (IllegalStateException e)
         {
            throw new AssertionFailedError("Failed at iteration " + i, e);
         }
         impulseCalculator.finalizeImpulse();
         updateVelocities(impulseCalculator, dt);

         double normalVelocityMagnitudePreImpulse = contactLinearVelocityNoImpulse.dot(collisionResult.getCollisionAxisForA());
         assertEquals(normalVelocityMagnitudePreImpulse < 0.0, impulseCalculator.isConstraintActive(), "Iteration " + i);
         assertContactResponseProperties("Iteration " + i, dt, contactLinearVelocityNoImpulse, impulseCalculator, EPSILON, POST_IMPULSE_VELOCITY_EPSILON);
      }
   }

   static void assertContactResponseProperties(String messagePrefix, double dt, FrameVector3D contactLinearVelocityNoImpulse,
                                               SingleContactImpulseCalculator impulseCalculator, double epsilon, double postImpulseVelocityEpsilon)
         throws Throwable
   {
      CollisionResult collisionResult = impulseCalculator.getCollisionResult();
      FrameVector3D collisionAxisForA = collisionResult.getCollisionAxisForA();
      RigidBodyBasics bodyA = collisionResult.getCollidableA().getRigidBody();
      RigidBodyBasics bodyB = collisionResult.getCollidableB().getRigidBody();
      DenseMatrix64F jointVelocityChangeA = impulseCalculator.getJointVelocityChangeA();
      DenseMatrix64F jointVelocityChangeB = impulseCalculator.getJointVelocityChangeB();

      double normalVelocityMagnitudePreImpulse = contactLinearVelocityNoImpulse.dot(collisionAxisForA);

      if (impulseCalculator.isConstraintActive())
      { // Contact was closing
         FrameVector3D contactLinearVelocityPostImpulse = computeContactVelocity(dt, collisionResult);
         double normalVelocityPostImpulse = contactLinearVelocityPostImpulse.dot(collisionAxisForA);
         assertEquals(0.0, normalVelocityPostImpulse, Math.max(1.0, Math.abs(normalVelocityMagnitudePreImpulse)) * postImpulseVelocityEpsilon, messagePrefix);

         assertEquals(bodyA.getBodyFixedFrame(), impulseCalculator.getImpulseA().getBodyFrame());
         FrameVector3D impulseOnA = new FrameVector3D(impulseCalculator.getImpulseA().getLinearPart());
         EuclidCoreTestTools.assertTuple3DIsSetToZero(messagePrefix, impulseCalculator.getImpulseA().getAngularPart());
         EuclidCoreTestTools.assertTuple3DIsSetToZero(messagePrefix, impulseCalculator.getImpulseB().getAngularPart());

         if (bodyB != null)
         {
            assertEquals(bodyB.getBodyFixedFrame(), impulseCalculator.getImpulseB().getBodyFrame());

            FrameVector3D impulseOnB = new FrameVector3D(impulseCalculator.getImpulseB().getLinearPart());
            impulseOnB.changeFrame(impulseOnA.getReferenceFrame());
            impulseOnB.negate();
            EuclidCoreTestTools.assertTuple3DEquals(messagePrefix, impulseOnA, impulseOnB, epsilon);
         }
         else
         {
            EuclidCoreTestTools.assertTuple3DIsSetToZero(messagePrefix, impulseCalculator.getImpulseB().getLinearPart());
         }

         FrameVector3D impulseNormal = extractNormalPart(impulseOnA, collisionAxisForA);
         FrameVector3D impulseTangential = extractTangentialPart(impulseOnA, collisionAxisForA);
         impulseNormal.changeFrame(worldFrame);
         impulseTangential.changeFrame(worldFrame);

         boolean isSticking = impulseTangential.length() < (impulseCalculator.getContactParameters().getCoefficientOfFriction() - epsilon)
               * impulseNormal.length();

         if (isSticking)
         {
            assertEquals(0.0,
                         contactLinearVelocityPostImpulse.length(),
                         Math.max(1.0, contactLinearVelocityNoImpulse.length()) * postImpulseVelocityEpsilon,
                         messagePrefix);
         }
         else
         {
            assertEquals(impulseTangential.length(),
                         impulseCalculator.getContactParameters().getCoefficientOfFriction() * impulseNormal.length(),
                         Math.max(1.0, Math.abs(impulseTangential.length())) * epsilon,
                         messagePrefix);
            FrameVector3D tangentialVelocityPostImpulse = extractTangentialPart(contactLinearVelocityPostImpulse, collisionAxisForA);
            try
            {
               assertTrue(impulseTangential.dot(tangentialVelocityPostImpulse) < 0.0,
                          messagePrefix + ", " + impulseTangential.dot(tangentialVelocityPostImpulse));
            }
            catch (Throwable e)
            {
               impulseCalculator.printForUnitTest();
               throw e;
            }
         }
      }
      else
      {
         assertNull(jointVelocityChangeA, messagePrefix);
         if (bodyB != null)
            assertNull(jointVelocityChangeB, messagePrefix);
      }

      if (bodyB == null)
         assertNull(jointVelocityChangeB);
   }

   static ForwardDynamicsCalculator setupForwardDynamicsCalculator(Vector3DReadOnly gravity, RigidBodyBasics rigidBody)
   {
      ForwardDynamicsCalculator forwardDynamicsCalculator = new ForwardDynamicsCalculator(MultiBodySystemTools.getRootBody(rigidBody));
      forwardDynamicsCalculator.setGravitionalAcceleration(gravity);
      forwardDynamicsCalculator.compute();
      return forwardDynamicsCalculator;
   }

   static CollisionResult nextCollisionResult(Random random, RigidBodyBasics contactingBodyA)
   {
      return nextCollisionResult(random, contactingBodyA, null);
   }

   static CollisionResult nextCollisionResult(Random random, RigidBodyBasics contactingBodyA, RigidBodyBasics contactingBodyB)
   {
      CollisionResult collisionResult = new CollisionResult();
      collisionResult.setCollidableA(nextCollidable(random, contactingBodyA));
      FrameVector3D collisionAxisForA = collisionResult.getCollisionAxisForA();
      FramePoint3D pointInBodyFrameA = collisionResult.getCollisionData().getPointOnA();
      FramePoint3D pointInBodyFrameB = collisionResult.getCollisionData().getPointOnB();
      FramePoint3D pointOnARootFrame = collisionResult.getPointOnARootFrame();
      FramePoint3D pointOnBRootFrame = collisionResult.getPointOnBRootFrame();

      collisionAxisForA.setIncludingFrame(worldFrame, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0));
      pointInBodyFrameA.setIncludingFrame(nextPointWithinRadiusOfCoM(random, worldFrame, contactingBodyA, 0.5));
      pointOnARootFrame.setIncludingFrame(pointInBodyFrameA);
      pointInBodyFrameA.changeFrame(contactingBodyA.getBodyFixedFrame());

      if (contactingBodyB != null)
      {
         collisionResult.setCollidableB(nextCollidable(random, contactingBodyB));
         pointInBodyFrameB.setIncludingFrame(pointInBodyFrameA);
         pointOnBRootFrame.setIncludingFrame(pointInBodyFrameB);
         pointOnBRootFrame.changeFrame(worldFrame);
         pointInBodyFrameB.changeFrame(contactingBodyB.getBodyFixedFrame());
      }
      else
      {
         collisionResult.setCollidableB(nextStaticCollidable(random));
      }

      return collisionResult;
   }

   static FrameVector3D computeContactVelocity(double dt, CollisionResult collisionResult)
   {
      return predictContactVelocity(dt, collisionResult, null, null);
   }

   static FrameVector3D predictContactVelocity(double dt, CollisionResult collisionResult, ForwardDynamicsCalculator forwardDynamicsCalculatorA,
                                               ForwardDynamicsCalculator forwardDynamicsCalculatorB)
   {
      RigidBodyBasics singleBodyA = collisionResult.getCollidableA().getRigidBody();
      FramePoint3D pointInBodyFrameA = collisionResult.getCollisionData().getPointOnA();

      FrameVector3D contactLinearVelocityPreImpulse;
      if (forwardDynamicsCalculatorA != null)
      {
         contactLinearVelocityPreImpulse = computeBodyPointVelocity(dt,
                                                                    singleBodyA,
                                                                    pointInBodyFrameA,
                                                                    forwardDynamicsCalculatorA.getAccelerationProvider(false));
      }
      else
      {
         contactLinearVelocityPreImpulse = computeBodyPointVelocity(dt, singleBodyA, pointInBodyFrameA);
      }

      RigidBodyBasics singleBodyB = collisionResult.getCollidableB().getRigidBody();
      if (singleBodyB != null)
      {
         FramePoint3D pointInBodyFrameB = collisionResult.getCollisionData().getPointOnB();
         if (forwardDynamicsCalculatorB != null)
         {
            contactLinearVelocityPreImpulse.sub(computeBodyPointVelocity(dt,
                                                                         singleBodyB,
                                                                         pointInBodyFrameB,
                                                                         forwardDynamicsCalculatorB.getAccelerationProvider(false)));
         }
         else
         {
            contactLinearVelocityPreImpulse.sub(computeBodyPointVelocity(dt, singleBodyB, pointInBodyFrameB));
         }
      }
      return contactLinearVelocityPreImpulse;
   }

   static void updateVelocities(SingleContactImpulseCalculator impulseCalculator, double dt)
   {
      if (!impulseCalculator.isConstraintActive())
         return;

      updateJointVelocities(dt,
                            impulseCalculator.getContactingBodyA(),
                            impulseCalculator.getForwardDynamicsCalculatorA(),
                            impulseCalculator.getJointVelocityChangeA());
      if (impulseCalculator.getContactingBodyB() != null)
         updateJointVelocities(dt,
                               impulseCalculator.getContactingBodyB(),
                               impulseCalculator.getForwardDynamicsCalculatorB(),
                               impulseCalculator.getJointVelocityChangeB());
   }

   static void updateJointVelocities(double dt, RigidBodyBasics rigidBody, ForwardDynamicsCalculator forwardDynamicsCalculator,
                                     DenseMatrix64F jointVelocityChange)
   {
      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(rigidBody);
      List<JointBasics> joints = Arrays.asList(MultiBodySystemTools.collectSubtreeJoints(rootBody));

      DenseMatrix64F jointVelocityMatrix = new DenseMatrix64F(MultiBodySystemTools.computeDegreesOfFreedom(joints), 1);
      MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, jointVelocityMatrix);
      CommonOps.addEquals(jointVelocityMatrix, jointVelocityChange);
      CommonOps.addEquals(jointVelocityMatrix, dt, forwardDynamicsCalculator.getJointAccelerationMatrix());
      MultiBodySystemTools.insertJointsState(joints, JointStateType.VELOCITY, jointVelocityMatrix);
      rootBody.updateFramesRecursively();
   }

   static FrameVector3D computeBodyPointVelocity(double dt, RigidBodyBasics rigidBody, FramePoint3D bodyFixedPoint)
   {
      return computeBodyPointVelocity(dt, rigidBody, bodyFixedPoint, null);
   }

   static FrameVector3D computeBodyPointVelocity(double dt, RigidBodyBasics rigidBody, FramePoint3D bodyFixedPoint,
                                                 RigidBodyAccelerationProvider noVelocityRigidBodyAccelerationProvider)
   {
      FrameVector3D contactLinearVelocity = new FrameVector3D();
      RigidBodyReadOnly rootBody = MultiBodySystemTools.getRootBody(rigidBody);
      if (noVelocityRigidBodyAccelerationProvider != null)
      {
         SingleContactImpulseCalculator.computeContactPointVelocity(dt,
                                                                    rootBody,
                                                                    rigidBody,
                                                                    noVelocityRigidBodyAccelerationProvider,
                                                                    bodyFixedPoint,
                                                                    contactLinearVelocity);
      }
      else
      {
         rigidBody.getBodyFixedFrame().getTwistOfFrame().getLinearVelocityAt(bodyFixedPoint, contactLinearVelocity);
      }
      contactLinearVelocity.changeFrame(worldFrame);
      return contactLinearVelocity;
   }

   static FrameVector3D extractNormalPart(FrameVector3DReadOnly input, FrameVector3DReadOnly normalAxis)
   {
      FrameVector3D axisCopy = new FrameVector3D(normalAxis);
      axisCopy.changeFrame(input.getReferenceFrame());
      FrameVector3D normalPart = new FrameVector3D(input.getReferenceFrame());
      normalPart.setAndScale(input.dot(axisCopy), axisCopy);
      return normalPart;
   }

   static FrameVector3D extractTangentialPart(FrameVector3DReadOnly input, FrameVector3DReadOnly normalAxis)
   {
      FrameVector3D axisCopy = new FrameVector3D(normalAxis);
      axisCopy.changeFrame(input.getReferenceFrame());
      FrameVector3D tangentialPart = new FrameVector3D(input.getReferenceFrame());
      tangentialPart.sub(input, extractNormalPart(input, normalAxis));
      return tangentialPart;
   }

   static FramePoint3D nextPointWithinRadiusOfCoM(Random random, ReferenceFrame referenceFrame, RigidBodyBasics rigidBody, double radius)
   {
      FramePoint3D next = new FramePoint3D(EuclidFrameRandomTools.nextFrameVector3DWithFixedLength(random,
                                                                                                   rigidBody.getBodyFixedFrame(),
                                                                                                   EuclidCoreRandomTools.nextDouble(random, 0.0, radius)));
      next.changeFrame(referenceFrame);
      return next;
   }

   static RigidBodyBasics nextSingleFloatingRigidBody(Random random, String name)
   {
      RigidBody rootBody = new RigidBody(name + "RootBody", worldFrame);
      SixDoFJoint floatingJoint = MultiBodySystemRandomTools.nextSixDoFJoint(random, name + "RootJoint", rootBody);
      RigidBody floatingBody = MultiBodySystemRandomTools.nextRigidBody(random, name + "Body", floatingJoint);
      floatingJoint.setSuccessor(floatingBody);
      floatingJoint.getJointPose().set(EuclidGeometryRandomTools.nextPose3D(random));
      floatingJoint.getJointTwist().set(MecanoRandomTools.nextSpatialVector(random, floatingJoint.getFrameAfterJoint()));
      //      floatingJoint.getJointWrench().set(MecanoRandomTools.nextSpatialForce(random, floatingJoint.getFrameAfterJoint()));

      double density = 8000.0 * random.nextDouble();
      Vector3D size = new Vector3D(random.nextDouble(), random.nextDouble(), random.nextDouble());
      double mass = density * size.getX() * size.getY() * size.getZ();
      floatingBody.getInertia().setMass(mass);
      floatingBody.getInertia().getMomentOfInertia().set(MomentOfInertiaFactory.solidBox(mass, size));
      rootBody.updateFramesRecursively();
      return floatingBody;
   }

   static Collidable nextStaticCollidable(Random random)
   {
      return new Collidable(null, -1, -1, EuclidFrameShapeRandomTools.nextFrameShape3D(random, worldFrame));
   }

   static Collidable nextCollidable(Random random, RigidBodyBasics rigidBody)
   {
      FrameShape3DBasics shape = EuclidFrameShapeRandomTools.nextFrameConvexShape3D(random,
                                                                               rigidBody.getBodyFixedFrame(),
                                                                               rigidBody.getInertia().getCenterOfMassOffset());
      return new Collidable(rigidBody, -1, -1, shape);
   }
}
