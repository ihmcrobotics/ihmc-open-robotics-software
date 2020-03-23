package us.ihmc.robotics.physics;

import static org.junit.jupiter.api.Assertions.*;

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
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DBasics;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.interfaces.SpatialAccelerationReadOnly;
import us.ihmc.mecano.tools.*;

public class SingleContactImpulseCalculatorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 10000;
   private static final double EPSILON = 2.0e-12;
   private static final double POST_IMPULSE_VELOCITY_EPSILON = 5.0e-11;
   private static final double GAMMA = 1.0e-10;

   @Test
   public void testFloatingBodyAgainstEnvironment() throws Throwable
   {
      Random random = new Random(57476);

      for (int i = 0; i < ITERATIONS; i++)
      {// Collision RobotA <=> Environment
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-6, 1.0e-2);
         Vector3DReadOnly gravity = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.0, 15.0));

         RigidBodyBasics singleBodyA = nextSingleFloatingRigidBody(random, "shoup");

         CollisionResult collisionResult = nextCollisionResult(random, singleBodyA);

         ForwardDynamicsCalculator forwardDynamicsCalculatorA = setupForwardDynamicsCalculator(gravity, singleBodyA);

         FrameVector3D contactLinearVelocityPreImpulse = predictContactVelocity(dt, collisionResult, forwardDynamicsCalculatorA, null);

         SingleContactImpulseCalculator impulseCalculator = new SingleContactImpulseCalculator(collisionResult, worldFrame, forwardDynamicsCalculatorA, null);
         impulseCalculator.setSpringConstant(0.0);
         impulseCalculator.setTolerance(GAMMA);
         impulseCalculator.initialize(dt);
         impulseCalculator.updateInertia(null, null);
         impulseCalculator.computeImpulse(dt);
         updateVelocities(impulseCalculator, dt);

         double normalVelocityMagnitudePreImpulse = contactLinearVelocityPreImpulse.dot(collisionResult.getCollisionAxisForA());
         assertEquals(normalVelocityMagnitudePreImpulse < 0.0, impulseCalculator.isConstraintActive(), "Iteration " + i);

         assertContactResponseProperties("Iteration " + i, dt, contactLinearVelocityPreImpulse, impulseCalculator, EPSILON, POST_IMPULSE_VELOCITY_EPSILON);
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

         ForwardDynamicsCalculator forwardDynamicsCalculatorA = setupForwardDynamicsCalculator(gravity, singleBodyA);
         ForwardDynamicsCalculator forwardDynamicsCalculatorB = setupForwardDynamicsCalculator(gravity, singleBodyB);

         FrameVector3D contactLinearVelocityPreImpulse = predictContactVelocity(dt, collisionResult, forwardDynamicsCalculatorA, forwardDynamicsCalculatorB);

         SingleContactImpulseCalculator impulseCalculator = new SingleContactImpulseCalculator(collisionResult,
                                                                                               worldFrame,
                                                                                               forwardDynamicsCalculatorA,
                                                                                               forwardDynamicsCalculatorB);
         impulseCalculator.setSpringConstant(0.0);
         impulseCalculator.setTolerance(GAMMA);
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

         updateVelocities(impulseCalculator, dt);

         double normalVelocityMagnitudePreImpulse = contactLinearVelocityPreImpulse.dot(collisionResult.getCollisionAxisForA());
         assertEquals(normalVelocityMagnitudePreImpulse < 0.0, impulseCalculator.isConstraintActive(), "Iteration " + i);
         assertContactResponseProperties("Iteration " + i, dt, contactLinearVelocityPreImpulse, impulseCalculator, EPSILON, POST_IMPULSE_VELOCITY_EPSILON);
      }
   }

   static void assertContactResponseProperties(String messagePrefix, double dt, FrameVector3D contactLinearVelocityPreImpulse,
                                               SingleContactImpulseCalculator impulseCalculator, double epsilon, double postImpulseVelocityEpsilon)
         throws Throwable
   {
      CollisionResult collisionResult = impulseCalculator.getCollisionResult();
      FrameVector3D collisionAxisForA = collisionResult.getCollisionAxisForA();
      RigidBodyBasics bodyA = collisionResult.getCollidableA().getRigidBody();
      RigidBodyBasics bodyB = collisionResult.getCollidableB().getRigidBody();
      DenseMatrix64F jointVelocityChangeA = impulseCalculator.getJointVelocityChangeA();
      DenseMatrix64F jointVelocityChangeB = impulseCalculator.getJointVelocityChangeB();

      double normalVelocityMagnitudePreImpulse = contactLinearVelocityPreImpulse.dot(collisionAxisForA);

      if (impulseCalculator.isConstraintActive())
      { // Contact was closing
         FrameVector3D contactLinearVelocityPostImpulse = computeContactVelocity(dt, collisionResult);
         double normalVelocityPostImpulse = contactLinearVelocityPostImpulse.dot(collisionAxisForA);
         assertEquals(0.0, normalVelocityPostImpulse, Math.max(1.0, Math.abs(normalVelocityMagnitudePreImpulse)) * postImpulseVelocityEpsilon, messagePrefix);

         assertEquals(bodyA.getBodyFixedFrame(), impulseCalculator.getContactImpulseA().getBodyFrame());
         FrameVector3D impulseOnA = new FrameVector3D(impulseCalculator.getContactImpulseA().getLinearPart());
         EuclidCoreTestTools.assertTuple3DIsSetToZero(messagePrefix, impulseCalculator.getContactImpulseA().getAngularPart());
         EuclidCoreTestTools.assertTuple3DIsSetToZero(messagePrefix, impulseCalculator.getContactImpulseB().getAngularPart());

         if (bodyB != null)
         {
            assertEquals(bodyB.getBodyFixedFrame(), impulseCalculator.getContactImpulseB().getBodyFrame());

            FrameVector3D impulseOnB = new FrameVector3D(impulseCalculator.getContactImpulseB().getLinearPart());
            impulseOnB.changeFrame(impulseOnA.getReferenceFrame());
            impulseOnB.negate();
            EuclidCoreTestTools.assertTuple3DEquals(messagePrefix, impulseOnA, impulseOnB, epsilon);
         }
         else
         {
            EuclidCoreTestTools.assertTuple3DIsSetToZero(messagePrefix, impulseCalculator.getContactImpulseB().getLinearPart());
         }

         FrameVector3D impulseNormal = extractNormalPart(impulseOnA, collisionAxisForA);
         FrameVector3D impulseTangential = extractTangentialPart(impulseOnA, collisionAxisForA);
         impulseNormal.changeFrame(worldFrame);
         impulseTangential.changeFrame(worldFrame);

         boolean isSticking = impulseTangential.length() < (impulseCalculator.getCoefficientOfFriction() - epsilon) * impulseNormal.length();

         if (isSticking)
         {
            assertEquals(0.0,
                         contactLinearVelocityPostImpulse.length(),
                         Math.max(1.0, contactLinearVelocityPreImpulse.length()) * postImpulseVelocityEpsilon,
                         messagePrefix);
         }
         else
         {
            assertEquals(impulseTangential.length(),
                         impulseCalculator.getCoefficientOfFriction() * impulseNormal.length(),
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
      FrameVector3D contactLinearVelocityPreImpulse = computeBodyPointVelocity(dt, singleBodyA, pointInBodyFrameA, forwardDynamicsCalculatorA);

      RigidBodyBasics singleBodyB = collisionResult.getCollidableB().getRigidBody();
      if (singleBodyB != null)
      {
         FramePoint3D pointInBodyFrameB = collisionResult.getCollisionData().getPointOnB();
         contactLinearVelocityPreImpulse.sub(computeBodyPointVelocity(dt, singleBodyB, pointInBodyFrameB, forwardDynamicsCalculatorB));
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
      DenseMatrix64F jointAccelerationMatrix = forwardDynamicsCalculator.getJointAccelerationMatrix();
      CommonOps.addEquals(jointVelocityMatrix, jointVelocityChange);
      CommonOps.addEquals(jointVelocityMatrix, dt, jointAccelerationMatrix);
      MultiBodySystemTools.insertJointsState(joints, JointStateType.VELOCITY, jointVelocityMatrix);
      rootBody.updateFramesRecursively();
   }

   static FrameVector3D computeBodyPointVelocity(double dt, RigidBodyBasics rigidBody, FramePoint3D bodyFixedPoint)
   {
      return computeBodyPointVelocity(dt, rigidBody, bodyFixedPoint, null);
   }

   static FrameVector3D computeBodyPointVelocity(double dt, RigidBodyBasics rigidBody, FramePoint3D bodyFixedPoint,
                                                 ForwardDynamicsCalculator forwardDynamicsCalculator)
   {
      FrameVector3D contactLinearVelocity = new FrameVector3D();
      RigidBodyReadOnly rootBody = MultiBodySystemTools.getRootBody(rigidBody);
      rigidBody.getBodyFixedFrame().getTwistOfFrame().getLinearVelocityAt(bodyFixedPoint, contactLinearVelocity);
      if (forwardDynamicsCalculator != null)
      {
         FrameVector3D contactLinearAcceleration = new FrameVector3D();
         SpatialAccelerationReadOnly bodyAcceleration = forwardDynamicsCalculator.getAccelerationProvider().getRelativeAcceleration(rootBody, rigidBody);
         bodyAcceleration.getLinearAccelerationAt(null, bodyFixedPoint, contactLinearAcceleration);
         contactLinearVelocity.scaleAdd(dt, contactLinearAcceleration, contactLinearVelocity);
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
      return new Collidable(null, -1, -1, EuclidShapeRandomTools.nextShape3D(random), worldFrame);
   }

   static Collidable nextCollidable(Random random, RigidBodyBasics rigidBody)
   {
      Shape3DBasics shape = EuclidShapeMissingRandomTools.nextConvexShape3D(random, rigidBody.getInertia().getCenterOfMassOffset());
      return new Collidable(rigidBody, -1, -1, shape, rigidBody.getBodyFixedFrame());
   }
}
