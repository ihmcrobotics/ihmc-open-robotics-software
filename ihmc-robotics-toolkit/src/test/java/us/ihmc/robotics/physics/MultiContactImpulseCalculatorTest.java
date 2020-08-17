package us.ihmc.robotics.physics;

import static us.ihmc.robotics.physics.SingleContactImpulseCalculatorTest.computeContactVelocity;
import static us.ihmc.robotics.physics.SingleContactImpulseCalculatorTest.nextCollidable;
import static us.ihmc.robotics.physics.SingleContactImpulseCalculatorTest.nextSingleFloatingRigidBody;
import static us.ihmc.robotics.physics.SingleContactImpulseCalculatorTest.nextStaticCollidable;
import static us.ihmc.robotics.physics.SingleContactImpulseCalculatorTest.setupForwardDynamicsCalculator;

import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;
import java.util.function.Function;
import java.util.stream.Collectors;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.opentest4j.AssertionFailedError;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameUnitVector3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.algorithms.ForwardDynamicsCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * The {@link MultiContactImpulseCalculator} often doesn't converge, which is still to be debugged
 * and improved.
 */
public class MultiContactImpulseCalculatorTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final int ITERATIONS = 500;
   private static final double TERMINAL_TOLERANCE = 5.0e-10;
   private static final double SINGLE_CONTACT_GAMMA = 1.0e-10;
   private static final double EPSILON = 2.0e-12;
   private static final double POST_IMPULSE_VELOCITY_EPSILON = 5.0e-8;

   @Test
   public void testTwoFloatingBodies() throws Throwable
   {
      Random random = new Random(4363567);

      for (int i = 0; i < ITERATIONS; i++)
      { // singleFloatingBodyA <=> singleFloatingBodyB and singleFloatingBodyA <=> Environment
         double dt = EuclidCoreRandomTools.nextDouble(random, 1.0e-6, 1.0e-3);
         Vector3DReadOnly gravity = EuclidCoreRandomTools.nextVector3DWithFixedLength(random, EuclidCoreRandomTools.nextDouble(random, 0.0, 15.0));

         RigidBodyBasics bodyA = nextSingleFloatingRigidBody(random, "blopA");
         RigidBodyBasics bodyB = nextSingleFloatingRigidBody(random, "blopB");
         RigidBodyBasics rootA = MultiBodySystemTools.getRootBody(bodyA);
         RigidBodyBasics rootB = MultiBodySystemTools.getRootBody(bodyB);
         CollisionResult bodyAToEnvironment = nextCollisionResult(random, bodyA);
         CollisionResult bodyAToBodyB = nextCollisionResult(random, bodyA, bodyB);

         MultiRobotCollisionGroup collisionGroup = new MultiRobotCollisionGroup();
         collisionGroup.getRootBodies().add(rootA);
         collisionGroup.getRootBodies().add(rootB);
         collisionGroup.getGroupCollisions().add(bodyAToEnvironment);
         collisionGroup.getGroupCollisions().add(bodyAToBodyB);

         Map<RigidBodyBasics, PhysicsEngineRobotData> physicsEngineRobotDataMap = toPhysicsEngineRobotDataMap(dt, gravity, bodyA, bodyB);
         Map<RigidBodyBasics, ForwardDynamicsCalculator> robotForwardDynamicsCalculatorMap = physicsEngineRobotDataMap.entrySet().stream()
                                                                                                                      .collect(Collectors.toMap(Entry::getKey,
                                                                                                                                                e -> e.getValue()
                                                                                                                                                      .getForwardDynamicsPlugin()
                                                                                                                                                      .getForwardDynamicsCalculator()));

         Map<CollisionResult, FrameVector3D> contactLinearVelocitiesNoImpulse = predictContactVelocity(dt,
                                                                                                       collisionGroup.getGroupCollisions(),
                                                                                                       robotForwardDynamicsCalculatorMap);

         MultiContactImpulseCalculator multiContactImpulseCalculator = new MultiContactImpulseCalculator(worldFrame);
         multiContactImpulseCalculator.configure(physicsEngineRobotDataMap, collisionGroup);
         multiContactImpulseCalculator.setContactParameters(ContactParameters.defaultIneslasticContactParameters(false));
         multiContactImpulseCalculator.setTolerance(TERMINAL_TOLERANCE);
         multiContactImpulseCalculator.setSingleContactTolerance(SINGLE_CONTACT_GAMMA);
         try
         {
            multiContactImpulseCalculator.computeImpulses(0.0, dt, false);
            if (!multiContactImpulseCalculator.hasConverged())
            {
               System.err.println("Did not converge");
               continue;
            }
            System.out.println("Completed in " + multiContactImpulseCalculator.getNumberOfIterations() + " iterations.");
         }
         catch (IllegalStateException e)
         {
            throw new AssertionFailedError("Failed at iteration " + i, e);
         }

         updateVelocities(dt, multiContactImpulseCalculator, robotForwardDynamicsCalculatorMap);

         List<SingleContactImpulseCalculator> impulseCalculators = multiContactImpulseCalculator.getImpulseCalculators();

         for (int j = 0; j < impulseCalculators.size(); j++)
         {
            SingleContactImpulseCalculator impulseCalculator = impulseCalculators.get(j);
            FrameVector3D contactLinearVelocityNoImpulse = contactLinearVelocitiesNoImpulse.get(impulseCalculator.getCollisionResult());
            String messagePrefix = "Iteration " + i + ", calc. index " + j;
            SingleContactImpulseCalculatorTest.assertContactResponseProperties(messagePrefix,
                                                                               dt,
                                                                               contactLinearVelocityNoImpulse,
                                                                               impulseCalculator,
                                                                               EPSILON,
                                                                               POST_IMPULSE_VELOCITY_EPSILON);
         }
      }
   }

   public Map<RigidBodyBasics, PhysicsEngineRobotData> toPhysicsEngineRobotDataMap(double dt, Vector3DReadOnly gravity, RigidBodyBasics... rigidBodies)
   {
      HashMap<RigidBodyBasics, PhysicsEngineRobotData> map = new HashMap<>();

      for (RigidBodyBasics rigidBody : rigidBodies)
      {
         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(rigidBody);
         PhysicsEngineRobotData physicsEngineRobotData = new PhysicsEngineRobotData(rigidBody.getName(), rootBody, null, null);
         physicsEngineRobotData.getForwardDynamicsPlugin().doScience(0.0, dt, gravity);
         map.put(rootBody, physicsEngineRobotData);
      }

      return map;
   }

   public Map<RigidBodyBasics, ForwardDynamicsCalculator> toForwardDynamicsCalculatorMap(Vector3DReadOnly gravity, RigidBodyBasics... rigidBodies)
   {
      HashMap<RigidBodyBasics, ForwardDynamicsCalculator> map = new HashMap<>();

      for (RigidBodyBasics rigidBody : rigidBodies)
      {
         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(rigidBody);
         map.put(rootBody, setupForwardDynamicsCalculator(gravity, rigidBody));
      }

      return map;
   }

   static void updateVelocities(double dt, MultiContactImpulseCalculator multiContactImpulseCalculator,
                                Map<RigidBodyBasics, ForwardDynamicsCalculator> calculatorMap)
   {
      Map<RigidBodyBasics, DMatrixRMaj> jointVelocityMatrixMap = new HashMap<>();

      for (SingleContactImpulseCalculator impulseCalculator : multiContactImpulseCalculator.getImpulseCalculators())
      {
         RigidBodyBasics bodyA = impulseCalculator.getContactingBodyA();
         RigidBodyBasics rootA = MultiBodySystemTools.getRootBody(bodyA);

         DMatrixRMaj jointVelocityMatrix = jointVelocityMatrixMap.get(rootA);
         if (jointVelocityMatrix == null)
         {
            List<JointBasics> joints = Arrays.asList(MultiBodySystemTools.collectSubtreeJoints(rootA));
            jointVelocityMatrix = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(joints), 1);
            MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, jointVelocityMatrix);
            CommonOps_DDRM.addEquals(jointVelocityMatrix, dt, calculatorMap.get(rootA).getJointAccelerationMatrix());
            jointVelocityMatrixMap.put(rootA, jointVelocityMatrix);
         }
         if (impulseCalculator.isConstraintActive())
            CommonOps_DDRM.addEquals(jointVelocityMatrix, impulseCalculator.getJointVelocityChangeA());

         RigidBodyBasics bodyB = impulseCalculator.getContactingBodyB();

         if (bodyB != null)
         {
            RigidBodyBasics rootB = MultiBodySystemTools.getRootBody(bodyB);

            jointVelocityMatrix = jointVelocityMatrixMap.get(rootB);
            if (jointVelocityMatrix == null)
            {
               List<JointBasics> joints = Arrays.asList(MultiBodySystemTools.collectSubtreeJoints(rootB));
               jointVelocityMatrix = new DMatrixRMaj(MultiBodySystemTools.computeDegreesOfFreedom(joints), 1);
               MultiBodySystemTools.extractJointsState(joints, JointStateType.VELOCITY, jointVelocityMatrix);
               CommonOps_DDRM.addEquals(jointVelocityMatrix, dt, calculatorMap.get(rootB).getJointAccelerationMatrix());
               jointVelocityMatrixMap.put(rootB, jointVelocityMatrix);
            }
            if (impulseCalculator.isConstraintActive())
               CommonOps_DDRM.addEquals(jointVelocityMatrix, impulseCalculator.getJointVelocityChangeB());
         }
      }

      for (Entry<RigidBodyBasics, DMatrixRMaj> entry : jointVelocityMatrixMap.entrySet())
      {
         RigidBodyBasics root = entry.getKey();
         List<JointBasics> joints = Arrays.asList(MultiBodySystemTools.collectSubtreeJoints(root));
         MultiBodySystemTools.insertJointsState(joints, JointStateType.VELOCITY, entry.getValue());
         root.updateFramesRecursively();
      }
   }

   static CollisionResult nextCollisionResult(Random random, RigidBodyBasics contactingBodyA)
   {
      return nextCollisionResult(random, contactingBodyA, null);
   }

   static CollisionResult nextCollisionResult(Random random, RigidBodyBasics contactingBodyA, RigidBodyBasics contactingBodyB)
   {
      CollisionResult collisionResult = new CollisionResult();
      Collidable collidableA = nextCollidable(random, contactingBodyA);
      collisionResult.setCollidableA(collidableA);
      FrameUnitVector3D collisionAxisForA = collisionResult.getCollisionAxisForA();
      FramePoint3D pointInBodyFrameA = collisionResult.getCollisionData().getPointOnA();
      FramePoint3D pointInBodyFrameB = collisionResult.getCollisionData().getPointOnB();
      FramePoint3D pointOnARootFrame = collisionResult.getPointOnARootFrame();
      FramePoint3D pointOnBRootFrame = collisionResult.getPointOnBRootFrame();

      collisionAxisForA.setIncludingFrame(worldFrame, EuclidCoreRandomTools.nextVector3DWithFixedLength(random, 1.0));
      FrameVector3D supportDirection = new FrameVector3D(collisionAxisForA);
      supportDirection.changeFrame(collidableA.getShape().getReferenceFrame());
      supportDirection.negate();
      pointInBodyFrameA.setIncludingFrame(collidableA.getShape().getSupportingVertex(supportDirection));
      pointInBodyFrameA.changeFrame(contactingBodyA.getBodyFixedFrame());
      pointOnARootFrame.setIncludingFrame(pointInBodyFrameA);
      pointOnARootFrame.changeFrame(worldFrame);

      if (contactingBodyB != null)
      {
         Collidable collidableB = nextCollidable(random, contactingBodyB);
         collisionResult.setCollidableB(collidableB);
         supportDirection.negate();
         supportDirection.changeFrame(collidableB.getShape().getReferenceFrame());
         pointOnBRootFrame.setIncludingFrame(collidableB.getShape().getSupportingVertex(supportDirection));
         pointOnBRootFrame.changeFrame(worldFrame);
         FrameVector3D translation = new FrameVector3D();
         translation.sub(pointOnARootFrame, pointOnBRootFrame);
         if (contactingBodyB.getParentJoint() instanceof SixDoFJointBasics)
         {
            SixDoFJointBasics floatingJoint = (SixDoFJointBasics) contactingBodyB.getParentJoint();
            floatingJoint.getJointPose().getPosition().add(translation);
            MultiBodySystemTools.getRootBody(contactingBodyB).updateFramesRecursively();
            pointOnBRootFrame.setIncludingFrame(pointOnARootFrame);
            pointInBodyFrameB.setIncludingFrame(pointOnBRootFrame);
            pointInBodyFrameB.changeFrame(contactingBodyB.getBodyFixedFrame());
         }
         else
         {
            throw new UnsupportedOperationException("Need to figure a more general approach");
         }
      }
      else
      {
         collisionResult.setCollidableB(nextStaticCollidable(random));
      }

      return collisionResult;
   }

   static Map<CollisionResult, FrameVector3D> computeContactVelocities(double dt, CollisionListResult collisionResults)
   {
      return collisionResults.stream().collect(Collectors.toMap(Function.identity(), collisionResult -> computeContactVelocity(dt, collisionResult)));
   }

   static Map<CollisionResult, FrameVector3D> predictContactVelocity(double dt, CollisionListResult collisionResults,
                                                                     Map<RigidBodyBasics, ForwardDynamicsCalculator> bodyToForwardDynamicsCalculatorMap)
   {
      return collisionResults.stream().collect(Collectors.toMap(Function.identity(), collisionResult ->
      {
         ForwardDynamicsCalculator calculatorA = bodyToForwardDynamicsCalculatorMap.get(collisionResult.getCollidableA().getRootBody());
         ForwardDynamicsCalculator calculatorB = bodyToForwardDynamicsCalculatorMap.get(collisionResult.getCollidableB().getRootBody());
         return SingleContactImpulseCalculatorTest.predictContactVelocity(dt, collisionResult, calculatorA, calculatorB);
      }));
   }
}
