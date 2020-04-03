package us.ihmc.robotics.physics;

import java.util.*;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import org.ejml.ops.RandomMatrices;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.mecano.algorithms.interfaces.RigidBodyTwistProvider;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.*;

public class CombinedRigidBodyTwistProvidersTest
{
   private static final int ITERATIONS = 1000;
   private static final double EPSILON = 1.0e-12;

   @Test
   public void testAgainstMovingReferenceFrame()
   {
      Random random = new Random(365754);

      for (int i = 0; i < ITERATIONS; i++)
      {
         List<JointBasics> jointChain = MultiBodySystemRandomTools.nextJointChain(random, 10);
         MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, jointChain);
         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(jointChain.get(0).getPredecessor());

         int numberOfProviders = random.nextInt(10) + 1;

         int nDoFs = SubtreeStreams.fromChildren(rootBody).mapToInt(JointReadOnly::getDegreesOfFreedom).sum();
         DenseMatrix64F totalJointVelocities = new DenseMatrix64F(nDoFs, 1);
         CombinedRigidBodyTwistProviders combinedRigidBodyTwistProviders = new CombinedRigidBodyTwistProviders(ReferenceFrame.getWorldFrame());

         for (int j = 0; j < numberOfProviders; j++)
         {
            TestHelper helper = new TestHelper(rootBody, "test" + i);
            helper.update(random);
            CommonOps.addEquals(totalJointVelocities, helper.jointVelocities);
            combinedRigidBodyTwistProviders.add(helper.toProvider());
         }

         MultiBodySystemTools.insertJointsState(jointChain, JointStateType.VELOCITY, totalJointVelocities);
         rootBody.updateFramesRecursively();

         for (int j = 0; j < 10; j++)
         {
            RigidBodyReadOnly body = jointChain.get(random.nextInt(jointChain.size())).getSuccessor();
            Twist expectedTwist = new Twist(body.getBodyFixedFrame().getTwistOfFrame());
            TwistReadOnly actualTwist = combinedRigidBodyTwistProviders.getTwistOfBody(body);
            MecanoTestTools.assertTwistEquals(expectedTwist, actualTwist, EPSILON);

            RigidBodyReadOnly base = jointChain.get(random.nextInt(jointChain.size())).getPredecessor();
            body.getBodyFixedFrame().getTwistRelativeToOther(base.getBodyFixedFrame(), expectedTwist);
            actualTwist = combinedRigidBodyTwistProviders.getRelativeTwist(base, body);
            MecanoTestTools.assertTwistEquals(expectedTwist, actualTwist, EPSILON);

            FramePoint3D bodyFixedPoint = EuclidFrameRandomTools.nextFramePoint3D(random, body.getBodyFixedFrame());
            FrameVector3D expectedLinearVelocity = new FrameVector3D();
            expectedTwist.getLinearVelocityAt(bodyFixedPoint, expectedLinearVelocity);
            FrameVector3DReadOnly actualLinearVelocity = combinedRigidBodyTwistProviders.getLinearVelocityOfBodyFixedPoint(base, body, bodyFixedPoint);
            EuclidFrameTestTools.assertFrameTuple3DEquals(expectedLinearVelocity, actualLinearVelocity, EPSILON);
         }
      }
   }

   private static class TestHelper
   {
      private final ReferenceFrame inertialFrame;
      private final RigidBodyBasics originalRootBody;
      private final List<JointBasics> originalJoints;
      private final RigidBodyBasics cloneRootBody;
      private final List<JointBasics> cloneJoints;
      private final DenseMatrix64F jointVelocities;

      private final Map<RigidBodyBasics, RigidBodyBasics> fromOriginalToCloneMap = new HashMap<>();

      public TestHelper(RigidBodyBasics rootBody, String name)
      {
         originalRootBody = rootBody;
         originalJoints = Arrays.asList(MultiBodySystemTools.collectSubtreeJoints(originalRootBody));
         inertialFrame = rootBody.getBodyFixedFrame().getRootFrame();
         cloneRootBody = MultiBodySystemFactories.cloneMultiBodySystem(rootBody, inertialFrame, name);
         cloneJoints = Arrays.asList(MultiBodySystemTools.collectSubtreeJoints(cloneRootBody));
         int nDoFs = SubtreeStreams.fromChildren(rootBody).mapToInt(JointReadOnly::getDegreesOfFreedom).sum();
         jointVelocities = new DenseMatrix64F(nDoFs, 1);

         List<? extends RigidBodyBasics> originalRigidBodies = originalRootBody.subtreeList();
         List<? extends RigidBodyBasics> cloneRigidBodies = cloneRootBody.subtreeList();

         for (int i = 0; i < originalRigidBodies.size(); i++)
         {
            fromOriginalToCloneMap.put(originalRigidBodies.get(i), cloneRigidBodies.get(i));
         }
      }

      public void update(Random random)
      {
         MultiBodySystemTools.copyJointsState(originalJoints, cloneJoints, JointStateType.CONFIGURATION);
         RandomMatrices.setRandom(jointVelocities, -2.0, 2.0, random);
         MultiBodySystemTools.insertJointsState(cloneJoints, JointStateType.VELOCITY, jointVelocities);
         cloneRootBody.updateFramesRecursively();
      }

      public RigidBodyTwistProvider toProvider()
      {
         Twist twist = new Twist();
         return RigidBodyTwistProvider.toRigidBodyTwistProvider(body ->
         {
            twist.setIncludingFrame(fromOriginalToCloneMap.get(body).getBodyFixedFrame().getTwistOfFrame());
            twist.setReferenceFrame(body.getBodyFixedFrame());
            twist.setBodyFrame(body.getBodyFixedFrame());
            return twist;
         }, inertialFrame);
      }
   }
}
