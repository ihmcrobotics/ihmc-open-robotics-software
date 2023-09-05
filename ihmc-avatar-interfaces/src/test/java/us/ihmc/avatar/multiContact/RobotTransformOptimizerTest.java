package us.ihmc.avatar.multiContact;

import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools.RandomFloatingRevoluteJointChain;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class RobotTransformOptimizerTest
{

   @Test
   public void testOnlyRootJointDiffer()
   {
      Random random = new Random(35346);
      RandomFloatingRevoluteJointChain robotA = new RandomFloatingRevoluteJointChain(random, 30);
      robotA.nextState(random, JointStateType.CONFIGURATION);
      RigidBody rootBodyA = robotA.getElevator();
      SixDoFJoint rootJointA = robotA.getRootJoint();
      List<? extends JointBasics> jointsA = rootJointA.subtreeList();

      RigidBodyBasics rootBodyB = MultiBodySystemFactories.cloneMultiBodySystem(rootBodyA, ReferenceFrame.getWorldFrame(), "clone");
      SixDoFJoint rootJointB = (SixDoFJoint) rootBodyB.getChildrenJoints().get(0);
      List<? extends JointBasics> jointsB = rootJointB.subtreeList();
      MultiBodySystemTools.copyJointsState(jointsA, jointsB, JointStateType.CONFIGURATION);

      RigidBodyTransform expectedTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);

      rootJointB.getJointPose().prependTransform(expectedTransform);
      expectedTransform.invert();

      rootBodyA.updateFramesRecursively();
      rootBodyB.updateFramesRecursively();

      RobotTransformOptimizer matcher = new RobotTransformOptimizer(rootBodyA, rootBodyB);
      matcher.addDefaultRigidBodySpatialErrorCalculators((bodyA, bodyB) -> !bodyA.isRootBody());
      matcher.compute();

      RigidBodyTransform actualTransform = matcher.getTransformFromBToA();

      EuclidCoreTestTools.assertOrientation3DGeometricallyEquals(expectedTransform.getRotation(), actualTransform.getRotation(), 1.0e-4);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(expectedTransform.getTranslation(), actualTransform.getTranslation(), 1.0e-3);
   }

}
