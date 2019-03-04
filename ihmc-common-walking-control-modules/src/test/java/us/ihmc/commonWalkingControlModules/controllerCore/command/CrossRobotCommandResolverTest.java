package us.ihmc.commonWalkingControlModules.controllerCore.command;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.robotModels.JointHashCodeResolver;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

class CrossRobotCommandResolverTest
{
   @Test
   void testResolveCenterOfPressureCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);
      long seed = random.nextLong();
      // By using the same seed on a fresh random, the two commands will be built the same way.
      CenterOfPressureCommand in = nextCenterOfPressureCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
      CenterOfPressureCommand expectedOut = nextCenterOfPressureCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
      CenterOfPressureCommand actualOut = new CenterOfPressureCommand();
      crossRobotCommandResolver.resolveCenterOfPressureCommand(in, actualOut);
      assertEquals(expectedOut, actualOut);
   }

   private CenterOfPressureCommand nextCenterOfPressureCommand(Random random, RigidBody rootBody, ReferenceFrame... possibleFrames)
   {
      CenterOfPressureCommand next = new CenterOfPressureCommand();
      next.setConstraintType(nextElementIn(random, ConstraintType.values()));
      next.setContactingRigidBody(nextElementIn(random, rootBody.subtreeList()));
      next.setWeight(nextFrameVector2D(random, possibleFrames));
      next.setDesiredCoP(nextFramePoint2D(random, possibleFrames));
      return next;
   }

   public static <E> E nextElementIn(Random random, E[] array)
   {
      return array[random.nextInt(array.length)];
   }

   public static <E> E nextElementIn(Random random, List<E> list)
   {
      return list.get(random.nextInt(list.size()));
   }

   public static FramePoint2D nextFramePoint2D(Random random, ReferenceFrame... possibleFrames)
   {
      return EuclidFrameRandomTools.nextFramePoint2D(random, nextElementIn(random, possibleFrames));
   }

   public static FramePoint3D nextFramePoint3D(Random random, ReferenceFrame... possibleFrames)
   {
      return EuclidFrameRandomTools.nextFramePoint3D(random, nextElementIn(random, possibleFrames));
   }

   public static FrameVector2D nextFrameVector2D(Random random, ReferenceFrame... possibleFrames)
   {
      return EuclidFrameRandomTools.nextFrameVector2D(random, nextElementIn(random, possibleFrames));
   }

   public static FrameVector3D nextFrameVector3D(Random random, ReferenceFrame... possibleFrames)
   {
      return EuclidFrameRandomTools.nextFrameVector3D(random, nextElementIn(random, possibleFrames));
   }

   private static class TestData
   {
      private final ReferenceFrame rootFrameA = ReferenceFrameTools.constructARootFrame("rootFrameA");
      private final ReferenceFrame[] frameTreeA;
      private final ReferenceFrame rootFrameB = ReferenceFrameTools.constructARootFrame("rootFrameB");
      private final ReferenceFrame[] frameTreeB;

      private final RigidBody rootBodyA = new RigidBody("rootBodyA", rootFrameA);
      private final RigidBody rootBodyB = new RigidBody("rootBodyB", rootFrameB);
      private final List<OneDoFJoint> chainA;
      private final List<OneDoFJoint> chainB;

      private final JointHashCodeResolver jointResolverForB = new JointHashCodeResolver();
      private final RigidBodyHashCodeResolver bodyResolverForB = new RigidBodyHashCodeResolver();
      private final ReferenceFrameHashCodeResolver frameResolverForB = new ReferenceFrameHashCodeResolver();

      public TestData(Random random, int numberOfFrames, int numberOfJoints)
      {
         frameTreeA = EuclidFrameRandomTools.nextReferenceFrameTree("frameTreeA", random, rootFrameA, numberOfFrames);
         frameTreeB = EuclidFrameRandomTools.nextReferenceFrameTree("frameTreeB", random, rootFrameB, numberOfFrames);

         for (int i = 0; i < frameTreeA.length; i++)
         { // We force the hash-codes for B to correspond to A, so it is as if the 2 trees are identical.
            frameResolverForB.put(frameTreeB[i], frameTreeA[i].hashCode());
         }

         chainA = MultiBodySystemRandomTools.nextOneDoFJointChain(random, "chainA", numberOfJoints);
         chainB = MultiBodySystemRandomTools.nextOneDoFJointChain(random, "chainB", numberOfJoints);

         bodyResolverForB.put(rootBodyB, rootBodyA.hashCode());

         for (int i = 0; i < chainA.size(); i++)
         {
            jointResolverForB.put(chainB.get(i), chainA.get(i).hashCode());
            bodyResolverForB.put(chainB.get(i).getSuccessor(), chainA.get(i).getSuccessor().hashCode());
         }
      }
   }
}
