package us.ihmc.commonWalkingControlModules.controllerCore.command;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import java.lang.reflect.Method;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.robotModels.JointHashCodeResolver;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

class CrossRobotCommandResolverTest
{
   private static final int ITERATIONS = 100;

   @SuppressWarnings("rawtypes")
   @Test
   void testInverseDynamicsCommandCoverage() throws Exception
   {
      boolean verbose = true;

      Set<Class<? extends InverseDynamicsCommand>> commandTypes = CrossRobotCommandRandomTools.getInverseDynamicsCommandTypes(InverseDynamicsCommandList.class,
                                                                                                                                  InverseDynamicsCommandBuffer.class);

      String errorMessage = "";

      if (!isResolveMethodAvailableForCommandList(InverseDynamicsCommandList.class, InverseDynamicsCommandBuffer.class, verbose))
         errorMessage += "Missing resolve method for: " + InverseDynamicsCommandList.class.getSimpleName() + "\n";

      for (Class<? extends InverseDynamicsCommand> commandType : commandTypes)
      {
         if (commandType.isInterface())
            continue;
         if (!isResolveMethodAvailableFor(commandType, verbose))
            errorMessage += "Missing resolve method for: " + commandType.getSimpleName() + "\n";
      }

      if (!errorMessage.isEmpty())
         fail("Missing at least one InverseDynamicsCommand:\n" + errorMessage);
   }

   @SuppressWarnings("rawtypes")
   @Test
   void testInverseKinematicsCommandCoverage() throws Exception
   {
      boolean verbose = true;

      Set<Class<? extends InverseKinematicsCommand>> commandTypes = CrossRobotCommandRandomTools.getInverseKinematicsCommandTypes(InverseKinematicsCommandList.class,
                                                                                                                                      InverseKinematicsCommandBuffer.class);

      String errorMessage = "";

      if (!isResolveMethodAvailableForCommandList(InverseKinematicsCommandList.class, InverseKinematicsCommandBuffer.class, verbose))
         errorMessage += "Missing resolve method for: " + InverseKinematicsCommandList.class.getSimpleName() + "\n";

      for (Class<? extends InverseKinematicsCommand> commandType : commandTypes)
      {
         if (commandType.isInterface())
            continue;
         if (!isResolveMethodAvailableFor(commandType, verbose))
            errorMessage += "Missing resolve method for: " + commandType.getSimpleName() + "\n";
      }

      if (!errorMessage.isEmpty())
         fail("Missing at least one InverseKinematicsCommand:\n" + errorMessage);
   }

   @SuppressWarnings("rawtypes")
   @Test
   void testVirtualModelControlCommandCoverage() throws Exception
   {
      boolean verbose = true;

      Set<Class<? extends VirtualModelControlCommand>> commandTypes = CrossRobotCommandRandomTools.getVirtualModelControlCommandTypes(VirtualModelControlCommandList.class,
                                                                                                                                          VirtualModelControlCommandBuffer.class);

      String errorMessage = "";

      if (!isResolveMethodAvailableForCommandList(VirtualModelControlCommandList.class, VirtualModelControlCommandBuffer.class, verbose))
         errorMessage += "Missing resolve method for: " + VirtualModelControlCommandList.class.getSimpleName() + "\n";

      for (Class<? extends VirtualModelControlCommand> commandType : commandTypes)
      {
         if (commandType.isInterface())
            continue;
         if (!isResolveMethodAvailableFor(commandType, verbose))
            errorMessage += "Missing resolve method for: " + commandType.getSimpleName() + "\n";
      }

      if (!errorMessage.isEmpty())
         fail("Missing at least one VirtualModelControlCommand:\n" + errorMessage);
   }

   @SuppressWarnings("rawtypes")
   @Test
   void testFeedbackControlCommandCoverage() throws Exception
   {
      boolean verbose = true;

      Set<Class<? extends FeedbackControlCommand>> commandTypes = CrossRobotCommandRandomTools.getFeedbackControlCommandTypes(FeedbackControlCommandList.class,
                                                                                                                                  FeedbackControlCommandBuffer.class);

      String errorMessage = "";

      if (!isResolveMethodAvailableForCommandList(FeedbackControlCommandList.class, FeedbackControlCommandBuffer.class, verbose))
         errorMessage += "Missing resolve method for: " + FeedbackControlCommandList.class.getSimpleName() + "\n";

      for (Class<? extends FeedbackControlCommand> commandType : commandTypes)
      {
         if (commandType.isInterface())
            continue;
         if (!isResolveMethodAvailableFor(commandType, verbose))
            errorMessage += "Missing resolve method for: " + commandType.getSimpleName() + "\n";
      }

      if (!errorMessage.isEmpty())
         fail("Missing at least one FeedbackControlCommand:\n" + errorMessage);
   }

   private static boolean isResolveMethodAvailableFor(Class<?> commandType, boolean verbose)
   {
      String methodName = "resolve" + commandType.getSimpleName();

      try
      {
         CrossRobotCommandResolver.class.getMethod(methodName, commandType, commandType);
         return true;
      }
      catch (NoSuchMethodException e)
      {
         return false;
      }
      catch (SecurityException e)
      {
         if (verbose)
            System.err.println("Encountered following error for method " + methodName + ", error: " + e.getMessage());
         return false;
      }
   }

   private static boolean isResolveMethodAvailableForCommandList(Class<?> commandListType, Class<?> commandRecyclingListType, boolean verbose)
   {
      String methodName = "resolve" + commandListType.getSimpleName();

      try
      {
         CrossRobotCommandResolver.class.getMethod(methodName, commandListType, commandRecyclingListType);
         return true;
      }
      catch (NoSuchMethodException e)
      {
         return false;
      }
      catch (SecurityException e)
      {
         if (verbose)
            System.err.println("Encountered following error for method " + methodName + ", error: " + e.getMessage());
         return false;
      }
   }

   @Test
   void testResolvingCommands() throws Exception
   {
      for (Class<?> commandType : CrossRobotCommandRandomTools.getAllCommandTypesWithoutBuffersAndInterfaces())
      {
         testForResolver(commandType);
      }
   }

   private static void testForResolver(Class<?> clazz) throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);
      for (int i = 0; i < ITERATIONS; i++)
      {
         // By using the same seed on a fresh random, the two commands will be built the same way.
         long seed = random.nextLong();
         Object in = CrossRobotCommandRandomTools.nextTypeInstance(clazz, new Random(seed), true, testData.rootBodyA, testData.frameTreeA);
         Object expectedOut = CrossRobotCommandRandomTools.nextTypeInstance(clazz, new Random(seed), true, testData.rootBodyB, testData.frameTreeB);

         // Find the resolve method by name only. This is robust to the parameter being an interface or a buffer.
         Method[] methods = CrossRobotCommandResolver.class.getMethods();
         String methodName = "resolve" + clazz.getSimpleName();
         Optional<Method> method = Arrays.asList(methods).stream().filter(m -> m.getName().equals(methodName)).findFirst();
         if (!method.isPresent())
         {
            fail("Could not find method " + methodName + " in " + CrossRobotCommandResolver.class.getSimpleName());
         }
         Method resolveMethod = method.get();

         // Doing this ensures that the output is a command buffer if needed.
         Object actualOut = resolveMethod.getParameters()[1].getType().newInstance();
         resolveMethod.invoke(crossRobotCommandResolver, in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   private static class TestData
   {
      private final ReferenceFrame rootFrameA = ReferenceFrameTools.constructARootFrame("rootFrameA");
      private final ReferenceFrame[] frameTreeA;
      private final ReferenceFrame rootFrameB = ReferenceFrameTools.constructARootFrame("rootFrameB");
      private final ReferenceFrame[] frameTreeB;

      private final RigidBodyBasics rootBodyA = new RigidBody("rootBody", rootFrameA);
      private final RigidBodyBasics rootBodyB;
      private final List<OneDoFJoint> chainA;
      private final List<OneDoFJoint> chainB;

      private final JointHashCodeResolver jointResolverForB = new JointHashCodeResolver();
      private final RigidBodyHashCodeResolver bodyResolverForB = new RigidBodyHashCodeResolver();
      private final ReferenceFrameHashCodeResolver frameResolverForB = new ReferenceFrameHashCodeResolver();

      public TestData(Random random, int numberOfFrames, int numberOfJoints)
      {

         frameResolverForB.put(rootFrameB, rootFrameA.hashCode());
         frameTreeA = EuclidFrameRandomTools.nextReferenceFrameTree("frameTreeA", random, rootFrameA, numberOfFrames);
         frameTreeB = new ReferenceFrame[frameTreeA.length];
         frameTreeB[0] = rootFrameB;

         for (int frameIndex = 1; frameIndex < frameTreeA.length; frameIndex++)
         {
            ReferenceFrame frameA = frameTreeA[frameIndex];
            ReferenceFrame parentFrameA = frameA.getParent();
            ReferenceFrame parentFrameB = frameResolverForB.getReferenceFrame(parentFrameA.hashCode());
            ReferenceFrame frameB = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("frameTreeB" + (frameIndex - 1), parentFrameB,
                                                                                                      frameA.getTransformToParent());
            frameTreeB[frameIndex] = frameB;
            frameResolverForB.put(frameB, frameA.hashCode());
         }

         chainA = MultiBodySystemRandomTools.nextOneDoFJointChain(random, "chain", rootBodyA, numberOfJoints);
         rootBodyB = MultiBodySystemFactories.cloneMultiBodySystem(rootBodyA, rootFrameB, "");
         chainB = SubtreeStreams.fromChildren(OneDoFJoint.class, rootBodyB).collect(Collectors.toList());

         bodyResolverForB.put(rootBodyB, rootBodyA.hashCode());
         frameResolverForB.put(rootBodyB.getBodyFixedFrame(), rootBodyA.getBodyFixedFrame().hashCode());

         for (int i = 0; i < chainA.size(); i++)
         {
            OneDoFJoint chainAJoint = chainA.get(i);
            OneDoFJoint chainBJoint = chainB.get(i);
            RigidBodyBasics chainABody = chainAJoint.getSuccessor();
            RigidBodyBasics chainBBody = chainBJoint.getSuccessor();

            jointResolverForB.put(chainBJoint, chainAJoint.hashCode());
            bodyResolverForB.put(chainBBody, chainABody.hashCode());
            frameResolverForB.put(chainBBody.getBodyFixedFrame(), chainABody.getBodyFixedFrame().hashCode());
            frameResolverForB.put(chainBJoint.getFrameAfterJoint(), chainAJoint.getFrameAfterJoint().hashCode());
            frameResolverForB.put(chainBJoint.getFrameBeforeJoint(), chainAJoint.getFrameBeforeJoint().hashCode());
         }
      }
   }
}
