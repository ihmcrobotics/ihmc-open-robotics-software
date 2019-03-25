package us.ihmc.commonWalkingControlModules.controllerCore.command;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OrientationFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.PointFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointLimitEnforcementCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointTorqueCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualForceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualTorqueCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
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

   @Test
   void testResolveControllerCoreCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);
      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         ControllerCoreCommand in = ControllerCoreCommandRandomTools.nextControllerCoreCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         ControllerCoreCommand expectedOut = ControllerCoreCommandRandomTools.nextControllerCoreCommand(new Random(seed), testData.rootBodyB,
                                                                                                        testData.frameTreeB);
         ControllerCoreCommandBuffer actualOut = new ControllerCoreCommandBuffer();
         crossRobotCommandResolver.resolveControllerCoreCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @SuppressWarnings("rawtypes")
   @Test
   void testInverseDynamicsCommandCoverage() throws Exception
   {
      boolean verbose = true;

      Set<Class<? extends InverseDynamicsCommand>> commandTypes = ControllerCoreCommandRandomTools.getInverseDynamicsCommandTypes(InverseDynamicsCommandList.class,
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

      Set<Class<? extends InverseKinematicsCommand>> commandTypes = ControllerCoreCommandRandomTools.getInverseKinematicsCommandTypes(InverseKinematicsCommandList.class,
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

      Set<Class<? extends VirtualModelControlCommand>> commandTypes = ControllerCoreCommandRandomTools.getVirtualModelControlCommandTypes(VirtualModelControlCommandList.class,
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

      Set<Class<? extends FeedbackControlCommand>> commandTypes = ControllerCoreCommandRandomTools.getFeedbackControlCommandTypes(FeedbackControlCommandList.class,
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
   void testResolveInverseDynamicsCommandList() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         InverseDynamicsCommandList in = ControllerCoreCommandRandomTools.nextInverseDynamicsCommandList(new Random(seed), testData.rootBodyA,
                                                                                                         testData.frameTreeA);
         InverseDynamicsCommandList expectedOut = ControllerCoreCommandRandomTools.nextInverseDynamicsCommandList(new Random(seed), testData.rootBodyB,
                                                                                                                  testData.frameTreeB);
         InverseDynamicsCommandBuffer actualOut = new InverseDynamicsCommandBuffer();
         crossRobotCommandResolver.resolveInverseDynamicsCommandList(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveInverseKinematicsCommandList() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         InverseKinematicsCommandList in = ControllerCoreCommandRandomTools.nextInverseKinematicsCommandList(new Random(seed), testData.rootBodyA,
                                                                                                             testData.frameTreeA);
         InverseKinematicsCommandList expectedOut = ControllerCoreCommandRandomTools.nextInverseKinematicsCommandList(new Random(seed), testData.rootBodyB,
                                                                                                                      testData.frameTreeB);
         InverseKinematicsCommandBuffer actualOut = new InverseKinematicsCommandBuffer();
         crossRobotCommandResolver.resolveInverseKinematicsCommandList(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveVirtualModelControlCommandList() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         VirtualModelControlCommandList in = ControllerCoreCommandRandomTools.nextVirtualModelControlCommandList(new Random(seed), testData.rootBodyA,
                                                                                                                 testData.frameTreeA);
         VirtualModelControlCommandList expectedOut = ControllerCoreCommandRandomTools.nextVirtualModelControlCommandList(new Random(seed), testData.rootBodyB,
                                                                                                                          testData.frameTreeB);
         VirtualModelControlCommandBuffer actualOut = new VirtualModelControlCommandBuffer();
         crossRobotCommandResolver.resolveVirtualModelControlCommandList(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveFeedbackControlCommandList() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         FeedbackControlCommandList in = ControllerCoreCommandRandomTools.nextFeedbackControlCommandList(new Random(seed), testData.rootBodyA,
                                                                                                         testData.frameTreeA);
         FeedbackControlCommandList expectedOut = ControllerCoreCommandRandomTools.nextFeedbackControlCommandList(new Random(seed), testData.rootBodyB,
                                                                                                                  testData.frameTreeB);
         FeedbackControlCommandBuffer actualOut = new FeedbackControlCommandBuffer();
         crossRobotCommandResolver.resolveFeedbackControlCommandList(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveLowLevelOneDoFJointDesiredDataHolder() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);
      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         LowLevelOneDoFJointDesiredDataHolder in = ControllerCoreCommandRandomTools.nextLowLevelOneDoFJointDesiredDataHolder(new Random(seed),
                                                                                                                             testData.rootBodyA,
                                                                                                                             testData.frameTreeA);
         LowLevelOneDoFJointDesiredDataHolder expectedOut = ControllerCoreCommandRandomTools.nextLowLevelOneDoFJointDesiredDataHolder(new Random(seed),
                                                                                                                                      testData.rootBodyB,
                                                                                                                                      testData.frameTreeB);
         LowLevelOneDoFJointDesiredDataHolder actualOut = new LowLevelOneDoFJointDesiredDataHolder();
         crossRobotCommandResolver.resolveLowLevelOneDoFJointDesiredDataHolder(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveCenterOfPressureCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);
      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         CenterOfPressureCommand in = ControllerCoreCommandRandomTools.nextCenterOfPressureCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         CenterOfPressureCommand expectedOut = ControllerCoreCommandRandomTools.nextCenterOfPressureCommand(new Random(seed), testData.rootBodyB,
                                                                                                            testData.frameTreeB);
         CenterOfPressureCommand actualOut = new CenterOfPressureCommand();
         crossRobotCommandResolver.resolveCenterOfPressureCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveContactWrenchCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         ContactWrenchCommand in = ControllerCoreCommandRandomTools.nextContactWrenchCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         ContactWrenchCommand expectedOut = ControllerCoreCommandRandomTools.nextContactWrenchCommand(new Random(seed), testData.rootBodyB,
                                                                                                      testData.frameTreeB);
         ContactWrenchCommand actualOut = new ContactWrenchCommand();
         crossRobotCommandResolver.resolveContactWrenchCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveExternalWrenchCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         ExternalWrenchCommand in = ControllerCoreCommandRandomTools.nextExternalWrenchCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         ExternalWrenchCommand expectedOut = ControllerCoreCommandRandomTools.nextExternalWrenchCommand(new Random(seed), testData.rootBodyB,
                                                                                                        testData.frameTreeB);
         ExternalWrenchCommand actualOut = new ExternalWrenchCommand();
         crossRobotCommandResolver.resolveExternalWrenchCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveInverseDynamicsOptimizationSettingsCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         InverseDynamicsOptimizationSettingsCommand in = ControllerCoreCommandRandomTools.nextInverseDynamicsOptimizationSettingsCommand(new Random(seed),
                                                                                                                                         testData.rootBodyA,
                                                                                                                                         testData.frameTreeA);
         InverseDynamicsOptimizationSettingsCommand expectedOut = ControllerCoreCommandRandomTools.nextInverseDynamicsOptimizationSettingsCommand(new Random(seed),
                                                                                                                                                  testData.rootBodyB,
                                                                                                                                                  testData.frameTreeB);
         InverseDynamicsOptimizationSettingsCommand actualOut = new InverseDynamicsOptimizationSettingsCommand();
         crossRobotCommandResolver.resolveInverseDynamicsOptimizationSettingsCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveJointAccelerationIntegrationCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         JointAccelerationIntegrationCommand in = ControllerCoreCommandRandomTools.nextJointAccelerationIntegrationCommand(new Random(seed), testData.rootBodyA,
                                                                                                                           testData.frameTreeA);
         JointAccelerationIntegrationCommand expectedOut = ControllerCoreCommandRandomTools.nextJointAccelerationIntegrationCommand(new Random(seed),
                                                                                                                                    testData.rootBodyB,
                                                                                                                                    testData.frameTreeB);
         JointAccelerationIntegrationCommand actualOut = new JointAccelerationIntegrationCommand();
         crossRobotCommandResolver.resolveJointAccelerationIntegrationCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveJointLimitEnforcementMethodCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         JointLimitEnforcementMethodCommand in = ControllerCoreCommandRandomTools.nextJointLimitEnforcementMethodCommand(new Random(seed), testData.rootBodyA,
                                                                                                                         testData.frameTreeA);
         JointLimitEnforcementMethodCommand expectedOut = ControllerCoreCommandRandomTools.nextJointLimitEnforcementMethodCommand(new Random(seed),
                                                                                                                                  testData.rootBodyB,
                                                                                                                                  testData.frameTreeB);
         JointLimitEnforcementMethodCommand actualOut = new JointLimitEnforcementMethodCommand();
         crossRobotCommandResolver.resolveJointLimitEnforcementMethodCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveJointspaceAccelerationCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         JointspaceAccelerationCommand in = ControllerCoreCommandRandomTools.nextJointspaceAccelerationCommand(new Random(seed), testData.rootBodyA,
                                                                                                               testData.frameTreeA);
         JointspaceAccelerationCommand expectedOut = ControllerCoreCommandRandomTools.nextJointspaceAccelerationCommand(new Random(seed), testData.rootBodyB,
                                                                                                                        testData.frameTreeB);
         JointspaceAccelerationCommand actualOut = new JointspaceAccelerationCommand();
         crossRobotCommandResolver.resolveJointspaceAccelerationCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveMomentumRateCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         MomentumRateCommand in = ControllerCoreCommandRandomTools.nextMomentumRateCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         MomentumRateCommand expectedOut = ControllerCoreCommandRandomTools.nextMomentumRateCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
         MomentumRateCommand actualOut = new MomentumRateCommand();
         crossRobotCommandResolver.resolveMomentumRateCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolvePlaneContactStateCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         PlaneContactStateCommand in = ControllerCoreCommandRandomTools.nextPlaneContactStateCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         PlaneContactStateCommand expectedOut = ControllerCoreCommandRandomTools.nextPlaneContactStateCommand(new Random(seed), testData.rootBodyB,
                                                                                                              testData.frameTreeB);
         PlaneContactStateCommand actualOut = new PlaneContactStateCommand();
         crossRobotCommandResolver.resolvePlaneContactStateCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveSpatialAccelerationCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         SpatialAccelerationCommand in = ControllerCoreCommandRandomTools.nextSpatialAccelerationCommand(new Random(seed), testData.rootBodyA,
                                                                                                         testData.frameTreeA);
         SpatialAccelerationCommand expectedOut = ControllerCoreCommandRandomTools.nextSpatialAccelerationCommand(new Random(seed), testData.rootBodyB,
                                                                                                                  testData.frameTreeB);
         SpatialAccelerationCommand actualOut = new SpatialAccelerationCommand();
         crossRobotCommandResolver.resolveSpatialAccelerationCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveInverseKinematicsOptimizationSettingsCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         InverseKinematicsOptimizationSettingsCommand in = ControllerCoreCommandRandomTools.nextInverseKinematicsOptimizationSettingsCommand(new Random(seed),
                                                                                                                                             testData.rootBodyA,
                                                                                                                                             testData.frameTreeA);
         InverseKinematicsOptimizationSettingsCommand expectedOut = ControllerCoreCommandRandomTools.nextInverseKinematicsOptimizationSettingsCommand(new Random(seed),
                                                                                                                                                      testData.rootBodyB,
                                                                                                                                                      testData.frameTreeB);
         InverseKinematicsOptimizationSettingsCommand actualOut = new InverseKinematicsOptimizationSettingsCommand();
         crossRobotCommandResolver.resolveInverseKinematicsOptimizationSettingsCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveJointLimitReductionCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         JointLimitReductionCommand in = ControllerCoreCommandRandomTools.nextJointLimitReductionCommand(new Random(seed), testData.rootBodyA,
                                                                                                         testData.frameTreeA);
         JointLimitReductionCommand expectedOut = ControllerCoreCommandRandomTools.nextJointLimitReductionCommand(new Random(seed), testData.rootBodyB,
                                                                                                                  testData.frameTreeB);
         JointLimitReductionCommand actualOut = new JointLimitReductionCommand();
         crossRobotCommandResolver.resolveJointLimitReductionCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveJointspaceVelocityCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         JointspaceVelocityCommand in = ControllerCoreCommandRandomTools.nextJointspaceVelocityCommand(new Random(seed), testData.rootBodyA,
                                                                                                       testData.frameTreeA);
         JointspaceVelocityCommand expectedOut = ControllerCoreCommandRandomTools.nextJointspaceVelocityCommand(new Random(seed), testData.rootBodyB,
                                                                                                                testData.frameTreeB);
         JointspaceVelocityCommand actualOut = new JointspaceVelocityCommand();
         crossRobotCommandResolver.resolveJointspaceVelocityCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveMomentumCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         MomentumCommand in = ControllerCoreCommandRandomTools.nextMomentumCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         MomentumCommand expectedOut = ControllerCoreCommandRandomTools.nextMomentumCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
         MomentumCommand actualOut = new MomentumCommand();
         crossRobotCommandResolver.resolveMomentumCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolvePrivilegedConfigurationCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         PrivilegedConfigurationCommand in = ControllerCoreCommandRandomTools.nextPrivilegedConfigurationCommand(new Random(seed), testData.rootBodyA,
                                                                                                                 testData.frameTreeA);
         PrivilegedConfigurationCommand expectedOut = ControllerCoreCommandRandomTools.nextPrivilegedConfigurationCommand(new Random(seed), testData.rootBodyB,
                                                                                                                          testData.frameTreeB);
         PrivilegedConfigurationCommand actualOut = new PrivilegedConfigurationCommand();
         crossRobotCommandResolver.resolvePrivilegedConfigurationCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolvePrivilegedJointSpaceCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         PrivilegedJointSpaceCommand in = ControllerCoreCommandRandomTools.nextPrivilegedJointSpaceCommand(new Random(seed), testData.rootBodyA,
                                                                                                           testData.frameTreeA);
         PrivilegedJointSpaceCommand expectedOut = ControllerCoreCommandRandomTools.nextPrivilegedJointSpaceCommand(new Random(seed), testData.rootBodyB,
                                                                                                                    testData.frameTreeB);
         PrivilegedJointSpaceCommand actualOut = new PrivilegedJointSpaceCommand();
         crossRobotCommandResolver.resolvePrivilegedJointSpaceCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveSpatialVelocityCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         SpatialVelocityCommand in = ControllerCoreCommandRandomTools.nextSpatialVelocityCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         SpatialVelocityCommand expectedOut = ControllerCoreCommandRandomTools.nextSpatialVelocityCommand(new Random(seed), testData.rootBodyB,
                                                                                                          testData.frameTreeB);
         SpatialVelocityCommand actualOut = new SpatialVelocityCommand();
         crossRobotCommandResolver.resolveSpatialVelocityCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveJointLimitEnforcementCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         JointLimitEnforcementCommand in = ControllerCoreCommandRandomTools.nextJointLimitEnforcementCommand(new Random(seed), testData.rootBodyA,
                                                                                                             testData.frameTreeA);
         JointLimitEnforcementCommand expectedOut = ControllerCoreCommandRandomTools.nextJointLimitEnforcementCommand(new Random(seed), testData.rootBodyB,
                                                                                                                      testData.frameTreeB);
         JointLimitEnforcementCommand actualOut = new JointLimitEnforcementCommand();
         crossRobotCommandResolver.resolveJointLimitEnforcementCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveJointTorqueCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         JointTorqueCommand in = ControllerCoreCommandRandomTools.nextJointTorqueCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         JointTorqueCommand expectedOut = ControllerCoreCommandRandomTools.nextJointTorqueCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
         JointTorqueCommand actualOut = new JointTorqueCommand();
         crossRobotCommandResolver.resolveJointTorqueCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveVirtualForceCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         VirtualForceCommand in = ControllerCoreCommandRandomTools.nextVirtualForceCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         VirtualForceCommand expectedOut = ControllerCoreCommandRandomTools.nextVirtualForceCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
         VirtualForceCommand actualOut = new VirtualForceCommand();
         crossRobotCommandResolver.resolveVirtualForceCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveVirtualModelControlOptimizationSettingsCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         VirtualModelControlOptimizationSettingsCommand in = ControllerCoreCommandRandomTools.nextVirtualModelControlOptimizationSettingsCommand(new Random(seed),
                                                                                                                                                 testData.rootBodyA,
                                                                                                                                                 testData.frameTreeA);
         VirtualModelControlOptimizationSettingsCommand expectedOut = ControllerCoreCommandRandomTools.nextVirtualModelControlOptimizationSettingsCommand(new Random(seed),
                                                                                                                                                          testData.rootBodyB,
                                                                                                                                                          testData.frameTreeB);
         VirtualModelControlOptimizationSettingsCommand actualOut = new VirtualModelControlOptimizationSettingsCommand();
         crossRobotCommandResolver.resolveVirtualModelControlOptimizationSettingsCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveVirtualTorqueCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         VirtualTorqueCommand in = ControllerCoreCommandRandomTools.nextVirtualTorqueCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         VirtualTorqueCommand expectedOut = ControllerCoreCommandRandomTools.nextVirtualTorqueCommand(new Random(seed), testData.rootBodyB,
                                                                                                      testData.frameTreeB);
         VirtualTorqueCommand actualOut = new VirtualTorqueCommand();
         crossRobotCommandResolver.resolveVirtualTorqueCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveVirtualWrenchCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         VirtualWrenchCommand in = ControllerCoreCommandRandomTools.nextVirtualWrenchCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         VirtualWrenchCommand expectedOut = ControllerCoreCommandRandomTools.nextVirtualWrenchCommand(new Random(seed), testData.rootBodyB,
                                                                                                      testData.frameTreeB);
         VirtualWrenchCommand actualOut = new VirtualWrenchCommand();
         crossRobotCommandResolver.resolveVirtualWrenchCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveCenterOfMassFeedbackControlCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         CenterOfMassFeedbackControlCommand in = ControllerCoreCommandRandomTools.nextCenterOfMassFeedbackControlCommand(new Random(seed), testData.rootBodyA,
                                                                                                                         testData.frameTreeA);
         CenterOfMassFeedbackControlCommand expectedOut = ControllerCoreCommandRandomTools.nextCenterOfMassFeedbackControlCommand(new Random(seed),
                                                                                                                                  testData.rootBodyB,
                                                                                                                                  testData.frameTreeB);
         CenterOfMassFeedbackControlCommand actualOut = new CenterOfMassFeedbackControlCommand();
         crossRobotCommandResolver.resolveCenterOfMassFeedbackControlCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveJointspaceFeedbackControlCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         JointspaceFeedbackControlCommand in = ControllerCoreCommandRandomTools.nextJointspaceFeedbackControlCommand(new Random(seed), testData.rootBodyA,
                                                                                                                     testData.frameTreeA);
         JointspaceFeedbackControlCommand expectedOut = ControllerCoreCommandRandomTools.nextJointspaceFeedbackControlCommand(new Random(seed),
                                                                                                                              testData.rootBodyB,
                                                                                                                              testData.frameTreeB);
         JointspaceFeedbackControlCommand actualOut = new JointspaceFeedbackControlCommand();
         crossRobotCommandResolver.resolveJointspaceFeedbackControlCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveOrientationFeedbackControlCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         OrientationFeedbackControlCommand in = ControllerCoreCommandRandomTools.nextOrientationFeedbackControlCommand(new Random(seed), testData.rootBodyA,
                                                                                                                       testData.frameTreeA);
         OrientationFeedbackControlCommand expectedOut = ControllerCoreCommandRandomTools.nextOrientationFeedbackControlCommand(new Random(seed),
                                                                                                                                testData.rootBodyB,
                                                                                                                                testData.frameTreeB);
         OrientationFeedbackControlCommand actualOut = new OrientationFeedbackControlCommand();
         crossRobotCommandResolver.resolveOrientationFeedbackControlCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolvePointFeedbackControlCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         PointFeedbackControlCommand in = ControllerCoreCommandRandomTools.nextPointFeedbackControlCommand(new Random(seed), testData.rootBodyA,
                                                                                                           testData.frameTreeA);
         PointFeedbackControlCommand expectedOut = ControllerCoreCommandRandomTools.nextPointFeedbackControlCommand(new Random(seed), testData.rootBodyB,
                                                                                                                    testData.frameTreeB);
         PointFeedbackControlCommand actualOut = new PointFeedbackControlCommand();
         crossRobotCommandResolver.resolvePointFeedbackControlCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @Test
   void testResolveSpatialFeedbackControlCommand() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         SpatialFeedbackControlCommand in = ControllerCoreCommandRandomTools.nextSpatialFeedbackControlCommand(new Random(seed), testData.rootBodyA,
                                                                                                               testData.frameTreeA);
         SpatialFeedbackControlCommand expectedOut = ControllerCoreCommandRandomTools.nextSpatialFeedbackControlCommand(new Random(seed), testData.rootBodyB,
                                                                                                                        testData.frameTreeB);
         SpatialFeedbackControlCommand actualOut = new SpatialFeedbackControlCommand();
         crossRobotCommandResolver.resolveSpatialFeedbackControlCommand(in, actualOut);
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
         rootBodyB = MultiBodySystemFactories.cloneMultiBodySystem(rootBodyA, rootFrameB, "Cloned");
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
