package us.ihmc.commonWalkingControlModules.controllerCore.command;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

import org.apache.commons.lang3.mutable.MutableInt;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.RandomMatrices;
import org.junit.jupiter.api.Test;
import org.reflections.Reflections;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
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
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointLimitEnforcementCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointTorqueCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualEffortCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualForceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualTorqueCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.OneDoFJointPrivilegedConfigurationParameters;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.robotModels.JointHashCodeResolver;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.kinematics.JointLimitData;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;

class CrossRobotCommandResolverTest
{
   private static final String CONTROLLER_CORE_COMMANDS_PACKAGE = "us.ihmc.commonWalkingControlModules.controllerCore.command";
   private static final int ITERATIONS = 100;

   @SuppressWarnings("rawtypes")
   @Test
   void testResolveControllerCoreCommand() throws Exception
   {
      Random random = new Random(657654);

      Reflections reflections = new Reflections(CONTROLLER_CORE_COMMANDS_PACKAGE);
      Set<Class<? extends InverseDynamicsCommand>> inverseDynamicsCommandsToGenerate = reflections.getSubTypesOf(InverseDynamicsCommand.class);
      inverseDynamicsCommandsToGenerate.remove(InverseDynamicsCommandList.class);
      inverseDynamicsCommandsToGenerate.remove(InverseDynamicsCommandBuffer.class);
      Set<Class<? extends InverseKinematicsCommand>> inverseKinematicsCommandsToGenerate = reflections.getSubTypesOf(InverseKinematicsCommand.class);
      inverseKinematicsCommandsToGenerate.remove(InverseKinematicsCommandList.class);
      inverseKinematicsCommandsToGenerate.remove(InverseKinematicsCommandBuffer.class);
      Set<Class<? extends VirtualModelControlCommand>> virtualModelControlCommandsToGenerate = reflections.getSubTypesOf(VirtualModelControlCommand.class);
      virtualModelControlCommandsToGenerate.remove(VirtualModelControlCommandList.class);
      virtualModelControlCommandsToGenerate.remove(VirtualModelControlCommandBuffer.class);
      virtualModelControlCommandsToGenerate.remove(VirtualEffortCommand.class);
      Set<Class<? extends FeedbackControlCommand>> feedbackControlCommandsToGenerate = reflections.getSubTypesOf(FeedbackControlCommand.class);
      feedbackControlCommandsToGenerate.remove(FeedbackControlCommandList.class);
      feedbackControlCommandsToGenerate.remove(FeedbackControlCommandBuffer.class);

      TestData testData = new TestData(random, 20, 20);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);
      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         ControllerCoreCommand in = nextControllerCoreCommand(new Random(seed), inverseDynamicsCommandsToGenerate, inverseKinematicsCommandsToGenerate,
                                                              virtualModelControlCommandsToGenerate, feedbackControlCommandsToGenerate, testData.rootBodyA,
                                                              testData.frameTreeA);
         ControllerCoreCommand expectedOut = nextControllerCoreCommand(new Random(seed), inverseDynamicsCommandsToGenerate, inverseKinematicsCommandsToGenerate,
                                                                       virtualModelControlCommandsToGenerate, feedbackControlCommandsToGenerate,
                                                                       testData.rootBodyB, testData.frameTreeB);
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

      Reflections reflections = new Reflections(CONTROLLER_CORE_COMMANDS_PACKAGE);
      Set<Class<? extends InverseDynamicsCommand>> commandTypes = reflections.getSubTypesOf(InverseDynamicsCommand.class);
      commandTypes.remove(InverseDynamicsCommandList.class);
      commandTypes.remove(InverseDynamicsCommandBuffer.class);

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

      Reflections reflections = new Reflections(CONTROLLER_CORE_COMMANDS_PACKAGE);
      Set<Class<? extends InverseKinematicsCommand>> commandTypes = reflections.getSubTypesOf(InverseKinematicsCommand.class);
      commandTypes.remove(InverseKinematicsCommandList.class);
      commandTypes.remove(InverseKinematicsCommandBuffer.class);

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

      Reflections reflections = new Reflections(CONTROLLER_CORE_COMMANDS_PACKAGE);
      Set<Class<? extends VirtualModelControlCommand>> commandTypes = reflections.getSubTypesOf(VirtualModelControlCommand.class);
      commandTypes.remove(VirtualModelControlCommandList.class);
      commandTypes.remove(VirtualModelControlCommandBuffer.class);

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

      Reflections reflections = new Reflections(CONTROLLER_CORE_COMMANDS_PACKAGE);
      Set<Class<? extends FeedbackControlCommand>> commandTypes = reflections.getSubTypesOf(FeedbackControlCommand.class);
      commandTypes.remove(FeedbackControlCommandList.class);
      commandTypes.remove(FeedbackControlCommandBuffer.class);

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

   @SuppressWarnings("rawtypes")
   @Test
   void testResolveInverseDynamicsCommandList() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      Reflections reflections = new Reflections(CONTROLLER_CORE_COMMANDS_PACKAGE);
      Set<Class<? extends InverseDynamicsCommand>> commandTypes = reflections.getSubTypesOf(InverseDynamicsCommand.class);
      commandTypes.remove(InverseDynamicsCommandList.class);
      commandTypes.remove(InverseDynamicsCommandBuffer.class);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         InverseDynamicsCommandList in = nextInverseDynamicsCommandList(new Random(seed), commandTypes, testData.rootBodyA, testData.frameTreeA);
         InverseDynamicsCommandList expectedOut = nextInverseDynamicsCommandList(new Random(seed), commandTypes, testData.rootBodyB, testData.frameTreeB);
         InverseDynamicsCommandBuffer actualOut = new InverseDynamicsCommandBuffer();
         crossRobotCommandResolver.resolveInverseDynamicsCommandList(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @SuppressWarnings("rawtypes")
   @Test
   void testResolveInverseKinematicsCommandList() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      Reflections reflections = new Reflections(CONTROLLER_CORE_COMMANDS_PACKAGE);
      Set<Class<? extends InverseKinematicsCommand>> commandTypes = reflections.getSubTypesOf(InverseKinematicsCommand.class);
      commandTypes.remove(InverseKinematicsCommandList.class);
      commandTypes.remove(InverseKinematicsCommandBuffer.class);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         InverseKinematicsCommandList in = nextInverseKinematicsCommandList(new Random(seed), commandTypes, testData.rootBodyA, testData.frameTreeA);
         InverseKinematicsCommandList expectedOut = nextInverseKinematicsCommandList(new Random(seed), commandTypes, testData.rootBodyB, testData.frameTreeB);
         InverseKinematicsCommandBuffer actualOut = new InverseKinematicsCommandBuffer();
         crossRobotCommandResolver.resolveInverseKinematicsCommandList(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @SuppressWarnings("rawtypes")
   @Test
   void testResolveVirtualModelControlCommandList() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      Reflections reflections = new Reflections(CONTROLLER_CORE_COMMANDS_PACKAGE);
      Set<Class<? extends VirtualModelControlCommand>> commandTypes = reflections.getSubTypesOf(VirtualModelControlCommand.class);
      commandTypes.remove(VirtualModelControlCommandList.class);
      commandTypes.remove(VirtualModelControlCommandBuffer.class);
      commandTypes.remove(VirtualEffortCommand.class);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         VirtualModelControlCommandList in = nextVirtualModelControlCommandList(new Random(seed), commandTypes, testData.rootBodyA, testData.frameTreeA);
         VirtualModelControlCommandList expectedOut = nextVirtualModelControlCommandList(new Random(seed), commandTypes, testData.rootBodyB,
                                                                                         testData.frameTreeB);
         VirtualModelControlCommandBuffer actualOut = new VirtualModelControlCommandBuffer();
         crossRobotCommandResolver.resolveVirtualModelControlCommandList(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   @SuppressWarnings("rawtypes")
   @Test
   void testResolveFeedbackControlCommandList() throws Exception
   {
      Random random = new Random(657654);

      TestData testData = new TestData(random, 20, 20);

      Reflections reflections = new Reflections(CONTROLLER_CORE_COMMANDS_PACKAGE);
      Set<Class<? extends FeedbackControlCommand>> commandTypes = reflections.getSubTypesOf(FeedbackControlCommand.class);
      commandTypes.remove(FeedbackControlCommandList.class);
      commandTypes.remove(FeedbackControlCommandBuffer.class);
      commandTypes.remove(VirtualEffortCommand.class);

      CrossRobotCommandResolver crossRobotCommandResolver = new CrossRobotCommandResolver(testData.frameResolverForB, testData.bodyResolverForB,
                                                                                          testData.jointResolverForB);

      for (int i = 0; i < ITERATIONS; i++)
      {
         long seed = random.nextLong();
         // By using the same seed on a fresh random, the two commands will be built the same way.
         FeedbackControlCommandList in = nextFeedbackControlCommandList(new Random(seed), commandTypes, testData.rootBodyA, testData.frameTreeA);
         FeedbackControlCommandList expectedOut = nextFeedbackControlCommandList(new Random(seed), commandTypes, testData.rootBodyB, testData.frameTreeB);
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
         LowLevelOneDoFJointDesiredDataHolder in = nextLowLevelOneDoFJointDesiredDataHolder(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         LowLevelOneDoFJointDesiredDataHolder expectedOut = nextLowLevelOneDoFJointDesiredDataHolder(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         CenterOfPressureCommand in = nextCenterOfPressureCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         CenterOfPressureCommand expectedOut = nextCenterOfPressureCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         ContactWrenchCommand in = nextContactWrenchCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         ContactWrenchCommand expectedOut = nextContactWrenchCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         ExternalWrenchCommand in = nextExternalWrenchCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         ExternalWrenchCommand expectedOut = nextExternalWrenchCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         InverseDynamicsOptimizationSettingsCommand in = nextInverseDynamicsOptimizationSettingsCommand(new Random(seed), testData.rootBodyA,
                                                                                                        testData.frameTreeA);
         InverseDynamicsOptimizationSettingsCommand expectedOut = nextInverseDynamicsOptimizationSettingsCommand(new Random(seed), testData.rootBodyB,
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
         JointAccelerationIntegrationCommand in = nextJointAccelerationIntegrationCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         JointAccelerationIntegrationCommand expectedOut = nextJointAccelerationIntegrationCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         JointLimitEnforcementMethodCommand in = nextJointLimitEnforcementMethodCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         JointLimitEnforcementMethodCommand expectedOut = nextJointLimitEnforcementMethodCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         JointspaceAccelerationCommand in = nextJointspaceAccelerationCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         JointspaceAccelerationCommand expectedOut = nextJointspaceAccelerationCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         MomentumRateCommand in = nextMomentumRateCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         MomentumRateCommand expectedOut = nextMomentumRateCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         PlaneContactStateCommand in = nextPlaneContactStateCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         PlaneContactStateCommand expectedOut = nextPlaneContactStateCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         SpatialAccelerationCommand in = nextSpatialAccelerationCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         SpatialAccelerationCommand expectedOut = nextSpatialAccelerationCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         InverseKinematicsOptimizationSettingsCommand in = nextInverseKinematicsOptimizationSettingsCommand(new Random(seed), testData.rootBodyA,
                                                                                                            testData.frameTreeA);
         InverseKinematicsOptimizationSettingsCommand expectedOut = nextInverseKinematicsOptimizationSettingsCommand(new Random(seed), testData.rootBodyB,
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
         JointLimitReductionCommand in = nextJointLimitReductionCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         JointLimitReductionCommand expectedOut = nextJointLimitReductionCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         JointspaceVelocityCommand in = nextJointspaceVelocityCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         JointspaceVelocityCommand expectedOut = nextJointspaceVelocityCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         MomentumCommand in = nextMomentumCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         MomentumCommand expectedOut = nextMomentumCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         PrivilegedConfigurationCommand in = nextPrivilegedConfigurationCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         PrivilegedConfigurationCommand expectedOut = nextPrivilegedConfigurationCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         PrivilegedJointSpaceCommand in = nextPrivilegedJointSpaceCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         PrivilegedJointSpaceCommand expectedOut = nextPrivilegedJointSpaceCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         SpatialVelocityCommand in = nextSpatialVelocityCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         SpatialVelocityCommand expectedOut = nextSpatialVelocityCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         JointLimitEnforcementCommand in = nextJointLimitEnforcementCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         JointLimitEnforcementCommand expectedOut = nextJointLimitEnforcementCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         JointTorqueCommand in = nextJointTorqueCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         JointTorqueCommand expectedOut = nextJointTorqueCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         VirtualForceCommand in = nextVirtualForceCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         VirtualForceCommand expectedOut = nextVirtualForceCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         VirtualModelControlOptimizationSettingsCommand in = nextVirtualModelControlOptimizationSettingsCommand(new Random(seed), testData.rootBodyA,
                                                                                                                testData.frameTreeA);
         VirtualModelControlOptimizationSettingsCommand expectedOut = nextVirtualModelControlOptimizationSettingsCommand(new Random(seed), testData.rootBodyB,
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
         VirtualTorqueCommand in = nextVirtualTorqueCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         VirtualTorqueCommand expectedOut = nextVirtualTorqueCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         VirtualWrenchCommand in = nextVirtualWrenchCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         VirtualWrenchCommand expectedOut = nextVirtualWrenchCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         CenterOfMassFeedbackControlCommand in = nextCenterOfMassFeedbackControlCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         CenterOfMassFeedbackControlCommand expectedOut = nextCenterOfMassFeedbackControlCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         JointspaceFeedbackControlCommand in = nextJointspaceFeedbackControlCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         JointspaceFeedbackControlCommand expectedOut = nextJointspaceFeedbackControlCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         OrientationFeedbackControlCommand in = nextOrientationFeedbackControlCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         OrientationFeedbackControlCommand expectedOut = nextOrientationFeedbackControlCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         PointFeedbackControlCommand in = nextPointFeedbackControlCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         PointFeedbackControlCommand expectedOut = nextPointFeedbackControlCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
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
         SpatialFeedbackControlCommand in = nextSpatialFeedbackControlCommand(new Random(seed), testData.rootBodyA, testData.frameTreeA);
         SpatialFeedbackControlCommand expectedOut = nextSpatialFeedbackControlCommand(new Random(seed), testData.rootBodyB, testData.frameTreeB);
         SpatialFeedbackControlCommand actualOut = new SpatialFeedbackControlCommand();
         crossRobotCommandResolver.resolveSpatialFeedbackControlCommand(in, actualOut);
         assertEquals(expectedOut, actualOut, "Iteration: " + i);
      }
   }

   public static CenterOfPressureCommand nextCenterOfPressureCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      CenterOfPressureCommand next = new CenterOfPressureCommand();
      next.setConstraintType(nextElementIn(random, ConstraintType.values()));
      next.setContactingRigidBody(nextElementIn(random, rootBody.subtreeList()));
      next.setWeight(nextFrameVector2D(random, possibleFrames));
      next.setDesiredCoP(nextFramePoint2D(random, possibleFrames));
      return next;
   }

   public static ContactWrenchCommand nextContactWrenchCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      ContactWrenchCommand next = new ContactWrenchCommand();
      next.setConstraintType(nextElementIn(random, ConstraintType.values()));
      next.setRigidBody(nextElementIn(random, rootBody.subtreeList()));
      next.getWrench().setIncludingFrame(nextWrench(random, possibleFrames));
      next.getWeightMatrix().set(nextWeightMatrix6D(random, possibleFrames));
      next.getSelectionMatrix().set(nextSelectionMatrix6D(random, possibleFrames));
      return next;
   }

   public static ExternalWrenchCommand nextExternalWrenchCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      ExternalWrenchCommand next = new ExternalWrenchCommand();
      next.setRigidBody(nextElementIn(random, rootBody.subtreeList()));
      next.getExternalWrench().setIncludingFrame(nextWrench(random, possibleFrames));
      return next;
   }

   public static InverseDynamicsOptimizationSettingsCommand nextInverseDynamicsOptimizationSettingsCommand(Random random, RigidBodyBasics rootBody,
                                                                                                           ReferenceFrame... possibleFrames)
   {
      InverseDynamicsOptimizationSettingsCommand next = new InverseDynamicsOptimizationSettingsCommand();
      next.setRhoMin(random.nextDouble());
      next.setJointAccelerationMax(random.nextDouble());
      next.setRhoWeight(random.nextDouble());
      next.setRhoRateWeight(random.nextDouble());
      next.setCenterOfPressureWeight(EuclidCoreRandomTools.nextPoint2D(random));
      next.setCenterOfPressureRateWeight(EuclidCoreRandomTools.nextPoint2D(random));
      next.setJointAccelerationWeight(random.nextDouble());
      next.setJointJerkWeight(random.nextDouble());
      next.setJointTorqueWeight(random.nextDouble());
      return next;
   }

   public static JointAccelerationIntegrationCommand nextJointAccelerationIntegrationCommand(Random random, RigidBodyBasics rootBody,
                                                                                             ReferenceFrame... possibleFrames)
   {
      JointAccelerationIntegrationCommand next = new JointAccelerationIntegrationCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         next.addJointToComputeDesiredPositionFor(allJoints.remove(random.nextInt(allJoints.size())));
         next.setJointParameters(jointIndex, nextJointAccelerationIntegrationParameters(random));
      }

      return next;
   }

   public static JointLimitEnforcementMethodCommand nextJointLimitEnforcementMethodCommand(Random random, RigidBodyBasics rootBody,
                                                                                           ReferenceFrame... possibleFrames)
   {
      JointLimitEnforcementMethodCommand next = new JointLimitEnforcementMethodCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         next.addLimitEnforcementMethod(allJoints.remove(random.nextInt(allJoints.size())), nextElementIn(random, JointLimitEnforcement.values()),
                                        nextJointLimitParameters(random));
      }

      return next;
   }

   public static JointspaceAccelerationCommand nextJointspaceAccelerationCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      JointspaceAccelerationCommand next = new JointspaceAccelerationCommand();

      List<JointBasics> allJoints = SubtreeStreams.fromChildren(rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         JointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         DenseMatrix64F desiredAcceleration = RandomMatrices.createRandom(joint.getDegreesOfFreedom(), 1, random);
         next.addJoint(joint, desiredAcceleration, random.nextDouble());
      }

      return next;
   }

   public static MomentumRateCommand nextMomentumRateCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      MomentumRateCommand next = new MomentumRateCommand();
      next.setMomentumRate(RandomMatrices.createRandom(6, 1, random));
      next.setWeights(nextWeightMatrix6D(random, possibleFrames));
      next.setSelectionMatrix(nextSelectionMatrix6D(random, possibleFrames));
      return next;
   }

   public static PlaneContactStateCommand nextPlaneContactStateCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      PlaneContactStateCommand next = new PlaneContactStateCommand();
      next.setContactingRigidBody(nextElementIn(random, rootBody.subtreeList()));
      next.setCoefficientOfFriction(random.nextDouble());
      next.setContactNormal(EuclidFrameRandomTools.nextFrameVector3D(random, nextElementIn(random, possibleFrames)));
      next.setUseHighCoPDamping(random.nextBoolean());
      next.setHasContactStateChanged(random.nextBoolean());
      if (random.nextBoolean())
         next.getContactFramePoseInBodyFixedFrame().set(EuclidCoreRandomTools.nextRigidBodyTransform(random));

      int numberOfContactPoints = random.nextInt(20);

      for (int i = 0; i < numberOfContactPoints; i++)
      {
         next.addPointInContact(EuclidFrameRandomTools.nextFramePoint3D(random, nextElementIn(random, possibleFrames)));
         if (random.nextBoolean())
            next.setMaxContactPointNormalForce(i, random.nextDouble());
         if (random.nextBoolean())
            next.setRhoWeight(i, random.nextDouble());
      }

      return next;
   }

   public static SpatialAccelerationCommand nextSpatialAccelerationCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      SpatialAccelerationCommand next = new SpatialAccelerationCommand();
      next.set(nextElementIn(random, rootBody.subtreeList()), nextElementIn(random, rootBody.subtreeList()));
      next.getControlFramePose().setIncludingFrame(nextFramePose3D(random, possibleFrames));
      next.getDesiredLinearAcceleration().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.getDesiredAngularAcceleration().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.setWeightMatrix(nextWeightMatrix6D(random, possibleFrames));
      next.setSelectionMatrix(nextSelectionMatrix6D(random, possibleFrames));
      if (random.nextBoolean())
         next.setPrimaryBase(nextElementIn(random, rootBody.subtreeList()));
      if (random.nextBoolean())
         next.setScaleSecondaryTaskJointWeight(true, random.nextDouble());
      return next;
   }

   public static InverseKinematicsOptimizationSettingsCommand nextInverseKinematicsOptimizationSettingsCommand(Random random, RigidBodyBasics rootBody,
                                                                                                               ReferenceFrame... possibleFrames)
   {
      InverseKinematicsOptimizationSettingsCommand next = new InverseKinematicsOptimizationSettingsCommand();
      next.setJointVelocityWeight(random.nextDouble());
      next.setJointAccelerationWeight(random.nextDouble());
      return next;
   }

   public static JointLimitReductionCommand nextJointLimitReductionCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      JointLimitReductionCommand next = new JointLimitReductionCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.addReductionFactor(joint, random.nextDouble());
      }

      return next;
   }

   public static JointspaceVelocityCommand nextJointspaceVelocityCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      JointspaceVelocityCommand next = new JointspaceVelocityCommand();

      List<JointBasics> allJoints = SubtreeStreams.fromChildren(rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         JointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.addJoint(joint, RandomMatrices.createRandom(joint.getDegreesOfFreedom(), 1, random), random.nextDouble());
      }

      return next;
   }

   public static MomentumCommand nextMomentumCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      MomentumCommand next = new MomentumCommand();
      next.setMomentum(RandomMatrices.createRandom(6, 1, random));
      next.getWeightMatrix().set(nextWeightMatrix6D(random, possibleFrames));
      next.getSelectionMatrix().set(nextSelectionMatrix6D(random, possibleFrames));
      return next;
   }

   public static PrivilegedConfigurationCommand nextPrivilegedConfigurationCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      PrivilegedConfigurationCommand next = new PrivilegedConfigurationCommand();

      if (random.nextBoolean())
         next.setDefaultParameters(nextOneDoFJointPrivilegedConfigurationParameters(random));

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.addJoint(joint, nextOneDoFJointPrivilegedConfigurationParameters(random));
      }

      if (random.nextBoolean())
         next.enable();
      else
         next.disable();

      return next;
   }

   public static PrivilegedJointSpaceCommand nextPrivilegedJointSpaceCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      PrivilegedJointSpaceCommand next = new PrivilegedJointSpaceCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));

         next.addJoint(joint, random.nextDouble());
         if (random.nextBoolean())
            next.setWeight(jointIndex, random.nextDouble());
      }

      if (random.nextBoolean())
         next.enable();
      else
         next.disable();

      return next;
   }

   public static SpatialVelocityCommand nextSpatialVelocityCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      SpatialVelocityCommand next = new SpatialVelocityCommand();
      next.set(nextElementIn(random, rootBody.subtreeList()), nextElementIn(random, rootBody.subtreeList()));
      next.getControlFramePose().setIncludingFrame(nextFramePose3D(random, possibleFrames));
      next.getDesiredLinearVelocity().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.getDesiredAngularVelocity().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.setWeightMatrix(nextWeightMatrix6D(random, possibleFrames));
      next.setSelectionMatrix(nextSelectionMatrix6D(random, possibleFrames));
      if (random.nextBoolean())
         next.setPrimaryBase(nextElementIn(random, rootBody.subtreeList()));
      if (random.nextBoolean())
         next.setScaleSecondaryTaskJointWeight(true, random.nextDouble());
      return next;
   }

   public static JointLimitEnforcementCommand nextJointLimitEnforcementCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      JointLimitEnforcementCommand next = new JointLimitEnforcementCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.addJoint(joint, nextJointLimitData(random));
      }

      return next;
   }

   public static JointTorqueCommand nextJointTorqueCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      JointTorqueCommand next = new JointTorqueCommand();

      List<JointBasics> allJoints = SubtreeStreams.fromChildren(rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         JointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.addJoint(joint, RandomMatrices.createRandom(joint.getDegreesOfFreedom(), 1, random));
      }

      return next;
   }

   public static VirtualForceCommand nextVirtualForceCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      VirtualForceCommand next = new VirtualForceCommand();

      next.set(nextElementIn(random, rootBody.subtreeList()), nextElementIn(random, rootBody.subtreeList()));
      next.getDesiredLinearForce().set(EuclidCoreRandomTools.nextVector3D(random));
      next.getControlFramePose().setIncludingFrame(nextFramePose3D(random, possibleFrames));
      next.setSelectionMatrix(nextSelectionMatrix3D(random, possibleFrames));

      return next;
   }

   public static VirtualModelControlOptimizationSettingsCommand nextVirtualModelControlOptimizationSettingsCommand(Random random, RigidBodyBasics rootBody,
                                                                                                                   ReferenceFrame... possibleFrames)
   {
      VirtualModelControlOptimizationSettingsCommand next = new VirtualModelControlOptimizationSettingsCommand();
      next.setRhoMin(random.nextDouble());
      next.setRhoWeight(random.nextDouble());
      next.setRhoRateWeight(random.nextDouble());
      next.setCenterOfPressureWeight(EuclidCoreRandomTools.nextPoint2D(random));
      next.setCenterOfPressureRateWeight(EuclidCoreRandomTools.nextPoint2D(random));
      next.setMomentumRateWeight(random.nextDouble());
      next.setMomentumAccelerationWeight(random.nextDouble());
      return next;
   }

   public static VirtualTorqueCommand nextVirtualTorqueCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      VirtualTorqueCommand next = new VirtualTorqueCommand();

      next.set(nextElementIn(random, rootBody.subtreeList()), nextElementIn(random, rootBody.subtreeList()));
      next.getDesiredAngularTorque().set(EuclidCoreRandomTools.nextVector3D(random));
      next.getControlFramePose().setIncludingFrame(nextFramePose3D(random, possibleFrames));
      next.setSelectionMatrix(nextSelectionMatrix3D(random, possibleFrames));

      return next;
   }

   public static VirtualWrenchCommand nextVirtualWrenchCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      VirtualWrenchCommand next = new VirtualWrenchCommand();

      next.set(nextElementIn(random, rootBody.subtreeList()), nextElementIn(random, rootBody.subtreeList()));
      next.getDesiredLinearForce().set(EuclidCoreRandomTools.nextVector3D(random));
      next.getDesiredAngularTorque().set(EuclidCoreRandomTools.nextVector3D(random));
      next.getControlFramePose().setIncludingFrame(nextFramePose3D(random, possibleFrames));
      next.setSelectionMatrix(nextSelectionMatrix6D(random, possibleFrames));

      return next;
   }

   public static CenterOfMassFeedbackControlCommand nextCenterOfMassFeedbackControlCommand(Random random, RigidBodyBasics rootBody,
                                                                                           ReferenceFrame... possibleFrames)
   {
      CenterOfMassFeedbackControlCommand next = new CenterOfMassFeedbackControlCommand();
      next.getDesiredPosition().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.getDesiredLinearVelocity().set(EuclidCoreRandomTools.nextVector3D(random));
      next.getFeedForwardLinearAction().set(EuclidCoreRandomTools.nextVector3D(random));
      next.setGains(nextPID3DGains(random));
      next.getMomentumRateCommand().set(nextMomentumRateCommand(random, rootBody, possibleFrames));
      return next;
   }

   public static JointspaceFeedbackControlCommand nextJointspaceFeedbackControlCommand(Random random, RigidBodyBasics rootBody,
                                                                                       ReferenceFrame... possibleFrames)
   {
      JointspaceFeedbackControlCommand next = new JointspaceFeedbackControlCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.addJoint(joint, random.nextDouble(), random.nextDouble(), random.nextDouble(), nextPDGains(random), random.nextDouble());
      }

      return next;
   }

   public static OrientationFeedbackControlCommand nextOrientationFeedbackControlCommand(Random random, RigidBodyBasics rootBody,
                                                                                         ReferenceFrame... possibleFrames)
   {
      OrientationFeedbackControlCommand next = new OrientationFeedbackControlCommand();
      next.getBodyFixedOrientationToControl().setIncludingFrame(nextFrameQuaternion(random, possibleFrames));
      next.getDesiredOrientation().setIncludingFrame(nextFrameQuaternion(random, possibleFrames));
      next.getDesiredAngularVelocity().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getFeedForwardAngularAction().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getGains().set(nextPID3DGains(random));
      next.setGainsFrame(nextElementIn(random, possibleFrames));
      next.getSpatialAccelerationCommand().set(nextSpatialAccelerationCommand(random, rootBody, possibleFrames));
      next.setControlBaseFrame(nextElementIn(random, possibleFrames));
      return next;
   }

   public static PointFeedbackControlCommand nextPointFeedbackControlCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      PointFeedbackControlCommand next = new PointFeedbackControlCommand();
      next.getBodyFixedPointToControl().setIncludingFrame(nextFramePoint3D(random, possibleFrames));
      next.getDesiredPosition().setIncludingFrame(nextFramePoint3D(random, possibleFrames));
      next.getDesiredLinearVelocity().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getFeedForwardLinearAction().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getGains().set(nextPID3DGains(random));
      next.setGainsFrame(nextElementIn(random, possibleFrames));
      next.getSpatialAccelerationCommand().set(nextSpatialAccelerationCommand(random, rootBody, possibleFrames));
      next.setControlBaseFrame(nextElementIn(random, possibleFrames));
      return next;
   }

   public static SpatialFeedbackControlCommand nextSpatialFeedbackControlCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      SpatialFeedbackControlCommand next = new SpatialFeedbackControlCommand();
      next.getControlFramePose().setIncludingFrame(nextFramePose3D(random, possibleFrames));
      next.getDesiredOrientation().setIncludingFrame(nextFrameQuaternion(random, possibleFrames));
      next.getDesiredAngularVelocity().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getFeedForwardAngularAction().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getGains().set(nextPIDSE3Gains(random));
      next.setGainsFrames(nextElementIn(random, possibleFrames), nextElementIn(random, possibleFrames));
      next.getSpatialAccelerationCommand().set(nextSpatialAccelerationCommand(random, rootBody, possibleFrames));
      next.setControlBaseFrame(nextElementIn(random, possibleFrames));
      next.getDesiredPosition().setIncludingFrame(nextFramePoint3D(random, possibleFrames));
      next.getDesiredLinearVelocity().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getFeedForwardLinearAction().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getSpatialAccelerationCommand().set(nextSpatialAccelerationCommand(random, rootBody, possibleFrames));
      next.setControlBaseFrame(nextElementIn(random, possibleFrames));
      return next;
   }

   @SuppressWarnings("rawtypes")
   public static InverseDynamicsCommandList nextInverseDynamicsCommandList(Random random,
                                                                           Collection<Class<? extends InverseDynamicsCommand>> commandsToGenerate,
                                                                           RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      InverseDynamicsCommandList next = new InverseDynamicsCommandList();

      List<Class<? extends InverseDynamicsCommand>> commandTypes = new ArrayList<>(commandsToGenerate);
      List<MutableInt> numberOfCommandsToGenerate = new ArrayList<>();
      commandTypes.forEach(c -> numberOfCommandsToGenerate.add(new MutableInt(random.nextInt(10))));

      while (!commandTypes.isEmpty())
      {
         int index = random.nextInt(commandTypes.size());
         Class<? extends InverseDynamicsCommand> commandType = commandTypes.get(index);
         if (numberOfCommandsToGenerate.get(index).getAndDecrement() == 0)
         {
            commandTypes.remove(index);
            numberOfCommandsToGenerate.remove(index);
         }

         Method randomGenerator = CrossRobotCommandResolverTest.class.getDeclaredMethod("next" + commandType.getSimpleName(), Random.class,
                                                                                        RigidBodyBasics.class, ReferenceFrame[].class);
         InverseDynamicsCommand<?> command = (InverseDynamicsCommand<?>) randomGenerator.invoke(null, random, rootBody, possibleFrames);
         next.addCommand(command);
      }
      return next;
   }

   @SuppressWarnings("rawtypes")
   public static InverseKinematicsCommandList nextInverseKinematicsCommandList(Random random,
                                                                               Collection<Class<? extends InverseKinematicsCommand>> commandsToGenerate,
                                                                               RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      InverseKinematicsCommandList next = new InverseKinematicsCommandList();

      List<Class<? extends InverseKinematicsCommand>> commandTypes = new ArrayList<>(commandsToGenerate);
      List<MutableInt> numberOfCommandsToGenerate = new ArrayList<>();
      commandTypes.forEach(c -> numberOfCommandsToGenerate.add(new MutableInt(random.nextInt(10))));

      while (!commandTypes.isEmpty())
      {
         int index = random.nextInt(commandTypes.size());
         Class<? extends InverseKinematicsCommand> commandType = commandTypes.get(index);
         if (numberOfCommandsToGenerate.get(index).getAndDecrement() == 0)
         {
            commandTypes.remove(index);
            numberOfCommandsToGenerate.remove(index);
         }

         Method randomGenerator = CrossRobotCommandResolverTest.class.getDeclaredMethod("next" + commandType.getSimpleName(), Random.class,
                                                                                        RigidBodyBasics.class, ReferenceFrame[].class);
         InverseKinematicsCommand<?> command = (InverseKinematicsCommand<?>) randomGenerator.invoke(null, random, rootBody, possibleFrames);
         next.addCommand(command);
      }
      return next;
   }

   @SuppressWarnings("rawtypes")
   public static VirtualModelControlCommandList nextVirtualModelControlCommandList(Random random,
                                                                                   Collection<Class<? extends VirtualModelControlCommand>> commandsToGenerate,
                                                                                   RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      VirtualModelControlCommandList next = new VirtualModelControlCommandList();

      List<Class<? extends VirtualModelControlCommand>> commandTypes = new ArrayList<>(commandsToGenerate);
      List<MutableInt> numberOfCommandsToGenerate = new ArrayList<>();
      commandTypes.forEach(c -> numberOfCommandsToGenerate.add(new MutableInt(random.nextInt(10))));

      while (!commandTypes.isEmpty())
      {
         int index = random.nextInt(commandTypes.size());
         Class<? extends VirtualModelControlCommand> commandType = commandTypes.get(index);
         if (numberOfCommandsToGenerate.get(index).getAndDecrement() == 0)
         {
            commandTypes.remove(index);
            numberOfCommandsToGenerate.remove(index);
         }

         Method randomGenerator = CrossRobotCommandResolverTest.class.getDeclaredMethod("next" + commandType.getSimpleName(), Random.class,
                                                                                        RigidBodyBasics.class, ReferenceFrame[].class);
         VirtualModelControlCommand<?> command = (VirtualModelControlCommand<?>) randomGenerator.invoke(null, random, rootBody, possibleFrames);
         next.addCommand(command);
      }
      return next;
   }

   @SuppressWarnings("rawtypes")
   public static FeedbackControlCommandList nextFeedbackControlCommandList(Random random,
                                                                           Collection<Class<? extends FeedbackControlCommand>> commandsToGenerate,
                                                                           RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      FeedbackControlCommandList next = new FeedbackControlCommandList();

      List<Class<? extends FeedbackControlCommand>> commandTypes = new ArrayList<>(commandsToGenerate);
      List<MutableInt> numberOfCommandsToGenerate = new ArrayList<>();
      commandTypes.forEach(c -> numberOfCommandsToGenerate.add(new MutableInt(random.nextInt(10))));

      while (!commandTypes.isEmpty())
      {
         int index = random.nextInt(commandTypes.size());
         Class<? extends FeedbackControlCommand> commandType = commandTypes.get(index);
         if (numberOfCommandsToGenerate.get(index).getAndDecrement() == 0)
         {
            commandTypes.remove(index);
            numberOfCommandsToGenerate.remove(index);
         }

         Method randomGenerator = CrossRobotCommandResolverTest.class.getDeclaredMethod("next" + commandType.getSimpleName(), Random.class,
                                                                                        RigidBodyBasics.class, ReferenceFrame[].class);
         FeedbackControlCommand<?> command = (FeedbackControlCommand<?>) randomGenerator.invoke(null, random, rootBody, possibleFrames);
         next.addCommand(command);
      }
      return next;
   }

   public static LowLevelOneDoFJointDesiredDataHolder nextLowLevelOneDoFJointDesiredDataHolder(Random random, RigidBodyBasics rootBody,
                                                                                               ReferenceFrame... possibleFrames)
   {
      LowLevelOneDoFJointDesiredDataHolder next = new LowLevelOneDoFJointDesiredDataHolder();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.registerJointWithEmptyData(joint).set(nextJointDesiredOutput(random));
      }

      return next;
   }

   @SuppressWarnings("rawtypes")
   public static ControllerCoreCommand nextControllerCoreCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      Reflections reflections = new Reflections(CONTROLLER_CORE_COMMANDS_PACKAGE);
      Set<Class<? extends InverseDynamicsCommand>> inverseDynamicsCommandsToGenerate = reflections.getSubTypesOf(InverseDynamicsCommand.class);
      inverseDynamicsCommandsToGenerate.remove(InverseDynamicsCommandList.class);
      inverseDynamicsCommandsToGenerate.remove(InverseDynamicsCommandBuffer.class);
      Set<Class<? extends InverseKinematicsCommand>> inverseKinematicsCommandsToGenerate = reflections.getSubTypesOf(InverseKinematicsCommand.class);
      inverseKinematicsCommandsToGenerate.remove(InverseKinematicsCommandList.class);
      inverseKinematicsCommandsToGenerate.remove(InverseKinematicsCommandBuffer.class);
      Set<Class<? extends VirtualModelControlCommand>> virtualModelControlCommandsToGenerate = reflections.getSubTypesOf(VirtualModelControlCommand.class);
      virtualModelControlCommandsToGenerate.remove(VirtualModelControlCommandList.class);
      virtualModelControlCommandsToGenerate.remove(VirtualModelControlCommandBuffer.class);
      virtualModelControlCommandsToGenerate.remove(VirtualEffortCommand.class);
      Set<Class<? extends FeedbackControlCommand>> feedbackControlCommandsToGenerate = reflections.getSubTypesOf(FeedbackControlCommand.class);
      feedbackControlCommandsToGenerate.remove(FeedbackControlCommandList.class);
      feedbackControlCommandsToGenerate.remove(FeedbackControlCommandBuffer.class);

      return nextControllerCoreCommand(random, inverseDynamicsCommandsToGenerate, inverseKinematicsCommandsToGenerate, virtualModelControlCommandsToGenerate,
                                       feedbackControlCommandsToGenerate, rootBody, possibleFrames);
   }

   @SuppressWarnings("rawtypes")
   public static ControllerCoreCommand nextControllerCoreCommand(Random random,
                                                                 Collection<Class<? extends InverseDynamicsCommand>> inverseDynamicsCommandsToGenerate,
                                                                 Collection<Class<? extends InverseKinematicsCommand>> inverseKinematicsCommandsToGenerate,
                                                                 Collection<Class<? extends VirtualModelControlCommand>> virtualModelControlCommandsToGenerate,
                                                                 Collection<Class<? extends FeedbackControlCommand>> feedbackControlCommandsToGenerate,
                                                                 RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, IllegalAccessException, InvocationTargetException
   {
      ControllerCoreCommand next = new ControllerCoreCommand(nextElementIn(random, WholeBodyControllerCoreMode.values()));

      next.getInverseDynamicsCommandList().set(nextInverseDynamicsCommandList(random, inverseDynamicsCommandsToGenerate, rootBody, possibleFrames));
      next.getInverseKinematicsCommandList().set(nextInverseKinematicsCommandList(random, inverseKinematicsCommandsToGenerate, rootBody, possibleFrames));
      next.getVirtualModelControlCommandList().set(nextVirtualModelControlCommandList(random, virtualModelControlCommandsToGenerate, rootBody, possibleFrames));
      next.getFeedbackControlCommandList().set(nextFeedbackControlCommandList(random, feedbackControlCommandsToGenerate, rootBody, possibleFrames));
      next.getLowLevelOneDoFJointDesiredDataHolder().overwriteWith(nextLowLevelOneDoFJointDesiredDataHolder(random, rootBody, possibleFrames));
      if (random.nextBoolean())
         next.requestReinitialization();
      return next;
   }

   @SafeVarargs
   public static <E> E nextElementIn(Random random, E... elements)
   {
      return elements[random.nextInt(elements.length)];
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

   public static FrameQuaternion nextFrameQuaternion(Random random, ReferenceFrame... possibleFrames)
   {
      return EuclidFrameRandomTools.nextFrameQuaternion(random, nextElementIn(random, possibleFrames));
   }

   public static FramePose3D nextFramePose3D(Random random, ReferenceFrame... possibleFrames)
   {
      return EuclidFrameRandomTools.nextFramePose3D(random, nextElementIn(random, possibleFrames));
   }

   public static Wrench nextWrench(Random random, ReferenceFrame... possibleFrames)
   {
      return MecanoRandomTools.nextWrench(random, nextElementIn(random, possibleFrames), nextElementIn(random, possibleFrames));
   }

   public static SpatialAcceleration nextSpatialAcceleration(Random random, ReferenceFrame... possibleFrames)
   {
      return MecanoRandomTools.nextSpatialAcceleration(random, nextElementIn(random, possibleFrames), nextElementIn(random, possibleFrames),
                                                       nextElementIn(random, possibleFrames));
   }

   public static WeightMatrix3D nextWeightMatrix3D(Random random, ReferenceFrame... possibleFrames)
   {
      WeightMatrix3D next = new WeightMatrix3D();
      next.setWeights(random.nextDouble(), random.nextDouble(), random.nextDouble());
      next.setWeightFrame(nextElementIn(random, possibleFrames));
      return next;
   }

   public static WeightMatrix6D nextWeightMatrix6D(Random random, ReferenceFrame... possibleFrames)
   {
      WeightMatrix6D next = new WeightMatrix6D();
      next.setAngularPart(nextWeightMatrix3D(random, possibleFrames));
      next.setLinearPart(nextWeightMatrix3D(random, possibleFrames));
      return next;
   }

   public static SelectionMatrix3D nextSelectionMatrix3D(Random random, ReferenceFrame... possibleFrames)
   {
      SelectionMatrix3D next = new SelectionMatrix3D();
      next.setAxisSelection(random.nextBoolean(), random.nextBoolean(), random.nextBoolean());
      next.setSelectionFrame(nextElementIn(random, possibleFrames));
      return next;
   }

   public static SelectionMatrix6D nextSelectionMatrix6D(Random random, ReferenceFrame... possibleFrames)
   {
      SelectionMatrix6D next = new SelectionMatrix6D();
      next.setAngularPart(nextSelectionMatrix3D(random, possibleFrames));
      next.setLinearPart(nextSelectionMatrix3D(random, possibleFrames));
      return next;
   }

   public static JointAccelerationIntegrationParameters nextJointAccelerationIntegrationParameters(Random random)
   {
      JointAccelerationIntegrationParameters next = new JointAccelerationIntegrationParameters();
      next.setPositionBreakFrequency(random.nextDouble());
      next.setVelocityBreakFrequency(random.nextDouble());
      next.setMaxPositionError(random.nextDouble());
      next.setMaxVelocity(random.nextDouble());
      return next;
   }

   public static JointLimitParameters nextJointLimitParameters(Random random)
   {
      JointLimitParameters next = new JointLimitParameters();
      next.setMaxAbsJointVelocity(random.nextDouble());
      next.setJointLimitDistanceForMaxVelocity(random.nextDouble());
      next.setJointLimitFilterBreakFrequency(random.nextDouble());
      next.setVelocityControlGain(random.nextDouble());
      return next;
   }

   public static OneDoFJointPrivilegedConfigurationParameters nextOneDoFJointPrivilegedConfigurationParameters(Random random)
   {
      OneDoFJointPrivilegedConfigurationParameters next = new OneDoFJointPrivilegedConfigurationParameters();
      if (random.nextBoolean())
         next.setPrivilegedConfiguration(random.nextDouble());
      if (random.nextBoolean())
         next.setPrivilegedConfigurationOption(nextElementIn(random, PrivilegedConfigurationOption.values()));
      if (random.nextBoolean())
         next.setWeight(random.nextDouble());
      if (random.nextBoolean())
         next.setConfigurationGain(random.nextDouble());
      if (random.nextBoolean())
         next.setVelocityGain(random.nextDouble());
      if (random.nextBoolean())
         next.setMaxVelocity(random.nextDouble());
      if (random.nextBoolean())
         next.setMaxAcceleration(random.nextDouble());
      return next;
   }

   public static JointLimitData nextJointLimitData(Random random)
   {
      JointLimitData next = new JointLimitData();
      next.setPositionSoftLowerLimit(random.nextDouble());
      next.setPositionSoftUpperLimit(random.nextDouble());
      next.setVelocityLowerLimit(random.nextDouble());
      next.setVelocityUpperLimit(random.nextDouble());
      next.setTorqueLowerLimit(random.nextDouble());
      next.setTorqueUpperLimit(random.nextDouble());
      next.setPositionLimitStiffness(random.nextDouble());
      next.setPositionLimitDamping(random.nextDouble());
      return next;
   }

   public static PDGains nextPDGains(Random random)
   {
      PDGains next = new PDGains();
      next.set(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
      return next;
   }

   public static PID3DGains nextPID3DGains(Random random)
   {
      PID3DGains next = new DefaultPID3DGains();

      next.setProportionalGains(random.nextDouble(), random.nextDouble(), random.nextDouble());
      next.setDerivativeGains(random.nextDouble(), random.nextDouble(), random.nextDouble());
      next.setIntegralGains(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
      next.setMaxDerivativeError(random.nextDouble());
      next.setMaxFeedbackAndFeedbackRate(random.nextDouble(), random.nextDouble());

      return next;
   }

   public static PIDSE3Gains nextPIDSE3Gains(Random random)
   {
      PIDSE3Gains next = new DefaultPIDSE3Gains();
      next.setPositionGains(nextPID3DGains(random));
      next.setOrientationGains(nextPID3DGains(random));
      return next;
   }

   public static JointDesiredOutput nextJointDesiredOutput(Random random)
   {
      JointDesiredOutput next = new JointDesiredOutput();
      next.setControlMode(nextElementIn(random, JointDesiredControlMode.values()));
      if (random.nextBoolean())
         next.setDesiredTorque(random.nextDouble());
      if (random.nextBoolean())
         next.setDesiredPosition(random.nextDouble());
      if (random.nextBoolean())
         next.setDesiredVelocity(random.nextDouble());
      if (random.nextBoolean())
         next.setDesiredAcceleration(random.nextDouble());
      next.setResetIntegrators(random.nextBoolean());
      if (random.nextBoolean())
         next.setStiffness(random.nextDouble());
      if (random.nextBoolean())
         next.setDamping(random.nextDouble());
      if (random.nextBoolean())
         next.setMasterGain(random.nextDouble());
      if (random.nextBoolean())
         next.setVelocityScaling(random.nextDouble());
      if (random.nextBoolean())
         next.setVelocityIntegrationBreakFrequency(random.nextDouble());
      if (random.nextBoolean())
         next.setPositionIntegrationBreakFrequency(random.nextDouble());
      if (random.nextBoolean())
         next.setMaxPositionError(random.nextDouble());
      if (random.nextBoolean())
         next.setMaxVelocityError(random.nextDouble());
      return next;
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
