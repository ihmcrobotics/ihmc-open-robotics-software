package us.ihmc.commonWalkingControlModules.controllerCore.command;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.stream.Collectors;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.RandomMatrices;
import org.junit.jupiter.api.Test;
import org.reflections.Reflections;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.CenterOfPressureCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ContactWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointLimitEnforcementMethodCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointspaceAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.MomentumRateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.SpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsOptimizationSettingsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointLimitReductionCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.JointspaceVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.MomentumCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedConfigurationCommand.PrivilegedConfigurationOption;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.PrivilegedJointSpaceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.SpatialVelocityCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointLimitEnforcementCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.JointTorqueCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualForceCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
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
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.kinematics.JointLimitData;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

class CrossRobotCommandResolverTest
{
   private static final String CONTROLLER_CORE_COMMANDS_PACKAGE = "us.ihmc.commonWalkingControlModules.controllerCore.command";
   private static final int ITERATIONS = 20;

   @SuppressWarnings("rawtypes")
   @Test
   void testInverseDynamicsCommandCoverage() throws Exception
   {
      boolean verbose = true;

      Reflections reflections = new Reflections(CONTROLLER_CORE_COMMANDS_PACKAGE);
      Set<Class<? extends InverseDynamicsCommand>> commandTypes = reflections.getSubTypesOf(InverseDynamicsCommand.class);

      String errorMessage = "";

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

      String errorMessage = "";

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

      String errorMessage = "";

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

      String errorMessage = "";

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

   public static CenterOfMassFeedbackControlCommand nextCenterOfMassFeedbackControlCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      CenterOfMassFeedbackControlCommand next = new CenterOfMassFeedbackControlCommand();
      next.getDesiredPosition().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.getDesiredLinearVelocity().set(EuclidCoreRandomTools.nextVector3D(random));
      next.getFeedForwardLinearAction().set(EuclidCoreRandomTools.nextVector3D(random));
      next.setGains(nextPID3DGains(random));
      next.getMomentumRateCommand().set(nextMomentumRateCommand(random, rootBody, possibleFrames));
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
