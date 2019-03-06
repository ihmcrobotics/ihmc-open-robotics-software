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
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitEnforcement;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointLimitParameters;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
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
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.robotModels.JointHashCodeResolver;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
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

   public static CenterOfPressureCommand nextCenterOfPressureCommand(Random random, RigidBody rootBody, ReferenceFrame... possibleFrames)
   {
      CenterOfPressureCommand next = new CenterOfPressureCommand();
      next.setConstraintType(nextElementIn(random, ConstraintType.values()));
      next.setContactingRigidBody(nextElementIn(random, rootBody.subtreeList()));
      next.setWeight(nextFrameVector2D(random, possibleFrames));
      next.setDesiredCoP(nextFramePoint2D(random, possibleFrames));
      return next;
   }

   public static ContactWrenchCommand nextContactWrenchCommand(Random random, RigidBody rootBody, ReferenceFrame... possibleFrames)
   {
      ContactWrenchCommand next = new ContactWrenchCommand();
      next.setConstraintType(nextElementIn(random, ConstraintType.values()));
      next.setRigidBody(nextElementIn(random, rootBody.subtreeList()));
      next.getWrench().setIncludingFrame(nextWrench(random, possibleFrames));
      next.getWeightMatrix().set(nextWeightMatrix6D(random, possibleFrames));
      next.getSelectionMatrix().set(nextSelectionMatrix6D(random, possibleFrames));
      return next;
   }

   public static ExternalWrenchCommand nextExternalWrenchCommand(Random random, RigidBody rootBody, ReferenceFrame... possibleFrames)
   {
      ExternalWrenchCommand next = new ExternalWrenchCommand();
      next.setRigidBody(nextElementIn(random, rootBody.subtreeList()));
      next.getExternalWrench().setIncludingFrame(nextWrench(random, possibleFrames));
      return next;
   }

   public static InverseDynamicsOptimizationSettingsCommand nextInverseDynamicsOptimizationSettingsCommand(Random random, RigidBody rootBody,
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

   public static JointAccelerationIntegrationCommand nextJointAccelerationIntegrationCommand(Random random, RigidBody rootBody,
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

   public static JointLimitEnforcementMethodCommand nextJointLimitEnforcementMethodCommand(Random random, RigidBody rootBody, ReferenceFrame... possibleFrames)
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

   public static JointspaceAccelerationCommand nextJointspaceAccelerationCommand(Random random, RigidBody rootBody, ReferenceFrame... possibleFrames)
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

   public static MomentumRateCommand nextMomentumRateCommand(Random random, RigidBody rootBody, ReferenceFrame... possibleFrames)
   {
      MomentumRateCommand next = new MomentumRateCommand();
      next.setMomentumRate(RandomMatrices.createRandom(6, 1, random));
      next.setWeights(nextWeightMatrix6D(random, possibleFrames));
      next.setSelectionMatrix(nextSelectionMatrix6D(random, possibleFrames));
      return next;
   }

   public static PlaneContactStateCommand nextPlaneContactStateCommand(Random random, RigidBody rootBody, ReferenceFrame... possibleFrames)
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

   public static Wrench nextWrench(Random random, ReferenceFrame... possibleFrames)
   {
      return MecanoRandomTools.nextWrench(random, nextElementIn(random, possibleFrames), nextElementIn(random, possibleFrames));
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

         chainA = MultiBodySystemRandomTools.nextOneDoFJointChain(random, "chainA", rootBodyA, numberOfJoints);
         chainB = MultiBodySystemRandomTools.nextOneDoFJointChain(random, "chainB", rootBodyB, numberOfJoints);

         bodyResolverForB.put(rootBodyB, rootBodyA.hashCode());

         for (int i = 0; i < chainA.size(); i++)
         {
            jointResolverForB.put(chainB.get(i), chainA.get(i).hashCode());
            bodyResolverForB.put(chainB.get(i).getSuccessor(), chainA.get(i).getSuccessor().hashCode());
         }
      }
   }
}
