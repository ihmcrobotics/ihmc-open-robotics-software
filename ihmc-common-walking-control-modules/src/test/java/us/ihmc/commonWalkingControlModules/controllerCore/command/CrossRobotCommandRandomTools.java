package us.ihmc.commonWalkingControlModules.controllerCore.command;

import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.apache.commons.lang3.mutable.MutableInt;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.RandomMatrices;
import org.reflections.Reflections;

import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.AtlasHumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextJointData;
import us.ihmc.commonWalkingControlModules.barrierScheduler.context.HumanoidRobotContextRootJointData;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModuleInput;
import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModuleOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.CenterOfMassFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandBuffer;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.JointspaceFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.OneDoFJointFeedbackControlCommand;
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
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
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
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.footstep.SimpleAdjustableFootstep;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.tools.MecanoRandomTools;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.PIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPIDSE3Gains;
import us.ihmc.robotics.controllers.pidGains.implementations.PDGains;
import us.ihmc.robotics.kinematics.JointLimitData;
import us.ihmc.robotics.lists.DenseMatrixArrayList;
import us.ihmc.robotics.lists.FrameTupleArrayList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix3D;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.sensors.ForceSensorData;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.weightMatrices.WeightMatrix3D;
import us.ihmc.robotics.weightMatrices.WeightMatrix6D;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.model.RobotMotionStatusHolder;
import us.ihmc.sensorProcessing.outputData.ImuData;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutput;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.LowLevelState;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolder;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.sensorProcessing.simulatedSensors.SensorDataContext;

public class CrossRobotCommandRandomTools
{
   public static interface ReflectionBuilder
   {
      Object get(Class<?> clazz) throws IllegalAccessException, IllegalArgumentException, InvocationTargetException, NoSuchMethodException, SecurityException;
   }

   private static Reflections controllerCoreReflections = null;
   @SuppressWarnings("rawtypes")
   private static Set<Class<? extends InverseDynamicsCommand>> allInverseDynamicsCommands = null;
   @SuppressWarnings("rawtypes")
   private static Set<Class<? extends InverseKinematicsCommand>> allInverseKinematicsCommands = null;
   @SuppressWarnings("rawtypes")
   private static Set<Class<? extends VirtualModelControlCommand>> allVirtualModelControlCommands = null;
   @SuppressWarnings("rawtypes")
   private static Set<Class<? extends FeedbackControlCommand>> allFeedbackControlCommands = null;

   private static final Set<Class<?>> additionalClassesToTest = createAdditionalClassesToTest();

   private static Set<Class<?>> createAdditionalClassesToTest()
   {
      HashSet<Class<?>> set = new HashSet<>();
      set.add(LinearMomentumRateControlModuleInput.class);
      set.add(LinearMomentumRateControlModuleOutput.class);
      set.add(ControllerCoreCommand.class);
      set.add(ControllerCoreOutput.class);
      set.add(CenterOfPressureDataHolder.class);
      set.add(HumanoidRobotContextJointData.class);
      set.add(RobotMotionStatusHolder.class);
      set.add(LowLevelOneDoFJointDesiredDataHolder.class);
      set.add(RawJointSensorDataHolderMap.class);
      set.add(ForceSensorDataHolder.class);
      set.add(SensorDataContext.class);
      set.add(ImuData.class);
      set.add(HumanoidRobotContextData.class);
      set.add(AtlasHumanoidRobotContextData.class);
      return set;
   }

   public static Reflections getControllerCoreReflectionsInstance()
   {
      if (controllerCoreReflections == null)
      {
         controllerCoreReflections = new Reflections("us.ihmc.commonWalkingControlModules.controllerCore.command");
      }
      return controllerCoreReflections;
   }

   @SafeVarargs
   @SuppressWarnings("rawtypes")
   public static Set<Class<? extends InverseDynamicsCommand>> getInverseDynamicsCommandTypes(Class<? extends InverseDynamicsCommand>... commandTypesToIgnore)
   {
      if (allInverseDynamicsCommands == null)
         allInverseDynamicsCommands = getControllerCoreReflectionsInstance().getSubTypesOf(InverseDynamicsCommand.class);

      Set<Class<? extends InverseDynamicsCommand>> commandTypes = new HashSet<>(allInverseDynamicsCommands);

      if (commandTypesToIgnore != null)
      {
         for (Class<? extends InverseDynamicsCommand> commandTypeToIgnore : commandTypesToIgnore)
            commandTypes.remove(commandTypeToIgnore);
      }
      return commandTypes;
   }

   @SafeVarargs
   @SuppressWarnings("rawtypes")
   public static Set<Class<? extends InverseKinematicsCommand>> getInverseKinematicsCommandTypes(Class<? extends InverseKinematicsCommand>... commandTypesToIgnore)
   {
      if (allInverseKinematicsCommands == null)
         allInverseKinematicsCommands = getControllerCoreReflectionsInstance().getSubTypesOf(InverseKinematicsCommand.class);

      Set<Class<? extends InverseKinematicsCommand>> commandTypes = new HashSet<>(allInverseKinematicsCommands);

      if (commandTypesToIgnore != null)
      {
         for (Class<? extends InverseKinematicsCommand> commandTypeToIgnore : commandTypesToIgnore)
            commandTypes.remove(commandTypeToIgnore);
      }
      return commandTypes;
   }

   @SafeVarargs
   @SuppressWarnings("rawtypes")
   public static Set<Class<? extends VirtualModelControlCommand>> getVirtualModelControlCommandTypes(Class<? extends VirtualModelControlCommand>... commandTypesToIgnore)
   {
      if (allVirtualModelControlCommands == null)
         allVirtualModelControlCommands = getControllerCoreReflectionsInstance().getSubTypesOf(VirtualModelControlCommand.class);

      Set<Class<? extends VirtualModelControlCommand>> commandTypes = new HashSet<>(allVirtualModelControlCommands);

      if (commandTypesToIgnore != null)
      {
         for (Class<? extends VirtualModelControlCommand> commandTypeToIgnore : commandTypesToIgnore)
            commandTypes.remove(commandTypeToIgnore);
      }
      return commandTypes;
   }

   @SafeVarargs
   @SuppressWarnings("rawtypes")
   public static Set<Class<? extends FeedbackControlCommand>> getFeedbackControlCommandTypes(Class<? extends FeedbackControlCommand>... commandTypesToIgnore)
   {
      if (allFeedbackControlCommands == null)
         allFeedbackControlCommands = getControllerCoreReflectionsInstance().getSubTypesOf(FeedbackControlCommand.class);

      Set<Class<? extends FeedbackControlCommand>> commandTypes = new HashSet<>(allFeedbackControlCommands);

      if (commandTypesToIgnore != null)
      {
         for (Class<? extends FeedbackControlCommand> commandTypeToIgnore : commandTypesToIgnore)
            commandTypes.remove(commandTypeToIgnore);
      }
      return commandTypes;
   }

   public static Set<Class<?>> getAdditionalClassesToTest()
   {
      return additionalClassesToTest;
   }

   public static Set<Class<?>> getAllCommandTypesWithoutBuffersAndInterfaces()
   {
      Set<Class<?>> allCommandTypes = new HashSet<>();
      allCommandTypes.addAll(getInverseDynamicsCommandTypes(InverseDynamicsCommandBuffer.class));
      allCommandTypes.addAll(getInverseKinematicsCommandTypes(InverseKinematicsCommandBuffer.class));
      allCommandTypes.addAll(getVirtualModelControlCommandTypes(VirtualModelControlCommandBuffer.class, VirtualEffortCommand.class));
      allCommandTypes.addAll(getFeedbackControlCommandTypes(FeedbackControlCommandBuffer.class));
      allCommandTypes.addAll(getAdditionalClassesToTest());
      return allCommandTypes;
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

   public static String nextString(Random random)
   {
      return Long.toString(random.nextLong());
   }

   public static Point2D nextPoint2D(Random random)
   {
      return EuclidCoreRandomTools.nextPoint2D(random);
   }

   public static Vector2D nextVector2D(Random random)
   {
      return EuclidCoreRandomTools.nextVector2D(random);
   }

   public static Point3D nextPoint3D(Random random)
   {
      return EuclidCoreRandomTools.nextPoint3D(random);
   }

   public static Vector3D nextVector3D(Random random)
   {
      return EuclidCoreRandomTools.nextVector3D(random);
   }

   public static Quaternion nextQuaternion(Random random)
   {
      return EuclidCoreRandomTools.nextQuaternion(random);
   }

   public static RigidBodyTransform nextRigidBodyTransform(Random random)
   {
      return EuclidCoreRandomTools.nextRigidBodyTransform(random);
   }

   public static FrameTupleArrayList<FramePoint3D> nextFrameTupleArrayList(Random random, int size, ReferenceFrame... possibleFrames)
   {
      FrameTupleArrayList<FramePoint3D> next = FrameTupleArrayList.createFramePointArrayList();
      while (next.size() < size)
         next.add().setIncludingFrame(nextFramePoint3D(random, possibleFrames));
      return next;
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

   public static DenseMatrix64F nextDenseMatrix64F(Random random)
   {
      return RandomMatrices.createRandom(random.nextInt(10), random.nextInt(10), random);
   }

   public static DenseMatrix64F nextDenseMatrix64F(Random random, int numRow, int numCol)
   {
      return RandomMatrices.createRandom(numRow, numCol, random);
   }

   public static DenseMatrixArrayList nextDenseMatrixArrayList(Random random, int numRow, int numCol, int size)
   {
      DenseMatrixArrayList next = new DenseMatrixArrayList();
      while (next.size() < size)
         next.add().set(nextDenseMatrix64F(random, numRow, numCol));
      return next;
   }

   public static TDoubleArrayList nextTDoubleArrayList(Random random, int size)
   {
      TDoubleArrayList next = new TDoubleArrayList();
      while (next.size() < size)
         next.add(random.nextDouble());
      return next;
   }

   public static <T> RecyclingArrayList<T> nextRecyclingArrayList(Class<T> elementType, int size, Random random, boolean ensureNonEmptyCommand,
                                                                  RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws Exception
   {
      RecyclingArrayList<T> next = new RecyclingArrayList<>(elementType);
      while (next.size() < size)
         next.add();
      randomizeRecyclingArrayList(next, (listElementType) -> nextTypeInstance(listElementType, random, true, rootBody, possibleFrames));
      return next;
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
      return nextDefaultPID3DGains(random);
   }

   public static DefaultPID3DGains nextDefaultPID3DGains(Random random)
   {
      DefaultPID3DGains next = new DefaultPID3DGains();

      next.setProportionalGains(random.nextDouble(), random.nextDouble(), random.nextDouble());
      next.setDerivativeGains(random.nextDouble(), random.nextDouble(), random.nextDouble());
      next.setIntegralGains(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
      next.setMaxProportionalError(random.nextDouble());
      next.setMaxDerivativeError(random.nextDouble());
      next.setMaxFeedbackAndFeedbackRate(random.nextDouble(), random.nextDouble());

      return next;
   }

   public static PIDSE3Gains nextPIDSE3Gains(Random random)
   {
      return nextDefaultPIDSE3Gains(random);
   }

   public static DefaultPIDSE3Gains nextDefaultPIDSE3Gains(Random random)
   {
      DefaultPIDSE3Gains next = new DefaultPIDSE3Gains();
      next.setPositionGains(nextDefaultPID3DGains(random));
      next.setOrientationGains(nextDefaultPID3DGains(random));
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
      return nextJointAccelerationIntegrationCommand(random, false, rootBody, possibleFrames);
   }

   public static JointAccelerationIntegrationCommand nextJointAccelerationIntegrationCommand(Random random, boolean ensureNonEmptyCommand,
                                                                                             RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      JointAccelerationIntegrationCommand next = new JointAccelerationIntegrationCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());
      if (ensureNonEmptyCommand)
         numberOfJoints = Math.max(numberOfJoints, 1);

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
      return nextJointLimitEnforcementMethodCommand(random, false, rootBody, possibleFrames);
   }

   public static JointLimitEnforcementMethodCommand nextJointLimitEnforcementMethodCommand(Random random, boolean ensureNonEmptyCommand,
                                                                                           RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      JointLimitEnforcementMethodCommand next = new JointLimitEnforcementMethodCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());
      if (ensureNonEmptyCommand)
         numberOfJoints = Math.max(numberOfJoints, 1);

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         next.addLimitEnforcementMethod(allJoints.remove(random.nextInt(allJoints.size())), nextElementIn(random, JointLimitEnforcement.values()),
                                        nextJointLimitParameters(random));
      }

      return next;
   }

   public static JointspaceAccelerationCommand nextJointspaceAccelerationCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      return nextJointspaceAccelerationCommand(random, false, rootBody, possibleFrames);
   }

   public static JointspaceAccelerationCommand nextJointspaceAccelerationCommand(Random random, boolean ensureNonEmptyCommand, RigidBodyBasics rootBody,
                                                                                 ReferenceFrame... possibleFrames)
   {
      JointspaceAccelerationCommand next = new JointspaceAccelerationCommand();

      List<JointBasics> allJoints = SubtreeStreams.fromChildren(rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());
      if (ensureNonEmptyCommand)
         numberOfJoints = Math.max(numberOfJoints, 1);

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
      return nextPlaneContactStateCommand(random, false, rootBody, possibleFrames);
   }

   public static PlaneContactStateCommand nextPlaneContactStateCommand(Random random, boolean ensureNonEmptyCommand, RigidBodyBasics rootBody,
                                                                       ReferenceFrame... possibleFrames)
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
      if (ensureNonEmptyCommand)
         numberOfContactPoints = Math.max(numberOfContactPoints, 1);

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
      return nextJointLimitReductionCommand(random, false, rootBody, possibleFrames);
   }

   public static JointLimitReductionCommand nextJointLimitReductionCommand(Random random, boolean ensureNonEmptyCommand, RigidBodyBasics rootBody,
                                                                           ReferenceFrame... possibleFrames)
   {
      JointLimitReductionCommand next = new JointLimitReductionCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());
      if (ensureNonEmptyCommand)
         numberOfJoints = Math.max(numberOfJoints, 1);

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.addReductionFactor(joint, random.nextDouble());
      }

      return next;
   }

   public static JointspaceVelocityCommand nextJointspaceVelocityCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      return nextJointspaceVelocityCommand(random, false, rootBody, possibleFrames);
   }

   public static JointspaceVelocityCommand nextJointspaceVelocityCommand(Random random, boolean ensureNonEmptyCommand, RigidBodyBasics rootBody,
                                                                         ReferenceFrame... possibleFrames)
   {
      JointspaceVelocityCommand next = new JointspaceVelocityCommand();

      List<JointBasics> allJoints = SubtreeStreams.fromChildren(rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());
      if (ensureNonEmptyCommand)
         numberOfJoints = Math.max(numberOfJoints, 1);

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
      return nextPrivilegedConfigurationCommand(random, false, rootBody, possibleFrames);
   }

   public static PrivilegedConfigurationCommand nextPrivilegedConfigurationCommand(Random random, boolean ensureNonEmptyCommand, RigidBodyBasics rootBody,
                                                                                   ReferenceFrame... possibleFrames)
   {
      PrivilegedConfigurationCommand next = new PrivilegedConfigurationCommand();

      if (random.nextBoolean())
         next.setDefaultParameters(nextOneDoFJointPrivilegedConfigurationParameters(random));

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());
      if (ensureNonEmptyCommand)
         numberOfJoints = Math.max(numberOfJoints, 1);

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
      return nextPrivilegedJointSpaceCommand(random, false, rootBody, possibleFrames);
   }

   public static PrivilegedJointSpaceCommand nextPrivilegedJointSpaceCommand(Random random, boolean ensureNonEmptyCommand, RigidBodyBasics rootBody,
                                                                             ReferenceFrame... possibleFrames)
   {
      PrivilegedJointSpaceCommand next = new PrivilegedJointSpaceCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());
      if (ensureNonEmptyCommand)
         numberOfJoints = Math.max(numberOfJoints, 1);

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

      if (random.nextBoolean())
      { // Setup as objective
         next.setWeightMatrix(nextWeightMatrix6D(random, possibleFrames));
      }
      else
      {
         switch (random.nextInt(3))
         {
         case 0:
            next.setAsGreaterOrEqualInequalityConstraint();
            break;
         case 1:
            next.setAsLessOrEqualInequalityConstraint();
            break;
         case 2:
         default:
            next.setAsHardEqualityConstraint();
            break;
         }
      }

      next.setSelectionMatrix(nextSelectionMatrix6D(random, possibleFrames));
      if (random.nextBoolean())
         next.setPrimaryBase(nextElementIn(random, rootBody.subtreeList()));
      if (random.nextBoolean())
         next.setScaleSecondaryTaskJointWeight(true, random.nextDouble());
      return next;
   }

   public static JointLimitEnforcementCommand nextJointLimitEnforcementCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      return nextJointLimitEnforcementCommand(random, false, rootBody, possibleFrames);
   }

   public static JointLimitEnforcementCommand nextJointLimitEnforcementCommand(Random random, boolean ensureNonEmptyCommand, RigidBodyBasics rootBody,
                                                                               ReferenceFrame... possibleFrames)
   {
      JointLimitEnforcementCommand next = new JointLimitEnforcementCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());
      if (ensureNonEmptyCommand)
         numberOfJoints = Math.max(numberOfJoints, 1);

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.addJoint(joint, nextJointLimitData(random));
      }

      return next;
   }

   public static JointTorqueCommand nextJointTorqueCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      return nextJointTorqueCommand(random, false, rootBody, possibleFrames);
   }

   public static JointTorqueCommand nextJointTorqueCommand(Random random, boolean ensureNonEmptyCommand, RigidBodyBasics rootBody,
                                                           ReferenceFrame... possibleFrames)
   {
      JointTorqueCommand next = new JointTorqueCommand();

      List<JointBasics> allJoints = SubtreeStreams.fromChildren(rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());
      if (ensureNonEmptyCommand)
         numberOfJoints = Math.max(numberOfJoints, 1);

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
      next.setControlMode(nextElementIn(random, WholeBodyControllerCoreMode.values()));
      next.getReferencePosition().set(EuclidCoreRandomTools.nextPoint3D(random));
      next.getReferenceLinearVelocity().set(EuclidCoreRandomTools.nextVector3D(random));
      next.getReferenceLinearAcceleration().set(EuclidCoreRandomTools.nextVector3D(random));
      next.setGains(nextDefaultPID3DGains(random));
      next.getMomentumRateCommand().set(nextMomentumRateCommand(random, rootBody, possibleFrames));
      return next;
   }

   public static OneDoFJointFeedbackControlCommand nextOneDoFJointFeedbackControlCommand(Random random, RigidBodyBasics rootBody,
                                                                                         ReferenceFrame... possibleFrames)
   {
      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      OneDoFJointBasics joint = allJoints.get(random.nextInt(allJoints.size()));
      return nextOneDoFJointFeedbackControlCommand(random, joint);
   }

   public static OneDoFJointFeedbackControlCommand nextOneDoFJointFeedbackControlCommand(Random random, OneDoFJointBasics joint)
   {
      OneDoFJointFeedbackControlCommand next = new OneDoFJointFeedbackControlCommand();
      next.setJoint(joint);
      next.setControlMode(nextElementIn(random, WholeBodyControllerCoreMode.values()));
      next.setReferencePosition(random.nextDouble());
      next.setReferenceVelocity(random.nextDouble());
      next.setReferenceAcceleration(random.nextDouble());
      next.setReferenceEffort(random.nextDouble());
      next.setGains(nextPDGains(random));
      next.setWeightForSolver(random.nextDouble());
      return next;
   }

   public static JointspaceFeedbackControlCommand nextJointspaceFeedbackControlCommand(Random random, RigidBodyBasics rootBody,
                                                                                       ReferenceFrame... possibleFrames)
   {
      return nextJointspaceFeedbackControlCommand(random, false, rootBody, possibleFrames);
   }

   public static JointspaceFeedbackControlCommand nextJointspaceFeedbackControlCommand(Random random, boolean ensureNonEmptyCommand, RigidBodyBasics rootBody,
                                                                                       ReferenceFrame... possibleFrames)
   {
      JointspaceFeedbackControlCommand next = new JointspaceFeedbackControlCommand();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());
      if (ensureNonEmptyCommand)
         numberOfJoints = Math.max(numberOfJoints, 1);

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.addCommand(nextOneDoFJointFeedbackControlCommand(random, joint));
      }

      return next;
   }

   public static OrientationFeedbackControlCommand nextOrientationFeedbackControlCommand(Random random, RigidBodyBasics rootBody,
                                                                                         ReferenceFrame... possibleFrames)
   {
      OrientationFeedbackControlCommand next = new OrientationFeedbackControlCommand();
      next.setControlMode(nextElementIn(random, WholeBodyControllerCoreMode.values()));
      next.getBodyFixedOrientationToControl().setIncludingFrame(nextFrameQuaternion(random, possibleFrames));
      next.getReferenceOrientation().setIncludingFrame(nextFrameQuaternion(random, possibleFrames));
      next.getReferenceAngularVelocity().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getReferenceAngularAcceleration().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getReferenceTorque().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getGains().set(nextDefaultPID3DGains(random));
      next.setGainsFrame(nextElementIn(random, possibleFrames));
      next.getSpatialAccelerationCommand().set(nextSpatialAccelerationCommand(random, rootBody, possibleFrames));
      next.setControlBaseFrame(nextElementIn(random, possibleFrames));
      return next;
   }

   public static PointFeedbackControlCommand nextPointFeedbackControlCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      PointFeedbackControlCommand next = new PointFeedbackControlCommand();
      next.setControlMode(nextElementIn(random, WholeBodyControllerCoreMode.values()));
      next.getBodyFixedPointToControl().setIncludingFrame(nextFramePoint3D(random, possibleFrames));
      next.getReferencePosition().setIncludingFrame(nextFramePoint3D(random, possibleFrames));
      next.getReferenceLinearVelocity().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getReferenceLinearAcceleration().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getReferenceForce().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getGains().set(nextDefaultPID3DGains(random));
      next.setGainsFrame(nextElementIn(random, possibleFrames));
      next.getSpatialAccelerationCommand().set(nextSpatialAccelerationCommand(random, rootBody, possibleFrames));
      next.setControlBaseFrame(nextElementIn(random, possibleFrames));
      return next;
   }

   public static SpatialFeedbackControlCommand nextSpatialFeedbackControlCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      SpatialFeedbackControlCommand next = new SpatialFeedbackControlCommand();
      next.setControlMode(nextElementIn(random, WholeBodyControllerCoreMode.values()));
      next.getControlFramePose().setIncludingFrame(nextFramePose3D(random, possibleFrames));
      next.getReferenceOrientation().setIncludingFrame(nextFrameQuaternion(random, possibleFrames));
      next.getReferenceAngularVelocity().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getReferenceAngularAcceleration().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getReferenceTorque().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getGains().set(nextDefaultPIDSE3Gains(random));
      next.setGainsFrames(nextElementIn(random, possibleFrames), nextElementIn(random, possibleFrames));
      next.getSpatialAccelerationCommand().set(nextSpatialAccelerationCommand(random, rootBody, possibleFrames));
      next.setControlBaseFrame(nextElementIn(random, possibleFrames));
      next.getReferencePosition().setIncludingFrame(nextFramePoint3D(random, possibleFrames));
      next.getReferenceLinearVelocity().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getReferenceLinearAcceleration().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getReferenceForce().setIncludingFrame(nextFrameVector3D(random, possibleFrames));
      next.getSpatialAccelerationCommand().set(nextSpatialAccelerationCommand(random, rootBody, possibleFrames));
      next.setControlBaseFrame(nextElementIn(random, possibleFrames));
      return next;
   }

   public static InverseDynamicsCommandList nextInverseDynamicsCommandList(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      return nextInverseDynamicsCommandList(random, getInverseDynamicsCommandTypes(InverseDynamicsCommandList.class, InverseDynamicsCommandBuffer.class),
                                            rootBody, possibleFrames);
   }

   @SuppressWarnings("rawtypes")
   public static InverseDynamicsCommandList nextInverseDynamicsCommandList(Random random,
                                                                           Collection<Class<? extends InverseDynamicsCommand>> commandsToGenerate,
                                                                           RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      InverseDynamicsCommandList next = new InverseDynamicsCommandList();

      List<Class<? extends InverseDynamicsCommand>> commandTypes = new ArrayList<>(commandsToGenerate);
      commandTypes.removeIf(Class::isInterface);
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

         Method randomGenerator = CrossRobotCommandRandomTools.class.getDeclaredMethod("next" + commandType.getSimpleName(), Random.class,
                                                                                           RigidBodyBasics.class, ReferenceFrame[].class);
         InverseDynamicsCommand<?> command = (InverseDynamicsCommand<?>) randomGenerator.invoke(null, random, rootBody, possibleFrames);
         next.addCommand(command);
      }
      return next;
   }

   public static InverseKinematicsCommandList nextInverseKinematicsCommandList(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      return nextInverseKinematicsCommandList(random,
                                              getInverseKinematicsCommandTypes(InverseKinematicsCommandList.class, InverseKinematicsCommandBuffer.class),
                                              rootBody, possibleFrames);
   }

   @SuppressWarnings("rawtypes")
   public static InverseKinematicsCommandList nextInverseKinematicsCommandList(Random random,
                                                                               Collection<Class<? extends InverseKinematicsCommand>> commandsToGenerate,
                                                                               RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      InverseKinematicsCommandList next = new InverseKinematicsCommandList();

      List<Class<? extends InverseKinematicsCommand>> commandTypes = new ArrayList<>(commandsToGenerate);
      commandTypes.removeIf(Class::isInterface);
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

         Method randomGenerator = CrossRobotCommandRandomTools.class.getDeclaredMethod("next" + commandType.getSimpleName(), Random.class,
                                                                                           RigidBodyBasics.class, ReferenceFrame[].class);
         InverseKinematicsCommand<?> command = (InverseKinematicsCommand<?>) randomGenerator.invoke(null, random, rootBody, possibleFrames);
         next.addCommand(command);
      }
      return next;
   }

   public static VirtualModelControlCommandList nextVirtualModelControlCommandList(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      return nextVirtualModelControlCommandList(random, getVirtualModelControlCommandTypes(VirtualModelControlCommandList.class,
                                                                                           VirtualModelControlCommandBuffer.class),
                                                rootBody, possibleFrames);
   }

   @SuppressWarnings("rawtypes")
   public static VirtualModelControlCommandList nextVirtualModelControlCommandList(Random random,
                                                                                   Collection<Class<? extends VirtualModelControlCommand>> commandsToGenerate,
                                                                                   RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      VirtualModelControlCommandList next = new VirtualModelControlCommandList();

      List<Class<? extends VirtualModelControlCommand>> commandTypes = new ArrayList<>(commandsToGenerate);
      commandTypes.removeIf(Class::isInterface);
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

         Method randomGenerator = CrossRobotCommandRandomTools.class.getDeclaredMethod("next" + commandType.getSimpleName(), Random.class,
                                                                                           RigidBodyBasics.class, ReferenceFrame[].class);
         VirtualModelControlCommand<?> command = (VirtualModelControlCommand<?>) randomGenerator.invoke(null, random, rootBody, possibleFrames);
         next.addCommand(command);
      }
      return next;
   }

   public static FeedbackControlCommandList nextFeedbackControlCommandList(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      return nextFeedbackControlCommandList(random, getFeedbackControlCommandTypes(FeedbackControlCommandList.class, FeedbackControlCommandBuffer.class),
                                            rootBody, possibleFrames);
   }

   @SuppressWarnings("rawtypes")
   public static FeedbackControlCommandList nextFeedbackControlCommandList(Random random,
                                                                           Collection<Class<? extends FeedbackControlCommand>> commandsToGenerate,
                                                                           RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      FeedbackControlCommandList next = new FeedbackControlCommandList();

      List<Class<? extends FeedbackControlCommand>> commandTypes = new ArrayList<>(commandsToGenerate);
      commandTypes.removeIf(Class::isInterface);
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

         Method randomGenerator = CrossRobotCommandRandomTools.class.getDeclaredMethod("next" + commandType.getSimpleName(), Random.class,
                                                                                           RigidBodyBasics.class, ReferenceFrame[].class);
         FeedbackControlCommand<?> command = (FeedbackControlCommand<?>) randomGenerator.invoke(null, random, rootBody, possibleFrames);
         next.addCommand(command);
      }
      return next;
   }

   public static JointDesiredOutputListBasics nextJointDesiredOutputListBasics(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      return nextLowLevelOneDoFJointDesiredDataHolder(random, false, rootBody, possibleFrames);
   }

   public static JointDesiredOutputListBasics nextJointDesiredOutputListBasics(Random random, boolean ensureNonEmptyCommand, RigidBodyBasics rootBody,
                                                                               ReferenceFrame... possibleFrames)
   {
      return nextLowLevelOneDoFJointDesiredDataHolder(random, ensureNonEmptyCommand, rootBody, possibleFrames);
   }

   public static LowLevelOneDoFJointDesiredDataHolder nextLowLevelOneDoFJointDesiredDataHolder(Random random, RigidBodyBasics rootBody,
                                                                                               ReferenceFrame... possibleFrames)
   {
      return nextLowLevelOneDoFJointDesiredDataHolder(random, false, rootBody, possibleFrames);
   }

   public static LowLevelOneDoFJointDesiredDataHolder nextLowLevelOneDoFJointDesiredDataHolder(Random random, boolean ensureNonEmptyCommand,
                                                                                               RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      LowLevelOneDoFJointDesiredDataHolder next = new LowLevelOneDoFJointDesiredDataHolder();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());
      if (ensureNonEmptyCommand)
         numberOfJoints = Math.max(numberOfJoints, 1);

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.registerJointWithEmptyData(joint).set(nextJointDesiredOutput(random));
      }

      return next;
   }

   public static CenterOfPressureDataHolder nextCenterOfPressureDataHolder(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      return nextCenterOfPressureDataHolder(random, false, rootBody, possibleFrames);
   }

   public static CenterOfPressureDataHolder nextCenterOfPressureDataHolder(Random random, boolean ensureNonEmptyCommand, RigidBodyBasics rootBody,
                                                                           ReferenceFrame... possibleFrames)
   {
      CenterOfPressureDataHolder next = new CenterOfPressureDataHolder();

      List<RigidBodyBasics> allBodies = SubtreeStreams.from(rootBody).collect(Collectors.toList());
      int numberOfBodies = random.nextInt(allBodies.size());
      if (ensureNonEmptyCommand)
         numberOfBodies = Math.max(numberOfBodies, 1);

      for (int bodyIndex = 0; bodyIndex < numberOfBodies; bodyIndex++)
      {
         RigidBodyBasics rigidBody = allBodies.remove(random.nextInt(allBodies.size()));
         next.registerRigidBody(rigidBody, nextFramePoint2D(random, possibleFrames));
      }

      return next;
   }

   public static HumanoidRobotContextJointData nextHumanoidRobotContextJointData(Random random)
   {
      return nextHumanoidRobotContextJointData(random, false);
   }

   public static HumanoidRobotContextJointData nextHumanoidRobotContextJointData(Random random, boolean ensureNonEmptyCommand)
   {
      HumanoidRobotContextJointData next = new HumanoidRobotContextJointData();
      int numberOfJoints = random.nextInt(50);
      if (ensureNonEmptyCommand)
         numberOfJoints = Math.max(numberOfJoints, 1);
      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         next.addJoint(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble());
      }
      next.setRootJointData(nextHumanoidRobotContextRootJointData(random));
      return next;
   }

   public static HumanoidRobotContextRootJointData nextHumanoidRobotContextRootJointData(Random random)
   {
      HumanoidRobotContextRootJointData next = new HumanoidRobotContextRootJointData();
      next.setRootJointOrientation(EuclidCoreRandomTools.nextQuaternion(random));
      next.setRootJointAngularVelocity(EuclidCoreRandomTools.nextVector3D(random));
      next.setRootJointAngularAcceleration(EuclidCoreRandomTools.nextVector3D(random));
      next.setRootJointLocation(EuclidCoreRandomTools.nextPoint3D(random));
      next.setRootJointLinearVelocity(EuclidCoreRandomTools.nextVector3D(random));
      next.setRootJointLinearAcceleration(EuclidCoreRandomTools.nextVector3D(random));
      return next;
   }

   public static RobotMotionStatusHolder nextRobotMotionStatusHolder(Random random)
   {
      RobotMotionStatusHolder next = new RobotMotionStatusHolder();
      next.setCurrentRobotMotionStatus(nextElementIn(random, RobotMotionStatus.values));
      return next;
   }

   public static RawJointSensorDataHolderMap nextRawJointSensorDataHolderMap(Random random, RigidBodyBasics rootBody)
   {
      return nextRawJointSensorDataHolderMap(random, false, rootBody);
   }

   public static RawJointSensorDataHolderMap nextRawJointSensorDataHolderMap(Random random, boolean ensureNonEmptyCommand, RigidBodyBasics rootBody)
   {
      RawJointSensorDataHolderMap next = new RawJointSensorDataHolderMap();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());
      if (ensureNonEmptyCommand)
         numberOfJoints = Math.max(numberOfJoints, 1);

      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.registerJoint(joint);
      }

      return next;
   }

   public static RawJointSensorDataHolder nextRawJointSensorDataHolder(Random random)
   {
      RawJointSensorDataHolder next = new RawJointSensorDataHolder();
      next.setIsEnabled(random.nextBoolean());
      next.setQ_raw(random.nextDouble());
      next.setQ_out_raw(random.nextDouble());
      next.setQd_out_raw(random.nextDouble());
      next.setQd_raw(random.nextDouble());
      next.setF_raw(random.nextDouble());
      next.setPsi_neg_raw(random.nextDouble());
      next.setPsi_pos_raw(random.nextDouble());
      next.setUsesOutputEncoderQ(random.nextBoolean());
      next.setUsesOutputEncoderQd(random.nextBoolean());
      next.setMotorCurrent(random.nextDouble());
      next.setCommandedMotorCurrent(random.nextDouble());
      next.setTemperature(random.nextDouble());
      next.setMotorAngle(0, random.nextDouble());
      next.setMotorAngle(1, random.nextDouble());
      return next;
   }

   public static HumanoidRobotContextData nextHumanoidRobotContextData(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      return nextHumanoidRobotContextData(random, false, rootBody, possibleFrames);
   }

   public static HumanoidRobotContextData nextHumanoidRobotContextData(Random random, boolean ensureNonEmptyCommand, RigidBodyBasics rootBody,
                                                                       ReferenceFrame... possibleFrames)
   {
      HumanoidRobotContextJointData processedJointData = nextHumanoidRobotContextJointData(random, ensureNonEmptyCommand);
      ForceSensorDataHolder forceSensorDataHolder = nextForceSensorDataHolder(random, ensureNonEmptyCommand, rootBody, possibleFrames);
      CenterOfPressureDataHolder centerOfPressureDataHolder = nextCenterOfPressureDataHolder(random, ensureNonEmptyCommand, rootBody, possibleFrames);
      RobotMotionStatusHolder robotMotionStatusHolder = nextRobotMotionStatusHolder(random);
      LowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList = nextLowLevelOneDoFJointDesiredDataHolder(random, ensureNonEmptyCommand, rootBody,
                                                                                                             possibleFrames);
      SensorDataContext sensorDataContext = nextSensorDataContext(random, ensureNonEmptyCommand, rootBody);
      HumanoidRobotContextData next = new HumanoidRobotContextData(processedJointData, forceSensorDataHolder, centerOfPressureDataHolder,
                                                                   robotMotionStatusHolder, jointDesiredOutputList, sensorDataContext);
      next.setTimestamp(random.nextLong());
      next.setControllerRan(random.nextBoolean());
      next.setEstimatorRan(random.nextBoolean());
      return next;
   }

   public static AtlasHumanoidRobotContextData nextAtlasHumanoidRobotContextData(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      return nextAtlasHumanoidRobotContextData(random, false, rootBody, possibleFrames);
   }

   public static AtlasHumanoidRobotContextData nextAtlasHumanoidRobotContextData(Random random, boolean ensureNonEmptyCommand, RigidBodyBasics rootBody,
                                                                                 ReferenceFrame... possibleFrames)
   {
      HumanoidRobotContextJointData processedJointData = nextHumanoidRobotContextJointData(random, ensureNonEmptyCommand);
      ForceSensorDataHolder forceSensorDataHolder = nextForceSensorDataHolder(random, ensureNonEmptyCommand, rootBody, possibleFrames);
      CenterOfPressureDataHolder centerOfPressureDataHolder = nextCenterOfPressureDataHolder(random, ensureNonEmptyCommand, rootBody, possibleFrames);
      RobotMotionStatusHolder robotMotionStatusHolder = nextRobotMotionStatusHolder(random);
      LowLevelOneDoFJointDesiredDataHolder jointDesiredOutputList = nextLowLevelOneDoFJointDesiredDataHolder(random, ensureNonEmptyCommand, rootBody,
                                                                                                             possibleFrames);
      SensorDataContext sensorDataContext = nextSensorDataContext(random, ensureNonEmptyCommand, rootBody);
      RawJointSensorDataHolderMap rawJointSensorDataHolderMap = nextRawJointSensorDataHolderMap(random, ensureNonEmptyCommand, rootBody);
      AtlasHumanoidRobotContextData next = new AtlasHumanoidRobotContextData(processedJointData, forceSensorDataHolder, centerOfPressureDataHolder,
                                                                             robotMotionStatusHolder, jointDesiredOutputList, sensorDataContext,
                                                                             rawJointSensorDataHolderMap);
      next.setTimestamp(random.nextLong());
      next.setControllerRan(random.nextBoolean());
      next.setEstimatorRan(random.nextBoolean());
      return next;
   }

   public static SensorDataContext nextSensorDataContext(Random random, RigidBodyBasics rootBody)
   {
      return nextSensorDataContext(random, false, rootBody);
   }

   public static SensorDataContext nextSensorDataContext(Random random, boolean ensureNonEmptyCommand, RigidBodyBasics rootBody)
   {
      SensorDataContext next = new SensorDataContext();

      List<OneDoFJointBasics> allJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList());
      int numberOfJoints = random.nextInt(allJoints.size());
      if (ensureNonEmptyCommand)
         numberOfJoints = Math.max(numberOfJoints, 1);
      for (int jointIndex = 0; jointIndex < numberOfJoints; jointIndex++)
      {
         OneDoFJointBasics joint = allJoints.remove(random.nextInt(allJoints.size()));
         next.registerJoint(joint.getName()).set(nextLowLevelState(random));
      }

      int numberOfImus = random.nextInt(10);
      if (ensureNonEmptyCommand)
         numberOfImus = Math.max(numberOfImus, 1);
      for (int imuIndex = 0; imuIndex < numberOfImus; imuIndex++)
      {
         next.registerImu("imu" + imuIndex).set(nextImuData(random));
      }

      int numberOfForceSensors = random.nextInt(10);
      if (ensureNonEmptyCommand)
         numberOfForceSensors = Math.max(numberOfForceSensors, 1);
      for (int forceSensorIndex = 0; forceSensorIndex < numberOfForceSensors; forceSensorIndex++)
      {
         next.registerForceSensor("ForceSensor" + forceSensorIndex).set(nextDenseMatrix64F(random, 6, 1));
      }

      return next;
   }

   public static ImuData nextImuData(Random random)
   {
      ImuData next = new ImuData();
      next.setAngularVelocity(nextVector3D(random));
      next.setLinearAcceleration(nextVector3D(random));
      next.setOrientation(nextQuaternion(random));
      return next;
   }

   public static LowLevelState nextLowLevelState(Random random)
   {
      LowLevelState next = new LowLevelState();
      next.setPosition(random.nextDouble());
      next.setVelocity(random.nextDouble());
      next.setAcceleration(random.nextDouble());
      next.setEffort(random.nextDouble());
      next.setPositionValid(random.nextBoolean());
      next.setVelocityValid(random.nextBoolean());
      next.setAccelerationValid(random.nextBoolean());
      next.setEffortValid(random.nextBoolean());
      return next;
   }

   public static ForceSensorDataHolder nextForceSensorDataHolder(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      return nextForceSensorDataHolder(random, false, rootBody, possibleFrames);
   }

   public static ForceSensorDataHolder nextForceSensorDataHolder(Random random, boolean ensureNonEmptyCommand, RigidBodyBasics rootBody,
                                                                 ReferenceFrame... possibleFrames)
   {
      ForceSensorDataHolder next = new ForceSensorDataHolder();
      int numberOfSensors = random.nextInt(20);
      if (ensureNonEmptyCommand)
         numberOfSensors = Math.max(numberOfSensors, 1);
      for (int sensorIndex = 0; sensorIndex < numberOfSensors; sensorIndex++)
      {
         next.registerForceSensor(nextForceSensorDefinition(random, rootBody, possibleFrames));
      }
      return next;
   }

   public static ForceSensorDefinition nextForceSensorDefinition(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      ForceSensorDefinition next = new ForceSensorDefinition();
      List<RigidBodyBasics> allBodies = SubtreeStreams.from(rootBody).collect(Collectors.toList());
      next.set("Sensor" + random.nextLong(), nextElementIn(random, allBodies), nextElementIn(random, possibleFrames));
      return next;
   }

   public static ForceSensorData nextForceSensorData(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      ForceSensorData next = new ForceSensorData();
      List<RigidBodyBasics> allBodies = SubtreeStreams.from(rootBody).collect(Collectors.toList());
      next.setFrameAndBody(nextElementIn(random, possibleFrames), nextElementIn(random, allBodies));
      next.setWrench(nextDenseMatrix64F(random, 6, 1));
      return next;
   }

   public static RootJointDesiredConfigurationData nextRootJointDesiredConfigurationData(Random random, ReferenceFrame... possibleFrames)
   {
      RootJointDesiredConfigurationData next = new RootJointDesiredConfigurationData();
      next.setDesiredConfiguration(nextFrameQuaternion(random, possibleFrames), nextFramePoint3D(random, possibleFrames));
      next.setDesiredVelocity(nextFrameVector3D(random, possibleFrames), nextFrameVector3D(random, possibleFrames));
      next.setDesiredAcceleration(nextFrameVector3D(random, possibleFrames), nextFrameVector3D(random, possibleFrames));
      return next;
   }

   public static ControllerCoreOutput nextControllerCoreOutput(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
   {
      ControllerCoreOutput next = new ControllerCoreOutput();
      next.setCenterOfPressureData(nextCenterOfPressureDataHolder(random, rootBody, possibleFrames));
      next.setLinearMomentumRate(nextFrameVector3D(random, possibleFrames));
      next.setRootJointDesiredConfigurationData(nextRootJointDesiredConfigurationData(random, possibleFrames));
      next.setLowLevelOneDoFJointDesiredDataHolder(nextJointDesiredOutputListBasics(random, rootBody, possibleFrames));
      return next;
   }

   public static ControllerCoreCommand nextControllerCoreCommand(Random random, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws NoSuchMethodException, SecurityException, IllegalAccessException, IllegalArgumentException, InvocationTargetException
   {
      return nextControllerCoreCommand(random, getInverseDynamicsCommandTypes(InverseDynamicsCommandList.class, InverseDynamicsCommandBuffer.class),
                                       getInverseKinematicsCommandTypes(InverseKinematicsCommandList.class, InverseKinematicsCommandBuffer.class),
                                       getVirtualModelControlCommandTypes(VirtualModelControlCommandList.class, VirtualModelControlCommandBuffer.class,
                                                                          VirtualEffortCommand.class),
                                       getFeedbackControlCommandTypes(FeedbackControlCommandList.class, FeedbackControlCommandBuffer.class), rootBody,
                                       possibleFrames);
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

   public static LinearMomentumRateControlModuleOutput nextLinearMomentumRateControlModuleOutput(Random random, ReferenceFrame... possibleFrames)
   {
      LinearMomentumRateControlModuleOutput next = new LinearMomentumRateControlModuleOutput();
      next.setDesiredCMP(nextFramePoint2D(random, possibleFrames));
      next.setEffectiveICPAdjustment(nextFrameVector3D(random, possibleFrames));
      next.setUsingStepAdjustment(random.nextBoolean());
      next.setFootstepWasAdjusted(random.nextBoolean());
      next.setFootstepSolution(nextFramePose3D(random, possibleFrames));
      return next;
   }

   public static LinearMomentumRateControlModuleInput nextLinearMomentumRateControlModuleInput(Random random, RigidBodyBasics rootBody,
                                                                                               ReferenceFrame... possibleFrames)
         throws Exception
   {
      LinearMomentumRateControlModuleInput next = new LinearMomentumRateControlModuleInput();
      next.setContactStateCommand(new SideDependentList<PlaneContactStateCommand>(nextPlaneContactStateCommand(random, rootBody, possibleFrames),
                                                                                  nextPlaneContactStateCommand(random, rootBody, possibleFrames)));
      next.setControlHeightWithMomentum(random.nextBoolean());
      next.setDesiredCapturePoint(nextFramePoint2D(random, possibleFrames));
      next.setDesiredCapturePointVelocity(nextFrameVector2D(random, possibleFrames));
      next.setDesiredCenterOfMassHeightAcceleration(random.nextDouble());
      next.setFinalTransferDuration(random.nextDouble());
      next.setFootsteps(nextRecyclingArrayList(SimpleAdjustableFootstep.class, random.nextInt(10), random, true, rootBody, possibleFrames));
      next.setSwingDurations(nextTDoubleArrayList(random, random.nextInt(10)));
      next.setTransferDurations(nextTDoubleArrayList(random, random.nextInt(10)));
      next.setInitializeForSingleSupport(random.nextBoolean());
      next.setInitializeForStanding(random.nextBoolean());
      next.setInitializeForTransfer(random.nextBoolean());
      next.setKeepCoPInsideSupportPolygon(random.nextBoolean());
      next.setMinimizeAngularMomentumRateZ(random.nextBoolean());
      next.setOmega0(random.nextDouble());
      next.setPerfectCMP(nextFramePoint2D(random, possibleFrames));
      next.setPerfectCoP(nextFramePoint2D(random, possibleFrames));
      next.setRemainingTimeInSwingUnderDisturbance(random.nextDouble());
      next.setSupportSide(nextElementIn(random, RobotSide.values));
      next.setTransferToSide(nextElementIn(random, RobotSide.values));
      return next;
   }

   public static SimpleAdjustableFootstep nextSimpleAdjustableFootstep(Random random, ReferenceFrame... possibleFrames)
   {
      SimpleAdjustableFootstep next = new SimpleAdjustableFootstep();
      next.setSoleFramePose(nextFramePose3D(random, possibleFrames));
      next.setRobotSide(nextElementIn(random, RobotSide.values));
      next.setFoothold(nextConvexPolygon2D(random));
      next.setIsAdjustable(random.nextBoolean());
      return next;
   }

   public static ConvexPolygon2D nextConvexPolygon2D(Random random)
   {
      ConvexPolygon2D next = new ConvexPolygon2D();
      int vertices = random.nextInt(10);
      for (int i = 0; i < vertices; i++)
      {
         next.addVertex(nextPoint2D(random));
      }
      next.update();
      return next;
   }

   public static void randomizeDoubleArray(Random random, double[] array)
   {
      for (int i = 0; i < array.length; i++)
      {
         array[i] = random.nextDouble();
      }
   }

   public static void randomizeTDoubleArrayList(Random random, TDoubleArrayList listToRandomize)
   {
      for (int i = 0; i < listToRandomize.size(); i++)
      {
         listToRandomize.set(i, random.nextDouble());
      }
   }

   public static void randomizeFrameTupleArrayList(Random random, FrameTupleArrayList<?> listToRandomize, ReferenceFrame... possibleFrames)
   {
      for (int i = 0; i < listToRandomize.size(); i++)
      {
         listToRandomize.get(i).setIncludingFrame(nextFramePoint3D(random, possibleFrames));
      }
   }

   public static void randomizeDenseMatrixArrayList(Random random, DenseMatrixArrayList listToRandomize)
   {
      for (int i = 0; i < listToRandomize.size(); i++)
      {
         listToRandomize.get(i).set(RandomMatrices.createRandom(random.nextInt(15) + 1, random.nextInt(15) + 1, random));
      }
   }

   public static void randomizeRecyclingArrayList(RecyclingArrayList<?> listToRandomize, ReflectionBuilder randomElementSupplier) throws Exception
   {
      if (listToRandomize.isEmpty())
         return;

      Field internalArrayField = RecyclingArrayList.class.getDeclaredField("values");
      internalArrayField.setAccessible(true);
      Object[] values = (Object[]) internalArrayField.get(listToRandomize);

      for (int i = 0; i < values.length; i++)
      {
         values[i] = randomElementSupplier.get(values[i].getClass());
      }
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public static void randomizeList(List listToRandomize, ReflectionBuilder randomElementSupplier) throws Exception
   {
      if (listToRandomize.isEmpty())
         return;

      if (listToRandomize instanceof RecyclingArrayList)
      {
         randomizeRecyclingArrayList((RecyclingArrayList<?>) listToRandomize, randomElementSupplier);
         return;
      }

      for (int i = 0; i < listToRandomize.size(); i++)
      {
         listToRandomize.set(i, randomElementSupplier.get(listToRandomize.get(i).getClass()));
      }
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public static void randomizeSideDependentList(SideDependentList listToRandomize, ReflectionBuilder randomElementSupplier) throws Exception
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         listToRandomize.put(robotSide, randomElementSupplier.get(listToRandomize.get(robotSide).getClass()));
      }
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public static void randomizeAtomicReference(AtomicReference referenceToRandomize, ReflectionBuilder randomElementSupplier) throws Exception
   {
      referenceToRandomize.set(randomElementSupplier.get(referenceToRandomize.get().getClass()));
   }

   @SuppressWarnings("rawtypes")
   public static void randomizeField(Random random, Field field, Object fieldOwnerInstance, RigidBodyBasics rootBody, ReferenceFrame... possibleFrames)
         throws Exception
   {
      field.setAccessible(true);
      Class<?> fieldType = field.getType();
      Object fieldInstance = field.get(fieldOwnerInstance);

      if (fieldType == DenseMatrixArrayList.class)
      {
         randomizeDenseMatrixArrayList(random, (DenseMatrixArrayList) fieldInstance);
      }
      else if (fieldType == FrameTupleArrayList.class)
      {
         randomizeFrameTupleArrayList(random, (FrameTupleArrayList<?>) fieldInstance, possibleFrames);
      }
      else if (fieldType == RecyclingArrayList.class)
      {
         randomizeRecyclingArrayList((RecyclingArrayList<?>) fieldInstance,
                                     (listElementType) -> nextTypeInstance(listElementType, random, true, rootBody, possibleFrames));
      }
      else if (fieldType == List.class)
      {
         randomizeList((List) fieldInstance, (listElementType) -> nextTypeInstance(listElementType, random, true, rootBody, possibleFrames));
      }
      else if (fieldType == TDoubleArrayList.class)
      {
         randomizeTDoubleArrayList(random, (TDoubleArrayList) fieldInstance);
      }
      else if (fieldType == double[].class)
      {
         randomizeDoubleArray(random, (double[]) fieldInstance);
      }
      else if (fieldType.isPrimitive())
      {
         randomizePrimitiveField(random, field, fieldOwnerInstance);
      }
      else if (fieldType == AtomicReference.class)
      {
         randomizeAtomicReference((AtomicReference) fieldInstance,
                                  (listElementType) -> nextTypeInstance(listElementType, random, true, rootBody, possibleFrames));
      }
      else if (fieldType == DenseMatrix64F.class)
      {
         field.set(fieldOwnerInstance, RandomMatrices.createRandom(10, 10, random));
      }
      else if (fieldType == SideDependentList.class)
      {
         randomizeSideDependentList((SideDependentList) fieldInstance,
                                    (listElementType) -> nextTypeInstance(listElementType, random, true, rootBody, possibleFrames));
      }
      else
      {
         field.set(fieldOwnerInstance, nextTypeInstance(fieldType, random, true, rootBody, possibleFrames));
      }
   }

   public static void randomizePrimitiveField(Random random, Field field, Object fieldOwnerInstance) throws Exception
   {
      field.setAccessible(true);
      Class<?> fieldType = field.getType();

      if (fieldType == double.class)
         field.set(fieldOwnerInstance, random.nextDouble());
      else if (fieldType == int.class)
         field.set(fieldOwnerInstance, random.nextInt(20));
      else if (fieldType == boolean.class)
         field.set(fieldOwnerInstance, random.nextBoolean());
      else if (fieldType == long.class)
         field.set(fieldOwnerInstance, random.nextLong());
      else
         throw new RuntimeException("Unhandled primitive: " + fieldType.getSimpleName());
   }

   public static Object nextTypeInstance(Class<?> typeToInstantiateRandomly, Random random, boolean ensureNonEmptyCommand, RigidBodyBasics rootBody,
                                         ReferenceFrame... possibleFrames)
         throws IllegalAccessException, IllegalArgumentException, InvocationTargetException, NoSuchMethodException, SecurityException
   {
      if (typeToInstantiateRandomly.isEnum())
      {
         return nextElementIn(random, typeToInstantiateRandomly.getEnumConstants());
      }
      else if (RigidBodyBasics.class.isAssignableFrom(typeToInstantiateRandomly))
      {
         return nextElementIn(random, rootBody.subtreeArray());
      }
      else if (typeToInstantiateRandomly == ReferenceFrame.class)
      {
         return nextElementIn(random, possibleFrames);
      }
      else if (OneDoFJointBasics.class.isAssignableFrom(typeToInstantiateRandomly))
      {
         return nextElementIn(random, SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList()));
      }
      else if (JointBasics.class.isAssignableFrom(typeToInstantiateRandomly))
      {
         List<JointBasics> allJoints = SubtreeStreams.fromChildren(JointBasics.class, rootBody).collect(Collectors.toList());
         return nextElementIn(random, allJoints);
      }

      String methodName = "next" + typeToInstantiateRandomly.getSimpleName();
      List<Method> potentialGenerators = Stream.of(CrossRobotCommandRandomTools.class.getDeclaredMethods()).filter(m -> m.getName().equals(methodName))
                                               .collect(Collectors.toList());

      if (potentialGenerators.size() > 1)
      {
         potentialGenerators.sort((m1, m2) -> Integer.compare(m1.getParameterCount(), m2.getParameterCount()));
      }
      else if (potentialGenerators.isEmpty())
      {
         throw new UnsupportedOperationException("Encountered problem finding random generator for: " + typeToInstantiateRandomly.getSimpleName());
      }

      Method generator = potentialGenerators.get(0);

      for (int i = 0; i < potentialGenerators.size(); i++)
      {
         if (Stream.of(potentialGenerators.get(i).getParameterTypes()).anyMatch(type -> type == boolean.class))
         {
            generator = potentialGenerators.get(i);
            break;
         }
      }

      Object[] arguments = new Object[generator.getParameterCount()];
      arguments[0] = random;

      for (int argIndex = 1; argIndex < generator.getParameterCount(); argIndex++)
      {
         Class<?> parameterType = generator.getParameterTypes()[argIndex];
         if (parameterType == RigidBodyBasics.class)
         {
            arguments[argIndex] = rootBody;
         }
         else if (parameterType == ReferenceFrame[].class)
         {
            arguments[argIndex] = possibleFrames;
         }
         else if (parameterType == OneDoFJointBasics.class)
         {
            arguments[argIndex] = nextElementIn(random, SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).collect(Collectors.toList()));
         }
         else if (parameterType == boolean.class)
         {
            arguments[argIndex] = ensureNonEmptyCommand;
         }
         else
         {
            throw new RuntimeException("Unexpected type: " + parameterType.getSimpleName() + " for " + generator.getName());
         }
      }

      return generator.invoke(null, arguments);
   }
}
