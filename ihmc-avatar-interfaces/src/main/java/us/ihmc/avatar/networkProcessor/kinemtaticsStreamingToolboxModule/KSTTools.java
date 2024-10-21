package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.ObjectCarryMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WholeBodyStreamingMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController.IKRobotStateUpdater;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController.RobotConfigurationDataBasedUpdater;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.WholeBodyStreamingMessagePublisher;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.KinematicsStreamingToolboxController.WholeBodyTrajectoryMessagePublisher;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.input.KSTInputFilter;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output.KSTOutputDataBasics;
import us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule.output.KSTOutputDataReadOnly;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ObjectCarryCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialVectorBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map.Entry;

public class KSTTools
{
   private final CommandInputManager commandInputManager;
   private final CommandInputManager ikCommandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final KinematicsStreamingToolboxParameters parameters;
   private final FullHumanoidRobotModel desiredFullRobotModel;
   private final FullHumanoidRobotModelFactory fullRobotModelFactory;
   private final DoubleProvider time;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final YoRegistry registry;

   private final FullHumanoidRobotModel currentFullRobotModel;
   private final FloatingJointBasics currentRootJoint;
   private final OneDoFJointBasics[] currentOneDoFJoint;

   private final HumanoidKinematicsToolboxController ikController;
   private final KSTStreamingMessageFactory streamingMessageFactory;
   private final KinematicsToolboxOutputConverter trajectoryMessageFactory;
   private final WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
   private final YoDouble streamIntegrationDuration;

   private IKRobotStateUpdater robotStateUpdater;

   private final double toolboxControllerPeriod;
   private final YoDouble walkingControllerMonotonicTime, walkingControllerWallTime;

   private final YoLong currentMessageId;

   private final YoBoolean hasNewInputCommand, hasPreviousInput;
   private final YoDouble latestInputReceivedTime, previousInputReceivedTime;
   private KinematicsStreamingToolboxInputCommand latestInput = null;
   private KinematicsStreamingToolboxInputCommand previousInput = null;

   private final KinematicsStreamingToolboxConfigurationCommand configurationCommand = new KinematicsStreamingToolboxConfigurationCommand();
   private final YoBoolean isNeckJointspaceOutputEnabled;
   private final YoBoolean isChestTaskspaceOutputEnabled;
   private final YoBoolean isPelvisTaskspaceOutputEnabled;
   private final SideDependentList<YoBoolean> areHandTaskspaceOutputsEnabled = new SideDependentList<>();
   private final SideDependentList<YoBoolean> areArmJointspaceOutputsEnabled = new SideDependentList<>();

   private final YoBoolean invalidUserInput;
   private final YoLong latestInputTimestampSource;
   private final YoDouble latestInputTimeSource;
   private final YoBoolean useStreamingPublisher;
   private final KSTInputFilter inputFilter;

   private WholeBodyTrajectoryMessagePublisher trajectoryMessagePublisher = m ->
   {
   };
   private WholeBodyStreamingMessagePublisher streamingMessagePublisher = null;
   private final KinematicsStreamingLogger logger;

   private final SideDependentList<ObjectCarryManager> objectCarryManagers = new SideDependentList<>();

   public KSTTools(CommandInputManager commandInputManager,
                   StatusMessageOutputManager statusOutputManager,
                   KinematicsStreamingToolboxParameters parameters,
                   FullHumanoidRobotModel desiredFullRobotModel,
                   FullHumanoidRobotModelFactory fullRobotModelFactory,
                   DoubleProvider time,
                   boolean runPostureOptimizer,
                   YoGraphicsListRegistry yoGraphicsListRegistry,
                   YoRegistry registry)
   {
      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;
      this.parameters = parameters;
      this.desiredFullRobotModel = desiredFullRobotModel;
      this.fullRobotModelFactory = fullRobotModelFactory;
      this.toolboxControllerPeriod = parameters.getToolboxUpdatePeriod();
      this.time = time;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.registry = registry;

      walkingControllerMonotonicTime = new YoDouble("walkingControllerMonotonicTime", registry);
      walkingControllerWallTime = new YoDouble("walkingControllerWallTime", registry);

      currentFullRobotModel = fullRobotModelFactory.createFullRobotModel();
      currentRootJoint = currentFullRobotModel.getRootJoint();
      currentOneDoFJoint = FullRobotModelUtils.getAllJointsExcludingHands(currentFullRobotModel);

      if (parameters.getJointCustomPositionLowerLimits() != null)
      {
         for (Entry<String, Double> entry : parameters.getJointCustomPositionLowerLimits().entrySet())
         {
            OneDoFJointBasics joint = desiredFullRobotModel.getOneDoFJointByName(entry.getKey());
            if (joint == null)
            {
               LogTools.warn("Couldn't find joint {}", entry.getKey());
               continue;
            }

            joint.setJointLimitLower(entry.getValue());
         }
      }

      if (parameters.getJointCustomPositionUpperLimits() != null)
      {
         for (Entry<String, Double> entry : parameters.getJointCustomPositionUpperLimits().entrySet())
         {
            OneDoFJointBasics joint = desiredFullRobotModel.getOneDoFJointByName(entry.getKey());
            if (joint == null)
            {
               LogTools.warn("Couldn't find joint {}", entry.getKey());
               continue;
            }

            joint.setJointLimitUpper(entry.getValue());
         }
      }

      for (RobotSide robotSide : RobotSide.values)
      {
         ObjectCarryManager objectCarryManager = new ObjectCarryManager(robotSide, parameters.getToolboxUpdatePeriod(), registry);
         objectCarryManagers.put(robotSide, objectCarryManager);
      }

      ikCommandInputManager = new CommandInputManager(HumanoidKinematicsToolboxController.class.getSimpleName(),
                                                      KinematicsToolboxModule.supportedCommands(),
                                                      32); // Need at least 17: 2*7 (for each arm) + 3 (for the neck).
      ikController = new HumanoidKinematicsToolboxController(ikCommandInputManager,
                                                             statusOutputManager,
                                                             desiredFullRobotModel,
                                                             toolboxControllerPeriod,
                                                             runPostureOptimizer,
                                                             yoGraphicsListRegistry,
                                                             registry);

      KinematicsStreamingToolboxCommandConverter commandConversionHelper = new KinematicsStreamingToolboxCommandConverter(desiredFullRobotModel,
                                                                                                                          ikController.getDesiredReferenceFrames());
      commandInputManager.registerConversionHelper(commandConversionHelper);
      commandConversionHelper.process(configurationCommand,
                                      parameters.getDefaultConfiguration()); // Initialize the configurationCommand from the parameters' message

      ikCommandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(desiredFullRobotModel, ikController.getDesiredReferenceFrames()));

      ikController.setPreserveUserCommandHistory(false);

      inputFilter = new KSTInputFilter(currentFullRobotModel, parameters, registry);

      currentMessageId = new YoLong("currentMessageId", registry);
      currentMessageId.set(1L);
      streamingMessageFactory = new KSTStreamingMessageFactory(fullRobotModelFactory, registry);
      streamingMessageFactory.setEnableVelocity(true);
      streamingMessageFactory.setEnableAcceleration(true);
      trajectoryMessageFactory = new KinematicsToolboxOutputConverter(fullRobotModelFactory);

      streamIntegrationDuration = new YoDouble("streamIntegrationDuration", registry);
      streamIntegrationDuration.set(parameters.getStreamIntegrationDuration());

      hasNewInputCommand = new YoBoolean("hasNewInputCommand", registry);
      hasPreviousInput = new YoBoolean("hasPreviousInput", registry);
      latestInputReceivedTime = new YoDouble("latestInputReceivedTime", registry);
      previousInputReceivedTime = new YoDouble("previousInputReceivedTime", registry);
      flushInputCommands();

      isNeckJointspaceOutputEnabled = new YoBoolean("isNeckJointspaceOutputEnabled", registry);
      isChestTaskspaceOutputEnabled = new YoBoolean("isChestTaskspaceOutputEnabled", registry);
      isPelvisTaskspaceOutputEnabled = new YoBoolean("isPelvisTaskspaceOutputEnabled", registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         YoBoolean isHandTaskspaceOutputEnabled = new YoBoolean("is" + robotSide.getPascalCaseName() + "HandTaskspaceOutputEnabled", registry);
         areHandTaskspaceOutputsEnabled.put(robotSide, isHandTaskspaceOutputEnabled);
         YoBoolean isArmJointspaceOutputEnabled = new YoBoolean("is" + robotSide.getPascalCaseName() + "ArmJointspaceOutputEnabled", registry);
         areArmJointspaceOutputsEnabled.put(robotSide, isArmJointspaceOutputEnabled);
      }

      invalidUserInput = new YoBoolean("invalidUserInput", registry);
      latestInputTimestampSource = new YoLong("latestInputTimestampSource", registry);
      latestInputTimeSource = new YoDouble("latestInputTimeSource", registry);

      useStreamingPublisher = new YoBoolean("useStreamingPublisher", registry);
      useStreamingPublisher.set(parameters.getUseStreamingPublisher());
      logger = new KinematicsStreamingLogger(registry);
   }

   public void update()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         objectCarryManagers.get(robotSide).update();
      }

      inputFilter.update();

      if (commandInputManager.isNewCommandAvailable(KinematicsStreamingToolboxConfigurationCommand.class))
      {
         configurationCommand.set(commandInputManager.pollNewestCommand(KinematicsStreamingToolboxConfigurationCommand.class));
      }

      isNeckJointspaceOutputEnabled.set(configurationCommand.isNeckJointspaceEnabled());
      isChestTaskspaceOutputEnabled.set(configurationCommand.isChestTaskspaceEnabled());
      isPelvisTaskspaceOutputEnabled.set(configurationCommand.isPelvisTaskspaceEnabled());

      for (RobotSide robotSide : RobotSide.values)
      {
         areHandTaskspaceOutputsEnabled.get(robotSide).set(configurationCommand.isHandTaskspaceEnabled(robotSide));
         areArmJointspaceOutputsEnabled.get(robotSide).set(configurationCommand.isArmJointspaceEnabled(robotSide));
      }

      boolean wasRobotUpdated = robotStateUpdater.updateRobotConfiguration(currentRootJoint, currentOneDoFJoint);

      if (wasRobotUpdated)
      {
         if (robotStateUpdater instanceof RobotConfigurationDataBasedUpdater rcdBasedUpdater)
         {
            RobotConfigurationData lastRobotConfigurationData = rcdBasedUpdater.getLastRobotConfigurationData();
            walkingControllerMonotonicTime.set(Conversions.nanosecondsToSeconds(lastRobotConfigurationData.getMonotonicTime()));
            walkingControllerWallTime.set(Conversions.nanosecondsToSeconds(lastRobotConfigurationData.getWallTime()));
         }

         currentFullRobotModel.updateFrames();
      }

      if (commandInputManager.isNewCommandAvailable(KinematicsStreamingToolboxInputCommand.class))
      {
         if (latestInput != null)
         {
            if (previousInput == null)
               previousInput = new KinematicsStreamingToolboxInputCommand();

            previousInput.set(latestInput);
            previousInputReceivedTime.set(latestInputReceivedTime.getValue());
            hasPreviousInput.set(true);
         }

         if (latestInput == null)
            latestInput = new KinematicsStreamingToolboxInputCommand();

         latestInput.set(commandInputManager.pollNewestCommand(KinematicsStreamingToolboxInputCommand.class));

//         ikController.updateUnoptimizedStablityData(latestInput);

         for (int i = latestInput.getNumberOfInputs() - 1; i >= 0; i--)
         {
            KinematicsToolboxRigidBodyCommand latestEndEffectorInput = latestInput.getInput(i);
            RigidBodyBasics endEffector = latestEndEffectorInput.getEndEffector();
            KinematicsToolboxRigidBodyCommand previousEndEffectorInput = hasPreviousInput.getValue() ? previousInput.getInputFor(endEffector) : null;

            if (!inputFilter.isInputValid(latestEndEffectorInput, previousEndEffectorInput))
               invalidUserInput.set(true);
         }

         if (latestInput.hasCenterOfMassInput())
         {
            KinematicsToolboxCenterOfMassCommand latestCenterOfMassInput = latestInput.getCenterOfMassInput();
            KinematicsToolboxCenterOfMassCommand previousCenterOfMassInput;
            if (hasPreviousInput.getValue() && previousInput.hasCenterOfMassInput())
               previousCenterOfMassInput = previousInput.getCenterOfMassInput();
            else
               previousCenterOfMassInput = null;

            if (!inputFilter.isInputValid(latestCenterOfMassInput, previousCenterOfMassInput))
               invalidUserInput.set(true);
         }

         latestInputTimestampSource.set(latestInput.getTimestamp());
         latestInputTimeSource.set(latestInput.getTimestamp() * 1.0e-9);

         if (latestInput.getTimestamp() <= 0)
            latestInput.setTimestamp(Conversions.secondsToNanoseconds(time.getValue()));

         latestInputReceivedTime.set(time.getValue());
         hasNewInputCommand.set(true);
      }
      else
      {
         hasNewInputCommand.set(false);
      }
   }

   public void onObjectCarryMessageReceived(ObjectCarryMessage objectCarryMessage)
   {
      RobotSide robotSide = RobotSide.fromByte(objectCarryMessage.getRobotSide());
      objectCarryManagers.get(robotSide).handleCommand(objectCarryMessage);
   }

   private class ObjectCarryManager
   {
      private final SpatialInertiaBasics handInertia;
      private final double nominalMass;
      private double objectCarryMass;
      private boolean isCarryingObject = false;
      private final RateLimitedYoVariable alphaLoadedCarriedObject;

      public ObjectCarryManager(RobotSide robotSide, double controlDT, YoRegistry registry)
      {
         handInertia = desiredFullRobotModel.getHand(robotSide).getInertia();
         nominalMass = handInertia.getMass();
         double loadDuration = 2.0;
         alphaLoadedCarriedObject = new RateLimitedYoVariable(robotSide + "alphaLoadedCarry", registry, 1.0 / loadDuration, controlDT);
      }

      void handleCommand(ObjectCarryMessage objectCarryMessage)
      {
         LogTools.info("Received object carry command!");

         isCarryingObject = objectCarryMessage.getIsPickingUp();
         if (isCarryingObject)
            objectCarryMass = objectCarryMessage.getObjectMass();
      }

      void update()
      {
         alphaLoadedCarriedObject.update(isCarryingObject ? 1.0 : 0.0);
         handInertia.setMass(nominalMass + alphaLoadedCarriedObject.getValue() * objectCarryMass);
      }
   }

   public void resetUserInvalidInputFlag()
   {
      invalidUserInput.set(false);
   }

   public boolean hasUserSubmittedInvalidInput()
   {
      return invalidUserInput.getValue();
   }

   public KinematicsStreamingToolboxParameters getParameters()
   {
      return parameters;
   }

   public double getTime()
   {
      return time.getValue();
   }

   public KinematicsStreamingToolboxConfigurationCommand getConfigurationCommand()
   {
      return configurationCommand;
   }

   public void getCurrentState(KSTOutputDataBasics currentStateToPack)
   {
      currentStateToPack.setFromRobot(currentRootJoint, currentOneDoFJoint);
   }

   public boolean hasNewInputCommand()
   {
      return hasNewInputCommand.getValue() && !hasUserSubmittedInvalidInput();
   }

   public KinematicsStreamingToolboxInputCommand getLatestInput()
   {
      return latestInput;
   }

   public double getLatestInputReceivedTime()
   {
      return latestInputReceivedTime.getValue();
   }

   public boolean hasPreviousInput()
   {
      return hasPreviousInput.getValue();
   }

   public KinematicsStreamingToolboxInputCommand getPreviousInput()
   {
      return hasPreviousInput.getValue() ? previousInput : null;
   }

   public double getPreviousInputReceivedTime()
   {
      return previousInputReceivedTime.getValue();
   }

   public void flushInputCommands()
   {
      latestInput = null;
      commandInputManager.clearAllCommands();
      hasNewInputCommand.set(false);
      hasPreviousInput.set(false);
      latestInputReceivedTime.set(-1.0);
      previousInputReceivedTime.set(-1.0);
   }

   public void setTrajectoryMessagerPublisher(WholeBodyTrajectoryMessagePublisher outputPublisher)
   {
      this.trajectoryMessagePublisher = outputPublisher;
   }

   public void setStreamingMessagePublisher(WholeBodyStreamingMessagePublisher streamingMessagePublisher)
   {
      this.streamingMessagePublisher = streamingMessagePublisher;
   }

   public void streamToController(KSTOutputDataReadOnly outputToPublish, boolean finalizeTrajectory)
   {
      if (finalizeTrajectory)
      {
         WholeBodyTrajectoryMessage messageToPublish = setupFinalizeTrajectoryMessage(outputToPublish);
         logger.update(messageToPublish);
         trajectoryMessagePublisher.publish(messageToPublish);
      }
      else if (streamingMessagePublisher == null || !useStreamingPublisher.getValue())
      {
         WholeBodyTrajectoryMessage messageToPublish = setupTrajectoryMessage(outputToPublish);
         logger.update(messageToPublish);
         trajectoryMessagePublisher.publish(messageToPublish);
      }
      else
      {
         WholeBodyStreamingMessage messageToPublish = setupStreamingMessage(outputToPublish);
         logger.update(messageToPublish);
         streamingMessagePublisher.publish(messageToPublish);
      }
   }

   public WholeBodyStreamingMessage setupStreamingMessage(KSTOutputDataReadOnly solutionToConvert)
   {
      streamingMessageFactory.update(currentMessageId.getValue(),
                                     Conversions.secondsToNanoseconds(time.getValue()),
                                     streamIntegrationDuration.getValue(),
                                     solutionToConvert::updateRobot);

      for (RobotSide robotSide : RobotSide.values)
      {
         if (areHandTaskspaceOutputsEnabled.get(robotSide).getValue())
            streamingMessageFactory.computeHandStreamingMessage(robotSide, configurationCommand.getHandTrajectoryFrameId(robotSide));

         if (areArmJointspaceOutputsEnabled.get(robotSide).getValue())
            streamingMessageFactory.computeArmStreamingMessage(robotSide);
      }

      if (isNeckJointspaceOutputEnabled.getValue())
         streamingMessageFactory.computeNeckStreamingMessage();
      if (isChestTaskspaceOutputEnabled.getValue())
         streamingMessageFactory.computeChestStreamingMessage(configurationCommand.getChestTrajectoryFrameId());
      if (isPelvisTaskspaceOutputEnabled.getValue())
         streamingMessageFactory.computePelvisStreamingMessage(configurationCommand.getPelvisTrajectoryFrameId());

      currentMessageId.increment();
      return streamingMessageFactory.getOutput();
   }

   public WholeBodyTrajectoryMessage setupTrajectoryMessage(KSTOutputDataReadOnly solutionToConvert)
   {
      HumanoidMessageTools.resetWholeBodyTrajectoryToolboxMessage(wholeBodyTrajectoryMessage);
      trajectoryMessageFactory.updateFullRobotModel(solutionToConvert::updateRobot);
      trajectoryMessageFactory.setMessageToCreate(wholeBodyTrajectoryMessage);
      trajectoryMessageFactory.setTrajectoryTime(0.0);
      trajectoryMessageFactory.setEnableVelocity(true);

      for (RobotSide robotSide : RobotSide.values)
      {
         if (areHandTaskspaceOutputsEnabled.get(robotSide).getValue())
            trajectoryMessageFactory.computeHandTrajectoryMessage(robotSide, configurationCommand.getHandTrajectoryFrameId(robotSide));

         if (areArmJointspaceOutputsEnabled.get(robotSide).getValue())
            trajectoryMessageFactory.computeArmTrajectoryMessage(robotSide);
      }

      if (isNeckJointspaceOutputEnabled.getValue())
         trajectoryMessageFactory.computeNeckTrajectoryMessage();
      if (isChestTaskspaceOutputEnabled.getValue())
         trajectoryMessageFactory.computeChestTrajectoryMessage(configurationCommand.getChestTrajectoryFrameId());
      if (isPelvisTaskspaceOutputEnabled.getValue())
         trajectoryMessageFactory.computePelvisTrajectoryMessage(configurationCommand.getPelvisTrajectoryFrameId());

      wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage().setEnableUserPelvisControl(true);
      HumanoidMessageTools.configureForStreaming(wholeBodyTrajectoryMessage,
                                                 streamIntegrationDuration.getValue(),
                                                 Conversions.secondsToNanoseconds(time.getValue()));
      setAllIDs(wholeBodyTrajectoryMessage, currentMessageId.getValue());
      currentMessageId.increment();
      return wholeBodyTrajectoryMessage;
   }

   public WholeBodyTrajectoryMessage setupFinalizeTrajectoryMessage(KSTOutputDataReadOnly solutionToConvert)
   {
      HumanoidMessageTools.resetWholeBodyTrajectoryToolboxMessage(wholeBodyTrajectoryMessage);
      trajectoryMessageFactory.updateFullRobotModel(solutionToConvert::updateRobot);
      trajectoryMessageFactory.setMessageToCreate(wholeBodyTrajectoryMessage);
      trajectoryMessageFactory.setTrajectoryTime(0.5);

      for (RobotSide robotSide : RobotSide.values)
      {
         if (areHandTaskspaceOutputsEnabled.get(robotSide).getValue())
            trajectoryMessageFactory.computeHandTrajectoryMessage(robotSide);

         if (areArmJointspaceOutputsEnabled.get(robotSide).getValue())
            trajectoryMessageFactory.computeArmTrajectoryMessage(robotSide);
      }

      if (isNeckJointspaceOutputEnabled.getValue())
         trajectoryMessageFactory.computeNeckTrajectoryMessage();
      if (isChestTaskspaceOutputEnabled.getValue())
         trajectoryMessageFactory.computeChestTrajectoryMessage();
      if (isPelvisTaskspaceOutputEnabled.getValue())
         trajectoryMessageFactory.computePelvisTrajectoryMessage();

      wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage().setEnableUserPelvisControl(false);
      HumanoidMessageTools.configureForOverriding(wholeBodyTrajectoryMessage);
      setAllIDs(wholeBodyTrajectoryMessage, currentMessageId.getValue());
      currentMessageId.increment();

      return wholeBodyTrajectoryMessage;
   }

   private static void setAllIDs(WholeBodyTrajectoryMessage message, long id)
   {
      message.setSequenceId(id);
      message.getLeftHandTrajectoryMessage().setSequenceId(id);
      message.getRightHandTrajectoryMessage().setSequenceId(id);
      message.getLeftArmTrajectoryMessage().setSequenceId(id);
      message.getRightArmTrajectoryMessage().setSequenceId(id);
      message.getChestTrajectoryMessage().setSequenceId(id);
      message.getSpineTrajectoryMessage().setSequenceId(id);
      message.getPelvisTrajectoryMessage().setSequenceId(id);
      message.getLeftFootTrajectoryMessage().setSequenceId(id);
      message.getRightFootTrajectoryMessage().setSequenceId(id);
      message.getNeckTrajectoryMessage().setSequenceId(id);
      message.getHeadTrajectoryMessage().setSequenceId(id);

      message.setUniqueId(id);
      message.getLeftHandTrajectoryMessage().setUniqueId(id);
      message.getRightHandTrajectoryMessage().setUniqueId(id);
      message.getLeftArmTrajectoryMessage().setUniqueId(id);
      message.getRightArmTrajectoryMessage().setUniqueId(id);
      message.getChestTrajectoryMessage().setUniqueId(id);
      message.getSpineTrajectoryMessage().setUniqueId(id);
      message.getPelvisTrajectoryMessage().setUniqueId(id);
      message.getLeftFootTrajectoryMessage().setUniqueId(id);
      message.getRightFootTrajectoryMessage().setUniqueId(id);
      message.getNeckTrajectoryMessage().setUniqueId(id);
      message.getHeadTrajectoryMessage().setUniqueId(id);

      message.getLeftHandTrajectoryMessage().getSe3Trajectory().getQueueingProperties().setMessageId(id);
      message.getRightHandTrajectoryMessage().getSe3Trajectory().getQueueingProperties().setMessageId(id);
      message.getLeftArmTrajectoryMessage().getJointspaceTrajectory().getQueueingProperties().setMessageId(id);
      message.getRightArmTrajectoryMessage().getJointspaceTrajectory().getQueueingProperties().setMessageId(id);
      message.getChestTrajectoryMessage().getSo3Trajectory().getQueueingProperties().setMessageId(id);
      message.getSpineTrajectoryMessage().getJointspaceTrajectory().getQueueingProperties().setMessageId(id);
      message.getPelvisTrajectoryMessage().getSe3Trajectory().getQueueingProperties().setMessageId(id);
      message.getLeftFootTrajectoryMessage().getSe3Trajectory().getQueueingProperties().setMessageId(id);
      message.getRightFootTrajectoryMessage().getSe3Trajectory().getQueueingProperties().setMessageId(id);
      message.getNeckTrajectoryMessage().getJointspaceTrajectory().getQueueingProperties().setMessageId(id);
      message.getHeadTrajectoryMessage().getSo3Trajectory().getQueueingProperties().setMessageId(id);
   }

   public void setRobotStateUpdater(IKRobotStateUpdater robotStateUpdater)
   {
      this.robotStateUpdater = robotStateUpdater;
      ikController.setDesiredRobotStateUpdater(robotStateUpdater);
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus newStatus)
   {
      ikController.updateCapturabilityBasedStatus(newStatus);
   }

   public CommandInputManager getCommandInputManager()
   {
      return commandInputManager;
   }

   public CommandInputManager getIKCommandInputManager()
   {
      return ikCommandInputManager;
   }

   public StatusMessageOutputManager getStatusOutputManager()
   {
      return statusOutputManager;
   }

   public FullHumanoidRobotModel getDesiredFullRobotModel()
   {
      return desiredFullRobotModel;
   }

   public FullHumanoidRobotModelFactory getFullRobotModelFactory()
   {
      return fullRobotModelFactory;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public KinematicsToolboxOutputConverter getOutputConverter()
   {
      return trajectoryMessageFactory;
   }

   public FullHumanoidRobotModel getCurrentFullRobotModel()
   {
      return currentFullRobotModel;
   }

   public HumanoidKinematicsToolboxController getIKController()
   {
      return ikController;
   }

   public KinematicsStreamingLogger getLogger()
   {
      return logger;
   }

   public double getToolboxControllerPeriod()
   {
      return toolboxControllerPeriod;
   }

   /**
    * [Unsafe] Copies the joint positions from the given array of joints to the given list of messages.
    * <p>
    * This method assumes that the given list of messages is ordered to match the given array of joints.
    * </p>
    *
    * @param joints           the array of joints to copy the positions from. Not modified.
    * @param messagesToUpdate the list of messages to update with the joint positions. Modified.
    */
   public static void copyJointDesiredPositions(OneDoFJointReadOnly[] joints, List<KinematicsToolboxOneDoFJointMessage> messagesToUpdate)
   {
      for (int i = 0; i < joints.length; i++)
      {
         messagesToUpdate.get(i).setDesiredPosition(joints[i].getQ());
      }
   }

   public static void updateFullRobotModel(RobotConfigurationData robotConfigurationData, FullHumanoidRobotModel fullRobotModelToUpdate)
   {
      OneDoFJointBasics[] joints = FullRobotModelUtils.getAllJointsExcludingHands(fullRobotModelToUpdate);
      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModelToUpdate.getForceSensorDefinitions();
      IMUDefinition[] imuDefinitions = fullRobotModelToUpdate.getIMUDefinitions();
      int jointNameHash = RobotConfigurationDataFactory.calculateJointNameHash(joints, forceSensorDefinitions, imuDefinitions);

      if (robotConfigurationData.getJointNameHash() != jointNameHash)
         throw new RuntimeException("Joint names do not match for RobotConfigurationData");

      for (int jointIndex = 0; jointIndex < joints.length; jointIndex++)
      {
         joints[jointIndex].setQ(robotConfigurationData.getJointAngles().get(jointIndex));
      }

      Pose3DBasics rootJointPose = fullRobotModelToUpdate.getRootJoint().getJointPose();
      rootJointPose.set(robotConfigurationData.getRootPosition(), robotConfigurationData.getRootOrientation());
   }

   public static void copyRobotState(FullHumanoidRobotModel source, FullHumanoidRobotModel destination, JointStateType stateSelection)
   {
      MultiBodySystemTools.copyJointsState(Collections.singletonList(source.getRootJoint()),
                                           Collections.singletonList(destination.getRootJoint()),
                                           stateSelection);
      MultiBodySystemTools.copyJointsState(Arrays.asList(source.getOneDoFJoints()), Arrays.asList(destination.getOneDoFJoints()), stateSelection);
   }

   public static void computeSpatialVelocity(double dt,
                                             FramePose3DReadOnly previousPose,
                                             FramePose3DReadOnly currentPose,
                                             FixedFrameSpatialVectorBasics spatialVelocityToPack)
   {
      computeLinearVelocity(dt, previousPose.getPosition(), currentPose.getPosition(), spatialVelocityToPack.getLinearPart());
      computeAngularVelocity(dt, previousPose.getOrientation(), currentPose.getOrientation(), spatialVelocityToPack.getAngularPart());
   }

   public static void computeLinearVelocity(double dt,
                                            FramePoint3DReadOnly previousPosition,
                                            FramePoint3DReadOnly currentPosition,
                                            FixedFrameVector3DBasics linearVelocityToPack)
   {
      linearVelocityToPack.sub(currentPosition, previousPosition);
      linearVelocityToPack.scale(1.0 / dt);
   }

   /**
    * Computes the angular velocity from finite difference. The result is the angular velocity
    * expressed in the local frame described by {@code currentOrientation}.
    */
   public static void computeAngularVelocity(double dt,
                                             FrameQuaternionReadOnly previousOrientation,
                                             FrameQuaternionReadOnly currentOrientation,
                                             FixedFrameVector3DBasics angularVelocityToPack)
   {
      previousOrientation.checkReferenceFrameMatch(currentOrientation);
      previousOrientation.checkReferenceFrameMatch(angularVelocityToPack);

      double qDot_x = currentOrientation.getX() - previousOrientation.getX();
      double qDot_y = currentOrientation.getY() - previousOrientation.getY();
      double qDot_z = currentOrientation.getZ() - previousOrientation.getZ();
      double qDot_s = currentOrientation.getS() - previousOrientation.getS();

      double qx = -currentOrientation.getX();
      double qy = -currentOrientation.getY();
      double qz = -currentOrientation.getZ();
      double qs = currentOrientation.getS();

      double wx = qs * qDot_x + qx * qDot_s + qy * qDot_z - qz * qDot_y;
      double wy = qs * qDot_y - qx * qDot_z + qy * qDot_s + qz * qDot_x;
      double wz = qs * qDot_z + qx * qDot_y - qy * qDot_x + qz * qDot_s;
      angularVelocityToPack.set(wx, wy, wz);
      angularVelocityToPack.scale(2.0 / dt);
   }

   public static void integrateLinearVelocity(double dt,
                                              FramePoint3DReadOnly initialPosition,
                                              FrameVector3DReadOnly linearVelocity,
                                              FixedFramePoint3DBasics finalPosition)
   {
      finalPosition.scaleAdd(dt, linearVelocity, initialPosition);
   }

   public static void integrateAngularVelocity(double dt,
                                               FrameQuaternionReadOnly initialOrientation,
                                               FrameVector3DReadOnly angularVelocity,
                                               boolean isAngularVelocityLocal,
                                               FixedFrameQuaternionBasics finalOrientation)
   {
      double qInit_x = initialOrientation.getX();
      double qInit_y = initialOrientation.getY();
      double qInit_z = initialOrientation.getZ();
      double qInit_s = initialOrientation.getS();

      double x = angularVelocity.getX() * dt;
      double y = angularVelocity.getY() * dt;
      double z = angularVelocity.getZ() * dt;
      finalOrientation.setRotationVector(x, y, z);

      double qInt_x = finalOrientation.getX();
      double qInt_y = finalOrientation.getY();
      double qInt_z = finalOrientation.getZ();
      double qInt_s = finalOrientation.getS();

      if (isAngularVelocityLocal)
         QuaternionTools.multiplyImpl(qInit_x, qInit_y, qInit_z, qInit_s, false, qInt_x, qInt_y, qInt_z, qInt_s, false, finalOrientation);
      else
         QuaternionTools.multiplyImpl(qInt_x, qInt_y, qInt_z, qInt_s, false, qInit_x, qInit_y, qInit_z, qInit_s, false, finalOrientation);
   }

   /**
    * Computes the minimum velocity that can be applied to the joint to reach the minimum joint position limit.
    *
    * @param qMin the minimum joint position limit.
    * @param q    the current joint position.
    * @param dt   the time step.
    * @return the minimum velocity that can be applied to the joint to reach the minimum joint position limit.
    */
   public static double computeJointMinVelocity(double qMin, double q, double dt)
   {
      if (!Double.isInfinite(qMin))
      {
         return (qMin - q) / dt;
      }
      else
      {
         return Double.NEGATIVE_INFINITY;
      }
   }

   /**
    * Computes the maximum velocity that can be applied to the joint to reach the maximum joint position limit.
    *
    * @param qMax the maximum joint position limit.
    * @param q    the current joint position.
    * @param dt   the time step.
    * @return the maximum velocity that can be applied to the joint to reach the maximum joint position limit.
    */
   public static double computeJointMaxVelocity(double qMax, double q, double dt)
   {
      if (!Double.isInfinite(qMax))
      {
         return (qMax - q) / dt;
      }
      else
      {
         return Double.POSITIVE_INFINITY;
      }
   }

   /**
    * Computes the minimum acceleration that can be applied to the joint to reach the minimum joint position limit.
    *
    * @param qMin the minimum joint position limit.
    * @param q    the current joint position.
    * @param qDot the current joint velocity.
    * @param dt   the time step.
    * @return the minimum acceleration that can be applied to the joint to reach the minimum joint position limit.
    */
   public static double computeJointMinAcceleration(double qMin, double q, double qDot, double dt)
   {
      return computeJointMinAcceleration(qMin, Double.NEGATIVE_INFINITY, q, qDot, dt);
   }

   /**
    * Computes the minimum acceleration that can be applied to the joint to reach the minimum joint position limit.
    *
    * @param qMin    the minimum joint position limit.
    * @param qDotMin the minimum joint velocity limit.
    * @param q       the current joint position.
    * @param qDot    the current joint velocity.
    * @param dt      the time step.
    * @return the minimum acceleration that can be applied to the joint to reach the minimum joint position limit.
    */
   public static double computeJointMinAcceleration(double qMin, double qDotMin, double q, double qDot, double dt)
   {
      double qDotMinPositionLimit;
      if (Double.isFinite(qMin))
         qDotMinPositionLimit = (qMin - q) / dt;
      else
         qDotMinPositionLimit = Double.NEGATIVE_INFINITY;

      if (Double.isFinite(qDotMin))
         qDotMin = Math.max(qDotMin, qDotMinPositionLimit);
      else
         qDotMin = qDotMinPositionLimit;

      if (Double.isFinite(qDotMin))
         return 2.0 * (qDotMin - qDot) / dt;
      else
         return Double.NEGATIVE_INFINITY;
   }

   /**
    * Computes the maximum acceleration that can be applied to the joint to reach the maximum joint position limit.
    *
    * @param qMax the maximum joint position limit.
    * @param q    the current joint position.
    * @param qDot the current joint velocity.
    * @param dt   the time step.
    * @return the maximum acceleration that can be applied to the joint to reach the maximum joint position limit.
    */
   public static double computeJointMaxAcceleration(double qMax, double q, double qDot, double dt)
   {
      return computeJointMaxAcceleration(qMax, Double.POSITIVE_INFINITY, q, qDot, dt);
   }

   /**
    * Computes the maximum acceleration that can be applied to the joint to reach the maximum joint position limit.
    *
    * @param qMax    the maximum joint position limit.
    * @param qDotMax the maximum joint velocity limit.
    * @param q       the current joint position.
    * @param qDot    the current joint velocity.
    * @param dt      the time step.
    * @return the maximum acceleration that can be applied to the joint to reach the maximum joint position limit.
    */
   public static double computeJointMaxAcceleration(double qMax, double qDotMax, double q, double qDot, double dt)
   {
      double qDotMaxPositionLimit;
      if (Double.isFinite(qMax))
         qDotMaxPositionLimit = (qMax - q) / dt;
      else
         qDotMaxPositionLimit = Double.POSITIVE_INFINITY;

      if (Double.isFinite(qDotMax))
         qDotMax = Math.min(qDotMax, qDotMaxPositionLimit);
      else
         qDotMax = qDotMaxPositionLimit;

      if (Double.isFinite(qDotMax))
         return 2.0 * (qDotMax - qDot) / dt;
      else
         return Double.POSITIVE_INFINITY;
   }
}
