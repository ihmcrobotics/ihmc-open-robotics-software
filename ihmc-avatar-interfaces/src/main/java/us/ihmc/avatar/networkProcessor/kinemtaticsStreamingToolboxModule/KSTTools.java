package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import java.util.Arrays;
import java.util.Collections;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class KSTTools
{
   private final CommandInputManager commandInputManager;
   private final CommandInputManager ikCommandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final FullHumanoidRobotModel desiredFullRobotModel;
   private final FullHumanoidRobotModelFactory fullRobotModelFactory;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final YoVariableRegistry registry;

   private final FullHumanoidRobotModel currentFullRobotModel;
   private final FloatingJointBasics currentRootJoint;
   private final OneDoFJointBasics[] currentOneDoFJoint;

   private final HumanoidKinematicsToolboxController ikController;
   private final KinematicsToolboxOutputConverter outputConverter;
   private final WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
   private final YoDouble streamIntegrationDuration;

   private final ConcurrentCopier<RobotConfigurationData> concurrentRobotConfigurationDataCopier = new ConcurrentCopier<>(RobotConfigurationData::new);
   private boolean hasRobotDataConfiguration = false;
   private final RobotConfigurationData robotConfigurationDataInternal = new RobotConfigurationData();
   private final ConcurrentCopier<CapturabilityBasedStatus> concurrentCapturabilityBasedStatusCopier = new ConcurrentCopier<>(CapturabilityBasedStatus::new);
   private boolean hasCapturabilityBasedStatus = false;
   private final CapturabilityBasedStatus capturabilityBasedStatusInternal = new CapturabilityBasedStatus();

   private final double walkingControllerPeriod;
   private final double toolboxControllerPeriod;
   private final YoDouble walkingControllerMonotonicTime, walkingControllerWallTime;

   private long currentMessageId = 1L;

   public KSTTools(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager, FullHumanoidRobotModel desiredFullRobotModel,
                   FullHumanoidRobotModelFactory fullRobotModelFactory, double walkingControllerPeriod, double toolboxControllerPeriod,
                   YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry registry)
   {
      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;
      this.desiredFullRobotModel = desiredFullRobotModel;
      this.fullRobotModelFactory = fullRobotModelFactory;
      this.walkingControllerPeriod = walkingControllerPeriod;
      this.toolboxControllerPeriod = toolboxControllerPeriod;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.registry = registry;

      walkingControllerMonotonicTime = new YoDouble("walkingControllerMonotonicTime", registry);
      walkingControllerWallTime = new YoDouble("walkingControllerWallTime", registry);

      currentFullRobotModel = fullRobotModelFactory.createFullRobotModel();
      currentRootJoint = currentFullRobotModel.getRootJoint();
      currentOneDoFJoint = FullRobotModelUtils.getAllJointsExcludingHands(currentFullRobotModel);

      ikCommandInputManager = new CommandInputManager(HumanoidKinematicsToolboxController.class.getSimpleName(), KinematicsToolboxModule.supportedCommands());
      ikController = new HumanoidKinematicsToolboxController(ikCommandInputManager,
                                                             statusOutputManager,
                                                             desiredFullRobotModel,
                                                             fullRobotModelFactory,
                                                             toolboxControllerPeriod,
                                                             yoGraphicsListRegistry,
                                                             registry);
      ikCommandInputManager.registerConversionHelper(new KinematicsToolboxCommandConverter(desiredFullRobotModel));

      ikController.setPreserveUserCommandHistory(false);

      outputConverter = new KinematicsToolboxOutputConverter(fullRobotModelFactory);

      streamIntegrationDuration = new YoDouble("streamIntegrationDuration", registry);
      streamIntegrationDuration.set(0.2);
   }

   public void update()
   {
      RobotConfigurationData newRobotConfigurationData = concurrentRobotConfigurationDataCopier.getCopyForReading();
      if (newRobotConfigurationData != null)
      {
         robotConfigurationDataInternal.set(newRobotConfigurationData);
         hasRobotDataConfiguration = true;
      }

      CapturabilityBasedStatus newCapturabilityBasedStatus = concurrentCapturabilityBasedStatusCopier.getCopyForReading();
      if (newCapturabilityBasedStatus != null)
      {
         capturabilityBasedStatusInternal.set(newCapturabilityBasedStatus);
         hasCapturabilityBasedStatus = true;
      }

      if (hasRobotDataConfiguration)
      {
         walkingControllerMonotonicTime.set(Conversions.nanosecondsToSeconds(robotConfigurationDataInternal.getMonotonicTime()));
         walkingControllerWallTime.set(Conversions.nanosecondsToSeconds(robotConfigurationDataInternal.getWallTime()));

         for (int jointIndex = 0; jointIndex < currentOneDoFJoint.length; jointIndex++)
         {
            currentOneDoFJoint[jointIndex].setQ(robotConfigurationDataInternal.getJointAngles().get(jointIndex));
         }

         Pose3DBasics rootJointPose = currentRootJoint.getJointPose();
         rootJointPose.set(robotConfigurationDataInternal.getRootTranslation(), robotConfigurationDataInternal.getRootOrientation());
         currentFullRobotModel.updateFrames();
      }
   }

   public void getCurrentState(KinematicsToolboxOutputStatus currentStateToPack)
   {
      MessageTools.packDesiredJointState(currentStateToPack, currentFullRobotModel.getRootJoint(), currentOneDoFJoint);
   }

   public void getCurrentState(YoKinematicsToolboxOutputStatus currentStateToPack)
   {
      KinematicsToolboxOutputStatus status = currentStateToPack.getStatus();
      MessageTools.packDesiredJointState(status, currentFullRobotModel.getRootJoint(), currentOneDoFJoint);
      currentStateToPack.set(status);
   }

   private KinematicsStreamingToolboxInputCommand latestInput = null;
   private boolean hasNewInputCommand = false;

   public KinematicsStreamingToolboxInputCommand pollInputCommand()
   {
      hasNewInputCommand = commandInputManager.isNewCommandAvailable(KinematicsStreamingToolboxInputCommand.class);

      if (hasNewInputCommand)
         latestInput = commandInputManager.pollNewestCommand(KinematicsStreamingToolboxInputCommand.class);

      return latestInput;
   }

   public boolean hasNewInputCommand()
   {
      return hasNewInputCommand;
   }

   public WholeBodyTrajectoryMessage setupStreamingMessage(KinematicsToolboxOutputStatus solutionToConvert)
   {
      outputConverter.updateFullRobotModel(solutionToConvert);
      outputConverter.setMessageToCreate(wholeBodyTrajectoryMessage);
      outputConverter.setTrajectoryTime(0.0);
      outputConverter.setEnableVelocity(true);

      outputConverter.computeHandTrajectoryMessages();
      outputConverter.computeArmTrajectoryMessages();
      outputConverter.computeNeckTrajectoryMessage();
      outputConverter.computeChestTrajectoryMessage(ReferenceFrame.getWorldFrame());
      outputConverter.computePelvisTrajectoryMessage();

      wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage().setEnableUserPelvisControl(true);
      HumanoidMessageTools.configureForStreaming(wholeBodyTrajectoryMessage, streamIntegrationDuration.getValue());
      setAllIDs(wholeBodyTrajectoryMessage, currentMessageId++);
      return wholeBodyTrajectoryMessage;
   }

   public WholeBodyTrajectoryMessage setupFinalizeStreamingMessage(KinematicsToolboxOutputStatus solutionToConvert)
   {
      outputConverter.updateFullRobotModel(solutionToConvert);
      outputConverter.setMessageToCreate(wholeBodyTrajectoryMessage);
      outputConverter.setTrajectoryTime(0.5);

      outputConverter.computeHandTrajectoryMessages();
      outputConverter.computeArmTrajectoryMessages();
      outputConverter.computeNeckTrajectoryMessage();
      outputConverter.computeChestTrajectoryMessage();
      outputConverter.computePelvisTrajectoryMessage();

      HumanoidMessageTools.configureForOverriding(wholeBodyTrajectoryMessage);
      setAllIDs(wholeBodyTrajectoryMessage, currentMessageId++);

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

   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      concurrentRobotConfigurationDataCopier.getCopyForWriting().set(newConfigurationData);
      concurrentRobotConfigurationDataCopier.commit();
      ikController.updateRobotConfigurationData(newConfigurationData);
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus newStatus)
   {
      concurrentCapturabilityBasedStatusCopier.getCopyForWriting().set(newStatus);
      concurrentCapturabilityBasedStatusCopier.commit();
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

   public YoVariableRegistry getRegistry()
   {
      return registry;
   }

   public KinematicsToolboxOutputConverter getOutputConverter()
   {
      return outputConverter;
   }

   public FullHumanoidRobotModel getCurrentFullRobotModel()
   {
      return currentFullRobotModel;
   }

   public HumanoidKinematicsToolboxController getIKController()
   {
      return ikController;
   }

   public RobotConfigurationData getRobotConfigurationData()
   {
      return hasRobotDataConfiguration ? robotConfigurationDataInternal : null;
   }

   public CapturabilityBasedStatus getCapturabilityBasedStatus()
   {
      return hasCapturabilityBasedStatus ? capturabilityBasedStatusInternal : null;
   }

   public double getWalkingControllerPeriod()
   {
      return walkingControllerPeriod;
   }

   public double getToolboxControllerPeriod()
   {
      return toolboxControllerPeriod;
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
      rootJointPose.set(robotConfigurationData.getRootTranslation(), robotConfigurationData.getRootOrientation());
   }

   public static void copyRobotState(FullHumanoidRobotModel source, FullHumanoidRobotModel destination, JointStateType stateSelection)
   {
      MultiBodySystemTools.copyJointsState(Collections.singletonList(source.getRootJoint()),
                                           Collections.singletonList(destination.getRootJoint()),
                                           stateSelection);
      MultiBodySystemTools.copyJointsState(Arrays.asList(source.getOneDoFJoints()), Arrays.asList(destination.getOneDoFJoints()), stateSelection);
   }
}
