package us.ihmc.avatar.networkProcessor.kinemtaticsStreamingToolboxModule;

import java.util.Arrays;
import java.util.Collections;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxCommandConverter;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
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
   private final OneDoFJointBasics[] currentOneDoFJoint;

   private final HumanoidKinematicsToolboxController ikController;
   private final KinematicsToolboxOutputConverter outputConverter;
   private final WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage = new WholeBodyTrajectoryMessage();
   private final YoDouble streamIntegrationDuration;

   private final AtomicReference<RobotConfigurationData> latestRobotConfigurationDataReference = new AtomicReference<>(null);
   private final AtomicReference<CapturabilityBasedStatus> latestCapturabilityBasedStatusReference = new AtomicReference<>(null);
   private final double walkingControllerPeriod;
   private final double toolboxControllerPeriod;

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

      currentFullRobotModel = fullRobotModelFactory.createFullRobotModel();
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
      streamIntegrationDuration.set(0.1);
   }

   public void update()
   {
      if (getRobotConfigurationData() != null)
      {
         updateFullRobotModel(getRobotConfigurationData(), currentFullRobotModel);
         currentFullRobotModel.updateFrames();
      }
   }

   public void getCurrentState(KinematicsToolboxOutputStatus currentStateToPack)
   {
      MessageTools.packDesiredJointState(currentStateToPack, currentFullRobotModel.getRootJoint(), currentOneDoFJoint);
   }

   private KinematicsStreamingToolboxInputCommand latestInput = null;

   public KinematicsStreamingToolboxInputCommand pollInputCommand()
   {
      if (commandInputManager.isNewCommandAvailable(KinematicsStreamingToolboxInputCommand.class))
      {
         latestInput = commandInputManager.pollNewestCommand(KinematicsStreamingToolboxInputCommand.class);
      }
      return latestInput;
   }

   public WholeBodyTrajectoryMessage convertIKOutput()
   {
      return setupWholeBodyTrajectoryMessage(ikController.getSolution());
   }

   public WholeBodyTrajectoryMessage setupWholeBodyTrajectoryMessage(KinematicsToolboxOutputStatus solutionToConvert)
   {
      outputConverter.updateFullRobotModel(solutionToConvert);
      outputConverter.setMessageToCreate(wholeBodyTrajectoryMessage);
      outputConverter.setTrajectoryTime(0.0);
      outputConverter.setEnableVelocity(true);

      outputConverter.computeHandTrajectoryMessages();
      outputConverter.computeArmTrajectoryMessages();
      outputConverter.computeNeckTrajectoryMessage();
      outputConverter.computeChestTrajectoryMessage();
      outputConverter.computePelvisTrajectoryMessage();

      HumanoidMessageTools.configureForStreaming(wholeBodyTrajectoryMessage, streamIntegrationDuration.getValue());
      return wholeBodyTrajectoryMessage;
   }

   public void updateRobotConfigurationData(RobotConfigurationData newConfigurationData)
   {
      latestRobotConfigurationDataReference.set(newConfigurationData);
      ikController.updateRobotConfigurationData(newConfigurationData);
   }

   public void updateCapturabilityBasedStatus(CapturabilityBasedStatus newStatus)
   {
      latestCapturabilityBasedStatusReference.set(newStatus);
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
      return latestRobotConfigurationDataReference.get();
   }

   public CapturabilityBasedStatus getCapturabilityBasedStatus()
   {
      return latestCapturabilityBasedStatusReference.get();
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
