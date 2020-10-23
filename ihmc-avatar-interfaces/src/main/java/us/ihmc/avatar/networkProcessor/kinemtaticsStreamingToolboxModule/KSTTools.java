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
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxOutputConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.KinematicsToolboxOutputConverter;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationDataFactory;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class KSTTools
{
   private final CommandInputManager commandInputManager;
   private final CommandInputManager ikCommandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final FullHumanoidRobotModel desiredFullRobotModel;
   private final FullHumanoidRobotModelFactory fullRobotModelFactory;
   private final DoubleProvider time;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final YoRegistry registry;

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

   private final YoBoolean hasNewInputCommand, hasPreviousInput;
   private final YoDouble latestInputReceivedTime, previousInputReceivedTime;
   private KinematicsStreamingToolboxInputCommand latestInput = null;
   private KinematicsStreamingToolboxInputCommand previousInput = new KinematicsStreamingToolboxInputCommand();

   private final KinematicsStreamingToolboxOutputConfigurationCommand outputConfiguration = new KinematicsStreamingToolboxOutputConfigurationCommand();
   private final YoBoolean isNeckJointspaceOutputEnabled;
   private final YoBoolean isChestTaskspaceOutputEnabled;
   private final YoBoolean isPelvisTaskspaceOutputEnabled;
   private final SideDependentList<YoBoolean> areHandTaskspaceOutputsEnabled = new SideDependentList<>();
   private final SideDependentList<YoBoolean> areArmJointspaceOutputsEnabled = new SideDependentList<>();

   public KSTTools(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager, FullHumanoidRobotModel desiredFullRobotModel,
                   FullHumanoidRobotModelFactory fullRobotModelFactory, double walkingControllerPeriod, double toolboxControllerPeriod, DoubleProvider time,
                   YoGraphicsListRegistry yoGraphicsListRegistry, YoRegistry registry)
   {
      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;
      this.desiredFullRobotModel = desiredFullRobotModel;
      this.fullRobotModelFactory = fullRobotModelFactory;
      this.walkingControllerPeriod = walkingControllerPeriod;
      this.toolboxControllerPeriod = toolboxControllerPeriod;
      this.time = time;
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
      ikController.minimizeAngularMomentum(true);

      outputConverter = new KinematicsToolboxOutputConverter(fullRobotModelFactory);

      streamIntegrationDuration = new YoDouble("streamIntegrationDuration", registry);
      streamIntegrationDuration.set(0.2);

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
   }

   public void update()
   {
      if (commandInputManager.isNewCommandAvailable(KinematicsStreamingToolboxOutputConfigurationCommand.class))
      {
         outputConfiguration.set(commandInputManager.pollNewestCommand(KinematicsStreamingToolboxOutputConfigurationCommand.class));
      }

      isNeckJointspaceOutputEnabled.set(outputConfiguration.isNeckJointspaceEnabled());
      isChestTaskspaceOutputEnabled.set(outputConfiguration.isChestTaskspaceEnabled());
      isPelvisTaskspaceOutputEnabled.set(outputConfiguration.isPelvisTaskspaceEnabled());

      for (RobotSide robotSide : RobotSide.values)
      {
         areHandTaskspaceOutputsEnabled.get(robotSide).set(outputConfiguration.isHandTaskspaceEnabled(robotSide));
         areArmJointspaceOutputsEnabled.get(robotSide).set(outputConfiguration.isArmJointspaceEnabled(robotSide));
      }

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

   public double getTime()
   {
      return time.getValue();
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

   public void pollInputCommand()
   {
      hasNewInputCommand.set(commandInputManager.isNewCommandAvailable(KinematicsStreamingToolboxInputCommand.class));

      if (hasNewInputCommand.getValue())
      {
         if (latestInput != null)
         {
            previousInput.set(latestInput);
            previousInputReceivedTime.set(latestInputReceivedTime.getValue());
            hasPreviousInput.set(true);
         }
         latestInput = commandInputManager.pollNewestCommand(KinematicsStreamingToolboxInputCommand.class);

         if (latestInput.getTimestamp() <= 0)
            latestInput.setTimestamp(Conversions.secondsToNanoseconds(time.getValue()));

         latestInputReceivedTime.set(time.getValue());
      }
   }
   
   public boolean hasNewInputCommand()
   {
      return hasNewInputCommand.getValue();
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

   public WholeBodyTrajectoryMessage setupStreamingMessage(KinematicsToolboxOutputStatus solutionToConvert)
   {
      HumanoidMessageTools.resetWholeBodyTrajectoryToolboxMessage(wholeBodyTrajectoryMessage);
      outputConverter.updateFullRobotModel(solutionToConvert);
      outputConverter.setMessageToCreate(wholeBodyTrajectoryMessage);
      outputConverter.setTrajectoryTime(0.0);
      outputConverter.setEnableVelocity(true);

      for (RobotSide robotSide : RobotSide.values)
      {
         if (areHandTaskspaceOutputsEnabled.get(robotSide).getValue())
            outputConverter.computeHandTrajectoryMessage(robotSide);

         if (areArmJointspaceOutputsEnabled.get(robotSide).getValue())
            outputConverter.computeArmTrajectoryMessage(robotSide);
      }

      if (isNeckJointspaceOutputEnabled.getValue())
         outputConverter.computeNeckTrajectoryMessage();
      if (isChestTaskspaceOutputEnabled.getValue())
         outputConverter.computeChestTrajectoryMessage(ReferenceFrame.getWorldFrame());
      if (isPelvisTaskspaceOutputEnabled.getValue())
         outputConverter.computePelvisTrajectoryMessage();

      wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage().setEnableUserPelvisControl(true);
      HumanoidMessageTools.configureForStreaming(wholeBodyTrajectoryMessage, streamIntegrationDuration.getValue(), Conversions.secondsToNanoseconds(time.getValue()));
      setAllIDs(wholeBodyTrajectoryMessage, currentMessageId++);
      return wholeBodyTrajectoryMessage;
   }

   public WholeBodyTrajectoryMessage setupFinalizeStreamingMessage(KinematicsToolboxOutputStatus solutionToConvert)
   {
      HumanoidMessageTools.resetWholeBodyTrajectoryToolboxMessage(wholeBodyTrajectoryMessage);
      outputConverter.updateFullRobotModel(solutionToConvert);
      outputConverter.setMessageToCreate(wholeBodyTrajectoryMessage);
      outputConverter.setTrajectoryTime(0.5);

      for (RobotSide robotSide : RobotSide.values)
      {
         if (areHandTaskspaceOutputsEnabled.get(robotSide).getValue())
            outputConverter.computeHandTrajectoryMessage(robotSide);

         if (areArmJointspaceOutputsEnabled.get(robotSide).getValue())
            outputConverter.computeArmTrajectoryMessage(robotSide);
      }

      if (isNeckJointspaceOutputEnabled.getValue())
         outputConverter.computeNeckTrajectoryMessage();
      if (isChestTaskspaceOutputEnabled.getValue())
         outputConverter.computeChestTrajectoryMessage();
      if (isPelvisTaskspaceOutputEnabled.getValue())
         outputConverter.computePelvisTrajectoryMessage();

      wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage().setEnableUserPelvisControl(true);
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

   public YoRegistry getRegistry()
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

   public static void computeLinearVelocity(double dt, FramePoint3DReadOnly previousPosition, FramePoint3DReadOnly currentPosition,
                                            FixedFrameVector3DBasics linearVelocityToPack)
   {
      linearVelocityToPack.sub(currentPosition, previousPosition);
      linearVelocityToPack.scale(1.0 / dt);
   }

   /**
    * Computes the angular velocity from finite difference. The result is the angular velocity
    * expressed in the local frame described by {@code currentOrientation}.
    */
   public static void computeAngularVelocity(double dt, FrameQuaternionReadOnly previousOrientation, FrameQuaternionReadOnly currentOrientation,
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

   public static void integrateLinearVelocity(double dt, FramePoint3DReadOnly initialPosition, FrameVector3DReadOnly linearVelocity,
                                              FixedFramePoint3DBasics finalPosition)
   {
      finalPosition.scaleAdd(dt, linearVelocity, initialPosition);
   }

   public static void integrateAngularVelocity(double dt, FrameQuaternionReadOnly initialOrientation, FrameVector3DReadOnly angularVelocity,
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

      double qFinal_x = qInit_s * qInt_x + qInit_x * qInt_s + qInit_y * qInt_z - qInit_z * qInt_y;
      double qFinal_y = qInit_s * qInt_y - qInit_x * qInt_z + qInit_y * qInt_s + qInit_z * qInt_x;
      double qFinal_z = qInit_s * qInt_z + qInit_x * qInt_y - qInit_y * qInt_x + qInit_z * qInt_s;
      double qFinal_s = qInit_s * qInt_s - qInit_x * qInt_x - qInit_y * qInt_y - qInit_z * qInt_z;
      finalOrientation.set(qFinal_x, qFinal_y, qFinal_z, qFinal_s);
   }
}
