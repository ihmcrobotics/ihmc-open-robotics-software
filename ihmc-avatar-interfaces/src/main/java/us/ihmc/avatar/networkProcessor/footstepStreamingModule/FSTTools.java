package us.ihmc.avatar.networkProcessor.footstepStreamingModule;

import controller_msgs.msg.dds.RobotConfigurationData;
import controller_msgs.msg.dds.WholeBodyStreamingMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import toolbox_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage;
import us.ihmc.avatar.networkProcessor.footstepStreamingModule.FootstepStreamingToolboxController.FootstepStreamingMessagePublisher;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxParameters;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.output.KSTOutputDataBasics;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.output.KSTOutputDataReadOnly;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.HumanoidKinematicsToolboxController;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxController.RobotConfigurationDataBasedUpdater;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxModule;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsStreamingToolboxAPI.KinematicsStreamingToolboxInputCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxCenterOfMassCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.List;

public class FSTTools
{
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusOutputManager;
   private final FootstepStreamingToolboxParameters parameters;
   private final DoubleProvider time;
   private final YoGraphicsListRegistry yoGraphicsListRegistry;
   private final YoRegistry registry;

   private final double toolboxControllerPeriod;
   private final YoDouble walkingControllerMonotonicTime, walkingControllerWallTime;

   private final YoLong currentMessageId;

   private final YoBoolean hasNewInputCommand, hasPreviousInput;
   private final YoDouble latestInputReceivedTime, previousInputReceivedTime;
   private KinematicsStreamingToolboxInputCommand latestInput = null;
   private KinematicsStreamingToolboxInputCommand previousInput = null;

   private final YoLong latestInputTimestampSource;
   private final YoDouble latestInputTimeSource;

   private FootstepStreamingMessagePublisher footstepStreamingMessagePublisher = m ->
   {
   };

   public FSTTools(CommandInputManager commandInputManager,
                   StatusMessageOutputManager statusOutputManager,
                   FootstepStreamingToolboxParameters parameters,
                   DoubleProvider time,
                   YoGraphicsListRegistry yoGraphicsListRegistry,
                   YoRegistry registry)
   {
      this.commandInputManager = commandInputManager;
      this.statusOutputManager = statusOutputManager;
      this.parameters = parameters;
      this.toolboxControllerPeriod = parameters.getToolboxUpdatePeriod();
      this.time = time;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.registry = registry;

      walkingControllerMonotonicTime = new YoDouble("walkingControllerMonotonicTime", registry);
      walkingControllerWallTime = new YoDouble("walkingControllerWallTime", registry);

      currentMessageId = new YoLong("currentMessageId", registry);
      currentMessageId.set(1L);

      hasNewInputCommand = new YoBoolean("hasNewInputCommand", registry);
      hasPreviousInput = new YoBoolean("hasPreviousInput", registry);
      latestInputReceivedTime = new YoDouble("latestInputReceivedTime", registry);
      previousInputReceivedTime = new YoDouble("previousInputReceivedTime", registry);
      flushInputCommands();

      latestInputTimestampSource = new YoLong("latestInputTimestampSource", registry);
      latestInputTimeSource = new YoDouble("latestInputTimeSource", registry);
   }

   public void update()
   {
      if (commandInputManager.isNewCommandAvailable(FootstepStreamingToolboxInputCommand.class))
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

         for (int i = latestInput.getNumberOfInputs() - 1; i >= 0; i--)
         {
            KinematicsToolboxRigidBodyCommand latestEndEffectorInput = latestInput.getInput(i);
            RigidBodyBasics endEffector = latestEndEffectorInput.getEndEffector();
            KinematicsToolboxRigidBodyCommand previousEndEffectorInput = hasPreviousInput.getValue() ? previousInput.getInputFor(endEffector) : null;
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

   public FootstepStreamingToolboxParameters getParameters()
   {
      return parameters;
   }

   public double getTime()
   {
      return time.getValue();
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

   public void setFootstepMessagerPublisher(FootstepStreamingMessagePublisher outputPublisher)
   {
      this.footstepStreamingMessagePublisher = outputPublisher;
   }

   public void streamToController(KSTOutputDataReadOnly outputToPublish, boolean finalizeTrajectory)
   {
      if (streamingMessagePublisher == null || !useStreamingPublisher.getValue())
         footstepStreamingMessagePublisher.publish(setupTrajectoryMessage(outputToPublish));
      else
         streamingMessagePublisher.publish(setupStreamingMessage(outputToPublish));
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

   public CommandInputManager getCommandInputManager()
   {
      return commandInputManager;
   }

   public StatusMessageOutputManager getStatusOutputManager()
   {
      return statusOutputManager;
   }

   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public double getToolboxControllerPeriod()
   {
      return toolboxControllerPeriod;
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
}
