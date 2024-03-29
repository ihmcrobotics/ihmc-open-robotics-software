package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories;

import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateArmDesiredAccelerationsMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateArmTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateChestTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateFootLoadBearingMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateFootTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateFootstepDataListMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateGoHomeMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateHandTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateHeadTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateNeckDesiredAccelerationsMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateNeckTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validatePelvisHeightTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validatePelvisOrientationTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validatePelvisTrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateSpineDesiredAccelerationsMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateSpineTrajectoryMessage;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import controller_msgs.msg.dds.*;
import ihmc_common_msgs.msg.dds.TextToSpeechPacket;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber.MessageValidator;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.MessageCollector.MessageIDExtractor;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;
import us.ihmc.humanoidRobotics.communication.directionalControlToolboxAPI.DirectionalControlInputCommand;
import us.ihmc.humanoidRobotics.communication.fastWalkingAPI.FastWalkingGaitParametersCommand;

public class ControllerAPIDefinition
{
   private static final List<Class<? extends Command<?, ?>>> controllerSupportedCommands;
   private static final List<Class<? extends Settable<?>>> controllerSupportedStatusMessages;

   static
   {
      List<Class<? extends Command<?, ?>>> commands = new ArrayList<>();

      /** Commands supported by bipedal walking controller {@link WalkingControllerState} */
      commands.add(ArmTrajectoryCommand.class);
      commands.add(HandTrajectoryCommand.class);
      commands.add(LegTrajectoryCommand.class);
      commands.add(FootTrajectoryCommand.class);
      commands.add(HeadTrajectoryCommand.class);
      commands.add(NeckTrajectoryCommand.class);
      commands.add(NeckDesiredAccelerationsCommand.class);
      commands.add(ChestTrajectoryCommand.class);
      commands.add(SpineTrajectoryCommand.class);
      commands.add(PelvisTrajectoryCommand.class);
      commands.add(PelvisOrientationTrajectoryCommand.class);
      commands.add(PelvisHeightTrajectoryCommand.class);
      commands.add(StopAllTrajectoryCommand.class);
      commands.add(FootstepDataListCommand.class);
      commands.add(GoHomeCommand.class);
      commands.add(FootLoadBearingCommand.class);
      commands.add(ArmDesiredAccelerationsCommand.class);
      commands.add(AutomaticManipulationAbortCommand.class);
      commands.add(HighLevelControllerStateCommand.class);
      commands.add(AbortWalkingCommand.class);
      commands.add(PrepareForLocomotionCommand.class);
      commands.add(PauseWalkingCommand.class);
      commands.add(SpineDesiredAccelerationsCommand.class);
      commands.add(HandLoadBearingCommand.class);
      commands.add(HandHybridJointspaceTaskspaceTrajectoryCommand.class);
      commands.add(HeadHybridJointspaceTaskspaceTrajectoryCommand.class);
      commands.add(ChestHybridJointspaceTaskspaceTrajectoryCommand.class);
      commands.add(ClearDelayQueueCommand.class);
      commands.add(MomentumTrajectoryCommand.class);
      commands.add(CenterOfMassTrajectoryCommand.class);
      commands.add(HandWrenchTrajectoryCommand.class);

      /** Commands supported by the fast-walking controller, not in this repo */
      commands.add(DirectionalControlInputCommand.class);
      commands.add(FastWalkingGaitParametersCommand.class);

      /** Commands supported by multi-contact controller, not in this repo */
      commands.add(MultiContactTrajectoryCommand.class);
      commands.add(MultiContactTrajectorySequenceCommand.class);
      commands.add(MultiContactBalanceStatusCommand.class);
      commands.add(MultiContactTimedContactSequenceCommand.class);

      /** Commands supported by the Crocoddyl control state */
      commands.add(CrocoddylSolverTrajectoryCommand.class);

      /** Command supported by the joint-space controller {@link JointspacePositionControllerState} */
      commands.add(WholeBodyJointspaceTrajectoryCommand.class);

      controllerSupportedCommands = Collections.unmodifiableList(commands);

      List<Class<? extends Settable<?>>> statusMessages = new ArrayList<>();

      /** Statuses supported by bipedal walking controller {@link WalkingControllerState} */
      statusMessages.add(CapturabilityBasedStatus.class);
      statusMessages.add(FootstepStatusMessage.class);
      statusMessages.add(PlanOffsetStatus.class);
      statusMessages.add(WalkingStatusMessage.class);
      statusMessages.add(WalkingControllerFailureStatusMessage.class);
      statusMessages.add(ManipulationAbortedStatus.class);
      statusMessages.add(HighLevelStateChangeStatusMessage.class);
      statusMessages.add(TextToSpeechPacket.class);
      statusMessages.add(ControllerCrashNotificationPacket.class);
      statusMessages.add(JointspaceTrajectoryStatusMessage.class);
      statusMessages.add(TaskspaceTrajectoryStatusMessage.class);
      statusMessages.add(JointDesiredOutputMessage.class);
      statusMessages.add(RobotDesiredConfigurationData.class);
      statusMessages.add(FootstepQueueStatusMessage.class);
      statusMessages.add(QueuedFootstepStatusMessage.class);
      statusMessages.add(WrenchTrajectoryStatusMessage.class);

      /** Statuses supported by multi-contact controller, not in this repo */
      statusMessages.add(MultiContactBalanceStatus.class);
      statusMessages.add(MultiContactTrajectoryStatus.class);

      controllerSupportedStatusMessages = Collections.unmodifiableList(statusMessages);
   }

   public static List<Class<? extends Command<?, ?>>> getControllerSupportedCommands()
   {
      return controllerSupportedCommands;
   }

   public static List<Class<? extends Settable<?>>> getControllerSupportedStatusMessages()
   {
      return controllerSupportedStatusMessages;
   }

   public static MessageValidator createDefaultMessageValidation()
   {
      Map<Class<? extends Settable<?>>, MessageValidator> validators = new HashMap<>();
      validators.put(ArmTrajectoryMessage.class, message -> validateArmTrajectoryMessage((ArmTrajectoryMessage) message));
      validators.put(HandTrajectoryMessage.class, message -> validateHandTrajectoryMessage((HandTrajectoryMessage) message));
      validators.put(FootTrajectoryMessage.class, message -> validateFootTrajectoryMessage((FootTrajectoryMessage) message));
      validators.put(HeadTrajectoryMessage.class, message -> validateHeadTrajectoryMessage((HeadTrajectoryMessage) message));
      validators.put(NeckTrajectoryMessage.class, message -> validateNeckTrajectoryMessage((NeckTrajectoryMessage) message));
      validators.put(NeckDesiredAccelerationsMessage.class, message -> validateNeckDesiredAccelerationsMessage((NeckDesiredAccelerationsMessage) message));
      validators.put(ChestTrajectoryMessage.class, message -> validateChestTrajectoryMessage((ChestTrajectoryMessage) message));
      validators.put(SpineTrajectoryMessage.class, message -> validateSpineTrajectoryMessage((SpineTrajectoryMessage) message));
      validators.put(PelvisTrajectoryMessage.class, message -> validatePelvisTrajectoryMessage((PelvisTrajectoryMessage) message));
      validators.put(PelvisOrientationTrajectoryMessage.class,
                     message -> validatePelvisOrientationTrajectoryMessage((PelvisOrientationTrajectoryMessage) message));
      validators.put(PelvisHeightTrajectoryMessage.class, message -> validatePelvisHeightTrajectoryMessage((PelvisHeightTrajectoryMessage) message));
      validators.put(FootstepDataListMessage.class, message -> validateFootstepDataListMessage((FootstepDataListMessage) message));
      validators.put(GoHomeMessage.class, message -> validateGoHomeMessage((GoHomeMessage) message));
      validators.put(FootLoadBearingMessage.class, message -> validateFootLoadBearingMessage((FootLoadBearingMessage) message));
      validators.put(ArmDesiredAccelerationsMessage.class, message -> validateArmDesiredAccelerationsMessage((ArmDesiredAccelerationsMessage) message));
      validators.put(SpineDesiredAccelerationsMessage.class, message -> validateSpineDesiredAccelerationsMessage((SpineDesiredAccelerationsMessage) message));

      return new MessageValidator()
      {
         @Override
         public String validate(Object message)
         {
            MessageValidator validator = validators.get(message.getClass());
            return validator == null ? null : validator.validate(message);
         }
      };
   }

   public static MessageIDExtractor createDefaultMessageIDExtractor()
   {
      Map<Class<? extends Settable<?>>, MessageIDExtractor> extractors = new HashMap<>();
      extractors.put(ArmTrajectoryMessage.class, m -> ((ArmTrajectoryMessage) m).getSequenceId());
      extractors.put(HandTrajectoryMessage.class, m -> ((HandTrajectoryMessage) m).getSequenceId());
      extractors.put(FootTrajectoryMessage.class, m -> ((FootTrajectoryMessage) m).getSequenceId());
      extractors.put(HeadTrajectoryMessage.class, m -> ((HeadTrajectoryMessage) m).getSequenceId());
      extractors.put(NeckTrajectoryMessage.class, m -> ((NeckTrajectoryMessage) m).getSequenceId());
      extractors.put(NeckDesiredAccelerationsMessage.class, m -> ((NeckDesiredAccelerationsMessage) m).getSequenceId());
      extractors.put(ChestTrajectoryMessage.class, m -> ((ChestTrajectoryMessage) m).getSequenceId());
      extractors.put(SpineTrajectoryMessage.class, m -> ((SpineTrajectoryMessage) m).getSequenceId());
      extractors.put(PelvisTrajectoryMessage.class, m -> ((PelvisTrajectoryMessage) m).getSequenceId());
      extractors.put(PelvisOrientationTrajectoryMessage.class, m -> ((PelvisOrientationTrajectoryMessage) m).getSequenceId());
      extractors.put(PelvisHeightTrajectoryMessage.class, m -> ((PelvisHeightTrajectoryMessage) m).getSequenceId());
      extractors.put(StopAllTrajectoryMessage.class, m -> ((StopAllTrajectoryMessage) m).getSequenceId());
      extractors.put(FootstepDataListMessage.class, m -> ((FootstepDataListMessage) m).getSequenceId());
      extractors.put(GoHomeMessage.class, m -> ((GoHomeMessage) m).getSequenceId());
      extractors.put(FootLoadBearingMessage.class, m -> ((FootLoadBearingMessage) m).getSequenceId());
      extractors.put(ArmDesiredAccelerationsMessage.class, m -> ((ArmDesiredAccelerationsMessage) m).getSequenceId());
      extractors.put(AutomaticManipulationAbortMessage.class, m -> ((AutomaticManipulationAbortMessage) m).getSequenceId());
      extractors.put(HighLevelStateMessage.class, m -> ((HighLevelStateMessage) m).getSequenceId());
      extractors.put(AbortWalkingMessage.class, m -> ((AbortWalkingMessage) m).getSequenceId());
      extractors.put(PrepareForLocomotionMessage.class, m -> ((PrepareForLocomotionMessage) m).getSequenceId());
      extractors.put(PauseWalkingMessage.class, m -> ((PauseWalkingMessage) m).getSequenceId());
      extractors.put(SpineDesiredAccelerationsMessage.class, m -> ((SpineDesiredAccelerationsMessage) m).getSequenceId());
      extractors.put(HandLoadBearingMessage.class, m -> ((HandLoadBearingMessage) m).getSequenceId());
      extractors.put(HandHybridJointspaceTaskspaceTrajectoryMessage.class, m -> ((HandHybridJointspaceTaskspaceTrajectoryMessage) m).getSequenceId());
      extractors.put(HeadHybridJointspaceTaskspaceTrajectoryMessage.class, m -> ((HeadHybridJointspaceTaskspaceTrajectoryMessage) m).getSequenceId());
      extractors.put(ChestHybridJointspaceTaskspaceTrajectoryMessage.class, m -> ((ChestHybridJointspaceTaskspaceTrajectoryMessage) m).getSequenceId());
      extractors.put(ClearDelayQueueMessage.class, m -> ((ClearDelayQueueMessage) m).getSequenceId());
      extractors.put(MomentumTrajectoryMessage.class, m -> ((MomentumTrajectoryMessage) m).getSequenceId());
      extractors.put(CenterOfMassTrajectoryMessage.class, m -> ((CenterOfMassTrajectoryMessage) m).getSequenceId());
      extractors.put(PlanarRegionsListMessage.class, m -> ((PlanarRegionsListMessage) m).getSequenceId());

      return new MessageIDExtractor()
      {
         @Override
         public long getMessageID(Object message)
         {
            MessageIDExtractor extractor = extractors.get(message.getClass());
            return extractor == null ? NO_ID : extractor.getMessageID(message);
         }
      };
   }
}
