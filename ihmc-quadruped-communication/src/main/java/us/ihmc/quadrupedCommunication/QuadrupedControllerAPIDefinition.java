package us.ihmc.quadrupedCommunication;

import static us.ihmc.commonWalkingControlModules.controllerAPI.input.MessageCollector.MessageIDExtractor.NO_ID;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateQuadrupedBodyHeightMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateQuadrupedBodyOrientationMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateQuadrupedTimedStepListMessage;
import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateSoleTrajectoryMessage;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import controller_msgs.msg.dds.*;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.MessageCollector;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.ROS2Tools.ROS2TopicQualifier;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.*;

public class QuadrupedControllerAPIDefinition
{
   private static final List<Class<? extends Command<?, ?>>> quadrupedSupportedCommands;
   private static final List<Class<? extends Settable<?>>> quadrupedSupportedStatusMessages;

   static
   {
      List<Class<? extends Command<?, ?>>> quadrupedCommands = new ArrayList<>();
      quadrupedCommands.add(QuadrupedTimedStepListCommand.class);
      quadrupedCommands.add(SoleTrajectoryCommand.class);
      quadrupedCommands.add(QuadrupedRequestedSteppingStateCommand.class);
      quadrupedCommands.add(QuadrupedBodyOrientationCommand.class);
      quadrupedCommands.add(QuadrupedBodyHeightCommand.class);
      quadrupedCommands.add(HighLevelControllerStateCommand.class);
      quadrupedCommands.add(PlanarRegionsListCommand.class);
      quadrupedCommands.add(QuadrupedBodyTrajectoryCommand.class);
      quadrupedCommands.add(PauseWalkingCommand.class);

//      quadrupedCommands.add(StopAllTrajectoryCommand.class); // TODO
//      quadrupedCommands.add(FootLoadBearingCommand.class); // TODO
//      quadrupedCommands.add(GoHomeCommand.class); // TODO
//      quadrupedCommands.add(QuadrupedBodyTranslationCommand.class); // TODO
//      quadrupedCommands.add(QuadrupedBodyPositionCommand.class); // TODO
//      quadrupedCommands.add(CenterOfMassTrajectoryCommand.class); // TODO
//      quadrupedCommands.add(AbortWalkingCommand.class);// TODO

      quadrupedSupportedCommands = Collections.unmodifiableList(quadrupedCommands);

      List<Class<? extends Settable<?>>> quadrupedStatusMessages = new ArrayList<>();
      quadrupedStatusMessages.add(QuadrupedSteppingStateChangeMessage.class);
      quadrupedStatusMessages.add(HighLevelStateChangeStatusMessage.class);
      quadrupedStatusMessages.add(QuadrupedFootstepStatusMessage.class);
      quadrupedStatusMessages.add(QuadrupedGroundPlaneMessage.class);
      quadrupedStatusMessages.add(WalkingControllerFailureStatusMessage.class);
      quadrupedStatusMessages.add(ControllerCrashNotificationPacket.class);

      quadrupedSupportedStatusMessages = Collections.unmodifiableList(quadrupedStatusMessages);

   }

   public static List<Class<? extends Command<?, ?>>> getQuadrupedSupportedCommands()
   {
      return quadrupedSupportedCommands;
   }

   public static List<Class<? extends Settable<?>>> getQuadrupedSupportedStatusMessages()
   {
      return quadrupedSupportedStatusMessages;
   }

   public static MessageTopicNameGenerator getSubscriberTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.QUADRUPED_CONTROL_MODULE, ROS2TopicQualifier.INPUT);
   }

   public static MessageTopicNameGenerator getPublisherTopicNameGenerator(String robotName)
   {
      return ROS2Tools.getTopicNameGenerator(robotName, ROS2Tools.QUADRUPED_CONTROL_MODULE, ROS2TopicQualifier.OUTPUT);
   }

   public static ControllerNetworkSubscriber.MessageValidator createDefaultMessageValidation()
   {
      Map<Class<? extends Settable<?>>, ControllerNetworkSubscriber.MessageValidator> validators = new HashMap<>();
      validators.put(SoleTrajectoryMessage.class, message -> validateSoleTrajectoryMessage((SoleTrajectoryMessage) message));
      validators.put(QuadrupedBodyOrientationMessage.class, message -> validateQuadrupedBodyOrientationMessage((QuadrupedBodyOrientationMessage) message));
      validators.put(QuadrupedBodyHeightMessage.class, message -> validateQuadrupedBodyHeightMessage((QuadrupedBodyHeightMessage) message));
      validators.put(QuadrupedTimedStepListMessage.class, message -> validateQuadrupedTimedStepListMessage((QuadrupedTimedStepListMessage) message));

      //      validators.put(PelvisTrajectoryMessage.class, message -> validatePelvisTrajectoryMessage((PelvisTrajectoryMessage) message));
      //      validators.put(GoHomeMessage.class, message -> validateGoHomeMessage((GoHomeMessage) message));
      //      validators.put(FootLoadBearingMessage.class, message -> validateFootLoadBearingMessage((FootLoadBearingMessage) message));

      return message -> validators.containsKey(message.getClass()) ? validators.get(message.getClass()).validate(message) : null;
   }

   public static MessageCollector.MessageIDExtractor createDefaultMessageIDExtractor()
   {
      Map<Class<? extends Settable<?>>, MessageCollector.MessageIDExtractor> extractors = new HashMap<>();
      extractors.put(SoleTrajectoryMessage.class, m -> ((SoleTrajectoryMessage) m).getSequenceId());
      extractors.put(QuadrupedBodyOrientationMessage.class, m -> ((QuadrupedBodyOrientationMessage) m).getSequenceId());
      extractors.put(QuadrupedBodyHeightMessage.class, m -> ((QuadrupedBodyHeightMessage) m).getSequenceId());
      extractors.put(QuadrupedTimedStepListMessage.class, m -> ((QuadrupedTimedStepListMessage) m).getSequenceId());
      extractors.put(QuadrupedRequestedSteppingStateMessage.class, m -> ((QuadrupedRequestedSteppingStateMessage) m).getSequenceId());

//      extractors.put(PelvisTrajectoryMessage.class, m -> ((PelvisTrajectoryMessage) m).getSequenceId());
//      extractors.put(StopAllTrajectoryMessage.class, m -> ((StopAllTrajectoryMessage) m).getSequenceId());
//      extractors.put(GoHomeMessage.class, m -> ((GoHomeMessage) m).getSequenceId());
//      extractors.put(FootLoadBearingMessage.class, m -> ((FootLoadBearingMessage) m).getSequenceId());
//      extractors.put(AbortWalkingMessage.class, m -> ((AbortWalkingMessage) m).getSequenceId());
//      extractors.put(PauseWalkingMessage.class, m -> ((PauseWalkingMessage) m).getSequenceId());
//      extractors.put(ChestHybridJointspaceTaskspaceTrajectoryMessage.class, m -> ((ChestHybridJointspaceTaskspaceTrajectoryMessage) m).getSequenceId());
//      extractors.put(ClearDelayQueueMessage.class, m -> ((ClearDelayQueueMessage) m).getSequenceId());
//      extractors.put(MomentumTrajectoryMessage.class, m -> ((MomentumTrajectoryMessage) m).getSequenceId());
//      extractors.put(CenterOfMassTrajectoryMessage.class, m -> ((CenterOfMassTrajectoryMessage) m).getSequenceId());
//      extractors.put(PlanarRegionsListMessage.class, m -> ((PlanarRegionsListMessage) m).getSequenceId());

      return message -> extractors.containsKey(message.getClass()) ? extractors.get(message.getClass()).getMessageID(message) : NO_ID;
   }
}


