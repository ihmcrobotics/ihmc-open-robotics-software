package us.ihmc.quadrupedRobotics.communication;

import controller_msgs.msg.dds.*;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.ControllerNetworkSubscriber;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.MessageCollector;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.QuadrupedTimedStepListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SoleTrajectoryCommand;
import us.ihmc.quadrupedRobotics.communication.commands.QuadrupedRequestedControllerStateCommand;
import us.ihmc.quadrupedRobotics.communication.commands.QuadrupedRequestedSteppingStateCommand;

import java.util.*;

import static us.ihmc.humanoidRobotics.communication.packets.PacketValidityChecker.validateQuadrupedTimedStepListMessage;

public class QuadrupedControllerAPIDefinition
{

   private static final List<Class<? extends Command<?, ?>>> quadrupedSupportedCommands;
   private static final List<Class<? extends Packet<?>>> quadrupedSupportedStatusMessages;

   static
   {

      List<Class<? extends Command<?, ?>>> quadrupedCommands = new ArrayList<>();
      quadrupedCommands.add(QuadrupedTimedStepListCommand.class);
      quadrupedCommands.add(SoleTrajectoryCommand.class);
      quadrupedCommands.add(QuadrupedRequestedSteppingStateCommand.class);
      quadrupedCommands.add(QuadrupedRequestedControllerStateCommand.class);

      quadrupedSupportedCommands = Collections.unmodifiableList(quadrupedCommands);

      List<Class<? extends Packet<?>>> quadrupedStatusMessages = new ArrayList<>();
      quadrupedStatusMessages.add(QuadrupedSteppingStateChangeMessage.class);
      quadrupedStatusMessages.add(QuadrupedControllerStateChangeMessage.class);

      quadrupedSupportedStatusMessages = Collections.unmodifiableList(quadrupedStatusMessages);

   }

   public static List<Class<? extends Command<?, ?>>> getQuadrupedSupportedCommands()
   {
      return quadrupedSupportedCommands;
   }

   public static List<Class<? extends Packet<?>>> getQuadrupedSupportedStatusMessages()
   {
      return quadrupedSupportedStatusMessages;
   }

   public static ControllerNetworkSubscriber.MessageValidator createDefaultMessageValidation()
   {
      Map<Class<? extends Packet<?>>, ControllerNetworkSubscriber.MessageValidator> validators = new HashMap<>();
//      validators.put(FootTrajectoryMessage.class, message -> validateFootTrajectoryMessage((FootTrajectoryMessage) message)); // TODO
//      validators.put(PelvisTrajectoryMessage.class, message -> validatePelvisTrajectoryMessage((PelvisTrajectoryMessage) message));
//      validators
//            .put(PelvisOrientationTrajectoryMessage.class, message -> validatePelvisOrientationTrajectoryMessage((PelvisOrientationTrajectoryMessage) message));
//      validators.put(PelvisHeightTrajectoryMessage.class, message -> validatePelvisHeightTrajectoryMessage((PelvisHeightTrajectoryMessage) message));
      validators.put(QuadrupedTimedStepListMessage.class, message -> validateQuadrupedTimedStepListMessage((QuadrupedTimedStepListMessage) message));
//      validators.put(GoHomeMessage.class, message -> validateGoHomeMessage((GoHomeMessage) message));
//      validators.put(FootLoadBearingMessage.class, message -> validateFootLoadBearingMessage((FootLoadBearingMessage) message));

      return new ControllerNetworkSubscriber.MessageValidator()
      {
         @Override
         public String validate(Packet<?> message)
         {
            ControllerNetworkSubscriber.MessageValidator validator = validators.get(message.getClass());
            return validator == null ? null : validator.validate(message);
         }
      };
   }

   public static MessageCollector.MessageIDExtractor createDefaultMessageIDExtractor()
   {
      Map<Class<? extends Packet<?>>, MessageCollector.MessageIDExtractor> extractors = new HashMap<>();
//      extractors.put(FootTrajectoryMessage.class, m -> ((FootTrajectoryMessage) m).getSequenceId()); // TODO
//      extractors.put(PelvisTrajectoryMessage.class, m -> ((PelvisTrajectoryMessage) m).getSequenceId());
//      extractors.put(PelvisOrientationTrajectoryMessage.class, m -> ((PelvisOrientationTrajectoryMessage) m).getSequenceId());
//      extractors.put(PelvisHeightTrajectoryMessage.class, m -> ((PelvisHeightTrajectoryMessage) m).getSequenceId());
//      extractors.put(StopAllTrajectoryMessage.class, m -> ((StopAllTrajectoryMessage) m).getSequenceId());
      extractors.put(QuadrupedTimedStepListMessage.class, m -> ((QuadrupedTimedStepListMessage) m).getSequenceId());
//      extractors.put(GoHomeMessage.class, m -> ((GoHomeMessage) m).getSequenceId());
//      extractors.put(FootLoadBearingMessage.class, m -> ((FootLoadBearingMessage) m).getSequenceId());
      extractors.put(QuadrupedRequestedSteppingStateMessage.class, m -> ((QuadrupedRequestedSteppingStateMessage) m).getSequenceId());
      extractors.put(QuadrupedRequestedControllerStateMessage.class, m -> ((QuadrupedRequestedControllerStateMessage) m).getSequenceId());
//      extractors.put(AbortWalkingMessage.class, m -> ((AbortWalkingMessage) m).getSequenceId());
//      extractors.put(PauseWalkingMessage.class, m -> ((PauseWalkingMessage) m).getSequenceId());
//      extractors.put(ChestHybridJointspaceTaskspaceTrajectoryMessage.class, m -> ((ChestHybridJointspaceTaskspaceTrajectoryMessage) m).getSequenceId());
//      extractors.put(ClearDelayQueueMessage.class, m -> ((ClearDelayQueueMessage) m).getSequenceId());
//      extractors.put(MomentumTrajectoryMessage.class, m -> ((MomentumTrajectoryMessage) m).getSequenceId());
//      extractors.put(CenterOfMassTrajectoryMessage.class, m -> ((CenterOfMassTrajectoryMessage) m).getSequenceId());
//      extractors.put(PlanarRegionsListMessage.class, m -> ((PlanarRegionsListMessage) m).getSequenceId());

      return new MessageCollector.MessageIDExtractor()
      {
         @Override
         public long getMessageID(Packet<?> message)
         {
            MessageCollector.MessageIDExtractor extractor = extractors.get(message.getClass());
            return extractor == null ? NO_ID : extractor.getMessageID(message);
         }
      };
   }
}


