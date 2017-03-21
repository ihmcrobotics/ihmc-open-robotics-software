package us.ihmc.humanoidRobotics.communication.controllerAPI.converter;

import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.FrameBasedMessage;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class FrameMessageCommandConverter implements CommandConversionInterface
{
   private ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver;

   public FrameMessageCommandConverter(ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver)
   {
      this.referenceFrameHashCodeResolver = referenceFrameHashCodeResolver;
   }

   @Override
   public <C extends Command<?, M>, M extends Packet<M>> boolean isConvertible(C command, M message)
   {
      return command instanceof FrameBasedCommand && message instanceof FrameBasedMessage;
   }
   
   @SuppressWarnings("unchecked")
   @Override
   public <C extends Command<?, M>, M extends Packet<M>> void process(C command, M message)
   {
      FrameBasedCommand<M> frameBasedCommand = (FrameBasedCommand<M>) command;
      FrameBasedMessage frameBasedMessage = (FrameBasedMessage) message;
      
      long trajectoryReferenceFrameID = frameBasedMessage.getTrajectoryReferenceFrameId();
      ReferenceFrame trajectoryReferenceFrame = referenceFrameHashCodeResolver.getReferenceFrameFromNameBaseHashCode(trajectoryReferenceFrameID);
      
      long expressedInReferenceFrameID = frameBasedMessage.getDataReferenceFrameId();
      ReferenceFrame expressedInReferenceFrame = referenceFrameHashCodeResolver.getReferenceFrameFromNameBaseHashCode(expressedInReferenceFrameID);
      
      frameBasedCommand.set(expressedInReferenceFrame, trajectoryReferenceFrame, message);
   }
}
