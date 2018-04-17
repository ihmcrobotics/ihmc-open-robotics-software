package us.ihmc.humanoidRobotics.communication.controllerAPI.converter;

import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.Packet;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class FrameMessageCommandConverter implements CommandConversionInterface
{
   private final ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver;

   public FrameMessageCommandConverter(ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver)
   {
      this.referenceFrameHashCodeResolver = referenceFrameHashCodeResolver;
   }

   /** {@inheritDoc} */
   @Override
   public <C extends Command<?, M>, M extends Packet<M>> boolean isConvertible(C command, M message)
   {
      return command instanceof FrameBasedCommand;
   }
   
   /** {@inheritDoc} */
   @SuppressWarnings("unchecked")
   @Override
   public <C extends Command<?, M>, M extends Packet<M>> void process(C command, M message)
   {
      FrameBasedCommand<M> frameBasedCommand = (FrameBasedCommand<M>) command;
      frameBasedCommand.set(referenceFrameHashCodeResolver, message);
   }
}
