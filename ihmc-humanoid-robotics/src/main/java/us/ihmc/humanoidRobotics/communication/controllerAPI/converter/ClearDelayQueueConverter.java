package us.ihmc.humanoidRobotics.communication.controllerAPI.converter;

import java.util.List;

import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ClearDelayQueueCommand;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.ClearDelayQueueMessage;

public class ClearDelayQueueConverter implements CommandConversionInterface
{
   private final TIntObjectHashMap<Class<? extends Command<?, ?>>> hashCodeToCommandClasses = new TIntObjectHashMap<>();
   private final TIntObjectHashMap<Class<? extends Packet<?>>> hashCodeToMessageClasses = new TIntObjectHashMap<>();

   public ClearDelayQueueConverter(List<Class<? extends Command<?, ?>>> commandsToRegister) throws InstantiationException, IllegalAccessException
   {
      for (Class<? extends Command<?, ?>> commandToRegister : commandsToRegister)
      {
         Class<? extends Packet<?>> messageClass = commandToRegister.newInstance().getMessageClass();
         int hashCode = messageClass.getSimpleName().hashCode();
         hashCodeToCommandClasses.put(hashCode, commandToRegister);
         hashCodeToMessageClasses.put(hashCode, messageClass);
      }
   }

   @Override
   public <C extends Command<?, M>, M extends Packet<M>> boolean isConvertible(C command, M message)
   {
      return command instanceof ClearDelayQueueCommand && message instanceof ClearDelayQueueMessage;
   }

   @Override
   public <C extends Command<?, M>, M extends Packet<M>> void process(C command, M message)
   {
      ClearDelayQueueCommand clearCommand = (ClearDelayQueueCommand) command;
      ClearDelayQueueMessage clearMessage = (ClearDelayQueueMessage) message;
      clearCommand.setCommandClassToClear(hashCodeToCommandClasses.get(clearMessage.getClassToClearSimpleNameBasedHashCode()));
      clearCommand.setMessageClassToClear(hashCodeToMessageClasses.get(clearMessage.getClassToClearSimpleNameBasedHashCode()));
      clearCommand.setClearAllDelayBuffers(clearMessage.getClearAllDelayBuffers());
   }
}
