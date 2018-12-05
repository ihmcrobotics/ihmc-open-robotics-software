package us.ihmc.humanoidRobotics.communication.controllerAPI.converter;

import java.util.List;

import controller_msgs.msg.dds.ClearDelayQueueMessage;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ClearDelayQueueCommand;

public class ClearDelayQueueConverter implements CommandConversionInterface
{
   private final TIntObjectHashMap<Class<? extends Command<?, ?>>> hashCodeToCommandClasses = new TIntObjectHashMap<>();
   private final TIntObjectHashMap<Class<? extends Settable<?>>> hashCodeToMessageClasses = new TIntObjectHashMap<>();

   public ClearDelayQueueConverter(List<Class<? extends Command<?, ?>>> commandsToRegister) throws InstantiationException, IllegalAccessException
   {
      for (Class<? extends Command<?, ?>> commandToRegister : commandsToRegister)
      {
         Class<? extends Settable<?>> messageClass = commandToRegister.newInstance().getMessageClass();
         int hashCode = messageClass.getSimpleName().hashCode();
         hashCodeToCommandClasses.put(hashCode, commandToRegister);
         hashCodeToMessageClasses.put(hashCode, messageClass);
      }
   }

   @Override
   public <C extends Command<?, M>, M extends Settable<M>> boolean isConvertible(C command, M message)
   {
      return command instanceof ClearDelayQueueCommand && message instanceof ClearDelayQueueMessage;
   }

   @Override
   public <C extends Command<?, M>, M extends Settable<M>> void process(C command, M message)
   {
      ClearDelayQueueCommand clearCommand = (ClearDelayQueueCommand) command;
      ClearDelayQueueMessage clearMessage = (ClearDelayQueueMessage) message;
      clearCommand.setCommandClassToClear(hashCodeToCommandClasses.get(clearMessage.getClassSimpleNameBasedHashCode()));
      clearCommand.setMessageClassToClear(hashCodeToMessageClasses.get(clearMessage.getClassSimpleNameBasedHashCode()));
      clearCommand.setClearAllDelayBuffers(clearMessage.getClearAllDelayBuffers());
   }
}
