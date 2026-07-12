package us.ihmc.humanoidRobotics.communication.controllerAPI.converter;

import controller_msgs.ClearDelayQueueMessage;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ClearDelayQueueCommand;
import us.ihmc.jros2.ROS2Message;

import java.util.List;

public class ClearDelayQueueConverter implements CommandConversionInterface
{
   private final TIntObjectHashMap<Class<? extends Command<?, ?>>> hashCodeToCommandClasses = new TIntObjectHashMap<>();
   private final TIntObjectHashMap<Class<? extends ROS2Message<?>>> hashCodeToMessageClasses = new TIntObjectHashMap<>();

   public ClearDelayQueueConverter(List<Class<? extends Command<?, ?>>> commandsToRegister) throws InstantiationException, IllegalAccessException
   {
      for (Class<? extends Command<?, ?>> commandToRegister : commandsToRegister)
      {
         Class<? extends ROS2Message<?>> messageClass = commandToRegister.newInstance().getMessageClass();
         int hashCode = messageClass.getSimpleName().hashCode();
         hashCodeToCommandClasses.put(hashCode, commandToRegister);
         hashCodeToMessageClasses.put(hashCode, messageClass);
      }
   }

   @Override
   public <C extends Command<?, M>, M extends ROS2Message<M>> boolean isConvertible(C command, M message)
   {
      return command instanceof ClearDelayQueueCommand && message instanceof ClearDelayQueueMessage;
   }

   @Override
   public <C extends Command<?, M>, M extends ROS2Message<M>> void process(C command, M message)
   {
      ClearDelayQueueCommand clearCommand = (ClearDelayQueueCommand) command;
      ClearDelayQueueMessage clearMessage = (ClearDelayQueueMessage) message;
      clearCommand.setCommandClassToClear(hashCodeToCommandClasses.get(clearMessage.getClassSimpleNameBasedHashCode()));
      clearCommand.setMessageClassToClear(hashCodeToMessageClasses.get(clearMessage.getClassSimpleNameBasedHashCode()));
      clearCommand.setClearAllDelayBuffers(clearMessage.getClearAllDelayBuffers());
   }
}
