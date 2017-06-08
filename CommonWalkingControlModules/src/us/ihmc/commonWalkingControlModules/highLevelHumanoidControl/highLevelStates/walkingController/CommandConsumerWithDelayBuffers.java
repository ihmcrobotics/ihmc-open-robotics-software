package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.controllerAPI.command.CompilableCommand;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class CommandConsumerWithDelayBuffers
{
   public static final int NUMBER_OF_COMMANDS_TO_QUEUE = 5;

   private final DoubleYoVariable yoTime;
   private final CommandInputManager commandInputManager;
   
   /** Controller's copy of the new commands to be processed. */
   private final Map<Class<? extends Command<?, ?>>, RecyclingArrayList<? extends Command<?, ?>>> commandsMap = new HashMap<>();
   private final Map<Class<?>, CommandPriorityQueue> priorityQueues = new HashMap<>();
   

   public CommandConsumerWithDelayBuffers(CommandInputManager commandInputManager, DoubleYoVariable yoTime)
   {
      this.yoTime = yoTime;
      this.commandInputManager = commandInputManager;
      registerNewCommands(commandInputManager.getListOfSupportedCommands());
   }
   
   @SuppressWarnings("unchecked")
   private <C extends Command<C, M>, M extends Packet<M>> void registerNewCommands(List<Class<? extends Command<?, ?>>> commandClasses)
   {
      for (int i = 0; i < commandClasses.size(); i++)
      {
         registerNewCommand((Class<C>) commandClasses.get(i));
      }
   }

   private <C extends Command<C, M>, M extends Packet<M>> void registerNewCommand(Class<C> commandClass)
   {
      commandsMap.put(commandClass, new RecyclingArrayList<>(NUMBER_OF_COMMANDS_TO_QUEUE, commandClass));
      priorityQueues.put(commandClass, new CommandPriorityQueue(NUMBER_OF_COMMANDS_TO_QUEUE));
   }
   
   private boolean isDelayedCommandAvailable(Class<? extends Command<?, ?>> commandClassToCheck)
   {
      Command<?, ?> command = priorityQueues.get(commandClassToCheck).peek();
      if(command != null)
      {
         double startTime = command.getExecutionDelayTime();
         if(yoTime.getDoubleValue() >= startTime)
         {
            return true;
         }
      }
      return false;
   }
   
   /**
    * Check if a new command to be processed is available.
    * @param commandClassToCheck class of the command to check availability.
    * @return true if at least one new command is available.
    */
   public boolean isNewCommandAvailable(Class<? extends Command<?, ?>> commandClassToCheck)
   {
      return isDelayedCommandAvailable(commandClassToCheck) || commandInputManager.isNewCommandAvailable(commandClassToCheck);
   }

   @SuppressWarnings("unchecked")
   public <C extends Command<C, ?>> C pollNewestCommand(Class<C> commandClassToPoll)
   {
      if(commandInputManager.isNewCommandAvailable(commandClassToPoll))
      {
         C command = commandInputManager.pollNewestCommand(commandClassToPoll);
         
         //see if the command has a delay
         if(command.getExecutionDelayTime() > Double.MIN_VALUE)
         {
            queueCommand(command);
         }
         else
         {
            //if we received a non delayed command then remove all delayed commands of this type
            priorityQueues.get(commandClassToPoll).clear();
            return command;
         }
      }
      
      if(isDelayedCommandAvailable(commandClassToPoll))
      {
         return (C) priorityQueues.get(commandClassToPoll).pop();
      }
      
      return null;
   }

   private <C extends Command<C, ?>> void queueCommand(Command<?,?> command)
   {
      RecyclingArrayList<? extends Command<?, ?>> recyclingArrayList = commandsMap.get(command.getClass());
      Command commandCopy = recyclingArrayList.add();
      commandCopy.set(command);
      commandCopy.setExecutionDelayTime(commandCopy.getExecutionDelayTime() + yoTime.getDoubleValue());
      priorityQueues.get(command.getClass()).add(commandCopy);
   }
   
   @SuppressWarnings("unchecked")
   public <C extends Command<C, ?>> List<C> pollNewCommands(Class<C> commandClassToPoll)
   {
      RecyclingArrayList<C> newCommands = (RecyclingArrayList<C>) commandInputManager.pollNewCommands(commandClassToPoll);
      
      int size = newCommands.size();
      int index = 0;
      CommandPriorityQueue commandPriorityQueue = priorityQueues.get(commandClassToPoll);
      while(index < size)
      {
         C command = newCommands.get(index);
         
         if(command.getExecutionDelayTime() > Double.MIN_VALUE)
         {
            queueCommand(command);
            newCommands.remove(index);
            index--;
            size--;
         }
         else
         {
            commandPriorityQueue.clear();
         }
         index++;
      }
      
      while(isDelayedCommandAvailable(commandClassToPoll))
      {
         Command<?, ?> queuedCommand = commandPriorityQueue.pop();
         newCommands.add().set((C) queuedCommand);
      }
      
      return newCommands;
   }

   public <C extends Command<C, ?>> void flushCommands(Class<C> commandClassToFlush)
   {
      CommandPriorityQueue queueableCommandPriorityQueue = priorityQueues.get(commandClassToFlush);
      if(queueableCommandPriorityQueue != null)
      {
         queueableCommandPriorityQueue.clear();
      }
      commandInputManager.flushCommands(commandClassToFlush);
   }

   public <C extends CompilableCommand<C, ?>> C pollAndCompileCommands(Class<C> commandClassToPoll)
   {
      return commandInputManager.pollAndCompileCommands(commandClassToPoll);
   }
}
