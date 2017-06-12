package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.lists.RecyclingArrayList;

/**
 * Pulls commands from the CommandInputManager and checks the delay time, If there is no delay or the delay is negative, it is available to be consumed
 * If the command has an execution delay it is put in a priority queue and held until the delay time has elapsed.
 *
 */
public class CommandConsumerWithDelayBuffers
{
   public static final int NUMBER_OF_COMMANDS_TO_QUEUE = 5;

   private final DoubleYoVariable yoTime;
   private final CommandInputManager commandInputManager;
   
   /** Controller's copy of the new commands to be processed. */
   private final Map<Class<? extends Command<?, ?>>, RecyclingArrayList<? extends Command<?, ?>>> commandsMap = new HashMap<>();
   private final Map<Class<?>, GarbageFreePriorityQueue<Command<?, ?>>> priorityQueues = new HashMap<>();
   

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
      CommandExecutionTimeComparator commandComparator = new CommandExecutionTimeComparator();
      priorityQueues.put(commandClass, new GarbageFreePriorityQueue<Command<?, ?>>(NUMBER_OF_COMMANDS_TO_QUEUE, Command.class, commandComparator));
   }
   
   /**
    * Returns true if a delayed command is ready to be executed
    * @param commandClassToCheck class to check
    * @return true if a command is ready to be executed, false otherwise. More than one command may be available if true
    */
   private boolean isDelayedCommandAvailable(Class<? extends Command<?, ?>> commandClassToCheck)
   {
      Command<?, ?> command = priorityQueues.get(commandClassToCheck).peek();
      if(command != null)
      {
         double startTime = command.getExecutionTime();
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

   /**
    * Poll the most recent available command.
    * After calling this method, no new command will be available excluding delayed commands
    * @param commandClassToPoll Used to know what type of command is to be polled.
    * @return the new command to be processed, returns null if there is no new available command.
    */
   @SuppressWarnings("unchecked")
   public <C extends Command<C, ?>> C pollNewestCommand(Class<C> commandClassToPoll)
   {
      RecyclingArrayList<C> newCommands = (RecyclingArrayList<C>) commandInputManager.pollNewCommands(commandClassToPoll);
      GarbageFreePriorityQueue<Command<?, ?>> commandPriorityQueue = processCommands(commandClassToPoll, newCommands);
      
      if(newCommands.size() > 0)
      {
         return newCommands.get(0);
      }
      
      if(isDelayedCommandAvailable(commandClassToPoll))
      {
         return (C) commandPriorityQueue.pop();
      }
      
      return null;
   }

   /**
    * Updates the execution delay and puts the command in a priority queue
    * @param command
    */
   @SuppressWarnings("unchecked")
   private <C extends Command<C, ?>> void queueCommand(Command<?,?> command)
   {
      RecyclingArrayList<? extends Command<?, ?>> recyclingArrayList = commandsMap.get(command.getClass());
      Command commandCopy = recyclingArrayList.add();
      commandCopy.set(command);
      commandCopy.setExecutionTime(commandCopy.getExecutionDelayTime() + yoTime.getDoubleValue());
      priorityQueues.get(command.getClass()).add(commandCopy);
   }
   
   /**
    * Poll all the new available commands.
    * After calling this method, no new command will be available excluding delayed commands.
    * @param commandClassToPoll Used to know what type of command is to be polled.
    * @return the new commands to be processed stored in a list, returns an empty list if there is no new available command.
    */
   @SuppressWarnings("unchecked")
   public <C extends Command<C, ?>> List<C> pollNewCommands(Class<C> commandClassToPoll)
   {
      RecyclingArrayList<C> newCommands = (RecyclingArrayList<C>) commandInputManager.pollNewCommands(commandClassToPoll);
      GarbageFreePriorityQueue<Command<?, ?>> commandPriorityQueue = processCommands(commandClassToPoll, newCommands);
      
      while(isDelayedCommandAvailable(commandClassToPoll))
      {
         Command<?, ?> queuedCommand = commandPriorityQueue.pop();
         newCommands.add().set((C) queuedCommand);
      }
      
      return newCommands;
   }

   /**
    * pulls delayed commands out of {@code newCommands} and queues them
    * If a non delayed command is found the queue is cleared. 
    * @param commandClassToPoll the command class
    * @param newCommands the commands received from the {@code CommandInputManager}
    * @return priority queue with new delayed commands added
    */
   private <C extends Command<C, ?>> GarbageFreePriorityQueue<Command<?, ?>> processCommands(Class<C> commandClassToPoll, RecyclingArrayList<C> newCommands)
   {
      int size = newCommands.size();
      int index = 0;
      GarbageFreePriorityQueue<Command<?, ?>> commandPriorityQueue = priorityQueues.get(commandClassToPoll);
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
      return commandPriorityQueue;
   }

   /**
    * clears commands from all buffers
    * @param commandClassToFlush the command class
    */
   public <C extends Command<C, ?>> void flushCommands(Class<C> commandClassToFlush)
   {
      GarbageFreePriorityQueue<Command<?, ?>> queueableCommandPriorityQueue = priorityQueues.get(commandClassToFlush);
      if(queueableCommandPriorityQueue != null)
      {
         queueableCommandPriorityQueue.clear();
      }
      commandInputManager.flushCommands(commandClassToFlush);
   }
}
