package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.lists.GarbageFreePriorityQueue;
import us.ihmc.robotics.lists.RecyclingArrayList;

/**
 * Pulls commands from the CommandInputManager and checks the delay time, If there is no delay or the delay is negative, it is available to be consumed
 * If the command has an execution delay it is put in a priority queue and held until the delay time has elapsed.
 * You must call update each tick to be able to consume commands
 */
public class CommandConsumerWithDelayBuffers
{
   public static final int NUMBER_OF_COMMANDS_TO_QUEUE = 10;

   private final DoubleYoVariable yoTime;
   private final CommandInputManager commandInputManager;
   
   /** Controller's copy of the new commands to be processed. */
   private final Map<Class<? extends Command<?, ?>>, RecyclingArrayList<? extends Command<?, ?>>> queuedCommands = new HashMap<>();
   private final Map<Class<? extends Command<?, ?>>, RecyclingArrayList<? extends Command<?, ?>>> outgoingCommands = new HashMap<>();
   private final Map<Class<?>, GarbageFreePriorityQueue<Command<?, ?>>> priorityQueues = new HashMap<>();
   private final List<Class<? extends Command<?, ?>>> listOfSupportedCommands;

   

   public CommandConsumerWithDelayBuffers(CommandInputManager commandInputManager, DoubleYoVariable yoTime)
   {
      this.yoTime = yoTime;
      this.commandInputManager = commandInputManager;
      listOfSupportedCommands = commandInputManager.getListOfSupportedCommands();
      registerNewCommands(listOfSupportedCommands);
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
      queuedCommands.put(commandClass, new RecyclingArrayList<>(NUMBER_OF_COMMANDS_TO_QUEUE, commandClass));
      outgoingCommands.put(commandClass, new RecyclingArrayList<>(NUMBER_OF_COMMANDS_TO_QUEUE, commandClass));
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
      GarbageFreePriorityQueue<Command<?, ?>> priorityQueue = priorityQueues.get(commandClassToCheck);
      Command<?, ?> command = priorityQueue.peek();
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
    * You MUST call this method each tick or you wont get any commands!
    * Polls commands from the commandInputManager and puts them in the delay queue. 
    */
   public <C extends Command<C, ?>> void update()
   {
      for(int i = 0; i < listOfSupportedCommands.size(); i++)
      {
         RecyclingArrayList<C> newCommands = (RecyclingArrayList<C>) commandInputManager.pollNewCommands((Class<C>) listOfSupportedCommands.get(i));
         for(int commandIndex = 0; commandIndex < newCommands.size(); commandIndex++)
         {
            C command = newCommands.get(commandIndex);
            if(command.getExecutionDelayTime() < Double.MIN_VALUE)
            {
               GarbageFreePriorityQueue<Command<?, ?>> priorityQueue = priorityQueues.get(command.getClass());
               priorityQueue.clear();
            }
            queueCommand(command);
         }  
      }
   }
   
   /**
    * Check if a new command to be processed is available.
    * @param <C>
    * @param clazz class of the command to check availability.
    * @return true if at least one new command is available.
    */
   public <C extends Command<C, ?>> boolean isNewCommandAvailable(Class<? extends Command<C,?>> clazz)
   {
      return isDelayedCommandAvailable(clazz);
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
      if(isDelayedCommandAvailable(commandClassToPoll))
      {
         return (C) priorityQueues.get(commandClassToPoll).pop();
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
      RecyclingArrayList<? extends Command<?, ?>> recyclingArrayList = queuedCommands.get(command.getClass());
      Command commandCopy = recyclingArrayList.add();
      commandCopy.set(command);
      commandCopy.setExecutionTime(commandCopy.getExecutionDelayTime() + yoTime.getDoubleValue());
      GarbageFreePriorityQueue<Command<?, ?>> priorityQueue = priorityQueues.get(command.getClass());
      priorityQueue.add(commandCopy);
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
      RecyclingArrayList<C> commands = (RecyclingArrayList<C>) outgoingCommands.get(commandClassToPoll);
      commands.clear();
      
      while(isDelayedCommandAvailable(commandClassToPoll))
      {
         GarbageFreePriorityQueue<Command<?, ?>> priorityQueue = priorityQueues.get(commandClassToPoll);
         Command<?, ?> queuedCommand =  priorityQueue.pop();
         commands.add().set((C) queuedCommand);
      }
      
      return commands;
   }

   /**
    * clears commands from all buffers
    * @param commandClassToFlush the command class
    */
   public <C extends Command<C, ?>> void flushCommands(Class<C> commandClassToFlush)
   {
      GarbageFreePriorityQueue<Command<?, ?>> queueableCommandPriorityQueue = priorityQueues.get(commandClassToFlush);
      queueableCommandPriorityQueue.clear();
      commandInputManager.flushCommands(commandClassToFlush);
   }
}
