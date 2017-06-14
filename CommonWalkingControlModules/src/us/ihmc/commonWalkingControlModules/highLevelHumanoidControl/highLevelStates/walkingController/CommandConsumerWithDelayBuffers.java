package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commons.PrintTools;
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
    * Check if a new command is available.
    * @param clazz class of the command to check availability.
    * @return true if at least one new command is available.
    */
   public <C extends Command<C, ?>> boolean isNewCommandAvailable(Class<? extends Command<?,?>> clazz)
   {
      GarbageFreePriorityQueue<Command<?, ?>> priorityQueue = priorityQueues.get(clazz);
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
    * Updates the execution delay and puts the command in a priority queue
    * @param command
    */
   @SuppressWarnings("unchecked")
   private void queueCommand(Command<?,?> command)
   {
      GarbageFreePriorityQueue<Command<?, ?>> priorityQueue = priorityQueues.get(command.getClass());
      if(priorityQueue.getSize() >= NUMBER_OF_COMMANDS_TO_QUEUE)
      {
         PrintTools.error("Tried to add " + command.getClass() + " to the delay queue, but the queue was full. Try increasing the queue size");
         return;
      }
      RecyclingArrayList<? extends Command<?, ?>> recyclingArrayList = queuedCommands.get(command.getClass());
      Command commandCopy = recyclingArrayList.add();
      commandCopy.set(command);
      commandCopy.setExecutionTime(commandCopy.getExecutionDelayTime() + yoTime.getDoubleValue());
      priorityQueue.add(commandCopy);
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
      if(isNewCommandAvailable(commandClassToPoll))
      {
         RecyclingArrayList<? extends Command<?, ?>> recyclingArrayList = queuedCommands.get(commandClassToPoll);
         GarbageFreePriorityQueue<Command<?, ?>> priorityQueue = priorityQueues.get(commandClassToPoll);
         Command<?, ?> command = priorityQueue.pop();
         recyclingArrayList.remove(command);
         return (C) command;
      }
      
      return null;
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
      GarbageFreePriorityQueue<Command<?, ?>> priorityQueue = priorityQueues.get(commandClassToPoll);
      RecyclingArrayList<? extends Command<?, ?>> recyclingArrayList = queuedCommands.get(commandClassToPoll);
      
      commands.clear();
      
      while(isNewCommandAvailable(commandClassToPoll))
      {
         Command<?, ?> queuedCommand =  priorityQueue.pop();
         recyclingArrayList.remove(queuedCommand);
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
