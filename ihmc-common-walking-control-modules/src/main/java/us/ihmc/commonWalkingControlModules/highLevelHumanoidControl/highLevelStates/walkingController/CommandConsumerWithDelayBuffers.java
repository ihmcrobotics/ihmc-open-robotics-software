package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.concurrent.Builder;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.ClearDelayQueueCommand;
import us.ihmc.log.LogTools;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Pulls commands from the CommandInputManager and checks the delay time, If there is no delay or the delay is negative, it is available to be consumed
 * If the command has an execution delay it is put in a priority queue and held until the delay time has elapsed.
 * You must call update each tick to be able to consume commands
 */
public class CommandConsumerWithDelayBuffers
{
   public static final int NUMBER_OF_COMMANDS_TO_QUEUE = 10;

   private final YoDouble yoTime;
   private final CommandInputManager commandInputManager;
   
   /** Controller's copy of the new commands to be processed. */
   private final Map<Class<? extends Command<?, ?>>, RecyclingArrayList<? extends Command<?, ?>>> queuedCommands = new HashMap<>();
   private final Map<Class<? extends Command<?, ?>>, RecyclingArrayList<? extends Command<?, ?>>> outgoingCommands = new HashMap<>();
   private final Map<Class<?>, PriorityQueue<Command<?, ?>>> priorityQueues = new HashMap<>();
   private final Map<Class<? extends Settable<?>>, Class<? extends Command<?,?>>> messageToCommandMap = new HashMap<>();
   private final List<Class<? extends Command<?, ?>>> listOfSupportedCommands;

   public CommandConsumerWithDelayBuffers(CommandInputManager commandInputManager, YoDouble yoTime)
   {
      this.yoTime = yoTime;
      this.commandInputManager = commandInputManager;
      listOfSupportedCommands = commandInputManager.getListOfSupportedCommands();
      registerNewCommands(listOfSupportedCommands);
   }
   
   @SuppressWarnings("unchecked")
   private <C extends Command<C, M>, M extends Settable<M>> void registerNewCommands(List<Class<? extends Command<?, ?>>> commandClasses)
   {
      for (int i = 0; i < commandClasses.size(); i++)
      {
         Class<? extends Command<?, ?>> commandClass = commandClasses.get(i);
         registerNewCommand((Class<C>) commandClass);

         Builder<? extends Command<?, ?>> commandConstructor = CommandInputManager.createBuilderWithEmptyConstructor(commandClass);
         Command<?, ?> command = commandConstructor.newInstance();
         messageToCommandMap.put(command.getMessageClass(), commandClass);
      }
   }

   private <C extends Command<C, M>, M extends Settable<M>> void registerNewCommand(Class<C> commandClass)
   {
      queuedCommands.put(commandClass, new RecyclingArrayList<>(NUMBER_OF_COMMANDS_TO_QUEUE, commandClass));
      outgoingCommands.put(commandClass, new RecyclingArrayList<>(NUMBER_OF_COMMANDS_TO_QUEUE, commandClass));
      CommandExecutionTimeComparator commandComparator = new CommandExecutionTimeComparator();
      priorityQueues.put(commandClass, new PriorityQueue<Command<?, ?>>(NUMBER_OF_COMMANDS_TO_QUEUE, commandComparator));
   }
   
   /**
    * You MUST call this method each tick or you wont get any commands!
    * Polls commands from the commandInputManager and puts them in the delay queue. 
    */
   @SuppressWarnings("unchecked")
   public <C extends Command<C, ?>> void update()
   {
      for(int i = 0; i < listOfSupportedCommands.size(); i++)
      {
         Class<? extends Command<?, ?>> commandClass = listOfSupportedCommands.get(i);
         RecyclingArrayList<C> newCommands = (RecyclingArrayList<C>) commandInputManager.pollNewCommands((Class<C>) commandClass);

         for(int commandIndex = 0; commandIndex < newCommands.size(); commandIndex++)
         {
            C command = newCommands.get(commandIndex);

            if(commandClass == ClearDelayQueueCommand.class)
            {
               ClearDelayQueueCommand clearDelayQueueCommand = (ClearDelayQueueCommand) command;
               handleClearQueueCommand(clearDelayQueueCommand);
               continue;
            }
            queueCommand(command);
         }  
      }
   }

   /**
    * Clears either all of the delay queues or a single delay queue depending on the
    * ClearDelayQueueCommand
    * @param clearDelayQueueCommand a command that dictates which queues to clear
    */
   private void handleClearQueueCommand(ClearDelayQueueCommand clearDelayQueueCommand)
   {
      if(clearDelayQueueCommand.getClearAllDelayBuffers())
      {
         for(int commandIndex = 0; commandIndex < listOfSupportedCommands.size(); commandIndex++)
         {
            Class<? extends Command<?, ?>> commandClassToFlush = listOfSupportedCommands.get(commandIndex);
            clearDelayQueue(commandClassToFlush);
         }
      }
      Class<? extends Settable<?>> messageClassToClear = clearDelayQueueCommand.getMessageClassToClear();
      if(messageClassToClear != null)
      {
         Class<? extends Command<?, ?>> commandClassToClear = messageToCommandMap.get(messageClassToClear);
         clearDelayQueueCommand.setCommandClassToClear(commandClassToClear);
      }

      Class<? extends Command<?, ?>> classToClear = clearDelayQueueCommand.getCommandClassToClear();
      if(classToClear != null)
      {
         clearDelayQueue(classToClear);
      }
   }

   /**
    * clears a single queue and it's associated RecyclingArrayList
    * @param commandClassToFlush the class to clear from the queues
    */
   private void clearDelayQueue(Class<? extends Command<?, ?>> commandClassToFlush)
   {
      PriorityQueue<Command<?, ?>> queueableCommandPriorityQueue = priorityQueues.get(commandClassToFlush);
      queueableCommandPriorityQueue.clear();
      RecyclingArrayList<? extends Command<?, ?>> recyclingArrayList = queuedCommands.get(commandClassToFlush);
      recyclingArrayList.clear();
   }

   /**
    * Check if a new command is available.
    * @param clazz class of the command to check availability.
    * @return true if at least one new command is available.
    */
   public <C extends Command<C, ?>> boolean isNewCommandAvailable(Class<? extends Command<?,?>> clazz)
   {
      PriorityQueue<Command<?, ?>> priorityQueue = priorityQueues.get(clazz);
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
   private <C extends Command<C, ?>> void queueCommand(C command)
   {
      PriorityQueue<Command<?, ?>> priorityQueue = priorityQueues.get(command.getClass());
      if(priorityQueue.size() >= NUMBER_OF_COMMANDS_TO_QUEUE)
      {
         LogTools.error("Tried to add {} to the delay queue, but the queue was full. Try increasing the queue size", command.getClass().getSimpleName());
         return;
      }
      RecyclingArrayList<? extends Command<?, ?>> recyclingArrayList = queuedCommands.get(command.getClass());
      C commandCopy = (C) recyclingArrayList.add();
      commandCopy.set(command);

      //not all commands implement setExecution time, if they don't the execution time will be 0 and should move to the front of the queue
      if(commandCopy.isDelayedExecutionSupported())
      {
         commandCopy.setExecutionTime(commandCopy.getExecutionDelayTime() + yoTime.getDoubleValue());
      }
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
         PriorityQueue<Command<?, ?>> priorityQueue = priorityQueues.get(commandClassToPoll);
         Command<?, ?> command = priorityQueue.poll();
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
      PriorityQueue<Command<?, ?>> priorityQueue = priorityQueues.get(commandClassToPoll);
      RecyclingArrayList<? extends Command<?, ?>> recyclingArrayList = queuedCommands.get(commandClassToPoll);
      
      commands.clear();
      
      while(isNewCommandAvailable(commandClassToPoll))
      {
         Command<?, ?> queuedCommand =  priorityQueue.poll();
         recyclingArrayList.remove(queuedCommand);
         commands.add().set((C) queuedCommand);
      }
      
      return commands;
   }

   /**
    * clears commands from all buffers
    * @param commandClassToFlush the command class
    * @deprecated Use {@link #clearCommands(Class<C>)} instead
    */
   @Deprecated
   public <C extends Command<C, ?>> void flushCommands(Class<C> commandClassToFlush)
   {
      clearCommands(commandClassToFlush);
   }

   /**
    * clears commands from all buffers
    * @param commandClassToFlush the command class
    */
   public <C extends Command<C, ?>> void clearCommands(Class<C> commandClassToFlush)
   {
      clearDelayQueue(commandClassToFlush);
      commandInputManager.clearCommands(commandClassToFlush);
   }
}
