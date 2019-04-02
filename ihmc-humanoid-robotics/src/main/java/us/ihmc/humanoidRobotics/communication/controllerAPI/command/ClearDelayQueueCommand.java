package us.ihmc.humanoidRobotics.communication.controllerAPI.command;

import controller_msgs.msg.dds.ClearDelayQueueMessage;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.controllerAPI.converter.ClearDelayQueueConverter;

/**
 * This command is used to clear the delay buffers on the controller, If you sent a command with a
 * delay and now you do not want them executed, use this command to remove them from the queue
 */
public class ClearDelayQueueCommand implements Command<ClearDelayQueueCommand, ClearDelayQueueMessage>
{
   private long sequenceId;
   private Class<? extends Command<?, ?>> commandClassToClear;
   private Class<? extends Settable<?>> messageClassToClear;

   private boolean clearAllDelayBuffers;

   /**
    * empty constructor, required
    */
   public ClearDelayQueueCommand()
   {
   }

   /**
    * set the class you want to clear
    * 
    * @param clazz the class you want to clear
    */
   public ClearDelayQueueCommand(Class<Command<?, ?>> clazz)
   {
      this.commandClassToClear = clazz;
   }

   /**
    * copy constructor
    * 
    * @param command the command to copy from
    */
   public ClearDelayQueueCommand(ClearDelayQueueCommand command)
   {
      set(command);
   }

   /**
    * set the class you want to clear
    * 
    * @param clazz the class you want to clear
    */
   public void setMessageClassToClear(Class<? extends Settable<?>> messageClassToClear)
   {
      this.messageClassToClear = messageClassToClear;
   }

   /**
    * set the class you want to clear
    * 
    * @param clazz the class you want to clear
    */
   public void setCommandClassToClear(Class<? extends Command<?, ?>> clazz)
   {
      this.commandClassToClear = clazz;
   }

   /**
    * get the class to clear
    * 
    * @param commandClassToClear the class to clear
    */
   public Class<? extends Command<?, ?>> getCommandClassToClear()
   {
      return commandClassToClear;
   }

   /**
    * get the class to clear
    * 
    * @param commandClassToClear the class to clear
    */
   public Class<? extends Settable<?>> getMessageClassToClear()
   {
      return messageClassToClear;
   }

   /**
    * set whether or not you want to clear all the delay buffers
    * 
    * @param whether or not to clear all the delay buffers
    */
   public void setClearAllDelayBuffers(boolean clearAll)
   {
      this.clearAllDelayBuffers = clearAll;
   }

   /**
    * get whether or not to clear all the delay buffers
    * 
    * @param whether or not to clear all the delay buffers
    */
   public boolean getClearAllDelayBuffers()
   {
      return clearAllDelayBuffers;
   }

   /**
    * Set this command to the contents of the other command
    * 
    * @param other the command to copy
    */
   @Override
   public void set(ClearDelayQueueCommand other)
   {
      sequenceId = other.sequenceId;
      commandClassToClear = other.commandClassToClear;
      messageClassToClear = other.messageClassToClear;
      clearAllDelayBuffers = other.clearAllDelayBuffers;
   }

   /**
    * sets the fields back to their default values
    */
   @Override
   public void clear()
   {
      sequenceId = 0;
      commandClassToClear = null;
      messageClassToClear = null;
      clearAllDelayBuffers = false;
   }

   /**
    * Set this command to the contents of the message
    * 
    * @param message the message to copy from
    */
   @Override
   public void setFromMessage(ClearDelayQueueMessage message)
   {
      throw new UnsupportedOperationException("A " + ClearDelayQueueConverter.class.getSimpleName() + " has to be used to convert a " + ClearDelayQueueMessage.class.getSimpleName() + " into a " + getClass().getSimpleName());
   }

   @Override
   public Class<ClearDelayQueueMessage> getMessageClass()
   {
      return ClearDelayQueueMessage.class;
   }

   /**
    * this command is always valid
    * 
    * @return
    */
   @Override
   public boolean isCommandValid()
   {
      return true;
   }

   @Override
   public long getSequenceId()
   {
      return sequenceId;
   }
}
