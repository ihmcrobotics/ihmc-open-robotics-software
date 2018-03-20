package us.ihmc.humanoidRobotics.communication.packets.wholebody;

import us.ihmc.communication.packets.Packet;

/**
 * This message is used to clear the delay buffers on the controller, If you sent a message with a
 * delay and now you do not want them executed, use this command
 */
public class ClearDelayQueueMessage extends Packet<ClearDelayQueueMessage>
{
   /** the class you want to clear **/
   public int classSimpleNameBasedHashCode;

   /** clear all the delay buffers **/
   public boolean clearAllDelayBuffers;

   /**
    * empty constructor, required for kryo
    */
   public ClearDelayQueueMessage()
   {
      uniqueId = VALID_MESSAGE_DEFAULT_ID;
   }

   @Override
   public void set(ClearDelayQueueMessage other)
   {
      classSimpleNameBasedHashCode = other.classSimpleNameBasedHashCode;
      clearAllDelayBuffers = other.clearAllDelayBuffers;
      setPacketInformation(other);
   }

   /**
    * set the class you want to clear.
    * 
    * @param classSimpleNameBasedHashCode the hash code of the class you want to clear: {@code classToClear.getSimpleName().hashCode()}.
    */
   public void setClassToClearSimpleNameBasedHashCode(int classSimpleNameBasedHashCode)
   {
      this.classSimpleNameBasedHashCode = classSimpleNameBasedHashCode;
   }

   /**
    * get the class to clear
    * 
    * @param clazz the class to clear
    */
   public int getClassToClearSimpleNameBasedHashCode()
   {
      return classSimpleNameBasedHashCode;
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

   @Override
   public boolean epsilonEquals(ClearDelayQueueMessage other, double epsilon)
   {
      if (this == other)
         return true;
      if (other == null)
         return false;
      if (classSimpleNameBasedHashCode != other.classSimpleNameBasedHashCode)
         return false;
      if (clearAllDelayBuffers != other.clearAllDelayBuffers)
         return false;
      return true;
   }
}
