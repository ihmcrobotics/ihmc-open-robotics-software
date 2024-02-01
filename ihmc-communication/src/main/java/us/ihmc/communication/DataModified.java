package us.ihmc.communication;

/**
 * This is used to keep track of when the operator has modified
 * some data using widgets, etc, and we need to send the updated
 * data off to some external process.
 *
 * This class can be instantiated once per message topic and passed
 * around to anything that might modify the data so that it can
 * mark it as modified.
 */
public class DataModified
{
   private boolean dataIsModified = false;

   /**
    * We've modified the data. Can be called more than once
    * in an update cycle.
    */
   public void markModified()
   {
      dataIsModified = true;
   }

   /**
    * We've sent the updated data off, or no longer need to.
    */
   public void clearModified()
   {
      dataIsModified = false;
   }

   /**
    * Primary getter.
    * @return if the data is marked as modified
    */
   public boolean getDataIsModified()
   {
      return dataIsModified;
   }
}
