package us.ihmc.robotDataLogger.rtps;

public interface ClearLogListener
{
   /** 
    * Gets called when a clear log message is broadcast
    * 
    * @param guid of the sender
    */
   public void clearLog(String guid);
}
