package us.ihmc.tools;


/**
 * Common interface to share timestamps between threads.
 * 
 * Implementations of this class are supposed to be thread-safe.
 *
 */
public interface TimestampProvider
{
   /**
    * 
    * Returns the current timestamp. 
    * 
    * The implementation of this function should be thread safe.
    * 
    * @return the current timestamp
    */
   public long getTimestamp();
}
