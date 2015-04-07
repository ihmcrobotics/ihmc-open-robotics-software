package us.ihmc.communication.net;

public interface NetworkedObjectCommunicator extends ObjectCommunicator
{
   /**
    * Same as consumeObject, but returns bytes send.
    * 
    * @param object
    * @return bytes send
    */
   public int send(Object object);
   
   /**
    * Disconnect the connection, but leave the executor listeners alive. This allows re-connecting at a later moment. 
    * Use close() when you want to shutdown the connection completely. 
    */
   public void closeConnection();
}
