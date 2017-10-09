package us.ihmc.communication.net;

public interface NetworkedObjectCommunicator extends ObjectCommunicator
{
   /**
    * Same as consumeObject, but returns bytes send.
    * 
    * @param object
    * @return bytes send
    */
   int send(Object object);

   /**
    * TCP connections may want to change settings like keepAlive or timeOut.
    */
   void attachStateListener(TcpNetStateListener stateListener);

   /**
    * Disconnect the connection, but leave the executor listeners alive. This allows re-connecting at a later moment. 
    * Use disconnect() when you want to shutdown the connection completely. 
    */
   void closeConnection();
}
