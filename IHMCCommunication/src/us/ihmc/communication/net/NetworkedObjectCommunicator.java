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
}
