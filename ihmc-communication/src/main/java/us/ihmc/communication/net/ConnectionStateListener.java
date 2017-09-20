package us.ihmc.communication.net;

public interface ConnectionStateListener
{
   public abstract void connected();
   
   public abstract void disconnected();
}
