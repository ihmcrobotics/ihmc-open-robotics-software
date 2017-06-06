package us.ihmc.communication.interfaces;

public interface Connectable
{
   public boolean isConnected();
   
   public void connect() throws Exception;
   
   public void disconnect() throws Exception;
}
