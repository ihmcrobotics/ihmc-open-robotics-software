package us.ihmc.communication.net;

import java.io.IOException;

public interface ObjectCommunicator extends GlobalObjectConsumer
{

   public abstract void attachStateListener(NetStateListener stateListener);
   public abstract <T> void attachListener(Class<T> clazz, ObjectConsumer<T> listener);
   public abstract <T> void detachListener(Class<T> clazz, ObjectConsumer<T> listener);
   public abstract void attachGlobalListener(GlobalObjectConsumer listener);
   public abstract void detachGlobalListener(GlobalObjectConsumer listener);
   
   public abstract boolean isConnected();
   
   public abstract void close();
   public abstract void connect() throws IOException;

}