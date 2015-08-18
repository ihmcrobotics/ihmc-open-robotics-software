package us.ihmc.communication.net;

import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashMap;

public class LocalObjectCommunicator implements ObjectCommunicator
{
   private final LinkedHashMap<Class<?>, ArrayList<ObjectConsumer<?>>> listeners = new LinkedHashMap<Class<?>, ArrayList<ObjectConsumer<?>>>();
   
   private final ArrayList<GlobalObjectConsumer> globalListeners = new ArrayList<GlobalObjectConsumer>();
   
   @SuppressWarnings("unchecked")
   public synchronized void consumeObject(Object object)
   {
      
      final ArrayList<ObjectConsumer<?>> classListeners = listeners.get(object.getClass());
      if(classListeners != null)
      {
         for(@SuppressWarnings("rawtypes") ObjectConsumer consumer : classListeners)
         {
            consumer.consumeObject(object);
         }
      }
   }

   public void attachStateListener(NetStateListener stateListener)
   {
      stateListener.connected();
   }

   public <T> void attachListener(Class<T> clazz, ObjectConsumer<T> listener)
   {
      if(!listeners.containsKey(clazz))
      {
         listeners.put(clazz, new ArrayList<ObjectConsumer<?>>());
      }
      listeners.get(clazz).add(listener);
   }
   
   public <T> void detachListener(Class<T> clazz, ObjectConsumer<T> listener)
   {
      if(listeners.containsKey(clazz))
      {
         ArrayList<ObjectConsumer<?>> listenerList = listeners.get(clazz);
         if (listenerList.contains(listener))
         {
            listenerList.remove(listener);
         }
      }
   }


   public boolean isConnected()
   {
      return true;
   }

   public void close()
   {
      
   }

   public void connect() throws IOException
   {
   }

	public void attachGlobalListener(GlobalObjectConsumer listener)
	{
		globalListeners.add(listener);
	}
	
	public void detachGlobalListener(GlobalObjectConsumer listener)
	{
	   if (globalListeners.contains(listener))
	   {
	      globalListeners.remove(listener);
	   }
	}

}
