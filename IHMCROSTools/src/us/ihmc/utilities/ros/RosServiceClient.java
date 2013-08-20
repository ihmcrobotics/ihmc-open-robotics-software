package us.ihmc.utilities.ros;

import org.ros.internal.message.Message;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

public class RosServiceClient<T extends Message, S extends Message>
{
   private final String requestType;
   
   private ServiceClient<T, S> client;
   
   private final Object syncObject = new Object(); 

   public RosServiceClient(String requestType)
   {
      this.requestType = requestType;
   }

   public String getRequestType()
   {
      return requestType;
   }
   
   public void setServiceClient(ServiceClient<T, S> client)
   {
      this.client = client;
      connected();
   }
   
   private void checkInitialized()
   {
      if(client == null)
      {
         throw new RuntimeException("RosServiceClient is not registered with RosMainNode");
      }
   }
   
   public void call(T request, ServiceResponseListener<S> response)
   {
      checkInitialized();
      client.call(request, response);
   }

   public T getMessage()
   {
      checkInitialized();
      
      return client.newMessage();
   }

   
   private void connected()
   {
      synchronized (syncObject)
      {
         syncObject.notifyAll();
      }
   }
   
   
   public void waitTillConnected()
   {
      while(client == null)
      {
         synchronized (syncObject)
         {
            try
            {
               syncObject.wait();
            }
            catch (InterruptedException e)
            {
            }
         }
      }
   }
}
