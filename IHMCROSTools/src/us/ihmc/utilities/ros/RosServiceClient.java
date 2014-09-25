package us.ihmc.utilities.ros;

import org.ros.exception.ServiceNotFoundException;
import org.ros.internal.message.Message;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

import us.ihmc.utilities.ThreadTools;

;

public class RosServiceClient<T extends Message, S extends Message>
{
   private final String requestType;

   private ServiceClient<T, S> client;

   private final Object syncObject = new Object();

   private ConnectedNode connectedNode = null;
   private String attachedServiceName = null;

   public RosServiceClient(String requestType)
   {
      this.requestType = requestType;
   }

   public ServiceClient<T, S> getClient()
   {
      return client;
   }

   public String getRequestType()
   {
      return requestType;
   }

   public void setServiceClient(ServiceClient<T, S> client, ConnectedNode connectedNode, String attachedServiceName)
   {
      this.connectedNode = connectedNode;
      this.attachedServiceName = attachedServiceName;
      this.client = client;
      connected();
   }

   private void checkInitialized()
   {
      if (client == null)
      {
         throw new RuntimeException("RosServiceClient is not registered with RosMainNode");
      }
   }
   

   public void call(T request, ServiceResponseListener<S> response)
   {
      final int DEFAULT_RETRY=20;
      call(request, response, DEFAULT_RETRY);
   }

   /**
    * @param request
    * @param response
    */
   public void call(T request, ServiceResponseListener<S> response, int retry)
   {
      checkInitialized();
  

      if(!client.isConnected())
      {
                client.shutdown();      
      
              //locate URI
              connectedNode.getLog().info("re-connecting to service " + attachedServiceName);

              for(int i=0;;i++)
              {
                 
                 if(i>retry)
                 {
                    connectedNode.getLog().error("Tried for " + retry + " times ... bailing out");
                    throw new RuntimeException("Tried for " + retry + " times... bailing out");
                 }
                 else
                 {
                    connectedNode.getLog().error("Attempt " + i +" to reconnect to service"  );
                 }
                 
                 
                 /** Approach 1, simpler but slower 
                  * 
                  */
                      try
                      {
                         client = connectedNode.newServiceClient(attachedServiceName, getRequestType());
                         break;
                      }
                      catch (ServiceNotFoundException e)
                      {
                         e.printStackTrace();
                      }                      

                 ThreadTools.sleep(3000); //wait for some socket to close due to timeout
              }
      }
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
      while (client == null)
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
