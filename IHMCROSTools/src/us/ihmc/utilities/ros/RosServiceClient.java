package us.ihmc.utilities.ros;

import  us.ihmc.utilities.ThreadTools;

import java.net.URI;

import org.jboss.netty.channel.ChannelException;
import org.ros.exception.RosRuntimeException;
import org.ros.internal.message.Message;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceClient;
import org.ros.node.service.ServiceResponseListener;

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

   /**
    * @param request
    * @param response
    */
   public void call(T request, ServiceResponseListener<S> response)
   {
      checkInitialized();
//      if(!client.isConnected())
//      {
//              client.shutdown();
//              //locate URI
//              connectedNode.getLog().info("re-connecting to service " + attachedServiceName);
//
//              for(int i=0;;i++)
//              {
//                 
//                 if(i>10)
//                 {
//                    connectedNode.getLog().error("Tried for 50 seconds ... bailing out");
//                    throw new RuntimeException("Tried for 50 seconds ... bailing out");
//                 }
//                 else
//                 {
//                    connectedNode.getLog().error("Attempt " + i +" to reconnect to serive"  );
//                 }
//                 
//                 
//                 URI serviceURI = connectedNode.lookupServiceUri(attachedServiceName);
//                 if (serviceURI != null)
//                 {
//                    try
//                    {
//                       client.connect(serviceURI);
//                       break;
//                    }
//                    catch (RosRuntimeException | ChannelException e)
//                    {
//                       connectedNode.getLog().error("connecting to " + serviceURI + " error:" + e.getMessage());
//                       e.printStackTrace();
//                    }
//                 }
//                 else
//                 {
//                    connectedNode.getLog().error("waiting for service " + attachedServiceName + " to be ready");
//                 }
//
//                 ThreadTools.sleep(5000); //wait for some socket to close due to timeout
//              }
//      }
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
