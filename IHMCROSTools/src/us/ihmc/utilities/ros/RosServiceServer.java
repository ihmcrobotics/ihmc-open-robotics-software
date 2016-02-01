package us.ihmc.utilities.ros;

import org.ros.exception.ServiceException;
import org.ros.internal.message.Message;
import org.ros.node.ConnectedNode;
import org.ros.node.service.ServiceResponseBuilder;
import org.ros.node.service.ServiceServer;

/**
 * 
 * @author tingfan
 *
 * @param <Q> requestMessageType
 * @param <R> responseMessageType
 */
public abstract class RosServiceServer<Q extends Message, R extends Message> implements ServiceResponseBuilder<Q, R>
{
   private final String requestType;

   private ServiceServer<Q, R> server;

   private final Object syncObject = new Object();

   private ConnectedNode connectedNode = null;
   private String attachedServiceName = null;

   public RosServiceServer(String requestType)
   {
      this.requestType = requestType;
   }

   public ServiceServer<Q, R> getServer()
   {
      return server;
   }

   public String getRequestType()
   {
      return requestType;
   }

   public void setServiceServer(ServiceServer<Q, R> server, ConnectedNode connectedNode, String attachedServiceName)
   {
      this.connectedNode = connectedNode;
      this.attachedServiceName = attachedServiceName;
      this.server = server;
      connected();
   }


   @Override
   public abstract void build(Q request, R response) throws ServiceException;


   private void connected()
   {
      synchronized (syncObject)
      {
         syncObject.notifyAll();
      }
   }

   public void waitTillConnected()
   {
      while (server == null)
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
