package us.ihmc.robotDataLogger.gui;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.robotDataLogger.interfaces.DataServerDiscoveryListener;
import us.ihmc.robotDataLogger.websocket.client.discovery.DataServerDiscoveryClient;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerDescription;

public class DataServerSelector
{
   private final DataServerSelectorJFrame selector = new DataServerSelectorJFrame();
   private final DataServerDiscoveryClient client;
   
   public DataServerSelector(List<HTTPDataServerDescription> hosts)
   {
      this.client = new DataServerDiscoveryClient(new Listener());
      
      for(HTTPDataServerDescription host : hosts)
      {
         if(host.isPersistant())
         {
            this.selector.addHost(host);
            this.client.addHost(host);
         }         
      }
      
      
   }
   
   public HTTPDataServerConnection select()
   {
      HTTPDataServerConnection connection = this.selector.select();
      this.selector.dispose();
      if(connection == null)
      {
         client.close();
      }
      else
      {
         client.close(connection);
      }
      return connection;
   }
   
   
   
   private class Listener implements DataServerDiscoveryListener
   {

      @Override
      public void connected(HTTPDataServerConnection connection)
      {
         selector.updateHost(connection);
      }

      @Override
      public void disconnected(HTTPDataServerConnection connection)
      {
         selector.updateHost(connection);         
      }
      
   }
   
   public static void main(String[] args)
   {
      ArrayList<HTTPDataServerDescription> hosts = new ArrayList<HTTPDataServerDescription>();
      hosts.add(new HTTPDataServerDescription("127.0.0.1", 8008, true));
      
      HTTPDataServerConnection connection = new DataServerSelector(hosts).select();
      System.out.println(connection);
      if(connection != null)
      {
         connection.close();
      }
      
   }
}
