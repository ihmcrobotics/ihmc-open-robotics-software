package us.ihmc.robotDataLogger.gui;

import java.io.IOException;
import java.net.InetAddress;
import java.net.UnknownHostException;
import java.util.List;

import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.StaticHostListLoader;
import us.ihmc.robotDataLogger.gui.DataServerSelectorJFrame.HostAddedListener;
import us.ihmc.robotDataLogger.interfaces.DataServerDiscoveryListener;
import us.ihmc.robotDataLogger.websocket.client.discovery.DataServerDiscoveryClient;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerDescription;

/**
 * Simple popup GUI to select which host to connect to.
 * 
 * Has the ability for the user to specify the host manually. Manually added IP's get stored for later use in ~/.ihmc/ControllerHosts.yaml
 * 
 * @author Jesper Smith
 *
 */
public class DataServerSelectorGUI implements HostAddedListener
{
   
   private final DataServerSelectorJFrame selector = new DataServerSelectorJFrame(this);
   private final DataServerDiscoveryClient client;
   
   
   public DataServerSelectorGUI(boolean enableAutoDiscovery)
   {
      this.client = new DataServerDiscoveryClient(new Listener(), true);
      
      List<HTTPDataServerDescription> hosts = StaticHostListLoader.load();
      
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
   
   

   @Override
   public void hostAdded(String host, String port)
   {
      try
      {
         int portInt = Integer.valueOf(port);
         InetAddress.getByName(host);
         
         HTTPDataServerDescription description = new HTTPDataServerDescription(host, portInt, true);
         this.selector.addHost(description);
         this.client.addHost(description);
         
         try
         {
            StaticHostListLoader.save(this.client.getPersistantHostList());
         }
         catch (IOException e)
         {
            LogTools.warn("Cannot save host list. " + e.getMessage());
         }
      }
      catch (NumberFormatException e)
      {
         LogTools.warn("Invalid port " + port);
      }
      catch (UnknownHostException e)
      {
         LogTools.warn("Invalid host. " + e.getMessage());
      }
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
   
   public static void main(String[] args) throws IOException
   {
      HTTPDataServerConnection connection = new DataServerSelectorGUI(true).select();
      System.out.println(connection);
      if(connection != null)
      {
         connection.close();
      }
      System.exit(0);
   }

}
