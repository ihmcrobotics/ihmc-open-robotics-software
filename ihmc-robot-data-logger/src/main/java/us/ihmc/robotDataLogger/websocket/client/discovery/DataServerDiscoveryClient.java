package us.ihmc.robotDataLogger.websocket.client.discovery;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.TimeUnit;

import us.ihmc.log.LogTools;
import us.ihmc.robotDataLogger.interfaces.DataServerDiscoveryListener;
import us.ihmc.robotDataLogger.util.DaemonThreadFactory;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection.HTTPDataServerConnectionListener;

/**
 * Client to discover active logging sessions.
 * 
 * This client will take a list of hardcoded hosts and optionally listens for auto discoverable hosts.
 * 
 * For every hosts, it tries to connect and download the announcement. If this is possible, it stays connected to the host and marks the host as connected.
 * 
 * If the host disconnects, or if the connection is refused it will clean up and try again.
 * 
 * @author Jesper Smith
 *
 */
public class DataServerDiscoveryClient implements DataServerLocationBroadcastReceiver.DataServerLocationFoundListener
{
   private final Object lock = new Object();
   private final DataServerDiscoveryListener listener;

   private final ThreadFactory daemonThreadFactory = DaemonThreadFactory.getNamedDaemonThreadFactory(getClass().getSimpleName());

   private final ScheduledExecutorService connectionExecutor = Executors.newSingleThreadScheduledExecutor(daemonThreadFactory);
   private final Executor listenerExecutor = Executors.newSingleThreadExecutor(daemonThreadFactory);

   private final HashMap<HTTPDataServerDescription, HTTPDataServerDescription> hosts = new HashMap<HTTPDataServerDescription, HTTPDataServerDescription>();

   private boolean clientClosed = false;
   private final HashSet<HTTPDataServerConnection> connections = new HashSet<HTTPDataServerConnection>();

   private final DataServerLocationBroadcastReceiver broadcastReceiver;
   
   public DataServerDiscoveryClient(DataServerDiscoveryListener listener, boolean enableAutoDiscovery)
   {
      this.listener = listener;
      
      DataServerLocationBroadcastReceiver broadcastReceiver = null;
      if(enableAutoDiscovery)
      {
         try
         {
            broadcastReceiver = new DataServerLocationBroadcastReceiver(this);
            broadcastReceiver.start();
         }
         catch (IOException e)
         {
            LogTools.warn("Cannot start broadcast receiver. " + e.getMessage());
            broadcastReceiver = null;
         }         
      }
      this.broadcastReceiver = broadcastReceiver;
      
   }
   
   public void addHosts(List<HTTPDataServerDescription> descriptions)
   {
      for(HTTPDataServerDescription description : descriptions)
      {
         addHost(description);
      }
   }

   @Override
   public void addHost(String host, int port, boolean persistant)
   {
      HTTPDataServerDescription description = new HTTPDataServerDescription(host, port, persistant);
      addHost(description);
   }
   
   public void addHost(HTTPDataServerDescription description)
   {
      synchronized (lock)
      {
         if (hosts.containsKey(description))
         {
            if (description.isPersistant())
            {
               LogTools.debug("{} already in list of hosts. Marking persistant", description);
               hosts.put(description, description);
            }
            else
            {
               LogTools.debug("{} already in list of hosts", description);
            }
         }
         else
         {
            hosts.put(description, description);
            connectionExecutor.execute(() -> tryConnection(description));
         }
      }
   }

   public void close()
   {
      close(null);
   }

   public void close(HTTPDataServerConnection connectionToKeep)
   {
      synchronized (lock)
      {
         clientClosed = true;
         if (connectionToKeep != null)
         {
            connections.remove(connectionToKeep);
         }

         for (HTTPDataServerConnection connection : connections)
         {
            connection.close();
         }
         
         broadcastReceiver.stop();
      }
   }

   private void tryConnection(HTTPDataServerDescription target)
   {
      LogTools.debug("Connecting to {}.", target);
      try
      {
         new HTTPDataServerConnection(target, new ConnectionListener());
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   private class ConnectionListener implements HTTPDataServerConnectionListener
   {
      @Override
      public void connected(HTTPDataServerConnection connection)
      {
         synchronized (lock)
         {
            LogTools.debug("Connected to {}.", connection.getTarget());

            connections.add(connection);
            listenerExecutor.execute(() -> listener.connected(connection));
         }
      }

      @Override
      public void disconnected(HTTPDataServerConnection connection)
      {
         listenerExecutor.execute(() -> listener.disconnected(connection));
      }

      @Override
      public void connectionRefused(HTTPDataServerDescription target)
      {
         synchronized (lock)
         {
            LogTools.debug("Connection refused to {}.", target);

            if (!clientClosed && hosts.get(target).isPersistant())
            {
               LogTools.debug("{} is marked persistant, reconnecting.", target);
               connectionExecutor.schedule(() -> tryConnection(target), 1, TimeUnit.SECONDS);
            }
            else
            {
               LogTools.debug("{} is volatile. Dropping.", target);
               hosts.remove(target);
            }

         }
      }

      @Override
      public void closed(HTTPDataServerConnection connection)
      {
         synchronized (lock)
         {
            LogTools.debug("Disconnected from {}.", connection.getTarget());
            connections.remove(connection);

            if (!clientClosed && hosts.get(connection.getTarget()).isPersistant())
            {
               LogTools.debug("{} is marked persistant, reconnecting.", connection.getTarget());
               connectionExecutor.execute(() -> tryConnection(connection.getTarget()));
            }
            else
            {
               LogTools.debug("{} is volatile. Dropping.", connection.getTarget());
               hosts.remove(connection.getTarget());
            }
         }
      }
   }
   
   public List<HTTPDataServerDescription> getPersistantHostList()
   {
      ArrayList<HTTPDataServerDescription> list = new ArrayList<>();
      
      for (HTTPDataServerDescription description : hosts.values())
      {
         if(description.isPersistant())
         {
            list.add(description);
         }
      }
      
      return list;
      
   }

   public static void main(String[] args)
   {
      DataServerDiscoveryClient client = new DataServerDiscoveryClient(new DataServerDiscoveryListener()
      {

         @Override
         public void disconnected(HTTPDataServerConnection connection)
         {
            System.out.println("Disconnected from " + connection.getTarget());
         }

         @Override
         public void connected(HTTPDataServerConnection connection)
         {
            System.out.println("Connected " + connection.getTarget());
            connection.close();
         }
      }, true);

      client.addHost("127.0.0.1", 8008, true);
   }
}
