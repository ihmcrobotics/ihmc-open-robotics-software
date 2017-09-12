package us.ihmc.communication.net.local;

import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.tools.thread.ThreadTools;

/* package-private */class IntraprocessCommunicationNetwork
{
   private static TIntObjectHashMap<IntraprocessCommunicator> communicators = null;

   private static void throwNotConnectedException(int port)
   {
      throw new RuntimeException("Client is not connected to port " + port + ". Make sure to call connect()");

   }

   /* package-private */ static int sendObject(IntraprocessObjectCommunicator sender, int port, Object object)
   {
      IntraprocessCommunicator communicator;
      synchronized (IntraprocessCommunicationNetwork.class)
      {
         if (communicators == null)
         {
            throwNotConnectedException(port);
         }

         communicator = communicators.get(port);
         if (communicator == null)
         {
            throwNotConnectedException(port);
         }
      }
      communicator.send(sender, object);
      return 0;
   }

   /* package-private */static void connect(IntraprocessObjectCommunicator client, int port)
   {

      synchronized (IntraprocessCommunicationNetwork.class)
      {
         IntraprocessCommunicator communicator;
         if (communicators == null)
         {
            communicators = new TIntObjectHashMap<>();
         }

         communicator = communicators.get(port);
         if (communicator == null)
         {
            communicator = new IntraprocessCommunicator(port);
            communicators.put(port, communicator);
         }
         communicator.connect(client);
      }

   }

   /* package-private */static void disconnect(IntraprocessObjectCommunicator client, int port)
   {

      synchronized (IntraprocessCommunicationNetwork.class)
      {
         if (communicators != null)
         {
            IntraprocessCommunicator communicator = communicators.get(port);
            if (communicator != null)
            {
               communicator.disconnect(client);
               if (!communicator.hasClients())
               {
                  communicators.remove(port);
               }
               if (communicators.size() == 0)
               {
                  communicators = null;
               }
            }
         }

      }
   }

   /* package-private */static synchronized boolean isConnected(IntraprocessObjectCommunicator client, int port)
   {
      IntraprocessCommunicator communicator;

      synchronized (IntraprocessCommunicationNetwork.class)
      {
         if (communicators != null)
         {
            communicator = communicators.get(port);
            if (communicator == null)
            {
               return false;
            }
         }
         else
         {
            return false;
         }
      }

      return communicator.isConnected(client);
   }

   private static class IntraprocessCommunicator
   {
      private final int port;
      private final ArrayList<IntraprocessObjectCommunicator> clients = new ArrayList<>();
      private ExecutorService callBackExecutor;

      private IntraprocessCommunicator(int port)
      {
         this.port = port;
         createCallbackExecuter();
      }

      private void createCallbackExecuter()
      {
         this.callBackExecutor = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory("IntraprocessCommunicatorCallback-" + port));
      }

      private synchronized boolean hasClients()
      {
         return clients.size() != 0;
      }

      private synchronized void connect(IntraprocessObjectCommunicator client)
      {
         clients.add(client);
         client.connected();
         
         if(callBackExecutor.isShutdown())
         {
            createCallbackExecuter();
         }
      }

      private synchronized void disconnect(IntraprocessObjectCommunicator client)
      {
         clients.remove(client);
         client.disconnected();

         callBackExecutor.shutdownNow();
      }

      private synchronized boolean isConnected(IntraprocessObjectCommunicator client)
      {
         for (int i = 0; i < clients.size(); i++)
         {
            if (clients.get(i) == client)
            {
               return true;
            }
         }

         return false;
      }

      private synchronized void send(IntraprocessObjectCommunicator sender, final Object object)
      {
         if (isConnected(sender))
         {
            for (int i = 0; i < clients.size(); i++)
            {
               final IntraprocessObjectCommunicator client = clients.get(i);
               if (client != sender)
               {
                  final Object copy = sender.copyPacket(object);
                  
                  callBackExecutor.execute(new Runnable()
                  {
                     @Override
                     public void run()
                     {
                        client.receiveObject(copy);
                     }
                  });
               }
            }
         }
         else
         {
            throwNotConnectedException(port);
         }
      }
   }

   // Helpers for test classes
   /* package-private */synchronized static int getOpenPorts()
   {
      if (communicators == null)
      {
         return 0;
      }
      else
      {
         return communicators.size();
      }
   }

   /* package-private */synchronized static boolean hasMap()
   {
      return communicators != null;
   }

   /* package-private */synchronized static void closeAllConnectionsForMyJUnitTests()
   {
      communicators = null;
   }
}
