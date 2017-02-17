package us.ihmc.communication.streamingData;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

import org.apache.commons.lang3.tuple.ImmutablePair;

import us.ihmc.tools.thread.ThreadTools;

public class StreamingDataTCPServer implements EstablishedAConnectionListener
{
   private boolean sendingData = true;
   private boolean justSendMostRecentObject = false;
   
   private final ArrayList<StreamingDataProducer> producers = new ArrayList<StreamingDataProducer>();
   private final ConcurrentLinkedQueue<ImmutablePair<Long, Object>> dataToSendToClients = new ConcurrentLinkedQueue<ImmutablePair<Long, Object>>();

   private final ConcurrentLinkedQueue<StreamingDataTCPConnectionToClient> clientConnections = new ConcurrentLinkedQueue<StreamingDataTCPConnectionToClient>();
   
  
   private final PersistentTCPServer server;
   
   public StreamingDataTCPServer(int port)
   {
      this.server = new PersistentTCPServer(port, this);
   }
   
   public void setJustSendMostRecentObject(boolean justSendMostRecentObject)
   {
      this.justSendMostRecentObject = justSendMostRecentObject;
   }
   
   public void registerStreamingDataProducer(StreamingDataProducer producer)
   {
      producers.add(producer);
      
      StreamingDataTCPServerConsumer consumer = new StreamingDataTCPServerConsumer(producer);
      producer.registerConsumer(consumer);
   }
   
   private class StreamingDataTCPServerConsumer implements StreamingDataConsumer
   {
      private final StreamingDataProducer producer;
      
      public StreamingDataTCPServerConsumer(StreamingDataProducer producer)
      {
         this.producer = producer;
      }
      
      public void consume(long dataIdentifier, Object dataObject)
      {
         synchronized(streamingDataTCPServerConnectionsDataSender)
         {
            dataToSendToClients.offer(new ImmutablePair<Long, Object>(dataIdentifier, dataObject)); 
            streamingDataTCPServerConnectionsDataSender.notifyAll();
         }
      }
      
      public boolean canHandle(Object object)
      {
         return true;
      }
      
      public long getDataIdentifier()
      {
         return producer.getDataIdentifier();
      }
   }

   private StreamingDataTCPServerConnectionsDataSender streamingDataTCPServerConnectionsDataSender;
   
   public void startOnAThread()
   {
      server.startOnAThread();

      if (streamingDataTCPServerConnectionsDataSender != null)
      {
         throw new RuntimeException("streamingDataTCPServerConnectionsDataSender != null. Only call startOnAThread() once!");
      }

      streamingDataTCPServerConnectionsDataSender = new StreamingDataTCPServerConnectionsDataSender();
      Thread thread = new Thread(streamingDataTCPServerConnectionsDataSender);
      thread.start();
   }
   
   public void closeAndBlockTillFullyClosed()
   {
      close();
      
      while(!isDoneClosing())
      {
         ThreadTools.sleep(10L);
      }
   }
   
   public void close()
   {
      sendingData = false;
      
      for (StreamingDataTCPConnectionToClient clientConnection : clientConnections)
      {
         clientConnection.close();
      }
      
      server.close();
   }
   
   
   
   public boolean isDoneClosing()
   {
      return server.isDoneClosing();
   }
   

   
   private class StreamingDataTCPServerConnectionsDataSender implements Runnable
   {
      private final ArrayList<StreamingDataTCPConnectionToClient> deadClients = new ArrayList<StreamingDataTCPServer.StreamingDataTCPConnectionToClient>();
      
      public void run()
      {
         while(sendingData)
         {
            synchronized(this)
            {
               if (dataToSendToClients.isEmpty())
               {
                  try
                  {
                     this.wait();
                  } 
                  catch (InterruptedException e)
                  {
                  }
               }
            }
            
            if (!dataToSendToClients.isEmpty())
            {
               ImmutablePair<Long, Object> identifierAndObject = null;

               if (justSendMostRecentObject)
               {
                  while(!dataToSendToClients.isEmpty())
                  {
                     identifierAndObject = dataToSendToClients.poll();
                  }
               }
               else
               {
                  identifierAndObject = dataToSendToClients.poll();
               }
               
               long dataIdentifier = identifierAndObject.getLeft();
               Object dataObject = identifierAndObject.getRight();
               
//               System.out.println("Sending data object to " + clientConnections.size() + " clients.");
               deadClients.clear();
               for (StreamingDataTCPConnectionToClient clientConnection : clientConnections)
               {
                  if (clientConnection.isConnected())
                  {
                     clientConnection.sendData(dataIdentifier, dataObject);
                  }
                  else
                  {
                     deadClients.add(clientConnection);
                  }
               }
               
               for (StreamingDataTCPConnectionToClient deadClient : deadClients)
               {
                  deadClient.close();
                  clientConnections.remove(deadClient);
               }
            }

         }
      }
   }
  
   
   private class StreamingDataTCPConnectionToClient
   {
      private ObjectOutputStream objectOutputStream;
      
      private boolean connected = false;
      
      public StreamingDataTCPConnectionToClient(ObjectOutputStream objectOutputStream)
      {
         this.objectOutputStream = objectOutputStream;
         connected = true;
      }
      
      public void close()
      {
         connected = false;

         if (objectOutputStream != null)
         {
            try
            {
               objectOutputStream.close();
            } 
            catch (IOException e)
            {
            }
         }

         objectOutputStream = null;
      }

      public boolean isConnected()
      {
         return connected;
      }
      
      public void sendData(long dataIdentifier, Object object)
      {
         if (connected)
         {
            try
            {
               objectOutputStream.writeLong(dataIdentifier);
               objectOutputStream.writeObject(object);
               objectOutputStream.flush();
            } 
            catch (IOException e)
            {
               connected = false;
            }
         }
      }
   }


   public void establishedAConnection(ObjectInputStream objectInputStream, ObjectOutputStream objectOutputStream)
   {
      if (objectInputStream != null)
      {
         throw new RuntimeException("Should not be getting objectInputStreams here!");
      }
      
      StreamingDataTCPConnectionToClient streamingDataTCPConnectionToClient = new StreamingDataTCPConnectionToClient(objectOutputStream);
      clientConnections.add(streamingDataTCPConnectionToClient);
   }

}
