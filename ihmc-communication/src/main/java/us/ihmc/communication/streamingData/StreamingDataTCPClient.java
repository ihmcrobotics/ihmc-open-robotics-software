package us.ihmc.communication.streamingData;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.util.ArrayList;

public class StreamingDataTCPClient implements EstablishedAConnectionListener
{   
   private final PersistentTCPClient persistentTCPClient;
   private StreamingDataTCPClientRunner streamingDataTCPClientRunner;
   
   private final ArrayList<StreamingDataConsumer> consumers = new ArrayList<StreamingDataConsumer>();

   public StreamingDataTCPClient(String serverIPAddress, int serverPort)
   {
      persistentTCPClient = new PersistentTCPClient(serverIPAddress, serverPort, this);
   }
   
   public void close()
   {
      persistentTCPClient.close();
      if (streamingDataTCPClientRunner != null) streamingDataTCPClientRunner.close();
   }
   
   public boolean isConnected()
   {
      return persistentTCPClient.isConnected();
   }

   public void registerStreamingDataConsumer(StreamingDataConsumer streamingDataConsumer)
   {
      consumers.add(streamingDataConsumer);
   }

   public void connectToServer(boolean persistentlyStayConnected)
   {
      persistentTCPClient.connectToServer(persistentlyStayConnected);
   }

   public void establishedAConnection(ObjectInputStream objectInputStream, ObjectOutputStream objectOutputStream)
   {
      if (streamingDataTCPClientRunner != null) streamingDataTCPClientRunner.close();
      createAndStartStreamingDataTCPClientRunner(objectInputStream); 
      
      if (objectOutputStream != null)
      {
         throw new RuntimeException("Should not be getting objectOutputStream here!");
      }
   }
   
   private void createAndStartStreamingDataTCPClientRunner(ObjectInputStream objectInputStream)
   {
      streamingDataTCPClientRunner = new StreamingDataTCPClientRunner(objectInputStream);
      Thread thread = new Thread(streamingDataTCPClientRunner);
      thread.setDaemon(true);
      thread.start();
   }


   private class StreamingDataTCPClientRunner implements Runnable
   {
      private final ObjectInputStream objectInputStream;
      private boolean connected = true;
      
      public StreamingDataTCPClientRunner(ObjectInputStream objectInputStream)
      {
         this.objectInputStream = objectInputStream;
      }
      
      public void close()
      {
         connected = false;
      }
      
      public void run()
      {
         while (connected)
         {
            try
            {
               long dataIdentifier = objectInputStream.readLong();
               Object dataObject = objectInputStream.readObject();

               for (StreamingDataConsumer streamingDataConsumer : consumers)
               {
                  boolean sameID = ((streamingDataConsumer.getDataIdentifier() == 0) || (streamingDataConsumer.getDataIdentifier() == dataIdentifier));
                  if (sameID && streamingDataConsumer.canHandle(dataObject))
                  {
                     streamingDataConsumer.consume(dataIdentifier, dataObject);
                  }
               }
            }
            catch (IOException e)
            {
               connected = false;
               persistentTCPClient.notifyConnectionBroke();
            }
            catch (ClassNotFoundException e)
            {
               System.err.println("Class not found!");
               e.printStackTrace();
               connected = false;
               persistentTCPClient.notifyConnectionBroke();
            }
         }
      }  
   }
}
