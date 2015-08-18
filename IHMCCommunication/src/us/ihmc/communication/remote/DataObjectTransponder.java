package us.ihmc.communication.remote;

import java.io.EOFException;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.net.Socket;
import java.net.SocketException;
import java.util.ArrayList;

import us.ihmc.communication.streamingData.StreamingDataConsumer;

/**
 * User: GrayThomas
 * Date: 12/10/12
 */
public class DataObjectTransponder implements StreamingDataConsumer
{
   protected static boolean DEBUG = false;
   private final ArrayList<StreamingDataConsumer> streamingDataConsumers = new ArrayList<StreamingDataConsumer>();
   private boolean hasClientHandler = false;
   private String name = "some DataObjectTransponder";
   protected Socket socket = null;
   protected ObjectInputStream objectInputStream = null;
   protected ObjectOutputStream objectOutputStream = null;
   private boolean connected = false;
   private boolean isSilent = false;


   void createNewDataReadingDaemon()
   {
      Thread clientThread = new Thread(new DataReadingThread());
      clientThread.setDaemon(true);
      clientThread.start();
   }
   int getNumberOfConsumers()
   {
      return streamingDataConsumers.size();
   }
   public void addStreamingDataConsumer(StreamingDataConsumer streamingDataConsumer)
   {
      streamingDataConsumers.add(streamingDataConsumer);
   }

   public void sendData(long dataObjectIdentifier, Object object) throws IOException
   {
      printIfDebug(getName() + "'s hasClientHandler is " + hasClientHandler + ".");
      printIfDebug(getName() + "'s objectOutputStream is " + objectOutputStream + ".");


      if (hasClientHandler)
      {
         printIfDebug(getName() + " is sending data on port: " + port);


         synchronized (this)
         {
            try
            {
               this.objectOutputStream.writeLong(dataObjectIdentifier);
               this.objectOutputStream.writeObject(object);
            }
            catch (IOException e)
            {
               printIfDebug(this.getName() + " had problem sending: " + e.getMessage());

               connected = false;
            }
         }

         if (!connected)
         {
            printIfDebug(getName() + "'s client on port " + port + " is dead");


            hasClientHandler = false;
         }
      }

   }

   public boolean isConnected()
   {
      return connected;
   }

   void connectToSocket(Socket socket) throws IOException
   {
      this.socket = socket;
      port = socket.getPort();
      hasClientHandler = true;
      connected = true;
      printIfDebug(getName()+" is opening output object stream");
      objectOutputStream = new ObjectOutputStream(socket.getOutputStream());
      printIfDebug(getName()+" has opened two streams: output "+ objectOutputStream.toString());
      printIfDebug(getName()+" is opening input object stream");
      objectInputStream = new ObjectInputStream(socket.getInputStream());
      printIfDebug(getName()+" has opened two streams: input "+ objectInputStream.toString());
      createNewDataReadingDaemon();

      synchronized (this)
      {
         notifyAll();         
      }
   }

   protected static void printIfDebug(String message)
   {
      if (DEBUG)
      {
         System.out.println(message);
      }
   }

   public String getName()
   {
      return name;
   }

   public void setName(String name)
   {
      this.name = name;
   }

   public int port;

   private class DataReadingThread implements Runnable
   {
      public void run()
      {
         while (connected)
         {
            try
            {
               long dataIdentifier = objectInputStream.readLong();
               Object obj = objectInputStream.readObject();
               printIfDebug(getName() + "'s DataReadingThread: received an object.");

               int consumersResponding = 0;
               for (StreamingDataConsumer streamingDataConsumer : streamingDataConsumers)
               {
                  if (dataIdentifier != streamingDataConsumer.getDataIdentifier() && streamingDataConsumer.canHandle(obj))
                  {
                     System.err.println("Can handle yet dataIdentifier != streamingDataConsumer.getDataIdentifier(). object = " + obj + " dataIdentifier = " + dataIdentifier);
                     System.err.println("streamingDataConsumer = " + streamingDataConsumer);
                     System.err.println("streamingDataConsumer.getDataIdentifier() = " + streamingDataConsumer.getDataIdentifier());
                     throw new RuntimeException();
                  }
                  
                  if ((dataIdentifier == streamingDataConsumer.getDataIdentifier()) && (streamingDataConsumer.canHandle(obj)))
                  {
                     consumersResponding ++;
                     streamingDataConsumer.consume(dataIdentifier, obj);
                  }
               }

               printIfDebug(getName() + "'s DataReadingThread: " + consumersResponding + " consumer(s) Responding");
               if (consumersResponding == 0 && !isSilent)
               {
                  System.out.println(getName()+" Number of Consumers: "+streamingDataConsumers.size());
                  throw new RuntimeException(getName() + ", DataObjectTransponder: DataReadingThread: unhandled packet");
               }
            }
            catch (SocketException se)
            {
               connected = false;
            }
            catch (EOFException e)
            {
               connected = false;
            }
            catch (ClassNotFoundException e)
            {
               e.printStackTrace();
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
            catch (RuntimeException e)
            {
               e.printStackTrace();
            }
         }

         closeSocket();
      }

      private void closeSocket()
      {
         try
         {
            socket.close();
         }
         catch (IOException xcp)
         {
            xcp.printStackTrace();
         }

         socket = null;
         objectInputStream = null;
      }

   }

   public void setIsSilent(boolean isSilent)
   {
      this.isSilent = isSilent;
   }

   public boolean canHandle(Object object)
   {
      return true;
   }

   public void consume(long dataObjectIdentifier, Object dataObject)
   {
      try
      {
         sendData(dataObjectIdentifier, dataObject);
      } 
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }

   public long getDataIdentifier()
   {
      return 0;
   }
}
