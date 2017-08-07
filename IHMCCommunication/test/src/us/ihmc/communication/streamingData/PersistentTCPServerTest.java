package us.ihmc.communication.streamingData;

import static org.junit.Assert.assertEquals;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.net.ConnectException;
import java.net.InetAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.thread.ThreadTools;

public class PersistentTCPServerTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.2)
	@Test(timeout = 30000)
   public void testPersistentTCPServer() throws UnknownHostException, IOException
   {
      EstablishedConnectionCounter establishedConnectionCounter = new EstablishedConnectionCounter();
      
      int port = 2957;
      
      PersistentTCPServer persistentTCPServer = new PersistentTCPServer(port, establishedConnectionCounter);
      persistentTCPServer.startOnAThread();
      
      assertEquals(0, establishedConnectionCounter.getNumberConnectionsEstablished());
      
      Socket socket = null;
      int errorCount = 0;
      do
      {
    	  try
    	  {
    		  socket = new Socket(InetAddress.getLocalHost(), port);
    	  }
    	  catch(ConnectException e)
    	  {
    		  errorCount++;
    		  if(errorCount > 5)
    			  throw e;
    		  ThreadTools.sleep(100); //allow server to setup
    	  }
      }while(socket == null);
      
      
      ObjectInputStream objectInputStreamOne = new ObjectInputStream(socket.getInputStream());
      
      while(establishedConnectionCounter.getNumberConnectionsEstablished() != 1)
      {
         ThreadTools.sleep(100L);
      }
      
      int integerToWrite = 88;
      establishedConnectionCounter.writeInteger(integerToWrite);
      
      int integerRead = objectInputStreamOne.readInt();
      assertEquals(integerToWrite, integerRead);
      
      assertEquals(1, establishedConnectionCounter.getNumberConnectionsEstablished());

      socket = new Socket(InetAddress.getLocalHost(), port);
      ObjectInputStream objectInputStreamTwo = new ObjectInputStream(socket.getInputStream());
      
      while(establishedConnectionCounter.getNumberConnectionsEstablished() != 2)
      {
         ThreadTools.sleep(100L);
      }
      
      integerToWrite = 99;
      establishedConnectionCounter.writeInteger(integerToWrite);
      
      integerRead = objectInputStreamOne.readInt();
      assertEquals(integerToWrite, integerRead);
      
      integerRead = objectInputStreamTwo.readInt();
      assertEquals(integerToWrite, integerRead);
      
      assertEquals(2, establishedConnectionCounter.getNumberConnectionsEstablished());
      
      objectInputStreamOne.close();
      
      integerToWrite = 101;
      establishedConnectionCounter.writeInteger(integerToWrite);

      while(establishedConnectionCounter.getNumberConnectionsEstablished() != 1)
      {
         ThreadTools.sleep(100L);
      }
      
      integerRead = objectInputStreamTwo.readInt();
      assertEquals(integerToWrite, integerRead);
      
      objectInputStreamTwo.close();
      
      integerToWrite = 202;
      establishedConnectionCounter.writeInteger(integerToWrite);

      while(establishedConnectionCounter.getNumberConnectionsEstablished() != 0)
      {
         ThreadTools.sleep(100L);
      }
      
      persistentTCPServer.close();

   }
   
   
   private static class EstablishedConnectionCounter implements EstablishedAConnectionListener
   {
      private int numberConnectionsEstablished = 0;
      private ArrayList<ObjectOutputStream> objectOutputStreams = new ArrayList<ObjectOutputStream>();
      
      public void establishedAConnection(ObjectInputStream objectInputStream, ObjectOutputStream objectOutputStream)
      {
         System.out.println("Established a connnection!!");
         this.objectOutputStreams.add(objectOutputStream);
         
         numberConnectionsEstablished++;
         
         if (objectInputStream != null)
         {
            throw new RuntimeException("Only expecting output stream here!");
         }
      }

      public int getNumberConnectionsEstablished()
      {
         return numberConnectionsEstablished;
      }
      
      public void writeInteger(int integerToWrite)
      {
         System.out.println("Writing Integer!!");

         ArrayList<ObjectOutputStream> deadStreams = new ArrayList<ObjectOutputStream>();
         
         for (ObjectOutputStream objectOutputStream : objectOutputStreams)
         {
            try
            {
               objectOutputStream.writeInt(integerToWrite);
               objectOutputStream.flush();
               System.out.println("Done Writing Integer!!");
            } 
            catch (IOException e)
            {
               System.out.println("Caught Dead Stream!!");
               deadStreams.add(objectOutputStream);
            }
         }
         
         for (ObjectOutputStream deadStream : deadStreams)
         {
            objectOutputStreams.remove(deadStream);
            numberConnectionsEstablished--;
         }
         
      }
   }

}
