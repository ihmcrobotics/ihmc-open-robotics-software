package us.ihmc.communication.streamingData;

import static junit.framework.Assert.assertTrue;
import static org.junit.Assert.assertEquals;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.thread.ThreadTools;

public class PersistentTCPClientTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout = 30000)
   public void test() throws IOException
   {
      String serverIPAddress = "localHost";
      int serverPort = 2009;
      
      EstablishedConnectionCounter establishedConnectionCounter = new EstablishedConnectionCounter();
      
      PersistentTCPClient persistentTCPClient = new PersistentTCPClient(serverIPAddress, serverPort, establishedConnectionCounter);
      persistentTCPClient.connectToServer(true);
      
      assertEquals(0, establishedConnectionCounter.getNumberConnectionsEstablished());
      
      int integerToRead = 77;
      
      ServerSocket serverSocket = new ServerSocket(serverPort);
      Socket socket = serverSocket.accept();
      OutputStream outputStream = socket.getOutputStream();
      ObjectOutputStream objectOutputStream = new ObjectOutputStream(outputStream);

      // wait until a connection is established
      while(establishedConnectionCounter.getNumberConnectionsEstablished() < 1)
      {
         ThreadTools.sleep(100L);
      }
      assertEquals(1, establishedConnectionCounter.getNumberConnectionsEstablished());
      assertTrue(persistentTCPClient.isConnected());
      
      objectOutputStream.writeInt(integerToRead);
      objectOutputStream.flush();
      
      int integerRead = establishedConnectionCounter.readInteger();
      assertEquals(integerToRead, integerRead);
      
      objectOutputStream.close();
      outputStream.close();
      socket.close();

      
      persistentTCPClient.notifyConnectionBroke();

      while(persistentTCPClient.isConnected())
      {
         ThreadTools.sleep(100L);
      }
      
      socket = serverSocket.accept();
      outputStream = socket.getOutputStream();
      objectOutputStream = new ObjectOutputStream(outputStream);
      
      // wait until the connection is automatically re-established
      while(establishedConnectionCounter.getNumberConnectionsEstablished() < 2)
      {
         ThreadTools.sleep(100L);
      }
      assertEquals(2, establishedConnectionCounter.getNumberConnectionsEstablished());
      assertTrue(persistentTCPClient.isConnected());

      
      objectOutputStream.writeInt(2*integerToRead);
      objectOutputStream.flush();
      
      int nextIntegerRead = establishedConnectionCounter.readInteger();
      assertEquals(2*integerToRead, nextIntegerRead);
   }
   

   private static class EstablishedConnectionCounter implements EstablishedAConnectionListener
   {
      private int numberConnectionsEstablished = 0;
      private ObjectInputStream objectInputStream;
      
      public void establishedAConnection(ObjectInputStream objectInputStream, ObjectOutputStream objectOutputStream)
      {
         System.out.println("Established a connnection!!");
         this.objectInputStream = objectInputStream;
         
         numberConnectionsEstablished++;
         
         if (objectOutputStream != null)
         {
            throw new RuntimeException("Only expecting input stream here!");
         }
      }

      public int getNumberConnectionsEstablished()
      {
         return numberConnectionsEstablished;
      }
      
      public int readInteger() throws IOException
      {
         System.out.println("Reading Integer!!");

         int returnInteger = objectInputStream.readInt();
         
         System.out.println("Done Reading Integer!!");

         return returnInteger;
      }
   }
}
