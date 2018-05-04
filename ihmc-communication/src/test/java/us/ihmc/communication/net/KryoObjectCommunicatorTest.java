package us.ihmc.communication.net;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.io.IOException;
import java.net.BindException;
import java.util.ArrayList;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

import geometry_msgs.msg.dds.PointStamped;
import geometry_msgs.msg.dds.Vector3Stamped;
import org.junit.Test;

import com.esotericsoftware.minlog.Log;

import us.ihmc.communication.packets.Packet;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.commons.thread.ThreadTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FLAKY})
public class KryoObjectCommunicatorTest
{

   // This test causes problems on Linux due to a bug in the way Java does its epoll wrapper
	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout=300000)
   public void testAutomaticReconnect() throws IOException, InterruptedException
   {
      int TCP_PORT = 49152 + (int)(Math.random() * (65535 - 49152));
      Log.set(Log.LEVEL_INFO);
      
      KryoObjectServer testServer = new KryoObjectServer(TCP_PORT, new NetClassList());
      boolean connected = false;
      
      do
      {
    	  try
    	  {
    		  testServer.connect();
    		  connected = true;
    	  }
    	  catch(BindException e)
    	  {
    		  TCP_PORT = 49152 + (int)(Math.random() * (65535 - 49152));
    		  testServer = new KryoObjectServer(TCP_PORT, new NetClassList()); //should find free port
    	  }
      }while(!connected);
      
      final KryoObjectServer server = testServer;
      KryoObjectClient client = new KryoObjectClient("127.0.0.1", TCP_PORT, new NetClassList());
      client.setDelayForReconnect(10);
      final CountDownLatch disconnectLatch = new CountDownLatch(5);

      
      client.setReconnectAutomatically(true);
      

      ConnectionStateListener netStateListener = new ConnectionStateListener()
      {
         
         @Override
         public void disconnected()
         {
            System.out.println("Disconnected " + disconnectLatch.getCount());
            disconnectLatch.countDown();
            if(disconnectLatch.getCount() > 0)
            {
               try
               {
                  ThreadTools.sleep(250);    // Server does not close for a maximum of 250 ms
                  server.connect();
               }
               catch (IOException e)
               {
                  fail();
               }
            }
         }
         
         @Override
         public void connected()
         {
            System.out.println("Connected");
            server.disconnect();
         }
      };
      client.attachStateListener(netStateListener);
      
      client.connect();
      server.connect();
      
      disconnectLatch.await(5L, TimeUnit.SECONDS);
      
      client.disconnect();
      server.disconnect();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000) 
   public void testStateListener() throws IOException, InterruptedException
   {
      int TCP_PORT = 49152 + (int)(Math.random() * (65535 - 49152));
      final CountDownLatch connectLatch = new CountDownLatch(2);
      final CountDownLatch disconnectLatch = new CountDownLatch(2);
      KryoObjectServer server = new KryoObjectServer(TCP_PORT, new NetClassList());
      KryoObjectClient client = new KryoObjectClient("127.0.0.1", TCP_PORT, new NetClassList());
      
      ConnectionStateListener netStateListener = new ConnectionStateListener()
      {
         
         @Override
         public void disconnected()
         {
            disconnectLatch.countDown();
         }
         
         @Override
         public void connected()
         {
            connectLatch.countDown();
         }
      };
      
      boolean connected = false;
      do
      {
	      server.attachStateListener(netStateListener);
	      client.attachStateListener(netStateListener);
	      try
	      {
	    	  server.connect();
	    	  client.connect();
	    	  connected = true;
	      }
	      catch(BindException e)
	      {
	    	  TCP_PORT = 49152 + (int)(Math.random() * (65535 - 49152));
	    	  server = new KryoObjectServer(TCP_PORT, new NetClassList());
	    	  client = new KryoObjectClient("127.0.0.1", TCP_PORT, new NetClassList()); //should find free port
	      }
      
      }while(!connected);
      
      server.disconnect();
      
      assertTrue(connectLatch.await(1, TimeUnit.SECONDS));
      assertTrue(disconnectLatch.await(1, TimeUnit.SECONDS));
      
      client.disconnect();
   }

	@SuppressWarnings("unused")
   @ContinuousIntegrationTest(estimatedDuration = 0.2)
	@Test(timeout=300000)
   public void testConnectionLimiter() throws IOException, InterruptedException
   {
      int TCP_PORT = 49152 + (int)(Math.random() * (65535 - 49152));
      Log.set(Log.LEVEL_ERROR);
      final CountDownLatch latch = new CountDownLatch(5);
      boolean connected = false;
      
      KryoObjectServer server = null;      
      ObjectConsumer<Vector3Stamped> objectListener = new ObjectConsumer<Vector3Stamped>()
      {
         @Override
         public void consumeObject(Vector3Stamped object)
         {
            latch.countDown();
         }
      };
      
      do
      {
    	  server = new KryoObjectServer(TCP_PORT, new NetClassList(Vector3Stamped.class));
    	  server.attachListener(Vector3Stamped.class, objectListener);
    	  server.setMaximumNumberOfConnections(5);
    	  try
    	  {
    		  server.connect();
    		  connected = true;
    	  }
	      catch(BindException e)
	      {
	    	  TCP_PORT = 49152 + (int)(Math.random() * (65535 - 49152));
	      }
      }while(!connected);
      ArrayList<KryoObjectClient> clients = new ArrayList<KryoObjectClient>();
      for(int i = 0; i < 7; i++)
      {
         KryoObjectClient client = new KryoObjectClient("127.0.0.1", TCP_PORT, new NetClassList(Vector3Stamped.class));
         client.connect();
         client.sendTCP(new Object());
         clients.add(client);
      }
      
      assertTrue(latch.await(1, TimeUnit.SECONDS));
      
      for(int i = 0; i < 5; i++)
      {
         assertTrue(clients.get(i).isConnected());
         clients.get(i).disconnect();
      }
      
      for(int i = 5; i < 7; i++)
      {
         assertFalse(clients.get(i).isConnected());
      }
      
      assertTrue(latch.await(1, TimeUnit.SECONDS));
      
      KryoObjectClient temp = clients.remove(0);
      temp = null;
      
      System.gc();
      
      KryoObjectClient client = new KryoObjectClient("127.0.0.1", TCP_PORT,  new NetClassList(Vector3Stamped.class));
      client.connect();
      assertTrue(client.isConnected());
      
      
      client.disconnect();
      server.disconnect();
      
      
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout=300000)
   public void testSendAndReceive() throws IOException, InterruptedException
   {
      int TCP_PORT = 49152 + (int)(Math.random() * (65535 - 49152));
      Log.set(Log.LEVEL_ERROR);
      int objectsToSend = 1000;
      final CountDownLatch latch = new CountDownLatch(objectsToSend * 4);
      
      NetClassList netClassList = new NetClassList();
      netClassList.registerPacketClass(TypeA.class);
      netClassList.registerPacketClass(TypeB.class);
      netClassList.registerPacketField(Vector3D.class);
      KryoObjectServer server = new KryoObjectServer(TCP_PORT, netClassList);
      KryoObjectClient client = new KryoObjectClient("127.0.0.1", TCP_PORT, netClassList);
      
      boolean connected = false;
      do
      {
	      try
	      {
	    	  server.connect();
	    	  client.connect();
	    	  connected = true;
	      }
	      catch (BindException e)
	      {
	    	  TCP_PORT = 49152 + (int)(Math.random() * (65535 - 49152));
	    	  server = new KryoObjectServer(TCP_PORT, netClassList);
	    	  client = new KryoObjectClient("127.0.0.1", TCP_PORT, netClassList); //should find free port
	      }
      }while(!connected);
      
      assertTrue(server.isConnected());
      assertTrue(client.isConnected());
      
      final TypeA aObjectToSend = new TypeA();
      aObjectToSend.a = "@!aedsf";
      aObjectToSend.b = 1;
      aObjectToSend.c = 0.2;
      aObjectToSend.testVector = new Vector3D(3.2, 1.1, 2.2);
      
      final TypeB bObjectToSend = new TypeB();
      bObjectToSend.a = 0.23;
      bObjectToSend.b = 2;
      bObjectToSend.c = 23.0f;
      
      
      ObjectConsumer<TypeA> aListener = new ObjectConsumer<TypeA>()
      {

         @Override
         public void consumeObject(TypeA object)
         {
            assertTrue(aObjectToSend.equals(object));
            latch.countDown();
         }
      };
      ObjectConsumer<TypeB> bListener = new ObjectConsumer<TypeB>()
            {
         
         @Override
         public void consumeObject(TypeB object)
         {
            assertTrue(bObjectToSend.equals(object));
            latch.countDown();
         }
      };
      server.attachListener(TypeA.class, aListener);
      server.attachListener(TypeB.class, bListener);
      client.attachListener(TypeA.class, aListener);
      client.attachListener(TypeB.class, bListener);
      
      
      for(int i = 0; i < objectsToSend; i++)
      {
         client.consumeObject(aObjectToSend);
         client.consumeObject(bObjectToSend);
         server.consumeObject(aObjectToSend);
         server.consumeObject(bObjectToSend);
      }
      
      assertTrue(latch.await(1, TimeUnit.SECONDS));
      
      client.disconnect();
      Thread.sleep(100);
      assertFalse(server.isConnected());
      assertFalse(client.isConnected());
      server.disconnect();
      
   }

   // Member classes need to be static in order for deserialization to work
   private static class TypeA extends Packet<TypeA>
   {
      public String a;
      public int b;
      public double c;
      public Vector3D testVector;
      
      @Override
      public String toString()
      {
         return a + b + c;
      }
      
      public boolean equals(TypeA other)
      {
         return other.a.equals(a) && other.b == b && other.c == c && other.testVector.equals(testVector);
      }

      @Override
      public boolean epsilonEquals(TypeA other, double epsilon)
      {
         return other.a.equals(a) && other.b == b && other.c == c && other.testVector.equals(testVector);
      }

      @Override
      public void set(TypeA other)
      {
         this.a = other.a;
         this.b = other.b;
         this.c = other.c;
         this.testVector = other.testVector;
      }
   }
   
   private static class TypeB extends Packet<TypeB>
   {
      public double a;
      public int b;
      public float c;
      
      @Override
      public String toString()
      {
         return a + " | " + b + " | " + c;
      }
      
      public boolean equals(TypeB other)
      {
         return other.a == a && other.b == b && other.c == c;
      }

      @Override
      public boolean epsilonEquals(TypeB other, double epsilon)
      {
         return other.a == a && other.b == b && other.c == c;
      }

      @Override
      public void set(TypeB other)
      {
         this.a = other.a;
         this.b = other.b;
         this.c = other.c;
      }
   }
}
