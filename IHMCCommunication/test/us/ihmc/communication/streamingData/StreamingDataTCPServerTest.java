package us.ihmc.communication.streamingData;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.thread.ThreadTools;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FLAKY})
public class StreamingDataTCPServerTest
{

	@ContinuousIntegrationTest(estimatedDuration = 1.0)
	@Test(timeout=90000)
   public void testTypicalUsage()
   {
      String hostname = "localhost";
      int port = 2003;
      
      StreamingDataTCPServer streamingDataTCPServer = new StreamingDataTCPServer(port);
      streamingDataTCPServer.startOnAThread();
      
      SimpleStreamingDataProducer simpleStreamingDataProducer = new SimpleStreamingDataProducer();
      streamingDataTCPServer.registerStreamingDataProducer(simpleStreamingDataProducer);
      simpleStreamingDataProducer.startProducingData();
      
//      System.out.println("Creating Client");
      
      StreamingDataTCPClient streamingDataTCPClient = new StreamingDataTCPClient(hostname, port);
      SimpleStreamingDataConsumer simpleStreamingDataConsumer = new SimpleStreamingDataConsumer();
      streamingDataTCPClient.registerStreamingDataConsumer(simpleStreamingDataConsumer);
      
//      System.out.println("Connecting Client to Server");

      streamingDataTCPClient.connectToServer(false);
      
      int numberOfDataObjectsToReceive = 100;
      while(simpleStreamingDataConsumer.getLargestIndexSeen() < numberOfDataObjectsToReceive)
      {
         ThreadTools.sleep(100L);
      }  
      
      streamingDataTCPClient.close();
      streamingDataTCPServer.closeAndBlockTillFullyClosed();
   }

	@ContinuousIntegrationTest(estimatedDuration = 2.2)
	@Test(timeout=90000)
   public void testMultipleProducersAndConsumersRobustly()
   {
      String hostname = "localhost";
      int port = 2003;
      
      StreamingDataTCPServer streamingDataTCPServer = new StreamingDataTCPServer(port);
      streamingDataTCPServer.startOnAThread();
      
      SimpleStreamingDataProducer simpleStreamingDataProducer = new SimpleStreamingDataProducer();
      streamingDataTCPServer.registerStreamingDataProducer(simpleStreamingDataProducer);
      simpleStreamingDataProducer.startProducingData();
      
      int numberOfClients = 100;
      
      ArrayList<StreamingDataTCPClient> clients = new ArrayList<StreamingDataTCPClient>();
      ArrayList<SimpleStreamingDataConsumer> consumers = new ArrayList<SimpleStreamingDataConsumer>();
      
      for (int i=0; i<numberOfClients; i++)
      {
         StreamingDataTCPClient streamingDataTCPClient = new StreamingDataTCPClient(hostname, port);
         streamingDataTCPClient.connectToServer(false);
         
         SimpleStreamingDataConsumer simpleStreamingDataConsumer = new SimpleStreamingDataConsumer();
         streamingDataTCPClient.registerStreamingDataConsumer(simpleStreamingDataConsumer);
         consumers.add(simpleStreamingDataConsumer);
         
         clients.add(streamingDataTCPClient);
      }
     
      int numberOfDataObjectsToReceive = 100;
      int consumerToCheckIndex = 0;
      while(consumerToCheckIndex < consumers.size())
      {
         SimpleStreamingDataConsumer consumer = consumers.get(consumerToCheckIndex);
         if (consumer.getLargestIndexSeen() > numberOfDataObjectsToReceive)
         {
            consumerToCheckIndex++;
         }
         
         ThreadTools.sleep(10L);
      }
      
      streamingDataTCPServer.closeAndBlockTillFullyClosed();
      
      for (StreamingDataTCPClient client : clients)
      {
         client.close();
      }
      
   }

	@ContinuousIntegrationTest(estimatedDuration = 3.1)
	@Test(timeout=90000)
   public void testPersistentConsumerToServerRestart()
   {
      String hostname = "localhost";
      int port = 2003;
      
      StreamingDataTCPServer streamingDataTCPServer = new StreamingDataTCPServer(port);
      streamingDataTCPServer.startOnAThread();
      
      SimpleStreamingDataProducer simpleStreamingDataProducer = new SimpleStreamingDataProducer();
      streamingDataTCPServer.registerStreamingDataProducer(simpleStreamingDataProducer);
      simpleStreamingDataProducer.startProducingData();

      StreamingDataTCPClient streamingDataTCPClient = new StreamingDataTCPClient(hostname, port);
      assertFalse(streamingDataTCPClient.isConnected());
      
      boolean persistentlyStayConnected = true;
      streamingDataTCPClient.connectToServer(persistentlyStayConnected);

      SimpleStreamingDataConsumer simpleStreamingDataConsumer = new SimpleStreamingDataConsumer();
      streamingDataTCPClient.registerStreamingDataConsumer(simpleStreamingDataConsumer);      
     
      int numberOfDataObjectsToReceive = 100;
      
      while(simpleStreamingDataConsumer.getLargestIndexSeen() < numberOfDataObjectsToReceive)
      {
         ThreadTools.sleep(10L);
      }
      
      assertTrue(streamingDataTCPClient.isConnected());

      streamingDataTCPServer.closeAndBlockTillFullyClosed();
      assertFalse(streamingDataTCPClient.isConnected());

      int numberSeenSoFar = simpleStreamingDataConsumer.getLargestIndexSeen();
      int numberOfAdditionalDataObjectsToSee = 200;

      streamingDataTCPServer = new StreamingDataTCPServer(port);
      streamingDataTCPServer.startOnAThread();
      streamingDataTCPServer.registerStreamingDataProducer(simpleStreamingDataProducer);

      boolean done = false;
      
      while(!done)
      {
         int largestIndexSeen = simpleStreamingDataConsumer.getLargestIndexSeen();
//         System.out.println("largestIndexSeen = " + largestIndexSeen);
         done = largestIndexSeen > numberSeenSoFar + numberOfAdditionalDataObjectsToSee;
         
         ThreadTools.sleep(10L);
      }
      assertTrue(streamingDataTCPClient.isConnected());

      streamingDataTCPClient.close(); 
      streamingDataTCPServer.closeAndBlockTillFullyClosed();
   }

   
   
}
