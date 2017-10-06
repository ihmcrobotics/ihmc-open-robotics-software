package us.ihmc.communication.streamingData;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.thread.ThreadTools;

public class StreamingDataProducerConsumerTest
{

	@ContinuousIntegrationTest(estimatedDuration = 1.1)
	@Test(timeout = 30000)
   public void testTypicalUsage()
   {
      SimpleStreamingDataProducer simpleStreamingDataProducer = new SimpleStreamingDataProducer();
      SimpleStreamingDataConsumer simpleStreamingDataConsumer = new SimpleStreamingDataConsumer();
      simpleStreamingDataProducer.registerConsumer(simpleStreamingDataConsumer);
      
      simpleStreamingDataProducer.startProducingData();
      
      while(simpleStreamingDataConsumer.getLargestIndexSeen() < 100)
      {
         ThreadTools.sleep(100L);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 1.1)
	@Test(timeout = 30000)
   public void testMultipleProducersAndConsumers()
   {
      SimpleStreamingDataProducer simpleStreamingDataProducerA = new SimpleStreamingDataProducer();
      SimpleStreamingDataProducer simpleStreamingDataProducerB = new SimpleStreamingDataProducer();
      
      SimpleStreamingDataConsumer simpleStreamingDataConsumerAOne = new SimpleStreamingDataConsumer();
      SimpleStreamingDataConsumer simpleStreamingDataConsumerATwo = new SimpleStreamingDataConsumer();
      
      SimpleStreamingDataConsumer simpleStreamingDataConsumerBOne = new SimpleStreamingDataConsumer();
      SimpleStreamingDataConsumer simpleStreamingDataConsumerBTwo = new SimpleStreamingDataConsumer();
      
      simpleStreamingDataProducerA.registerConsumer(simpleStreamingDataConsumerAOne);
      simpleStreamingDataProducerA.registerConsumer(simpleStreamingDataConsumerATwo);
      
      simpleStreamingDataProducerB.registerConsumer(simpleStreamingDataConsumerBOne);
      simpleStreamingDataProducerB.registerConsumer(simpleStreamingDataConsumerBTwo);
      
      simpleStreamingDataProducerA.startProducingData();
      simpleStreamingDataProducerB.startProducingData();
      
      boolean done = false;
      
      while(!done)
      {
         ThreadTools.sleep(100L);
         done = (simpleStreamingDataConsumerAOne.getLargestIndexSeen() > 100) && (simpleStreamingDataConsumerATwo.getLargestIndexSeen() > 100) && 
               (simpleStreamingDataConsumerBOne.getLargestIndexSeen() > 100) && (simpleStreamingDataConsumerBTwo.getLargestIndexSeen() > 100); 
      }
   }
   

}
