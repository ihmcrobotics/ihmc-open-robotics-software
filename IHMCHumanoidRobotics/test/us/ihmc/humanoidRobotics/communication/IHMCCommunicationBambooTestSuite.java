package us.ihmc.humanoidRobotics.communication;

import org.junit.runner.RunWith;
import org.junit.runners.Suite;
import org.junit.runners.Suite.SuiteClasses;

@RunWith(Suite.class)
@SuiteClasses
({
   
   us.ihmc.communication.streamingData.PersistentTCPClientTest.class, 
   us.ihmc.communication.streamingData.PersistentTCPServerTest.class,
   us.ihmc.communication.streamingData.StreamingDataProducerConsumerTest.class, 
   us.ihmc.communication.streamingData.StreamingDataTCPServerTest.class,
   us.ihmc.communication.net.KryoObjectCommunicatorTest.class, 
   us.ihmc.humanoidRobotics.communication.remote.serialization.JointConfigurationDataSenderTest.class, 
   
   us.ihmc.humanoidRobotics.communication.networkProcessor.NetworkProcessorTest.class,   
})
public class IHMCCommunicationBambooTestSuite
{
   public static void main(String[] args)
   {
   }
}
