package us.ihmc.communication.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.testing.TestPlanSuite;
import us.ihmc.tools.testing.TestPlanSuite.TestSuiteTarget;
import us.ihmc.tools.testing.TestPlanTarget;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(TestPlanSuite.class)
@TestSuiteTarget(TestPlanTarget.Fast)
@SuiteClasses
({
   us.ihmc.communication.kryo.KryoAnnotationTest.class,
   us.ihmc.communication.net.KryoStreamSerializerTest.class,
   us.ihmc.communication.net.local.InterprocessObjectCommunicatorTest.class,
   us.ihmc.communication.remote.DataObjectTransponderTest.class,
   us.ihmc.communication.streamingData.StreamingDataProducerConsumerTest.class,
   us.ihmc.communication.streamingData.PersistentTCPServerTest.class,
   us.ihmc.communication.streamingData.PersistentTCPClientTest.class
})

public class IHMCCommunicationAFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
