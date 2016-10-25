package us.ihmc.communication.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite.ContinuousIntegrationSuiteCategory;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(ContinuousIntegrationSuite.class)
@ContinuousIntegrationSuiteCategory(IntegrationCategory.FLAKY)
@SuiteClasses
({
   us.ihmc.communication.net.KryoObjectCommunicatorTest.class,
   us.ihmc.communication.remote.DataObjectTransponderTest.class,
   us.ihmc.communication.streamingData.StreamingDataTCPServerTest.class
})

public class IHMCCommunicationAFlakyTestSuite
{
   public static void main(String[] args)
   {

   }
}
