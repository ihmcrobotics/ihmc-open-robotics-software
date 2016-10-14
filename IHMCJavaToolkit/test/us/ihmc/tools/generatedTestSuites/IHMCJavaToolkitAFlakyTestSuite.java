package us.ihmc.tools.generatedTestSuites;

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
   us.ihmc.tools.processManagement.ProcessSpawnerTest.class,
   us.ihmc.tools.thread.ThreadToolsTest.class
})

public class IHMCJavaToolkitAFlakyTestSuite
{
   public static void main(String[] args)
   {

   }
}
