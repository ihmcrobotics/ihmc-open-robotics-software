package us.ihmc.systemIdentification.generatedTestSuites;

import org.junit.runner.RunWith;
import org.junit.runners.Suite.SuiteClasses;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationSuite.ContinuousIntegrationSuiteCategory;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

/** WARNING: AUTO-GENERATED FILE. DO NOT MAKE MANUAL CHANGES TO THIS FILE. **/
@RunWith(ContinuousIntegrationSuite.class)
@ContinuousIntegrationSuiteCategory(IntegrationCategory.FAST)
@SuiteClasses
({
   us.ihmc.systemIdentification.frictionId.frictionModels.AsymmetricCoulombViscousFrictionModelTest.class,
   us.ihmc.systemIdentification.frictionId.frictionModels.AsymmetricCoulombViscousStribeckFrictionModelTest.class,
   us.ihmc.systemIdentification.frictionId.frictionModels.NoCompensationFrictionModelTest.class,
   us.ihmc.systemIdentification.frictionId.frictionModels.PressureBasedFrictionModelTest.class
})

public class IHMCSystemIdentificationAFastTestSuite
{
   public static void main(String[] args)
   {

   }
}
