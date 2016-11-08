package us.ihmc.llaQuadrupedController;

import static org.junit.Assert.fail;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;

@ContinuousIntegrationPlan(categories = IntegrationCategory.CODE_QUALITY)

public class SimulationConstructionSetDependencyTest
{
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 100000)
   @Ignore
   public void testSimulationConstructionSetIsNotADependency()
   {
      ClassLoader loader = SimulationConstructionSetDependencyTest.class.getClassLoader();
      try
      {
         Class scs = loader.loadClass("us.ihmc.simulationconstructionset.SimulationConstructionSet");
      }
      catch (ClassNotFoundException e)
      {
         return;
      }
      
      fail("us.ihmc.simulationConstructionSet.SimulationConstructionSet is on the LLAQuardupedController classpath");
      
   }
   
}
