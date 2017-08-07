package us.ihmc.jMonkeyEngineToolkit.jme;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.jMonkeyEngineToolkit.examples.Graphics3DAdapterExampleOne;

@ContinuousIntegrationPlan(categories={IntegrationCategory.UI})
public class JMEGraphics3dAdapterTest
{

	@ContinuousIntegrationTest(estimatedDuration = 6.2)
	@Test(timeout = 31000)
   public void testSimpleObject()
   {
      JMEGraphics3DAdapter renderer = new JMEGraphics3DAdapter();
      Graphics3DAdapterExampleOne example1 = new Graphics3DAdapterExampleOne();

      assertTrue(example1.doExample(renderer));
   }

}
