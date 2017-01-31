package us.ihmc.jMonkeyEngineToolkit.jme;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.jMonkeyEngineToolkit.examples.Graphics3DAdapterExampleOne;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphics3DAdapter;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

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
