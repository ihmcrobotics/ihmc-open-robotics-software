package us.ihmc.tools.color;

import static org.junit.Assert.assertTrue;

import java.awt.Color;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.thread.RunnableThatThrows;

@ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class GradientTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCreateGradient()
   {
      Color[] gradient = Gradient.createGradient(Color.BLUE, Color.YELLOW, 5);
      
      assertTrue("Color[" + 0 + "] not correct: " + gradient[0], gradient[0].equals(new Color(0, 0, 255)));
      assertTrue("Color[" + 1 + "] not correct: " + gradient[1], gradient[1].equals(new Color(51, 51, 204)));
      assertTrue("Color[" + 2 + "] not correct: " + gradient[2], gradient[2].equals(new Color(102, 102, 153)));
      assertTrue("Color[" + 3 + "] not correct: " + gradient[3], gradient[3].equals(new Color(153, 153, 102)));
      assertTrue("Color[" + 4 + "] not correct: " + gradient[4], gradient[4].equals(new Color(204, 204, 51)));
   }
   
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCreateMultiGradient()
   {
      Color[] gradient = Gradient.createMultiGradient(new Color[]{Color.BLUE, Color.YELLOW}, 5);
      
      assertTrue("Color[" + 0 + "] not correct: " + gradient[0], gradient[0].equals(new Color(0, 0, 255)));
      assertTrue("Color[" + 1 + "] not correct: " + gradient[1], gradient[1].equals(new Color(51, 51, 204)));
      assertTrue("Color[" + 2 + "] not correct: " + gradient[2], gradient[2].equals(new Color(102, 102, 153)));
      assertTrue("Color[" + 3 + "] not correct: " + gradient[3], gradient[3].equals(new Color(153, 153, 102)));
      assertTrue("Color[" + 4 + "] not correct: " + gradient[4], gradient[4].equals(new Color(204, 204, 51)));
      
      JUnitTools.assertExceptionThrown(IllegalArgumentException.class, new RunnableThatThrows()
      {
         @Override
         public void run() throws Throwable
         {
            Gradient.createMultiGradient(new Color[]{Color.BLUE}, 5);
         }
      });
   }
}
