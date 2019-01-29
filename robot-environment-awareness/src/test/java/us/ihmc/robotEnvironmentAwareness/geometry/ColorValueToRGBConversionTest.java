package us.ihmc.robotEnvironmentAwareness.geometry;

import static org.junit.Assert.assertTrue;

import java.awt.Color;
import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.StereoVisionPointCloudViewer;

public class ColorValueToRGBConversionTest
{
   private static final int NUMBER_OF_TESTS = 100;
   private static final double EPS = 1.0e-4;
   private static final double rgbScale = 255.0;

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testRandomColor()
   {
      Random random = new Random(0612L);

      for (int i = 0; i < NUMBER_OF_TESTS; i++)
      {
         int randomColorValue = random.nextInt();
         Color randomColor = new Color(randomColorValue);
         javafx.scene.paint.Color convertedRandomColor = StereoVisionPointCloudViewer.intToColor(randomColorValue);

         assertTrue("Red color conversion is working incorrect.", Math.abs(convertedRandomColor.getRed() * rgbScale - randomColor.getRed()) < EPS);
         assertTrue("Green color conversion is working incorrect.", Math.abs(convertedRandomColor.getGreen() * rgbScale - randomColor.getGreen()) < EPS);
         assertTrue("Blue color conversion is working incorrect.", Math.abs(convertedRandomColor.getBlue() * rgbScale - randomColor.getBlue()) < EPS);
      }
   }
}
