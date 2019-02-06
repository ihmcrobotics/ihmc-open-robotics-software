package us.ihmc.robotEnvironmentAwareness.geometry;

import static us.ihmc.robotics.Assert.*;

import java.awt.Color;
import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.continuousIntegration.ContinuousIntegrationSuite.ContinuousIntegrationSuiteCategory;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.StereoVisionPointCloudViewer;

@ContinuousIntegrationSuiteCategory(IntegrationCategory.FAST)
public class ColorValueToRGBConversionTest
{
   private static final int NUMBER_OF_TESTS = 100;
   private static final double EPS = 1.0e-4;
   private static final double rgbScale = 255.0;

   @Test
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
