package us.ihmc.robotics.controllers.stiction;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class ConstantStictionModelTest
{
   private static final int iters = 100;
   private static final double epsilon = 1e-8;

   @Test
   public void testGetStictionMagnitude()
   {
      YoRegistry registry = new YoRegistry("test");
      YoDouble constantStictionProvider = new YoDouble("constantStictionProvider", registry);
      StictionModel stictionModel = new ConstantStictionModel(constantStictionProvider);

      assertEquals(0.0, stictionModel.getStictionMagnitude(), epsilon);

      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         double expectedMagnitude = RandomNumbers.nextDouble(random, 0.0, 1000.0);
         constantStictionProvider.set(expectedMagnitude);

         assertEquals(expectedMagnitude, stictionModel.getStictionMagnitude(), epsilon);
      }
   }
}
