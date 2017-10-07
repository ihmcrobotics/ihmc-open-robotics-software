package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.Axis;

public class AxisTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDirection()
   {
      Axis xAxis = Axis.X;
      Axis yAxis = Axis.Y;
      Axis zAxis = Axis.Z;
      
      assertEquals(xAxis.ordinal(), 0, 1e-7);
      assertEquals(yAxis.ordinal(), 1, 1e-7);
      assertEquals(zAxis.ordinal(), 2, 1e-7);

      assertEquals(3, Axis.values().length);

      boolean touchedX = false;
      boolean touchedY = false;
      boolean touchedZ = false;

      for (Axis axis : Axis.values())
      {
         switch (axis)
         {
            case X :
               touchedX = true;

               break;

            case Y :
               touchedY = true;

               break;

            case Z :
               touchedZ = true;

               break;
         }
      }

      assertTrue(touchedX);
      assertTrue(touchedY);
      assertTrue(touchedZ);

   }

}
