package us.ihmc.robotics.geometry;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class DirectionTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testDirection()
   {
      Direction xDirection = Direction.X;
      Direction yDirection = Direction.Y;
      Direction zDirection = Direction.Z;
      
      assertEquals(xDirection.getIndex(), 0, 1e-7);
      assertEquals(yDirection.getIndex(), 1, 1e-7);
      assertEquals(zDirection.getIndex(), 2, 1e-7);

      assertEquals(3, Direction.values().length);
      assertEquals(2, Direction.values2D().length);

      boolean touchedX = false;
      boolean touchedY = false;
      boolean touchedZ = false;

      for (Direction direction : Direction.values())
      {
         switch (direction)
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

      touchedX = false;
      touchedY = false;
      touchedZ = false;

      for (Direction direction : Direction.values2D())
      {
         switch (direction)
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
      assertFalse(touchedZ);

   }

}
