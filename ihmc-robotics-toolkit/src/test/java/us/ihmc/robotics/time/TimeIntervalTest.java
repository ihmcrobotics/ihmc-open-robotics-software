package us.ihmc.robotics.time;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.time.TimeInterval;

import static us.ihmc.robotics.Assert.*;

public class TimeIntervalTest
{
   @Test
   public void testAccessors()
   {
      double epsilon = 1e-6;

      TimeInterval timeInterval = new TimeInterval(0.0, 1.0);
      assertEquals(timeInterval.getStartTime(), 0.0, epsilon);
      assertEquals(timeInterval.getEndTime(), 1.0, epsilon);
      assertEquals(timeInterval.getDuration(), 1.0, epsilon);

      timeInterval.setInterval(2.0, 3.0);
      assertEquals(timeInterval.getStartTime(), 2.0, epsilon);
      assertEquals(timeInterval.getEndTime(), 3.0, epsilon);
      assertEquals(timeInterval.getDuration(), 1.0, epsilon);

      timeInterval.setStartTime(4.0);
      timeInterval.setEndTime(5.0);
      assertEquals(timeInterval.getStartTime(), 4.0, epsilon);
      assertEquals(timeInterval.getEndTime(), 5.0, epsilon);
      assertEquals(timeInterval.getDuration(), 1.0, epsilon);

      TimeInterval other = new TimeInterval(6.0, 7.0);
      timeInterval.set(other);
      assertTrue(timeInterval.epsilonEquals(other, epsilon));
   }

   @Test
   public void testShiftInterval()
   {
      double epsilon = 1e-6;

      TimeInterval timeInterval = new TimeInterval(0.0, 1.0);
      timeInterval.shiftInterval(10.0);
      assertEquals(timeInterval.getStartTime(), 10.0, epsilon);
      assertEquals(timeInterval.getEndTime(), 11.0, epsilon);
   }
}
