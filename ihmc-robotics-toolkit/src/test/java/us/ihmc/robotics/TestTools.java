package us.ihmc.robotics;

import static org.hamcrest.MatcherAssert.*;
import static org.hamcrest.Matchers.*;

public class TestTools
{
   public static void assertEpsilonEquals(double a, double b, double epsilon)
   {
      assertThat(a, closeTo(b, epsilon));
   }

   public static void assertEpsilonEquals(double a, double b, double epsilon, String reason)
   {
      assertThat(reason, a, closeTo(b, epsilon));
   }
}
