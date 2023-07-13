package us.ihmc.robotics;

/**
 * @deprecated You should use {@link org.junit.jupiter.api.Assertions} directly instead.
 *             This was created as part of the transition from JUnit 4 to JUnit 5 and should
 *             be removed.
 */
public class Assert
{
   @Deprecated
   static public void assertTrue(String message, boolean condition)
   {
      org.junit.jupiter.api.Assertions.assertTrue(condition, message);
   }

   @Deprecated
   static public void assertTrue(boolean condition)
   {
      org.junit.jupiter.api.Assertions.assertTrue(condition);
   }

   @Deprecated
   static public void assertFalse(String message, boolean condition)
   {
      org.junit.jupiter.api.Assertions.assertFalse(condition, message);
   }

   @Deprecated
   static public void assertFalse(boolean condition)
   {
      org.junit.jupiter.api.Assertions.assertFalse(condition);
   }

   @Deprecated
   static public void fail(String message)
   {
      org.junit.jupiter.api.Assertions.fail(message);
   }

   @Deprecated
   static public void fail()
   {
      org.junit.jupiter.api.Assertions.fail();
   }

   @Deprecated
   static public void assertEquals(String message, Object expected, Object actual)
   {
      org.junit.jupiter.api.Assertions.assertEquals(expected, actual, message);
   }

   @Deprecated
   static public void assertEquals(Object expected, Object actual)
   {
      org.junit.jupiter.api.Assertions.assertEquals(expected, actual);
   }

   @Deprecated
   static public void assertNotEquals(Object expected, Object actual)
   {
      org.junit.jupiter.api.Assertions.assertNotEquals(expected, actual);
   }

   @Deprecated
   static public void assertNotEquals(long first, long second)
   {
      org.junit.jupiter.api.Assertions.assertNotEquals(first, second);
   }

   @Deprecated
   static public void assertNotEquals(String message, double first, double second, double delta)
   {
      org.junit.jupiter.api.Assertions.assertNotEquals(first, second, message);
   }

   @Deprecated
   static public void assertNotEquals(double first, double second, double delta)
   {
      org.junit.jupiter.api.Assertions.assertNotEquals(first, second);
   }

   @Deprecated
   public static void assertArrayEquals(Object[] expecteds, Object[] actuals)
   {
      org.junit.jupiter.api.Assertions.assertArrayEquals(expecteds, actuals);
   }

   @Deprecated
   public static void assertArrayEquals(int[] expecteds, int[] actuals)
   {
      org.junit.jupiter.api.Assertions.assertArrayEquals(expecteds, actuals);
   }

   @Deprecated
   public static void assertArrayEquals(byte[] expecteds, byte[] actuals)
   {
      org.junit.jupiter.api.Assertions.assertArrayEquals(expecteds, actuals);
   }

   @Deprecated
   public static void assertArrayEquals(double[] expecteds, double[] actuals, double delta)
   {
      if (delta == 0.0)
         org.junit.jupiter.api.Assertions.assertArrayEquals(expecteds, actuals);
      else
         org.junit.jupiter.api.Assertions.assertArrayEquals(expecteds, actuals, delta);
   }

   @Deprecated
   public static void assertArrayEquals(float[] expecteds, float[] actuals, float delta)
   {
      if (delta == 0.0)
         org.junit.jupiter.api.Assertions.assertArrayEquals(expecteds, actuals);
      else
         org.junit.jupiter.api.Assertions.assertArrayEquals(expecteds, actuals, delta);
   }

   @Deprecated
   public static void assertArrayEquals(String string, double[] data, double[] ds, double delta)
   {
      if (delta == 0.0)
         org.junit.jupiter.api.Assertions.assertArrayEquals(data, ds, string);
      else
         org.junit.jupiter.api.Assertions.assertArrayEquals(data, ds, delta, string);
   }

   @Deprecated
   static public void assertEquals(String message, double expected, double actual, double delta)
   {
      if (delta == 0.0)
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual, message);
      else
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual, delta, message);
   }

   @Deprecated
   static public void assertEquals(String message, float expected, float actual, float delta)
   {
      if (delta == 0.0)
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual, message);
      else
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual, delta, message);
   }

   @Deprecated
   static public void assertEquals(long expected, long actual)
   {
      org.junit.jupiter.api.Assertions.assertEquals(expected, actual);
   }

   @Deprecated
   static public void assertEquals(String message, long expected, long actual)
   {
      org.junit.jupiter.api.Assertions.assertEquals(expected, actual, message);
   }

   @Deprecated
   static public void assertEquals(double expected, double actual, double delta)
   {
      if (delta == 0.0)
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual);
      else
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual, delta);
   }

   @Deprecated
   static public void assertEquals(float expected, float actual, float delta)
   {
      if (delta == 0.0)
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual);
      else
         org.junit.jupiter.api.Assertions.assertEquals(expected, actual, delta);
   }

   @Deprecated
   static public void assertNotNull(String message, Object object)
   {
      org.junit.jupiter.api.Assertions.assertNotNull(object, message);
   }

   @Deprecated
   static public void assertNotNull(Object object)
   {
      org.junit.jupiter.api.Assertions.assertNotNull(object);
   }

   @Deprecated
   static public void assertNull(String message, Object object)
   {
      org.junit.jupiter.api.Assertions.assertNull(object, message);
   }

   @Deprecated
   static public void assertNull(Object object)
   {
      org.junit.jupiter.api.Assertions.assertNull(object);
   }

   @Deprecated
   static public void assertSame(Object expected, Object actual)
   {
      org.junit.jupiter.api.Assertions.assertSame(expected, actual);
   }

   @Deprecated
   static public void assertNotSame(Object unexpected, Object actual)
   {
      org.junit.jupiter.api.Assertions.assertNotSame(unexpected, actual);
   }
}
