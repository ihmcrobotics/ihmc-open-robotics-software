package us.ihmc.robotics;

public class Assert
{
   static public void assertTrue(String message, boolean condition)
   {
      org.junit.Assert.assertTrue(message, condition);
   }

   static public void assertTrue(boolean condition)
   {
      org.junit.Assert.assertTrue(condition);
   }

   static public void assertFalse(String message, boolean condition)
   {
      org.junit.Assert.assertFalse(message, condition);
   }

   static public void assertFalse(boolean condition)
   {
      org.junit.Assert.assertFalse(condition);
   }

   static public void fail(String message)
   {
      org.junit.Assert.fail(message);
   }

   static public void fail()
   {
      org.junit.Assert.fail();
   }

   static public void assertEquals(String message, Object expected, Object actual)
   {
      org.junit.Assert.assertEquals(message, expected, actual);
   }

   static public void assertEquals(Object expected, Object actual)
   {
      org.junit.Assert.assertEquals(expected, actual);
   }

   static public void assertNotEquals(Object expected, Object actual)
   {
      org.junit.Assert.assertNotEquals(expected, actual);
   }

   static public void assertNotEquals(long first, long second)
   {
      org.junit.Assert.assertNotEquals(first, second);
   }

   static public void assertNotEquals(String message, double first, double second, double delta)
   {
      org.junit.Assert.assertNotEquals(message, first, second);
   }

   static public void assertNotEquals(double first, double second, double delta)
   {
      org.junit.Assert.assertNotEquals(first, second);
   }

   public static void assertArrayEquals(Object[] expecteds, Object[] actuals)
   {
      org.junit.Assert.assertArrayEquals(expecteds, actuals);
   }

   public static void assertArrayEquals(int[] expecteds, int[] actuals)
   {
      org.junit.Assert.assertArrayEquals(expecteds, actuals);
   }

   public static void assertArrayEquals(byte[] expecteds, byte[] actuals)
   {
      org.junit.Assert.assertArrayEquals(expecteds, actuals);
   }

   public static void assertArrayEquals(double[] expecteds, double[] actuals, double delta)
   {
      org.junit.Assert.assertArrayEquals(expecteds, actuals, delta);
   }

   public static void assertArrayEquals(float[] expecteds, float[] actuals, float delta)
   {
      org.junit.Assert.assertArrayEquals(expecteds, actuals, delta);
   }

   public static void assertArrayEquals(String string, double[] data, double[] ds, double delta)
   {
      org.junit.Assert.assertArrayEquals(string, data, ds, delta);
   }

   static public void assertEquals(String message, double expected, double actual, double delta)
   {
      org.junit.Assert.assertEquals(message, expected, actual, delta);
   }

   static public void assertEquals(String message, float expected, float actual, float delta)
   {
      org.junit.Assert.assertEquals(message, expected, actual, delta);
   }

   static public void assertEquals(long expected, long actual)
   {
      org.junit.Assert.assertEquals(expected, actual);
   }

   static public void assertEquals(String message, long expected, long actual)
   {
      org.junit.Assert.assertEquals(message, expected, actual);
   }

   static public void assertEquals(double expected, double actual, double delta)
   {
      org.junit.Assert.assertEquals(expected, actual, delta);
   }

   static public void assertEquals(float expected, float actual, float delta)
   {
      org.junit.Assert.assertEquals(expected, actual, delta);
   }

   static public void assertNotNull(String message, Object object)
   {
      org.junit.Assert.assertNotNull(message, object);
   }

   static public void assertNotNull(Object object)
   {
      org.junit.Assert.assertNotNull(object);
   }

   static public void assertNull(String message, Object object)
   {
      org.junit.Assert.assertNull(message, object);
   }

   static public void assertNull(Object object)
   {
      org.junit.Assert.assertNull(object);
   }

   static public void assertSame(Object expected, Object actual)
   {
      org.junit.Assert.assertSame(expected, actual);
   }

   static public void assertNotSame(Object unexpected, Object actual)
   {
      org.junit.Assert.assertNotSame(unexpected, actual);
   }
}