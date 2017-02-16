package us.ihmc.robotics.time;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.Conversions;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class TimeToolsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testToSeconds()
   {
      long timestamp = 1500000000;

      assertEquals(1.5, Conversions.nanoSecondstoSeconds(timestamp), 1e-22);

      assertEquals(-1.5, Conversions.nanoSecondstoSeconds(-timestamp), 1e-22);


   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testToNanoSeconds()
   {
      double time = 1.5;

      assertEquals(1500000000, Conversions.secondsToNanoSeconds(time));
      assertEquals(-1500000000, Conversions.secondsToNanoSeconds(-time));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMicroSecondsToNanoseconds()
   {
      long mSecs = 2;

      Random random = new Random();

      for (int i = 0; i < 100; i++)
      {
         mSecs = (long) random.nextFloat() * 1000;
         assertEquals(mSecs * 1e3, Conversions.microSecondsToNanoseconds(mSecs), 1e-6);
         assertEquals(-mSecs * 1e3, Conversions.microSecondsToNanoseconds(-mSecs), 1e-6);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSecondsToMilliseconds()
   {
      long secs = 2;

      Random random = new Random();

      for (int i = 0; i < 100; i++)
      {
         secs = (long) random.nextFloat() * 1000;
         assertEquals(secs * 1e3, Conversions.secondsToMilliSeconds(secs), 1e-6);
         assertEquals(-secs * 1e3, Conversions.secondsToMilliSeconds(-secs), 1e-6);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMillisecondsToSeconds()
   {
      long mSecs = 2;

      Random random = new Random();

      for (int i = 0; i < 100; i++)
      {
         mSecs = (long) random.nextFloat() * 1000;
         assertEquals(mSecs * 1e-3, Conversions.milliSecondsToSeconds(mSecs), 1e-6);
         assertEquals(-mSecs * 1e-3, Conversions.milliSecondsToSeconds(-mSecs), 1e-6);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMillisecondsToMinutes()
   {
      long mSecs = 2;

      Random random = new Random();

      for (int i = 0; i < 100; i++)
      {
         mSecs = (long) random.nextFloat() * 1000;
         assertEquals((mSecs * 1e-3) / 60.0, Conversions.milliSecondsToMinutes(mSecs), 1e-6);
         assertEquals((-mSecs * 1e-3) / 60.0, Conversions.milliSecondsToMinutes(-mSecs), 1e-6);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMillisecondsToNanoSeconds()
   {
      int mSecs = 2;

      Random random = new Random();

      for (int i = 0; i < 100; i++)
      {
         mSecs = (int) random.nextFloat() * 1000;
         assertEquals((mSecs * 1e6), Conversions.milliSecondsToNanoSeconds(mSecs), 1e-6);
         assertEquals((-mSecs * 1e6), Conversions.milliSecondsToNanoSeconds(-mSecs), 1e-6);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMicroSecondsToSeconds()
   {
      int mSecs = 2;

      Random random = new Random();

      for (int i = 0; i < 100; i++)
      {
         mSecs = (int) random.nextFloat() * 1000;
         assertEquals((mSecs * 1e-6), Conversions.microSecondsToSeconds(mSecs), 1e-6);
         assertEquals((-mSecs * 1e-6), Conversions.microSecondsToSeconds(-mSecs), 1e-6);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMinutesToSeconds()
   {
      int mins = 2;

      Random random = new Random();

      for (int i = 0; i < 100; i++)
      {
         mins = (int) random.nextFloat() * 1000;
         assertEquals((mins * 60), Conversions.minutesToSeconds(mins), 1e-6);
         assertEquals((-mins * 60), Conversions.minutesToSeconds(-mins), 1e-6);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSecondsToMinutes()
   {
      int secs = 2;

      Random random = new Random();

      for (int i = 0; i < 100; i++)
      {
         secs = (int) random.nextFloat() * 1000;
         assertEquals((secs / 60.0), Conversions.secondsToMinutes(secs), 1e-6);
         assertEquals((-secs / 60.0), Conversions.secondsToMinutes(-secs), 1e-6);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testNanoSecondsToMilliSeconds()
   {
      int nSecs = 2;

      Random random = new Random();

      for (int i = 0; i < 100; i++)
      {
         nSecs = (int) random.nextFloat() * 1000;
         assertEquals((nSecs * 1e-6), Conversions.nanoSecondsToMillis(nSecs), 1e-6);
         assertEquals((-nSecs * 1e-6), Conversions.nanoSecondsToMillis(-nSecs), 1e-6);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testNanoSecondsToMicroSeconds()
   {
      int nSecs = 2;

      Random random = new Random();

      for (int i = 0; i < 100; i++)
      {
         nSecs = (int) random.nextFloat() * 1000;
         assertEquals((nSecs * 1e-3), Conversions.nanoSecondsToMicroseconds(nSecs), 1e-6);
         assertEquals((-nSecs * 1e-3), Conversions.nanoSecondsToMicroseconds(-nSecs), 1e-6);
      }
   }
}
