package us.ihmc.commons;

import static org.junit.Assert.assertEquals;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class ConversionsTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void kibibytesToBytes()
   {
      Random rand = new Random();
      for (int i = 0; i < 1000; i++)
      {
         int kibibytes = rand.nextInt();
         assertEquals(Conversions.kibibytesToBytes(kibibytes), kibibytes * 1024, 1e-12);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void kilobytesToBytes()
   {
      Random rand = new Random();
      for (int i = 0; i < 1000; i++)
      {
         int kilobytes = rand.nextInt();
         assertEquals(Conversions.kilobytesToBytes(kilobytes), kilobytes * 1000, 1e-12);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void megabytesToBytes()
   {
      Random rand = new Random();
      for (int i = 0; i < 1000; i++)
      {
         int megabytes = rand.nextInt();
         assertEquals(Conversions.megabytesToBytes(megabytes), megabytes * 1000000, 1e-12);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void mebibytesToBytes()
   {
      Random rand = new Random();
      for (int i = 0; i < 1000; i++)
      {
         int mebibytes = rand.nextInt();
         assertEquals(Conversions.mebibytesToBytes(mebibytes), mebibytes * 1048576, 1e-12);
      }
   }

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

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(Conversions.class, ConversionsTest.class);
   }
}
