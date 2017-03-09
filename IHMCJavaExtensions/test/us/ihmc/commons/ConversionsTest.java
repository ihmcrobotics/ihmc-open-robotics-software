package us.ihmc.commons;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testMagnitudeToDecibels()
   {
      double epsilon = 1e-10;
      assertEquals(20.0, Conversions.convertMagnitudeToDecibels(10.0), epsilon);
      assertEquals(40.0, Conversions.convertMagnitudeToDecibels(100.0), epsilon);
      assertEquals(28.691378080683975, Conversions.convertMagnitudeToDecibels(27.2), epsilon);

      double[] magnitudes = new double[] {10.0, 100.0, 27.2};
      double[] decibels = Conversions.convertMagnitudeToDecibels(magnitudes);

      assertEquals(20.0, decibels[0], epsilon);
      assertEquals(40.0, decibels[1], epsilon);
      assertEquals(28.691378080683975, decibels[2], epsilon);

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testNaN()
   {
      double magnitude = -1.0;
      double decibels = Conversions.convertMagnitudeToDecibels(magnitude);
      assertTrue(Double.isNaN(decibels));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testNegativeInfinity()
   {
      double magnitude = 0.0;
      double decibels = Conversions.convertMagnitudeToDecibels(magnitude);
      assertTrue(Double.isInfinite(decibels));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRadiansToDegrees()
   {
      double[] phaseInRadians = new double[] {0.0, Math.PI / 4.0, Math.PI, Math.PI * 2.0, Math.PI * 4.0};
      double[] phaseInDegrees = Conversions.convertRadianToDegrees(phaseInRadians);

      double epsilon = 1e-10;
      assertEquals(0.0, phaseInDegrees[0], epsilon);
      assertEquals(45.0, phaseInDegrees[1], epsilon);
      assertEquals(180.0, phaseInDegrees[2], epsilon);
      assertEquals(360.0, phaseInDegrees[3], epsilon);
      assertEquals(720.0, phaseInDegrees[4], epsilon);

      phaseInRadians = new double[] {-0.0, -Math.PI / 4.0, -Math.PI, -Math.PI * 2.0, -Math.PI * 4.0};
      phaseInDegrees = Conversions.convertRadianToDegrees(phaseInRadians);

      assertEquals(-0.0, phaseInDegrees[0], epsilon);
      assertEquals(-45.0, phaseInDegrees[1], epsilon);
      assertEquals(-180.0, phaseInDegrees[2], epsilon);
      assertEquals(-360.0, phaseInDegrees[3], epsilon);
      assertEquals(-720.0, phaseInDegrees[4], epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRadiansPerSecondToHz()
   {
      double[] freqInRadPerSecond = new double[] {0.0, Math.PI / 4.0, Math.PI, Math.PI * 2.0, Math.PI * 4.0};
      double[] freqInHz = Conversions.convertRadPerSecondToHz(freqInRadPerSecond);

      double epsilon = 1e-10;
      assertEquals(0.0, freqInHz[0], epsilon);
      assertEquals(0.125, freqInHz[1], epsilon);
      assertEquals(0.5, freqInHz[2], epsilon);
      assertEquals(1.0, freqInHz[3], epsilon);
      assertEquals(2.0, freqInHz[4], epsilon);

      freqInRadPerSecond = new double[] {-0.0, -Math.PI / 4.0, -Math.PI, -Math.PI * 2.0, -Math.PI * 4.0};
      freqInHz = Conversions.convertRadPerSecondToHz(freqInRadPerSecond);

      assertEquals(-0.0, freqInHz[0], epsilon);
      assertEquals(-0.125, freqInHz[1], epsilon);
      assertEquals(-0.5, freqInHz[2], epsilon);
      assertEquals(-1.0, freqInHz[3], epsilon);
      assertEquals(-2.0, freqInHz[4], epsilon);
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(Conversions.class, ConversionsTest.class);
   }
}
