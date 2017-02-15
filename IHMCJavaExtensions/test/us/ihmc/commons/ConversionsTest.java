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
}
