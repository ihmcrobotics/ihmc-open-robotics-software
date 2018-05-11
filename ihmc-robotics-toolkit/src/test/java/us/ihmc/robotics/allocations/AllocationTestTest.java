package us.ihmc.robotics.allocations;

import java.util.Collections;
import java.util.List;

import org.junit.Assert;
import org.junit.Test;

import us.ihmc.commons.PrintTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple3D.Vector3D;

public class AllocationTestTest
{
   private enum MyEnum
   {
      A, B, C, D
   };

   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 3000)
   public void testSettingOfVector()
   {
      Vector3D someVector = new Vector3D();
      List<Throwable> allocations = new Tester().runAndCollectAllocations(() -> {
         someVector.set(1.0, 1.0, 1.0);
      });

      Assert.assertEquals(0, allocations.size());
   }

   @SuppressWarnings("unused")
   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 3000)
   public void testAllocationOfArray()
   {
      List<Throwable> allocations = new Tester().runAndCollectAllocations(() -> {
         double[] someArray = new double[12];
      });

      Assert.assertEquals(1, allocations.size());
      Assert.assertTrue(allocations.get(0).getMessage().contains(double.class.getSimpleName() + "[]"));
      PrintTools.info(allocations.get(0).getMessage());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 3000)
   public void testSingleAllocation()
   {
      List<Throwable> allocations = new Tester().runAndCollectAllocations(() -> {
         new Vector3D();
      });

      Assert.assertEquals(1, allocations.size());
      Assert.assertTrue(allocations.get(0).getMessage().contains(Vector3D.class.getSimpleName()));
      PrintTools.info(allocations.get(0).getMessage());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 3000)
   public void testSwitchTable()
   {
      // First time the switch statement is called for an enum a switch table is generated:
      List<Throwable> allocations = new Tester().runAndCollectAllocations(() -> {
         switch (MyEnum.A)
         {
         default:
            break;
         }
      });
      Assert.assertEquals(2, allocations.size());
      allocations.forEach(a -> a.printStackTrace());

      // The second time there are no allocations:
      allocations = new Tester().runAndCollectAllocations(() -> {
         switch (MyEnum.B)
         {
         default:
            break;
         }
      });
      Assert.assertEquals(0, allocations.size());
   }

   private class Tester implements AllocationTest
   {
      @Override
      public List<Class<?>> getClassesOfInterest()
      {
         return Collections.emptyList();
      }

      @Override
      public List<Class<?>> getClassesToIgnore()
      {
         return Collections.emptyList();
      }
   }
}
