package us.ihmc.robotics.filters;

import static org.junit.Assert.assertEquals;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class ZeroLagLowPassFilterTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testZeroFiltering()
   {
      double[] list = new double[10];
      for(int i=0; i<10; i++)
      {
         list[i]=i+ 1;
      }
  
      double[] filteredList = ZeroLagLowPassFilter.getFilteredArray(list, 0.0);
      
      assertEquals(list.length, filteredList.length);
      
      for(int i=0; i < list.length; i++)
      {
         assertEquals(list[i], filteredList[i], 1e-8);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testZeroNonZeroAlpha()
   {
      double[] list = new double[10];
      for(int i=0; i<10; i++)
      {
         list[i]=i+ 1;
      }
  
      double[] filteredList = ZeroLagLowPassFilter.getFilteredArray(list, 0.5);
      
      assertEquals(list.length, filteredList.length);
      
      double[] answer = new double[]{ 1.083089195191860, 2.041178390383720, 3.019856780767441, 4.008463561534882, 5.001302123069763, 5.994791746139526, 6.985677242279053, 7.969401359558106, 8.937826156616211, 9.875164031982422};
      
      assertEquals(answer.length, filteredList.length);

      for(int i=0; i < list.length; i++)
      {
         assertEquals(answer[i], filteredList[i], 1e-8);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testZeroNonZeroAlpha2()
   {
      double[] list = new double[10];
      for(int i=0; i<10; i++)
      {
         list[i]=i+ 1;
      }
  
      double[] filteredList = ZeroLagLowPassFilter.getFilteredArray(list, 0.3);
      
      assertEquals(list.length, filteredList.length);
      
      double[] answer = new double[]{  1.008900871140671, 2.002669570468903, 3.000798568229678, 4.000231894098929, 5.000043980329763, 5.999927901099211, 6.999694060330704, 7.998960518102349, 8.996529155441165, 9.988428746667216};
      
      assertEquals(answer.length, filteredList.length);

      for(int i=0; i < list.length; i++)
      {
         assertEquals(answer[i], filteredList[i], 1e-8);
      }
   }
}
