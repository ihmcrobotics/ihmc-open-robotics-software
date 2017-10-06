package us.ihmc.tools.lists;

import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class PairListTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimplePairList()
   {
      PairList<LeftObject, RightObject> pairList = new PairList<>();
      pairList.add(new LeftObject(), new RightObject());
      assertTrue("Yo not 5", pairList.first(0).yo == 5);
      assertTrue("Hi not 0.1f", pairList.second(0).hi == 0.1f);
      
      for (int i = 1; i < 5; i++)
      {
         pairList.add(new LeftObject(), new RightObject());
         assertTrue("Index wrong.", pairList.get(i).left.equals(pairList.first(i)));
         assertTrue("Index wrong.", pairList.get(i).right.equals(pairList.second(i)));
      }
   }
   
   private class LeftObject
   {
      int yo = 5;
   }
   
   private class RightObject
   {
      float hi = 0.1f;
   }
}
