package us.ihmc.commons.lists;


import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class PairListTest
{
   @Test
   public void testSimplePairList()
   {
      PairList<LeftObject, RightObject> pairList = new PairList<>();
      pairList.add(new LeftObject(), new RightObject());
      assertTrue(pairList.first(0).yo == 5, "Yo not 5");
      assertTrue(pairList.second(0).hi == 0.1f, "Hi not 0.1f");
      
      for (int i = 1; i < 5; i++)
      {
         pairList.add(new LeftObject(), new RightObject());
         assertTrue(pairList.get(i).left.equals(pairList.first(i)), "Index wrong.");
         assertTrue(pairList.get(i).right.equals(pairList.second(i)), "Index wrong.");
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
