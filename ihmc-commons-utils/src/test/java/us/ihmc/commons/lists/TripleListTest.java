package us.ihmc.commons.lists;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class TripleListTest
{
   @Test
   public void testSimpleTripleList()
   {
      TripleList<LeftObject, MiddleObject, RightObject> tripleList = new TripleList<>();
      tripleList.add(new LeftObject(), new MiddleObject(), new RightObject());
      assertTrue(tripleList.first(0).yo == 5, "Yo not 5");
      assertTrue(tripleList.second(0).fi == 6.7f, "Fi not 6.7f");
      assertTrue(tripleList.third(0).hi == 0.1f, "Hi not 0.1f");
      
      for (int i = 1; i < 5; i++)
      {
         tripleList.add(new LeftObject(), new MiddleObject(), new RightObject());
         assertTrue(tripleList.get(i).left.equals(tripleList.first(i)), "Index wrong.");
         assertTrue(tripleList.get(i).middle.equals(tripleList.second(i)), "Index wrong.");
         assertTrue(tripleList.get(i).right.equals(tripleList.third(i)) ,"Index wrong.");
      }
   }
   
   private class LeftObject
   {
      int yo = 5;
   }

   private class MiddleObject
   {
      double fi = 6.7f;
   }
   
   private class RightObject
   {
      float hi = 0.1f;
   }
}
