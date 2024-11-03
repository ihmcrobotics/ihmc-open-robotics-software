package us.ihmc.tools.lists;

import org.junit.jupiter.api.Test;

import static us.ihmc.robotics.Assert.assertTrue;

public class TripleListTest
{
   @Test
   public void testSimpleTripleList()
   {
      TripleList<LeftObject, MiddleObject, RightObject> tripleList = new TripleList<>();
      tripleList.add(new LeftObject(), new MiddleObject(), new RightObject());
      assertTrue("Yo not 5", tripleList.first(0).yo == 5);
      assertTrue("Fi not 6.7f", tripleList.second(0).fi == 6.7f);
      assertTrue("Hi not 0.1f", tripleList.third(0).hi == 0.1f);
      
      for (int i = 1; i < 5; i++)
      {
         tripleList.add(new LeftObject(), new MiddleObject(), new RightObject());
         assertTrue("Index wrong.", tripleList.get(i).left.equals(tripleList.first(i)));
         assertTrue("Index wrong.", tripleList.get(i).middle.equals(tripleList.second(i)));
         assertTrue("Index wrong.", tripleList.get(i).right.equals(tripleList.third(i)));
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
