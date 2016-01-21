package us.ihmc.tools.maps;

import static org.junit.Assert.*;

import org.junit.Test;

public class CircularLongmapTest
{
   @Test
   public void testFillingMap()
   {
      CircularLongMap map = new CircularLongMap(100);

      for (int i = 0; i < 100; i++)
      {
         map.insert(i, i * 10);
      }


      for (int i = 0; i < 100; i++)
      {
         long value = map.getValue(false, i);
         assertEquals(i * 10, value);
      }

      for (int i = 100; i < 150; i++)
      {
         map.insert(i, i * 10);
      }
      for (int i = 50; i < 150; i++)
      {
         long value = map.getValue(false, i);
         assertEquals(i * 10, value);
      }
   }

   @Test
   public void testMatchNearest()
   {
      CircularLongMap map = new CircularLongMap(100);

      for (int i = 0; i < 100; i++)
      {
         map.insert(i * 10, i * 50);
      }
      map.print();
      for (int i = 0; i < 100; i++)
      {
         long value = map.getValue(true, i * 10 + 6);
         assertEquals(i * 50, value);
      }

      for (int i = 100; i < 150; i++)
      {
         map.insert(i * 10, i * 50);
      }
      map.print();
      for (int i = 50; i < 150; i++)
      {
         long value = map.getValue(true, i * 10 + 2);
         assertEquals(i * 50, value);
      }
   }

   @Test
   public void testEdgeCases()
   {
      CircularLongMap map = new CircularLongMap(100);
      for (int i = 4; i < 54; i++)
      {
         map.insert(i, i * 10);
      }

      try
      {
         map.getValue(false, 3);
         fail();
      }
      catch (Exception e)
      {

      }
      
      
      assertEquals(40, map.getValue(true, 3));
      
      for(int i = 54; i < 104; i++)
      {
         map.insert(i, i * 10);
      }
      
      assertEquals(40, map.getValue(true, 3));
      assertEquals(40, map.getValue(true, 4));

      
      for(int i = 105; i < 125; i+=2)
      {
         map.insert(i, i*10);
      }
      
      assertEquals(140, map.getValue(true, 13));
      assertEquals(140, map.getValue(true, 14));
      assertEquals(150, map.getValue(true, 15));
      
      assertEquals(1030, map.getValue(true, 103));
      assertEquals(1030, map.getValue(true, 104));
      assertEquals(1050, map.getValue(true, 105));
      
      
   }
}
