package us.ihmc.commons.robotics.robotSide;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

public class QuadrantDependentListTest
{
   @Test
   public void testMultipleIterations()
   {
      QuadrantDependentList<String> quadrantDependentList = new QuadrantDependentList<>();
      
      for (RobotQuadrant robotQuadrant : quadrantDependentList.keySet())
      {
         System.out.println(quadrantDependentList.get(robotQuadrant));
      }
      
      quadrantDependentList.set(RobotQuadrant.FRONT_LEFT, "first");
      quadrantDependentList.set(RobotQuadrant.FRONT_RIGHT, "second");
      quadrantDependentList.set(RobotQuadrant.HIND_RIGHT, "third");
      quadrantDependentList.set(RobotQuadrant.HIND_LEFT, "fourth");
      
      for (RobotQuadrant robotQuadrant : quadrantDependentList.keySet())
      {
         System.out.println(quadrantDependentList.get(robotQuadrant));
      }
   }
   
   @Test
   public void testIteratorOrdering()
   {
      QuadrantDependentList<String> quadrantDependentList = new QuadrantDependentList<>("1", "2", "3", "4");
      quadrantDependentList.set(RobotQuadrant.FRONT_LEFT, "first");
      quadrantDependentList.set(RobotQuadrant.FRONT_RIGHT, "second");
      quadrantDependentList.set(RobotQuadrant.HIND_RIGHT, "third");
      quadrantDependentList.set(RobotQuadrant.HIND_LEFT, "fourth");
      
      int i = 0;
      for (RobotQuadrant robotQuadrant : quadrantDependentList.keySet())
      {
         switch (i++)
         {
         case 0:
            assertEquals("not first", "first", quadrantDependentList.get(robotQuadrant));
            break;
         case 1:
            assertEquals("not second", "second", quadrantDependentList.get(robotQuadrant));
            break;
         case 2:
            assertEquals("not third", "third", quadrantDependentList.get(robotQuadrant));
            break;
         case 3:
            assertEquals("not fourth", "fourth", quadrantDependentList.get(robotQuadrant));
            break;
         }
      }
      
      quadrantDependentList = new QuadrantDependentList<>();
      quadrantDependentList.set(RobotQuadrant.FRONT_RIGHT, "second");
      quadrantDependentList.set(RobotQuadrant.FRONT_LEFT, "first");
      quadrantDependentList.set(RobotQuadrant.HIND_LEFT, "fourth");
      quadrantDependentList.set(RobotQuadrant.HIND_RIGHT, "third");
      
      i = 0;
      for (RobotQuadrant robotQuadrant : quadrantDependentList.keySet())
      {
         switch (i++)
         {
         case 0:
            assertEquals("first", quadrantDependentList.get(robotQuadrant), "not first");
            break;
         case 1:
            assertEquals("second", quadrantDependentList.get(robotQuadrant), "not second");
            break;
         case 2:
            assertEquals("third", quadrantDependentList.get(robotQuadrant), "not third");
            break;
         case 3:
            assertEquals("fourth", quadrantDependentList.get(robotQuadrant), "not fourth");
            break;
         }
      }
      
      quadrantDependentList = new QuadrantDependentList<>();
      quadrantDependentList.set(RobotQuadrant.FRONT_RIGHT, "second");
      quadrantDependentList.set(RobotQuadrant.FRONT_LEFT, "first");
      quadrantDependentList.set(RobotQuadrant.HIND_LEFT, "fourth");
      quadrantDependentList.set(RobotQuadrant.HIND_RIGHT, "third");
      
      i = 0;
      for (RobotQuadrant robotQuadrant : quadrantDependentList.keySet())
      {
         switch (i++)
         {
         case 0:
            assertEquals(RobotQuadrant.FRONT_LEFT, robotQuadrant, "not FRONT_LEFT");
            break;
         case 1:
            assertEquals(RobotQuadrant.FRONT_RIGHT, robotQuadrant, "not FRONT_RIGHT");
            break;
         case 2:
            assertEquals(RobotQuadrant.HIND_RIGHT, robotQuadrant,"not HIND_RIGHT");
            break;
         case 3:
            assertEquals(RobotQuadrant.HIND_LEFT, robotQuadrant, "not HIND_LEFT");
            break;
         }
      }
      
      quadrantDependentList = new QuadrantDependentList<>();
      quadrantDependentList.set(RobotQuadrant.FRONT_RIGHT, "second");
      quadrantDependentList.set(RobotQuadrant.FRONT_LEFT, "first");
      quadrantDependentList.set(RobotQuadrant.HIND_RIGHT, "third");
      
      i = 0;
      for (RobotQuadrant robotQuadrant : quadrantDependentList.keySet())
      {
         switch (i++)
         {
         case 0:
            assertEquals(RobotQuadrant.FRONT_LEFT, robotQuadrant, "not FRONT_LEFT");
            break;
         case 1:
            assertEquals(RobotQuadrant.FRONT_RIGHT, robotQuadrant, "not FRONT_RIGHT");
            break;
         case 2:
            assertEquals(RobotQuadrant.HIND_RIGHT, robotQuadrant, "not HIND_RIGHT");
            break;
         }
      }
   }
}
