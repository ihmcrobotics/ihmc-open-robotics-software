package us.ihmc.robotics.robotSide;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.continuousIntegration.IntegrationCategory;

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
            assertEquals("not FRONT_LEFT", RobotQuadrant.FRONT_LEFT, robotQuadrant);
            break;
         case 1:
            assertEquals("not FRONT_RIGHT", RobotQuadrant.FRONT_RIGHT, robotQuadrant);
            break;
         case 2:
            assertEquals("not HIND_RIGHT", RobotQuadrant.HIND_RIGHT, robotQuadrant);
            break;
         case 3:
            assertEquals("not HIND_LEFT", RobotQuadrant.HIND_LEFT, robotQuadrant);
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
            assertEquals("not FRONT_LEFT", RobotQuadrant.FRONT_LEFT, robotQuadrant);
            break;
         case 1:
            assertEquals("not FRONT_RIGHT", RobotQuadrant.FRONT_RIGHT, robotQuadrant);
            break;
         case 2:
            assertEquals("not HIND_RIGHT", RobotQuadrant.HIND_RIGHT, robotQuadrant);
            break;
         }
      }
   }
}
