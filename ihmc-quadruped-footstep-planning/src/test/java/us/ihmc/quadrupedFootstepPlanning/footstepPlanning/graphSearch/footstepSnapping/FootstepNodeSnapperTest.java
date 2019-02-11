package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class FootstepNodeSnapperTest
{
   private final Random random = new Random(320L);
   private final double epsilon = 1e-8;

   private int[] frontLeftXIndices = new int[] {-30, 0, 23, 87, -100, 42};
   private int[] frontRightXIndices = new int[] {-30, 0, 23, 87, -100, 42};
   private int[] hindLeftXIndices = new int[] {-30, 0, 23, 87, -100, 42};
   private int[] hindRightXIndices = new int[] {-30, 0, 23, 87, -100, 42};
   private int[] frontLeftYIndices = new int[] {-35, 0, -777, 87, -50, 28};
   private int[] frontRightYIndices = new int[] {-35, 0, -777, 87, -50, 28};
   private int[] hindLeftYIndices = new int[] {-35, 0, -777, 87, -50, 28};
   private int[] hindRightYIndices = new int[] {-35, 0, -777, 87, -50, 28};

   @Disabled
   @Test
   public void testFootstepCacheing()
   {
      TestSnapper testSnapper = new TestSnapper();
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(new PlanarRegion());
      testSnapper.setPlanarRegions(planarRegionsList);

      for (int i = 0; i < frontLeftXIndices.length; i++)
      {
         for (int j = 0; j < frontLeftYIndices.length; j++)
         {
            for (int k = 0; k < frontRightXIndices.length; k++)
            {
               for (int l = 0; l < frontRightYIndices.length; l++)
               {
                  for (int m = 0; m < hindLeftXIndices.length; m++)
                  {
                     for (int n = 0; n < hindLeftYIndices.length; n++)
                     {
                        for (int o = 0; o < hindRightXIndices.length; o++)
                        {
                           for (int p = 0; p < hindRightYIndices.length; p++)
                           {
                              RobotQuadrant robotQuadrant = RobotQuadrant.generateRandomRobotQuadrant(random);

                              double frontLeftX = FootstepNode.gridSizeXY * frontLeftXIndices[i];
                              double frontLeftY = FootstepNode.gridSizeXY * frontLeftYIndices[j];
                              double frontRightX = FootstepNode.gridSizeXY * frontRightXIndices[k];
                              double frontRightY = FootstepNode.gridSizeXY * frontRightYIndices[l];
                              double hindLeftX = FootstepNode.gridSizeXY * hindLeftXIndices[m];
                              double hindLeftY = FootstepNode.gridSizeXY * hindLeftYIndices[n];
                              double hindRightX = FootstepNode.gridSizeXY * hindRightXIndices[o];
                              double hindRightY = FootstepNode.gridSizeXY * hindRightYIndices[p];

                              double length = 1.0;
                              double width = 0.5;

                              testSnapper.snapFootstepNode(
                                    new FootstepNode(robotQuadrant, frontLeftX, frontLeftY, frontRightX, frontRightY, hindLeftX, hindLeftY, hindRightX,
                                                     hindRightY, length, width));
                              assertTrue(testSnapper.dirtyBit);
                              testSnapper.dirtyBit = false;

                              testSnapper.snapFootstepNode(
                                    new FootstepNode(robotQuadrant, frontLeftX, frontLeftY, frontRightX, frontRightY, hindLeftX, hindLeftY, hindRightX,
                                                      hindRightY, length, width));
                              assertTrue(!testSnapper.dirtyBit);
                           }
                        }
                     }
                  }
               }
            }
         }
      }
   }

   @Test
   public void testWithoutPlanarRegions()
   {
      TestSnapper testSnapper = new TestSnapper();


      for (int i = 0; i < frontLeftXIndices.length; i++)
      {
         for (int j = 0; j < frontLeftYIndices.length; j++)
         {
            for (int k = 0; k < frontRightXIndices.length; k++)
            {
               for (int l = 0; l < frontRightYIndices.length; l++)
               {
                  for (int m = 0; m < hindLeftXIndices.length; m++)
                  {
                     for (int n = 0; n < hindLeftYIndices.length; n++)
                     {
                        for (int o = 0; o < hindRightXIndices.length; o++)
                        {
                           for (int p = 0; p < hindRightYIndices.length; p++)
                           {
                              RobotQuadrant newQuadrant = RobotQuadrant.generateRandomRobotQuadrant(random);

                              double frontLeftX = FootstepNode.gridSizeXY * frontLeftXIndices[i];
                              double frontLeftY = FootstepNode.gridSizeXY * frontLeftYIndices[j];
                              double frontRightX = FootstepNode.gridSizeXY * frontRightXIndices[k];
                              double frontRightY = FootstepNode.gridSizeXY * frontRightYIndices[l];
                              double hindLeftX = FootstepNode.gridSizeXY * hindLeftXIndices[m];
                              double hindLeftY = FootstepNode.gridSizeXY * hindLeftYIndices[n];
                              double hindRightX = FootstepNode.gridSizeXY * hindRightXIndices[o];
                              double hindRightY = FootstepNode.gridSizeXY * hindRightYIndices[p];

                              double length = 1.0;
                              double width = 0.5;

                              FootstepNodeSnapData snapData = testSnapper.snapFootstepNode(
                                    new FootstepNode(newQuadrant, frontLeftX, frontLeftY, frontRightX, frontRightY, hindLeftX, hindLeftY, hindRightX,
                                                     hindRightY, length, width));
                              assertTrue(!testSnapper.dirtyBit);

                              for (RobotQuadrant robotQuadrant : RobotQuadrant.values())
                              {
                                 assertTrue(snapData.getSnapTransform(robotQuadrant).epsilonEquals(new RigidBodyTransform(), epsilon));
                              }
                           }
                        }
                     }
                  }
               }
            }
         }
      }
   }

   private class TestSnapper extends FootstepNodeSnapper
   {
      boolean dirtyBit = false;

      @Override
      protected FootstepNodeSnapData snapInternal(FootstepNode footstepNode)
      {
         dirtyBit = true;
         return FootstepNodeSnapData.emptyData();
      }
   }
}
