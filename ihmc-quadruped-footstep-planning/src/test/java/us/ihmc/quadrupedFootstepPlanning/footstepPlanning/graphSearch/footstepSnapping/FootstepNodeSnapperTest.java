package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.Ignore;
import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Random;

import static junit.framework.TestCase.assertTrue;

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

   @Ignore
   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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

                              testSnapper.snapFootstepNode(
                                    new FootstepNode(robotQuadrant, frontLeftXIndices[i], frontLeftYIndices[j], frontRightXIndices[k], frontRightYIndices[l],
                                                     hindLeftXIndices[m], hindLeftYIndices[n], hindRightXIndices[o], hindRightYIndices[p]));
                              assertTrue(testSnapper.dirtyBit);
                              testSnapper.dirtyBit = false;

                              testSnapper.snapFootstepNode(
                                    new FootstepNode(robotQuadrant, frontLeftXIndices[i], frontLeftYIndices[j], frontRightXIndices[k], frontRightYIndices[l],
                                                     hindLeftXIndices[m], hindLeftYIndices[n], hindRightXIndices[o], hindRightYIndices[p]));
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

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
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

                              FootstepNodeSnapData snapData = testSnapper.snapFootstepNode(
                                    new FootstepNode(newQuadrant, frontLeftXIndices[i], frontLeftYIndices[j], frontRightXIndices[k], frontRightYIndices[l],
                                                     hindLeftXIndices[m], hindLeftYIndices[n], hindRightXIndices[o], hindRightYIndices[p]));
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
