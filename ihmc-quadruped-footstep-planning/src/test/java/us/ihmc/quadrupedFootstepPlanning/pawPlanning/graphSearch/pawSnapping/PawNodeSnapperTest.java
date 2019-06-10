package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping;

import gnu.trove.map.hash.TIntObjectHashMap;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class PawNodeSnapperTest
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

                              double frontLeftX = PawNode.gridSizeXY * frontLeftXIndices[i];
                              double frontLeftY = PawNode.gridSizeXY * frontLeftYIndices[j];
                              double frontRightX = PawNode.gridSizeXY * frontRightXIndices[k];
                              double frontRightY = PawNode.gridSizeXY * frontRightYIndices[l];
                              double hindLeftX = PawNode.gridSizeXY * hindLeftXIndices[m];
                              double hindLeftY = PawNode.gridSizeXY * hindLeftYIndices[n];
                              double hindRightX = PawNode.gridSizeXY * hindRightXIndices[o];
                              double hindRightY = PawNode.gridSizeXY * hindRightYIndices[p];

                              double length = 1.0;
                              double width = 0.5;

                              int xIndex;
                              int yIndex;
                              switch (robotQuadrant)
                              {
                              case FRONT_LEFT:
                                 xIndex = PawNode.snapToGrid(frontLeftX);
                                 yIndex = PawNode.snapToGrid(frontLeftY);
                                 break;
                              case FRONT_RIGHT:
                                 xIndex = PawNode.snapToGrid(frontRightX);
                                 yIndex = PawNode.snapToGrid(frontRightY);
                                 break;
                              case HIND_LEFT:
                                 xIndex = PawNode.snapToGrid(hindLeftX);
                                 yIndex = PawNode.snapToGrid(hindLeftY);
                                 break;
                              default:
                                 xIndex = PawNode.snapToGrid(hindRightX);
                                 yIndex = PawNode.snapToGrid(hindRightY);
                                 break;
                              }

                              testSnapper.dirtyBit = true;
                              String string = "i " + i + " j " + j + " k " + k + " l " + l + " m " + m + " n " + n + " o " + o + " p " + p;

                              testSnapper.snapPawNode(robotQuadrant, xIndex, yIndex);
                              assertTrue(string, testSnapper.dirtyBit);
                              testSnapper.dirtyBit = false;

                              testSnapper.snapPawNode(robotQuadrant, xIndex, yIndex);
                              assertFalse(string, testSnapper.dirtyBit);
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

                              double frontLeftX = PawNode.gridSizeXY * frontLeftXIndices[i];
                              double frontLeftY = PawNode.gridSizeXY * frontLeftYIndices[j];
                              double frontRightX = PawNode.gridSizeXY * frontRightXIndices[k];
                              double frontRightY = PawNode.gridSizeXY * frontRightYIndices[l];
                              double hindLeftX = PawNode.gridSizeXY * hindLeftXIndices[m];
                              double hindLeftY = PawNode.gridSizeXY * hindLeftYIndices[n];
                              double hindRightX = PawNode.gridSizeXY * hindRightXIndices[o];
                              double hindRightY = PawNode.gridSizeXY * hindRightYIndices[p];

                              double length = 1.0;
                              double width = 0.5;

                              int xIndex;
                              int yIndex;
                              switch (newQuadrant)
                              {
                              case FRONT_LEFT:
                                 xIndex = PawNode.snapToGrid(frontLeftX);
                                 yIndex = PawNode.snapToGrid(frontLeftY);
                                 break;
                              case FRONT_RIGHT:
                                 xIndex = PawNode.snapToGrid(frontRightX);
                                 yIndex = PawNode.snapToGrid(frontRightY);
                                 break;
                              case HIND_LEFT:
                                 xIndex = PawNode.snapToGrid(hindLeftX);
                                 yIndex = PawNode.snapToGrid(hindLeftY);
                                 break;
                              default:
                                 xIndex = PawNode.snapToGrid(hindRightX);
                                 yIndex = PawNode.snapToGrid(hindRightY);
                                 break;
                              }

                              PawNodeSnapData snapData = testSnapper.snapPawNode(newQuadrant, xIndex, yIndex);
                              assertTrue(!testSnapper.dirtyBit);
                              assertTrue(snapData.getSnapTransform().epsilonEquals(new RigidBodyTransform(), epsilon));
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
   public void testHashMapStorage()
   {
      Random random = new Random(3823L);
      int numTrials = 100;
      PawNode nodeA, nodeB;

      for (int i = 0; i < numTrials; i++)
      {
         // test for exact same transform
         RobotQuadrant robotQuadrant = RobotQuadrant.FRONT_LEFT;
         Point2DReadOnly frontLeft = EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly frontRight = EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly otherFrontRight = EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly hindLeft = EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly otherHindLeft = EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly hindRight = EuclidCoreRandomTools.nextPoint2D(random, 1.0);
         Point2DReadOnly otherHindRight = EuclidCoreRandomTools.nextPoint2D(random, 1.0);


         double yaw = PawNode.computeNominalYaw(frontLeft.getX(), frontLeft.getY(), frontRight.getX(), frontRight.getY(), hindLeft.getX(), hindLeft.getY(),
                                                hindRight.getX(), hindRight.getY());

         nodeA = new PawNode(robotQuadrant, frontLeft, frontRight, hindLeft, hindRight, yaw, 1.5, 0.5);
         nodeB = new PawNode(robotQuadrant, frontLeft, otherFrontRight, otherHindLeft, otherHindRight, yaw, 1.5, 0.5);

         TIntObjectHashMap<PawNodeSnapData> snapDataHolder = new TIntObjectHashMap<>();

         assertFalse("number : " + i, snapDataHolder.containsKey(nodeB.hashCode()));

         snapDataHolder.put(nodeA.hashCode(), PawNodeSnapData.emptyData());

         assertTrue("number : " + i, snapDataHolder.containsKey(nodeA.hashCode()));

         if (nodeA.getYawIndex() == nodeB.getYawIndex())
         {
            assertEquals("number : " + i, nodeA.hashCode(), nodeB.hashCode());
            assertTrue("number : " + i, snapDataHolder.containsKey(nodeB.hashCode()));
         }
      }
   }


   private class TestSnapper extends PawNodeSnapper
   {
      boolean dirtyBit = false;

      @Override
      protected PawNodeSnapData snapInternal(RobotQuadrant robotQuadrant, int xIndex, int yIndex)
      {
         dirtyBit = true;
         return PawNodeSnapData.emptyData();
      }
   }
}
