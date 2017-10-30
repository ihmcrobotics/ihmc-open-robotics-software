package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.Test;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Random;

import static junit.framework.TestCase.assertTrue;

@ContinuousIntegrationAnnotations.ContinuousIntegrationPlan(categories = IntegrationCategory.FAST)
public class FootstepNodeSnapperTest
{
   private final Random random = new Random(320L);
   private final double epsilon = 1e-8;

   private int[] xIndices = new int[]{-30, 0, 23, 87, -100, 42};
   private int[] yIndices = new int[]{-35, 0, -777, 87, -50, 28};
   private int[] yawIndices = new int[]{-2, 4, 0};

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFootstepCacheing()
   {
      TestSnapper testSnapper = new TestSnapper();
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(new PlanarRegion());
      testSnapper.setPlanarRegions(planarRegionsList);

      for (int i = 0; i < xIndices.length; i++)
      {
         for (int j = 0; j < yIndices.length; j++)
         {
            for (int k = 0; k < yawIndices.length; k++)
            {
               RobotSide robotSide = RobotSide.generateRandomRobotSide(random);

               testSnapper.snapFootstepNode(new FootstepNode(xIndices[i], yIndices[j], yawIndices[k], robotSide));
               assertTrue(testSnapper.dirtyBit);
               testSnapper.dirtyBit = false;

               testSnapper.snapFootstepNode(new FootstepNode(xIndices[i], yIndices[j], yawIndices[k], robotSide));
               assertTrue(!testSnapper.dirtyBit);
            }
         }
      }
   }

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testWithoutPlanarRegions()
   {
      TestSnapper testSnapper = new TestSnapper();

      for (int i = 0; i < xIndices.length; i++)
      {
         for (int j = 0; j < yIndices.length; j++)
         {
            for (int k = 0; k < yawIndices.length; k++)
            {
               RobotSide robotSide = RobotSide.generateRandomRobotSide(random);

               FootstepNodeSnapData snapData = testSnapper.snapFootstepNode(new FootstepNode(xIndices[i], yIndices[j], yawIndices[k], robotSide));
               assertTrue(!testSnapper.dirtyBit);

               assertTrue(snapData.getSnapTransform().epsilonEquals(new RigidBodyTransform(), epsilon));
               assertTrue(snapData.getCroppedFoothold().isEmpty());
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
