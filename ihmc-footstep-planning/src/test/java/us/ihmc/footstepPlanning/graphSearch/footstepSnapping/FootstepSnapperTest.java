package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Random;

import static us.ihmc.robotics.Assert.*;

public class FootstepSnapperTest
{
   private final Random random = new Random(320L);
   private final double epsilon = 1e-8;

   private int[] xIndices = new int[]{-30, 0, 23, 87, -100, 42};
   private int[] yIndices = new int[]{-35, 0, -777, 87, -50, 28};
   private int[] yawIndices = new int[]{-2, 4, 0};

   @Test
   public void testFootstepCacheing()
   {
      TestSnapper testSnapper = new TestSnapper();
      testSnapper.setPlanarRegions(PlanarRegionsList.flatGround(1.0));

      for (int i = 0; i < xIndices.length; i++)
      {
         for (int j = 0; j < yIndices.length; j++)
         {
            for (int k = 0; k < yawIndices.length; k++)
            {
               RobotSide robotSide = RobotSide.generateRandomRobotSide(random);

               testSnapper.snapFootstep(new DiscreteFootstep(xIndices[i], yIndices[j], yawIndices[k], robotSide));
               assertTrue(testSnapper.dirtyBit);

               testSnapper.dirtyBit = false;
               testSnapper.snapFootstep(new DiscreteFootstep(xIndices[i], yIndices[j], yawIndices[k], robotSide));
               assertFalse(testSnapper.dirtyBit);
            }
         }
      }
   }

   @Test
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

               FootstepSnapData snapData = testSnapper.snapFootstep(new DiscreteFootstep(xIndices[i], yIndices[j], yawIndices[k], robotSide));
               assertTrue(!testSnapper.dirtyBit);

               assertTrue(snapData.getSnapTransform().epsilonEquals(new RigidBodyTransform(), epsilon));
               assertTrue(snapData.getCroppedFoothold().isEmpty());
            }
         }
      }
   }

   private class TestSnapper extends FootstepSnapAndWiggler
   {
      boolean dirtyBit = false;

      public TestSnapper()
      {
         super(PlannerTools.createDefaultFootPolygons(), new DefaultFootstepPlannerParameters());
      }

      @Override
      protected FootstepSnapData computeSnapTransform(DiscreteFootstep footstepToSnap, DiscreteFootstep stanceStep)
      {
         dirtyBit = true;
         return FootstepSnapData.emptyData();
      }
   }
}
