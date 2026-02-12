package us.ihmc.footstepPlanning.graphSearch.footstepSnapping;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlanarRegionToHeightMapConverter;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.perception.gpuMapping.TerrainMapMessageTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class FootstepSnapperTest
{
   private final Random random = new Random(320L);
   private final double epsilon = 1e-8;

   private int[] xIndices = new int[] {-30, 0, 23, 87, -100, 42};
   private int[] yIndices = new int[] {-35, 0, -777, 87, -50, 28};
   private int[] yawIndices = new int[] {-2, 4, 0};

   @Test
   public void testFootstepCacheing()
   {
      TestSnapper testSnapper = new TestSnapper();
      PlanarRegionsList planarRegionsList = PlanarRegionsList.flatGround(1.0);
      // Applying this transform is necessary to make the height map get populated. Otherwise, the height map is left with unoccupied cells, as it's at zero
      // height which is equivalent to the groun dheight.
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.appendTranslation(0.0, 0.0, 0.10);
      planarRegionsList.getPlanarRegion(0).applyTransform(transform);
      TerrainMapData terrainMapData = TerrainMapMessageTools.unpackMessage(PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(
            planarRegionsList));
      testSnapper.setTerrainMapData(terrainMapData);

      for (int i = 0; i < xIndices.length; i++)
      {
         for (int j = 0; j < yIndices.length; j++)
         {
            for (int k = 0; k < yawIndices.length; k++)
            {
               RobotSide robotSide = RobotSide.generateRandomRobotSide(random);

               // We have to pull the footstep out to use the same footstep object. The caching isn't done in a hash map anymore. Instead, the snap transform
               // is populated in the footstep itself. This is done to avoid computation of having to retrieve the transform from a hash map with many entries.
               // However, it means that the snaps aren't cached via a hashmap, but rather the footstep itself, so the same step object must be reused.
               DiscreteFootstep discreteFootstep = new DiscreteFootstep(xIndices[i], yIndices[j], yawIndices[k], robotSide);

               testSnapper.snapFootstep(discreteFootstep);
               assertTrue(testSnapper.dirtyBit);

               testSnapper.dirtyBit = false;
               testSnapper.snapFootstep(discreteFootstep);
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
