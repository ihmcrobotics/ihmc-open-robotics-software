package us.ihmc.footstepPlanning.polygonSnapping;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.footstepPlanning.graphSearch.EnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.robotics.robotSide.SideDependentList;

public class HeightMapSnapWigglerTest
{
   @Test
   public void testWiggleWithAndWithoutOverhang()
   {
      double gridResolution = 0.05;
      double gridSizeXY = 0.3;
      double gridCenterXY = 0.0;
      TerrainMapData terrainmapData = new TerrainMapData(gridResolution, gridSizeXY, gridCenterXY, gridCenterXY);

      // We want to set the environment to a fixed height, so a flat plane.
      for (double x = -0.10; x <= 0.15; x += gridResolution)
      {
         for (double y = -0.20; y <= 0.2; y += gridResolution)
         {
            terrainmapData.setHeight(x, y, 0.2);
         }
      }

      // Define the foot polygon
      ConvexPolygon2D polygonToSnap = new ConvexPolygon2D();

      double footLength = 0.2;
      double footWidth = 0.1;
      polygonToSnap.addVertex(footLength / 2.0, footWidth / 2.0);
      polygonToSnap.addVertex(footLength / 2.0, -footWidth / 2.0);
      polygonToSnap.addVertex(-footLength / 2.0, -footWidth / 2.0);
      polygonToSnap.addVertex(-footLength / 2.0, footWidth / 2.0);
      polygonToSnap.update();

      // Set up the environment.
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(new ConvexPolygon2D(polygonToSnap), new ConvexPolygon2D(polygonToSnap));
      EnvironmentHandler environmentHandler = new EnvironmentHandler();
      HeightMapPolygonSnapper snapper = new HeightMapPolygonSnapper();
      HeightMapSnapWiggler wiggler = new HeightMapSnapWiggler(footPolygons, new WiggleParameters());
      environmentHandler.setTerrainMapData(terrainmapData);

      // Perform the snap itself
      FootstepSnapData snapData = new FootstepSnapData();
      DiscreteFootstep footstep = new DiscreteFootstep(0.0, 0.0);

      snapData.getSnapTransform().set(snapper.snapPolygonToHeightMap(polygonToSnap, environmentHandler));

      wiggler.computeWiggleTransform(footstep, environmentHandler, snapData);

      Assertions.assertFalse(snapData.getWiggleTransformInWorld().hasRotation());
      Assertions.assertFalse(snapData.getWiggleTransformInWorld().hasTranslation());

      // make the foot overhang by a 3 cm. As the min X of the height map is -0.1 m, and the foot length is 0.2 m, this means that the the heel of hte foot is
      // overhanging by 3 cm.
      footstep = new DiscreteFootstep(-0.03, 0.0);

      wiggler.computeWiggleTransform(footstep, environmentHandler, snapData);

      Assertions.assertFalse(snapData.getWiggleTransformInWorld().hasRotation());
      Assertions.assertTrue(snapData.getWiggleTransformInWorld().hasTranslation());

      Assertions.assertEquals(-footstep.getX(), snapData.getWiggleTransformInWorld().getTranslation().getX(), 2e-2);
      Assertions.assertEquals(0.0, snapData.getWiggleTransformInWorld().getTranslation().getY(), 1e-3);
   }
}
