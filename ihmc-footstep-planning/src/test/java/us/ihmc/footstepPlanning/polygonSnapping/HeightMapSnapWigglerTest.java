package us.ihmc.footstepPlanning.polygonSnapping;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.pubsub.common.SampleIdentity;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.Random;

public class HeightMapSnapWigglerTest
{
   @Test
   public void testWiggleWithAndWithoutOverhang()
   {
      double gridResolution = 0.05;
      double gridSizeXY = 0.3;
      double gridCenterXY = 0.0;
      HeightMapData heightMapData = new HeightMapData(gridResolution, gridSizeXY, gridCenterXY, gridCenterXY);

      for (double x = -0.10; x <= 0.15; x += gridResolution)
      {
         for (double y = -0.20; y <= 0.2; y += gridResolution)
         {
            heightMapData.setHeightAt(x, y, 0.2);
         }
      }

      ConvexPolygon2D polygonToSnap = new ConvexPolygon2D();

      double footLength = 0.2;
      double footWidth = 0.1;
      polygonToSnap.addVertex(footLength / 2.0, footWidth / 2.0);
      polygonToSnap.addVertex(footLength / 2.0, -footWidth / 2.0);
      polygonToSnap.addVertex(-footLength / 2.0, -footWidth / 2.0);
      polygonToSnap.addVertex(-footLength / 2.0, footWidth / 2.0);
      polygonToSnap.update();

      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(new ConvexPolygon2D(polygonToSnap), new ConvexPolygon2D(polygonToSnap));
      FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
      HeightMapPolygonSnapper snapper = new HeightMapPolygonSnapper();
      HeightMapSnapWiggler wiggler = new HeightMapSnapWiggler(footPolygons, new WiggleParameters());
      environmentHandler.setHeightMap(heightMapData);

      FootstepSnapData snapData = new FootstepSnapData();
      DiscreteFootstep footstep = new DiscreteFootstep(0.0, 0.0);

      double snapHeightThreshold = 0.05;
      double minInclineAngle = Math.toRadians(45.0);
      snapData.getSnapTransform().set(snapper.snapPolygonToHeightMap(polygonToSnap, environmentHandler, snapHeightThreshold, minInclineAngle));

      wiggler.computeWiggleTransform(footstep, environmentHandler, snapData, snapHeightThreshold, minInclineAngle);

      Assertions.assertFalse(snapData.getWiggleTransformInWorld().hasRotation());
      Assertions.assertFalse(snapData.getWiggleTransformInWorld().hasTranslation());

      // make the foot overhang by a fair bit
      footstep = new DiscreteFootstep(-0.03, 0.0);

      wiggler.computeWiggleTransform(footstep, environmentHandler, snapData, snapHeightThreshold, minInclineAngle);

      Assertions.assertFalse(snapData.getWiggleTransformInWorld().hasRotation());
      Assertions.assertTrue(snapData.getWiggleTransformInWorld().hasTranslation());

      Assertions.assertEquals(-footstep.getX(), snapData.getWiggleTransformInWorld().getTranslation().getX(), 2e-2);
      Assertions.assertEquals(0.0, snapData.getWiggleTransformInWorld().getTranslation().getY(), 1e-3);
   }
}
