package us.ihmc.pathPlanning.visibilityGraphs.tools;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegions;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegion;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertTrue;

public class NavigableRegionToolsTest
{
   @Test
   public void testGetNavigableRegionContainingThisPoint()
   {
      ConvexPolygon2D planarRegionPolygon = new ConvexPolygon2D();
      planarRegionPolygon.addVertex(0.5, 0.5);
      planarRegionPolygon.addVertex(-0.5, 0.5);
      planarRegionPolygon.addVertex(-0.5, -0.5);
      planarRegionPolygon.addVertex(0.5, -0.5);
      planarRegionPolygon.update();

      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(), planarRegionPolygon);
      List<PlanarRegion> planarRegions = new ArrayList<>();
      planarRegions.add(planarRegion);

      double navigableRegionExtrusionDistance = 0.05;

      VisibilityGraphsParametersBasics parameters = new DefaultVisibilityGraphParameters();
      parameters.setNavigableExtrusionDistance(navigableRegionExtrusionDistance);

      NavigableRegions navigableRegions = new NavigableRegions(parameters, planarRegions);
      navigableRegions.createNavigableRegions();

      ConvexPolygon2D scaledRegion = new ConvexPolygon2D();
      ConvexPolygonScaler scaler = new ConvexPolygonScaler();
      scaler.scaleConvexPolygon(planarRegionPolygon, 0.5 * navigableRegionExtrusionDistance, scaledRegion);

      Random random = new Random(1738L);
      for (int i = 0; i < 1000; i++)
      {
         int startIndex = RandomNumbers.nextInt(random, 0, 3);
         double alpha = RandomNumbers.nextDouble(random, 0.0, 1.0);

         Point2D pointToCheck = new Point2D();
         pointToCheck.interpolate(scaledRegion.getVertex(startIndex), scaledRegion.getNextVertex(startIndex), alpha);

         NavigableRegion containingRegion = NavigableRegionTools.getNavigableRegionContainingThisPoint(new Point3D(pointToCheck), navigableRegions);
         assertTrue("Should be no region, since we're outside the navigable region", containingRegion == null);
      }
   }
}
