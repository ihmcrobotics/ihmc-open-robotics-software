package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;

public class PawCliffDetectionToolsTest
{
   @Test
   public void testFindHighestNearbyPoint2()
   {

      ConvexPolygon2D groundRegion = new ConvexPolygon2D();
      groundRegion.addVertex(5.0, 5.0);
      groundRegion.addVertex(5.0, -5.0);
      groundRegion.addVertex(-5.0, 5.0);
      groundRegion.addVertex(-5.0, -5.0);
      groundRegion.update();


      ConvexPolygon2D cliffRegion = new ConvexPolygon2D();
      cliffRegion.addVertex(0.2, 0.2);
      cliffRegion.addVertex(0.2, -0.2);
      cliffRegion.addVertex(-0.2, 0.2);
      cliffRegion.addVertex(-0.2, -0.2);
      cliffRegion.update();

      RigidBodyTransform higherCliff = new RigidBodyTransform();
      RigidBodyTransform lowerCliff = new RigidBodyTransform();
      higherCliff.setTranslation(0.5, 0.0, 0.3);
      lowerCliff.setTranslation(0.5, 0.0, 0.05);

      PlanarRegion ground = new PlanarRegion(new RigidBodyTransform(), groundRegion);
      PlanarRegion higherCliffRegion = new PlanarRegion(higherCliff, cliffRegion);
      PlanarRegion lowerCliffRegion = new PlanarRegion(lowerCliff, cliffRegion);

      List<PlanarRegion> regions = new ArrayList<>();
      regions.add(ground);
      regions.add(higherCliffRegion);
      regions.add(lowerCliffRegion);
      PlanarRegionsList planarRegionsList = new PlanarRegionsList(regions);

      double heightToAvoid = 0.25;
      Assert.assertEquals(PawCliffDetectionTools.findHighestNearbyPoint(planarRegionsList, new Point3D(0.25, 0.0, 0.0), 0.0, new Point3D(), 0.2, -0.1, 0.1, -0.1), 0.3, 1e-5);
      Assert.assertTrue(PawCliffDetectionTools.isNearCliff(planarRegionsList, new Point3D(0.25, 0.0, 0.0), 0.0, heightToAvoid, 0.2, -0.1, 0.1, -0.1));
      heightToAvoid = 0.34;
      Assert.assertFalse(PawCliffDetectionTools.isNearCliff(planarRegionsList, new Point3D(0.25, 0.0, 0.0), 0.0, heightToAvoid, 0.2, -0.1, 0.1, -0.1));
   }
}
