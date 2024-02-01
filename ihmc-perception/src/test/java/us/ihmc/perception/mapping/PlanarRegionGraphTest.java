package us.ihmc.perception.mapping;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import static us.ihmc.robotics.Assert.assertEquals;

public class PlanarRegionGraphTest
{
   @Test
   public void testGraphWithFullOverLap()
   {
      PlanarRegionGraph graph = new PlanarRegionGraph();

      ConvexPolygon2D region1Polygon = new ConvexPolygon2D();
      region1Polygon.addVertex(0.2, 0.2);
      region1Polygon.addVertex(0.2, -0.2);
      region1Polygon.addVertex(-0.2, -0.2);
      region1Polygon.addVertex(-0.2, 0.2);
      region1Polygon.update();

      RigidBodyTransform region2Transform = new RigidBodyTransform();
      region2Transform.getTranslation().set(0.1, 0.0, 0.0);

      RigidBodyTransform region3Transform = new RigidBodyTransform();
      region3Transform.getTranslation().set(0.25, 0.0, 0.0);

      RigidBodyTransform region4Transform = new RigidBodyTransform();
      region3Transform.getTranslation().set(0.25, 0.3, 0.0);

      PlanarRegion region1 = new PlanarRegion(new RigidBodyTransform(), region1Polygon);
      region1.setRegionId(1);
      PlanarRegion region2 = new PlanarRegion(region2Transform, region1Polygon);
      region2.setRegionId(2);
      PlanarRegion region3 = new PlanarRegion(region3Transform, region1Polygon);
      region3.setRegionId(3);
      PlanarRegion region4 = new PlanarRegion(region4Transform, region1Polygon);
      region4.setRegionId(4);


      graph.addRootOfBranch(region1);
      graph.addRootOfBranch(region3);
      graph.addRootOfBranch(region4);
      graph.addEdge(region1, region2);
      graph.addEdge(region2, region3);

      graph.collapseGraphByMerging(0.05);

      PlanarRegionsList planarRegionsList = graph.getAsPlanarRegionsList();
      assertEquals(2, planarRegionsList.getNumberOfPlanarRegions());
      assertEquals(1, planarRegionsList.getPlanarRegion(0).getRegionId());
      assertEquals(4, planarRegionsList.getPlanarRegion(1).getRegionId());
   }

   @Disabled
   @Test
   public void testGraphWithOneOverLap()
   {
      PlanarRegionGraph graph = new PlanarRegionGraph();

      ConvexPolygon2D region1Polygon = new ConvexPolygon2D();
      region1Polygon.addVertex(0.2, 0.2);
      region1Polygon.addVertex(0.2, -0.2);
      region1Polygon.addVertex(-0.2, -0.2);
      region1Polygon.addVertex(-0.2, 0.2);
      region1Polygon.update();

      RigidBodyTransform region2Transform = new RigidBodyTransform();
      region2Transform.getTranslation().set(0.1, 0.0, 0.0);

      RigidBodyTransform region3Transform = new RigidBodyTransform();
      region3Transform.getTranslation().set(0.6, 0.0, 0.0);

      PlanarRegion region1 = new PlanarRegion(new RigidBodyTransform(), region1Polygon);
      region1.setRegionId(1);
      PlanarRegion region2 = new PlanarRegion(region2Transform, region1Polygon);
      region2.setRegionId(2);
      PlanarRegion region3 = new PlanarRegion(region3Transform, region1Polygon);
      region3.setRegionId(3);

      graph.addRootOfBranch(region1);
      graph.addRootOfBranch(region3);
      graph.addEdge(region1, region2);
      graph.addEdge(region2, region3);

      graph.collapseGraphByMerging(0.05);

      assertEquals(2, graph.getAsPlanarRegionsList().getNumberOfPlanarRegions());
   }
}
