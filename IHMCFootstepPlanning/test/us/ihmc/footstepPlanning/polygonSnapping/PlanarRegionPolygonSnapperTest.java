package us.ihmc.footstepPlanning.polygonSnapping;

import static org.junit.Assert.*;

import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.MutationTestingTools;

public class PlanarRegionPolygonSnapperTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSnapPolygonToPlanarRegion()
   {
      ConvexPolygon2d polygonToSnap = new ConvexPolygon2d();
      polygonToSnap.addVertex(1.0, 1.0);
      polygonToSnap.addVertex(-1.0, 1.0);
      polygonToSnap.addVertex(-1.0, -1.0);
      polygonToSnap.addVertex(1.0, -1.0);
      polygonToSnap.update();

      ArrayList<ConvexPolygon2d> planarRegionConvexPolygons = new ArrayList<>();
      ConvexPolygon2d planarRegionPolygon = new ConvexPolygon2d();
      planarRegionPolygon.addVertex(10.0, 10.0);
      planarRegionPolygon.addVertex(-10.0, 10.0);
      planarRegionPolygon.addVertex(-10.0, -10.0);
      planarRegionPolygon.addVertex(10.0, -10.0);
      planarRegionPolygon.update();
      planarRegionConvexPolygons.add(planarRegionPolygon);

      RigidBodyTransform planarRegionTransformToWorld = new RigidBodyTransform();

      PlanarRegion planarRegionToSnapTo = new PlanarRegion(planarRegionTransformToWorld, planarRegionConvexPolygons);
      RigidBodyTransform polygonSnappingTransform = PlanarRegionPolygonSnapper.snapPolygonToPlanarRegion(polygonToSnap, planarRegionToSnapTo);

      RigidBodyTransform identityTransform = new RigidBodyTransform();
      assertTrue(polygonSnappingTransform.epsilonEquals(identityTransform, 1e-7));
   }

   public static void main(String[] args)
   {
      String targetTests = PlanarRegionPolygonSnapperTest.class.getName();
      String targetClassesInSamePackage = PlanarRegionPolygonSnapper.class.getName();
      MutationTestingTools.doPITMutationTestAndOpenResult(targetTests, targetClassesInSamePackage);
   }
}
