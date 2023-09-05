package us.ihmc.simulationConstructionSetTools.util.ground;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationConstructionSetTools.util.ground.RotatableConvexPolygonTerrainObject;

public class CombinedTerrainObjectTest
{

   //TODO: Flesh out this test case to be a really good one.

	@Test
   public void testTwoIntersectingBoxes()
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("Combined Terrain Object to Test");
      Point3D pointToCheck = new Point3D();
      Point3D expectedIntersection = new Point3D();
      Vector3D expectedNormal = new Vector3D();

      Point3D resultIntersection = new Point3D();
      Vector3D resultNormal = new Vector3D();
      
      setupTwoIntersectingBoxesMadeFromPolygons(combinedTerrainObject);
      
      pointToCheck.set(0.4, 0.45, 0.25);
      expectedIntersection.set(0.4, 0.5, 0.25);
      expectedNormal.set(0.0, 1.0, 0.0);

      combinedTerrainObject.checkIfInside(pointToCheck.getX(), pointToCheck.getY(), pointToCheck.getZ(), resultIntersection, resultNormal);
      EuclidCoreTestTools.assertEquals(expectedIntersection, resultIntersection, 1e-4);
      EuclidCoreTestTools.assertEquals(expectedNormal, resultNormal, 1e-4);
   }

   private void setupTwoIntersectingBoxesMadeFromPolygons(CombinedTerrainObject3D combinedTerrainObject)
   {
      Vector3D normalVector = new Vector3D(0.0, 0.0, 1.0);
      double[][] firstVertices = {{0.0,0.0},{1.0,0.0},{1.0,1.0},{0.0,1.0}};
      ConvexPolygon2D firstConvexPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(firstVertices));
      RotatableConvexPolygonTerrainObject firstBox = new RotatableConvexPolygonTerrainObject(normalVector, firstConvexPolygon, 1.0);
      combinedTerrainObject.addTerrainObject(firstBox);

      double[][] secondVertices = {{-0.5,-0.5},{0.5,-0.5},{-0.5,0.5},{0.5,0.5}};
      ConvexPolygon2D secondConvexPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(secondVertices));
      RotatableConvexPolygonTerrainObject secondBox = new RotatableConvexPolygonTerrainObject(normalVector, secondConvexPolygon, 0.5);
      combinedTerrainObject.addTerrainObject(secondBox);
   }

}
