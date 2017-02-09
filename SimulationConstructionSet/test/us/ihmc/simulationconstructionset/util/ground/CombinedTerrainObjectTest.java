package us.ihmc.simulationconstructionset.util.ground;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.tools.testing.JUnitTools;

public class CombinedTerrainObjectTest
{

   //TODO: Flesh out this test case to be a really good one.

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout=300000)
   public void testTwoIntersectingBoxes()
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("Combined Terrain Object to Test");
      Point3d pointToCheck = new Point3d();
      Point3d expectedIntersection = new Point3d();
      Vector3d expectedNormal = new Vector3d();

      Point3d resultIntersection = new Point3d();
      Vector3d resultNormal = new Vector3d();
      
      setupTwoIntersectingBoxesMadeFromPolygons(combinedTerrainObject);
      
      pointToCheck.set(0.4, 0.45, 0.25);
      expectedIntersection.set(0.4, 0.5, 0.25);
      expectedNormal.set(0.0, 1.0, 0.0);

      combinedTerrainObject.checkIfInside(pointToCheck.getX(), pointToCheck.getY(), pointToCheck.getZ(), resultIntersection, resultNormal);
      JUnitTools.assertTuple3dEquals(expectedIntersection, resultIntersection, 1e-4);
      JUnitTools.assertTuple3dEquals(expectedNormal, resultNormal, 1e-4);
   }

   private void setupTwoIntersectingBoxesMadeFromPolygons(CombinedTerrainObject3D combinedTerrainObject)
   {
      Vector3d normalVector = new Vector3d(0.0, 0.0, 1.0);
      double[][] firstVertices = {{0.0,0.0},{1.0,0.0},{1.0,1.0},{0.0,1.0}};
      ConvexPolygon2d firstConvexPolygon = new ConvexPolygon2d(firstVertices);
      RotatableConvexPolygonTerrainObject firstBox = new RotatableConvexPolygonTerrainObject(normalVector, firstConvexPolygon, 1.0);
      combinedTerrainObject.addTerrainObject(firstBox);

      double[][] secondVertices = {{-0.5,-0.5},{0.5,-0.5},{-0.5,0.5},{0.5,0.5}};
      ConvexPolygon2d secondConvexPolygon = new ConvexPolygon2d(secondVertices);
      RotatableConvexPolygonTerrainObject secondBox = new RotatableConvexPolygonTerrainObject(normalVector, secondConvexPolygon, 0.5);
      combinedTerrainObject.addTerrainObject(secondBox);
   }

}
