package com.yobotics.simulationconstructionset.util.ground;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.utilities.math.geometry.ConvexPolygon2d;
import us.ihmc.utilities.test.JUnitTools;

public class CombinedTerrainObjectTest
{
   private CombinedTerrainObject combinedTerrainObject;
   private Point3d pointToCheck;
   private Point3d expected;
   private Point3d resultPoint;
   private Vector3d resultVector;
   
   @Before
   public void setUp() throws Exception
   {
      combinedTerrainObject = new CombinedTerrainObject("Combined Terrain Object to Test");
      pointToCheck = new Point3d();
      expected = new Point3d();
      resultPoint = new Point3d();
      resultVector = new Vector3d();
   }

   @Test
   public void testTwoIntersectingBoxes()
   {
      setupTwoIntersectingBoxesMadeFromPolygons();
      
      pointToCheck.set(0.4, 0.45, 0.25);
      expected.set(0.4, 0.5, 0.25);
      combinedTerrainObject.closestIntersectionAndNormalAt(pointToCheck.x, pointToCheck.y, pointToCheck.z, resultPoint, resultVector);
      JUnitTools.assertTuple3dEquals(expected, resultPoint, 1e-4);
      combinedTerrainObject.closestIntersectionTo(pointToCheck.x, pointToCheck.y, pointToCheck.z, resultPoint);
      JUnitTools.assertTuple3dEquals(expected, resultPoint, 1e-4);
      expected.set(0.0, 1.0, 0.0);
      JUnitTools.assertTuple3dEquals(expected, resultVector, 1e-4);
      combinedTerrainObject.surfaceNormalAt(pointToCheck.x, pointToCheck.y, pointToCheck.z, resultVector);
      JUnitTools.assertTuple3dEquals(expected, resultVector, 1e-4);
   }

   private void setupTwoIntersectingBoxesMadeFromPolygons()
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
