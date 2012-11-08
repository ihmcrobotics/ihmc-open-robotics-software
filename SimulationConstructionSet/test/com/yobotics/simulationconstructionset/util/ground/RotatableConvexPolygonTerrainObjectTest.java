package com.yobotics.simulationconstructionset.util.ground;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Before;
import org.junit.Test;

import us.ihmc.utilities.math.geometry.ConvexPolygon2d;
import us.ihmc.utilities.math.geometry.Plane3d;
import us.ihmc.utilities.test.JUnitTools;

import com.yobotics.simulationconstructionset.SimulationConstructionSet;

public class RotatableConvexPolygonTerrainObjectTest
{
   private RotatableConvexPolygonTerrainObject flatTopFaceOctagon3d, inclinedTopFaceOctagon3d;
   private Vector3d normalZVector, normalYZVector;
   private ConvexPolygon2d convexPolygon;
   private double[][] pointList;
   private double centroidHeight;
   private double epsilon = 1e-8;

   @Before
   public void setUp() throws Exception
   {
      normalZVector = new Vector3d(0.0, 0.0, 1.0);
      double[][] pointList =
      {
         {2.0, 1.0}, {1.0, 2.0}, {-1.0, 2.0}, {-2.0, 1.0}, {-2.0, -1.0}, {-1.0, -2.0}, {1.0, -2.0}, {2.0, -1.0}
      };
      this.pointList = pointList;
      convexPolygon = new ConvexPolygon2d(pointList);

      centroidHeight = 1.0;

      flatTopFaceOctagon3d = new RotatableConvexPolygonTerrainObject(normalZVector, convexPolygon, centroidHeight);

      normalYZVector = new Vector3d(0.0, 1.0, 1.0);

      inclinedTopFaceOctagon3d = new RotatableConvexPolygonTerrainObject(normalYZVector, convexPolygon, centroidHeight);
   }

   @Test
   public void testHeightAt()
   {
      Point2d centroid = convexPolygon.getCentroidCopy();
      assertEquals(centroidHeight, flatTopFaceOctagon3d.heightAt(centroid.getX(), centroid.getY(), centroidHeight), epsilon);
      double expectedY;
      for (double[] point : pointList)
      {
         expectedY = centroidHeight - point[1];
         assertEquals(expectedY, inclinedTopFaceOctagon3d.heightAt(point[0], point[1], centroidHeight), epsilon);
      }
   }

   @Test
   public void testIsInside()
   {
      assertTrue(flatTopFaceOctagon3d.isClose(0.0, 0.0, 0.5));    // Point Inside
      assertFalse(flatTopFaceOctagon3d.isClose(0.0, 0.0, 1.5));    // Point On the top Outside
      assertFalse(flatTopFaceOctagon3d.isClose(2.0, 2.0, 1.5));    // Point Outside

      assertFalse(inclinedTopFaceOctagon3d.isClose(0.0, 1.0, centroidHeight));    // Point Outside
      assertTrue(inclinedTopFaceOctagon3d.isClose(0.0, -1.0, centroidHeight));    // Point Inside
      assertTrue(inclinedTopFaceOctagon3d.isClose(0.0, 0.0, centroidHeight));    // Point is on the center of the top surface

   }

   @Test
   public void testClosestIntersectionTo()
   {
      Point3d pointToPack = new Point3d();

      // Point above centroid
      flatTopFaceOctagon3d.closestIntersectionTo(0.0, 0.0, 1.5, pointToPack);
      Point3d expectedPoint = new Point3d(0.0, 0.0, 1.0);
      JUnitTools.assertTuple3dEquals(expectedPoint, pointToPack, epsilon);

      // Point outside, but below top surface
      flatTopFaceOctagon3d.closestIntersectionTo(3.0, 1.5, 0.5, pointToPack);
      expectedPoint.set(2.0, 1.0, 0.5);
      JUnitTools.assertTuple3dEquals(expectedPoint, pointToPack, epsilon);

      // Point outside and above top surface
      flatTopFaceOctagon3d.closestIntersectionTo(3.0, 1.5, 1.5, pointToPack);
      expectedPoint.set(2.0, 1.0, 1.0);
      JUnitTools.assertTuple3dEquals(expectedPoint, pointToPack, epsilon);

      // Point centroid, inclined top surface
      inclinedTopFaceOctagon3d.closestIntersectionTo(0.0, 0.0, 1.0, pointToPack);
      expectedPoint = new Point3d(0.0, 0.0, 1.0);
      JUnitTools.assertTuple3dEquals(expectedPoint, pointToPack, epsilon);
      
      // Point high above centroid, inclined top surface
      inclinedTopFaceOctagon3d.closestIntersectionTo(0.0, 0.0, 15.0, pointToPack);
      expectedPoint = new Point3d(0.0, -2.0, 3.0);
      JUnitTools.assertTuple3dEquals(expectedPoint, pointToPack, epsilon);
   }
   
   @Test
   public void testIsInsedeTheFace(){
	   Point3d faceCenter = new Point3d(1.0,0.0,0.0);
	   Vector3d faceNormal = new Vector3d(1.0,0.0,0.0);	   
	   Plane3d facePlane = new Plane3d(faceCenter,faceNormal);
	   ArrayList<Point3d> faceVertices3d = new ArrayList<Point3d>();
	   faceVertices3d.add(new Point3d(1.0,-2.0,0.0));
	   faceVertices3d.add(new Point3d(1.0,0.0,-2.0));
	   faceVertices3d.add(new Point3d(1.0,2.0,0.0));
	   faceVertices3d.add(new Point3d(1.0,0.0,2.0));
	   //Expected conversions v1=(2.0, 0.0) v2=(0.0, 2.0) v3=(-2.0, 0.0) v4=(0.0, -2.0)	   
	   Point3d pointToCheck = new Point3d(1.0,-1.0,0.0); //Point inside (1.0, 0.0)
	   assertTrue(flatTopFaceOctagon3d.isInsadeTheFace(facePlane,faceVertices3d,pointToCheck));
	   
	   pointToCheck.set(1.0,-1.0,1.0);//Point on the edge (1.0, 1.0)
	   assertTrue(flatTopFaceOctagon3d.isInsadeTheFace(facePlane,faceVertices3d,pointToCheck));
	   
	   pointToCheck.set(1.0, 1.0, 2.0);//Point outside (-1.0, -2.0)
	   assertFalse(flatTopFaceOctagon3d.isInsadeTheFace(facePlane,faceVertices3d,pointToCheck));
   }

   @Test
   public void testSurfaceNormalAt()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testClosestIntersectionAndNormalAt()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testGetXMin()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testGetXMax()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testGetYMin()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testGetYMax()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testGetXTiles()
   {
      fail("Not yet implemented");
   }

   @Test
   public void testGetYTiles()
   {
      fail("Not yet implemented");
   }
   
   @Test
   public void testSetupInEnvironment()
   {
      SimulationConstructionSet scs = new SimulationConstructionSet();
      scs.addStaticLinkGraphics(flatTopFaceOctagon3d.getLinkGraphics());
      
      scs.setGroundVisible(false);

      scs.startOnAThread();
      
      while(true)
      {
         try
         {
            Thread.sleep(1000);
         } 
         catch (InterruptedException e)
         {
         }
         
      }
   }

}
