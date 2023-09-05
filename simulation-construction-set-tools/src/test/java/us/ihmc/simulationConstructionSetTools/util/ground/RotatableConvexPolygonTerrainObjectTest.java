package us.ihmc.simulationConstructionSetTools.util.ground;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.util.ground.RotatableConvexPolygonTerrainObject;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class RotatableConvexPolygonTerrainObjectTest
{
   private RotatableConvexPolygonTerrainObject flatTopFaceOctagon3d, inclinedTopFaceOctagon3d, inclinedTopFaceOctagon3dSecond;
   private Vector3D normalZVector, normalYZVector;
   private ConvexPolygon2D convexPolygon;
   private double[][] pointList;
   private double centroidHeight;
   private double epsilon = 1e-8;

   @BeforeEach
   public void setUp() throws Exception
   {
      normalZVector = new Vector3D(0.0, 0.0, 1.0);
      double[][] pointList =
      {
         {2.0, 1.0}, {1.0, 2.0}, {-1.0, 2.0}, {-2.0, 1.0}, {-2.0, -1.0}, {-1.0, -2.0}, {1.0, -2.0}, {2.0, -1.0}
      };
      this.pointList = pointList;
      convexPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointList));

      centroidHeight = 1.0;

      flatTopFaceOctagon3d = new RotatableConvexPolygonTerrainObject(normalZVector, convexPolygon, centroidHeight);

      normalYZVector = new Vector3D(0.0, 1.0, 1.0);

      inclinedTopFaceOctagon3d = new RotatableConvexPolygonTerrainObject(normalYZVector, convexPolygon, centroidHeight);

      inclinedTopFaceOctagon3dSecond = new RotatableConvexPolygonTerrainObject(normalYZVector, convexPolygon, 3.0);
   }

	@Test
   public void testHeightAt()
   {
      Point2DReadOnly centroid = convexPolygon.getCentroid();
      assertEquals(centroidHeight, flatTopFaceOctagon3d.heightAt(centroid.getX(), centroid.getY(), centroidHeight), epsilon);
      double expectedY;
      for (double[] point : pointList)
      {
         expectedY = centroidHeight - point[1];
         assertEquals(expectedY, inclinedTopFaceOctagon3d.heightAt(point[0], point[1], centroidHeight), epsilon);
      }

      assertEquals(0.0, flatTopFaceOctagon3d.heightAt(5.0, 5.0, 5.0), epsilon);
   }

	@Test
   public void testIsClose()
   {
      assertTrue(flatTopFaceOctagon3d.isClose(0.0, 0.0, 0.5));    // Point Inside
      assertFalse(flatTopFaceOctagon3d.isClose(0.0, 0.0, 1.5));    // Point On the top Outside
      assertFalse(flatTopFaceOctagon3d.isClose(2.0, 2.0, 1.5));    // Point Outside

      assertTrue(inclinedTopFaceOctagon3d.isClose(0.0, 1.0, centroidHeight));    // Point Outside
      assertTrue(inclinedTopFaceOctagon3d.isClose(0.0, -1.0, centroidHeight));    // Point Inside
      assertTrue(inclinedTopFaceOctagon3d.isClose(0.0, 0.0, centroidHeight));    // Point is on the center of the top surface

   }

	@Test
   public void testClosestIntersectionTo()
   {
      Point3D pointToPack = new Point3D();
      Vector3D normalToPack = new Vector3D();

      Point3D expectedPoint = new Point3D(2.0, 0.0, 0.5);
      flatTopFaceOctagon3d.checkIfInside(3.0, 0.0, 0.5, pointToPack, normalToPack);    // Point on lateral surface
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);

      expectedPoint.set(-1.5, -1.5, 0.0);
      flatTopFaceOctagon3d.checkIfInside(-4.0, -4.0, 0.0, pointToPack, normalToPack);    // Point on lateral surface
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);

      expectedPoint.set(0.0, 2.0, 0.9);
      flatTopFaceOctagon3d.checkIfInside(0.0, 2.3, 0.9, pointToPack, normalToPack);    // Point on lateral surface
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);

      expectedPoint.set(1.0, -1.0, 1.0);
      flatTopFaceOctagon3d.checkIfInside(1.0, -1.0, 1.1, pointToPack, normalToPack);    // Point on top surface
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);

      expectedPoint.set(1.5, -1.5, 1.0);
      flatTopFaceOctagon3d.checkIfInside(1.5, -1.5, 1.0, pointToPack, normalToPack);    // Point on top surface edge
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);

      expectedPoint.set(0.0, 1.5, 1.5);
      inclinedTopFaceOctagon3dSecond.checkIfInside(0.0, 2.0, 2.0, pointToPack, normalToPack);    // Point on top (inclined) surface
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);

      expectedPoint.set(0.0, 2.0, 0.5);
      inclinedTopFaceOctagon3dSecond.checkIfInside(0.0, 3.0, 0.5, pointToPack, normalToPack);    // Point on lateral surface
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);

      expectedPoint = new Point3D(1.0, 2.0, 0.5);
      flatTopFaceOctagon3d.checkIfInside(1.1, 5.0, 0.5, pointToPack, normalToPack);    // Point on lateral edge
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);

      expectedPoint = new Point3D(1.0, 2.0, 1.0);
      flatTopFaceOctagon3d.checkIfInside(1.1, 5.0, 2.0, pointToPack, normalToPack);    // Point on the top corner
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);

      expectedPoint = new Point3D(-1.0, -2.0, 1.0);
      flatTopFaceOctagon3d.checkIfInside(-2.0, -4.0, 1.5, pointToPack, normalToPack);    // Point on the top corner
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);

      expectedPoint = new Point3D(-2.0, 0.0, 1.0);
      flatTopFaceOctagon3d.checkIfInside(-3.0, 0.0, 1.5, pointToPack, normalToPack);    // Point on the top edge
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);

      expectedPoint = new Point3D(2.0, 0.0, 0.5);
      flatTopFaceOctagon3d.checkIfInside(1.99, 0.0, 0.5, pointToPack, normalToPack);    // Point just inside the rightmost vertical plane
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);

      expectedPoint = new Point3D(2.0, 0.0, 0.5);
      inclinedTopFaceOctagon3dSecond.checkIfInside(1.99, 0.0, 0.5, pointToPack, normalToPack);    // Point just inside the rightmost vertical plane
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);

      expectedPoint = new Point3D(2.0, 0.0, 0.5);
      flatTopFaceOctagon3d.checkIfInside(1.5, 0.0, 0.5, pointToPack, normalToPack);    // Point just inside the rightmost vertical plane
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);

      expectedPoint = new Point3D(2.0, 0.0, 0.5);
      inclinedTopFaceOctagon3dSecond.checkIfInside(1.5, 0.0, 0.5, pointToPack, normalToPack);    // Point just inside the rightmost vertical plane
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);

      expectedPoint = new Point3D(1.49, 0.0, 1.0);
      flatTopFaceOctagon3d.checkIfInside(1.49, 0.0, 0.5, pointToPack, normalToPack);    // Point just inside the rightmost vertical plane
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);

      expectedPoint = new Point3D(2.0, 0.0, 0.5);
      inclinedTopFaceOctagon3dSecond.checkIfInside(1.49, 0.0, 0.5, pointToPack, normalToPack);    // Point just inside the rightmost vertical plane
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);

   }

	@Test
   public void testIsInsideTheFace()
   {
      Point3D faceCenter = new Point3D(1.0, 0.0, 0.0);
      Vector3D faceNormal = new Vector3D(1.0, 0.0, 0.0);
      Plane3D facePlane = new Plane3D(faceCenter, faceNormal);
      ArrayList<Point3D> faceVertices3d = new ArrayList<Point3D>();
      faceVertices3d.add(new Point3D(1.0, -2.0, 0.0));
      faceVertices3d.add(new Point3D(1.0, 0.0, -2.0));
      faceVertices3d.add(new Point3D(1.0, 2.0, 0.0));
      faceVertices3d.add(new Point3D(1.0, 0.0, 2.0));

      // Expected conversions v1=(2.0, 0.0) v2=(0.0, 2.0) v3=(-2.0, 0.0) v4=(0.0, -2.0)
      Point3D pointToCheck = new Point3D(1.0, -1.0, 0.0);    // Point inside (1.0, 0.0)
      assertTrue(flatTopFaceOctagon3d.isInsideTheFace(facePlane, faceVertices3d, pointToCheck));

      pointToCheck.set(1.0, -1.0, 1.0);    // Point on the edge (1.0, 1.0)
      assertTrue(flatTopFaceOctagon3d.isInsideTheFace(facePlane, faceVertices3d, pointToCheck));

      pointToCheck.set(1.0, 1.0, 2.0);    // Point outside (-1.0, -2.0)
      assertFalse(flatTopFaceOctagon3d.isInsideTheFace(facePlane, faceVertices3d, pointToCheck));
   }

	@Test
   public void testSurfaceNormalAt()
   {
      Vector3D normalToPack = new Vector3D();
      Point3D pointToPack = new Point3D();

      flatTopFaceOctagon3d.checkIfInside(0.0, 0.0, 1.01, pointToPack, normalToPack);
      EuclidCoreTestTools.assertEquals(new Vector3D(0.0, 0.0, 1.0), normalToPack, 1e-4);

      flatTopFaceOctagon3d.checkIfInside(0.0, 0.0, 0.99, pointToPack, normalToPack);
      EuclidCoreTestTools.assertEquals(new Vector3D(0.0, 0.0, 1.0), normalToPack, 1e-4);

      Vector3D expected = new Vector3D(1.0, 0.0, 0.5);
      expected.normalize();
      flatTopFaceOctagon3d.checkIfInside(3.0, 0.0, 1.5, pointToPack, normalToPack);    // Point on top surface edge
      EuclidCoreTestTools.assertEquals(expected, normalToPack, epsilon);
   }

	@Test
   public void testClosestIntersectionAndNormalAt()
   {
      Point3D pointToPack = new Point3D();
      Vector3D normalToPack = new Vector3D();
      Vector3D expectedVector = new Vector3D();

      Point3D expectedPoint = new Point3D(2.0, 0.0, 0.5);
      expectedVector.set(1.0, 0.0, 0.0);
      flatTopFaceOctagon3d.checkIfInside(3.0, 0.0, 0.5, pointToPack, normalToPack);    // Point on lateral surface
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);

      expectedPoint.set(-1.5, -1.5, 0.0);
      expectedVector.set(-1.0, -1.0, 0.0);
      expectedVector.normalize();
      flatTopFaceOctagon3d.checkIfInside(-4.0, -4.0, 0.0, pointToPack, normalToPack);    // Point on lateral surface
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);

      expectedPoint.set(0.0, 2.0, 0.9);
      expectedVector.set(0.0, 1.0, 0.0);
      flatTopFaceOctagon3d.checkIfInside(0.0, 2.3, 0.9, pointToPack, normalToPack);    // Point on lateral surface
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);

      expectedPoint.set(1.0, -1.0, 1.0);
      expectedVector.set(0.0, 0.0, 1.0);
      flatTopFaceOctagon3d.checkIfInside(1.0, -1.0, 1.1, pointToPack, normalToPack);    // Point on top surface
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);

      expectedPoint.set(1.5, -1.5, 1.0);
      expectedVector.set(1.0, -1.0, 0.0);
      expectedVector.normalize();
      flatTopFaceOctagon3d.checkIfInside(1.5, -1.5, 1.0, pointToPack, normalToPack);    // Point on top surface edge
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);

      expectedPoint.set(0.0, 1.5, 1.5);
      expectedVector.set(0.0, 1.0, 1.0);
      expectedVector.normalize();
      inclinedTopFaceOctagon3dSecond.checkIfInside(0.0, 2.0, 2.0, pointToPack, normalToPack);    // Point on top (inclined) surface
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);

      expectedPoint.set(0.0, 2.0, 0.5);
      expectedVector.set(.0, 1.0, 0.0);
      inclinedTopFaceOctagon3dSecond.checkIfInside(0.0, 3.0, 0.5, pointToPack, normalToPack);    // Point on lateral surface
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);

      expectedPoint = new Point3D(1.0, 2.0, 0.5);
      expectedVector.set(1.1, 5.0, 0.5);
      expectedVector.sub(expectedPoint);
      expectedVector.normalize();
      flatTopFaceOctagon3d.checkIfInside(1.1, 5.0, 0.5, pointToPack, normalToPack);    // Point on lateral edge
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);

      expectedPoint = new Point3D(1.0, 2.0, 1.0);
      expectedVector.set(1.1, 5.0, 2.0);
      expectedVector.sub(expectedPoint);
      expectedVector.normalize();
      flatTopFaceOctagon3d.checkIfInside(1.1, 5.0, 2.0, pointToPack, normalToPack);    // Point on the top corner
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);

      expectedPoint = new Point3D(-1.0, -2.0, 1.0);
      expectedVector.set(-2.0, -4.0, 1.5);
      expectedVector.sub(expectedPoint);
      expectedVector.normalize();
      flatTopFaceOctagon3d.checkIfInside(-2.0, -4.0, 1.5, pointToPack, normalToPack);    // Point on the top corner
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);

      expectedPoint = new Point3D(-2.0, 0.0, 1.0);
      expectedVector.set(-3.0, 0.0, 1.5);
      expectedVector.sub(expectedPoint);
      expectedVector.normalize();
      flatTopFaceOctagon3d.checkIfInside(-3.0, 0.0, 1.5, pointToPack, normalToPack);    // Point on the top edge
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);

      expectedPoint = new Point3D(2.0, 0.0, 0.5);
      expectedVector.set(1.0, 0.0, 0.0);
      flatTopFaceOctagon3d.checkIfInside(1.99, 0.0, 0.5, pointToPack, normalToPack);    // Point just inside the rightmost vertical plane
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);

      expectedPoint = new Point3D(2.0, 0.0, 0.5);
      inclinedTopFaceOctagon3dSecond.checkIfInside(1.99, 0.0, 0.5, pointToPack, normalToPack);    // Point just inside the rightmost vertical plane
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);

      expectedPoint = new Point3D(2.0, 0.0, 0.5);
      flatTopFaceOctagon3d.checkIfInside(1.5, 0.0, 0.5, pointToPack, normalToPack);    // Point just inside the rightmost vertical plane
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);

      expectedPoint = new Point3D(2.0, 0.0, 0.5);
      inclinedTopFaceOctagon3dSecond.checkIfInside(1.5, 0.0, 0.5, pointToPack, normalToPack);    // Point just inside the rightmost vertical plane
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);

      expectedPoint = new Point3D(1.49, 0.0, 1.0);
      expectedVector.set(0.0, 0.0, 1.0);
      flatTopFaceOctagon3d.checkIfInside(1.49, 0.0, 0.5, pointToPack, normalToPack);    // Point just inside the rightmost vertical plane
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);

      expectedPoint = new Point3D(2.0, 0.0, 0.5);
      expectedVector.set(1.0, 0.0, 0.0);
      inclinedTopFaceOctagon3dSecond.checkIfInside(1.49, 0.0, 0.5, pointToPack, normalToPack);    // Point just inside the rightmost vertical plane
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);
      
      expectedPoint = new Point3D(0.0, -1.85, 4.85);
      expectedVector.set(0.0, 1.0, 1.0);
      expectedVector.normalize();
      inclinedTopFaceOctagon3dSecond.checkIfInside(0.0, -1.8, 4.9, pointToPack, normalToPack);    // Point just inside the 'back' (lowest y) vertical plane
      EuclidCoreTestTools.assertEquals(expectedPoint, pointToPack, epsilon);
      EuclidCoreTestTools.assertEquals(expectedVector, normalToPack, epsilon);
   }

	@Test
   public void testGetXMin()
   {
      assertEquals(-2.0, flatTopFaceOctagon3d.getBoundingBox().getMinX(), epsilon);
   }

	@Test
   public void testGetXMax()
   {
      assertEquals(2.0, flatTopFaceOctagon3d.getBoundingBox().getMaxX(), epsilon);
   }

	@Test
   public void testGetYMin()
   {
      assertEquals(-2.0, flatTopFaceOctagon3d.getBoundingBox().getMinY(), epsilon);
   }

	@Test
   public void testGetYMax()
   {
      assertEquals(2.0, flatTopFaceOctagon3d.getBoundingBox().getMaxY(), epsilon);
   }

   public void testSetupInEnvironment()
   {
      // Not an actual test, could be given @Test for visual confirmation though
      SimulationConstructionSet scs = new SimulationConstructionSet();
      scs.addStaticLinkGraphics(inclinedTopFaceOctagon3d.getLinkGraphics());

      scs.setGroundVisible(false);

      scs.startOnAThread();

      while (true)
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
