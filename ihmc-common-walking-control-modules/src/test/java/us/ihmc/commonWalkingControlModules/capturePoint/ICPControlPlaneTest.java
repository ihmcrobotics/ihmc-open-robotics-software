package us.ihmc.commonWalkingControlModules.capturePoint;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ICPControlPlaneTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
   public void testProjectPointForwardAndLeftOntoPlane()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      double gravity = 9.81;

      ReferenceFrame centerOfMassFrame = createCenterOfMassFrame(0.1, 0.1, 1.0);
      double planeHeightInCoMFrame = -1.0; //

      ICPControlPlane icpControlPlane = new ICPControlPlane(centerOfMassFrame, gravity, registry);
      icpControlPlane.setOmega0(Math.sqrt(-gravity / planeHeightInCoMFrame));


      // test plane height
      assertEquals(planeHeightInCoMFrame, icpControlPlane.getControlPlaneHeight(), 1e-10);

      // test point a little below the plane
      FramePoint3D pointToProject = new FramePoint3D(centerOfMassFrame, 0.15, 0.15, -1.1);
      pointToProject.changeFrame(worldFrame);

      FramePoint3D projectedPoint = new FramePoint3D(worldFrame);
      FramePoint3D expectedProjectedPoint = new FramePoint3D(centerOfMassFrame);

      icpControlPlane.projectPointOntoControlPlane(worldFrame, pointToProject, projectedPoint);

      expectedProjectedPoint.setX(0.15 / 1.1);
      expectedProjectedPoint.setY(0.15 / 1.1);
      expectedProjectedPoint.setZ(-1.0);
      expectedProjectedPoint.changeFrame(worldFrame);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test point a little above the plane
      pointToProject = new FramePoint3D(centerOfMassFrame, 0.15, 0.15, -0.9);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointOntoControlPlane(worldFrame, pointToProject, projectedPoint);

      expectedProjectedPoint.setX(0.15 / 0.9);
      expectedProjectedPoint.setY(0.15 / 0.9);
      expectedProjectedPoint.setZ(-1.0);
      expectedProjectedPoint.changeFrame(worldFrame);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test point on the plane
      pointToProject = new FramePoint3D(centerOfMassFrame, 0.15, 0.15, -1.0);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointOntoControlPlane(worldFrame, pointToProject, projectedPoint);

      expectedProjectedPoint.setX(0.15);
      expectedProjectedPoint.setY(0.15);
      expectedProjectedPoint.setZ(-1.0);
      expectedProjectedPoint.changeFrame(worldFrame);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test point slightly above the plane
      pointToProject = new FramePoint3D(centerOfMassFrame, 0.15, 0.15, -0.95);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointOntoControlPlane(worldFrame, pointToProject, projectedPoint);

      expectedProjectedPoint.setX(0.15 / 0.95);
      expectedProjectedPoint.setY(0.15 / 0.95);
      expectedProjectedPoint.setZ(-1.0);
      expectedProjectedPoint.changeFrame(worldFrame);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test point slightly below the plane
      pointToProject = new FramePoint3D(centerOfMassFrame, 0.15, 0.15, -1.05);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointOntoControlPlane(worldFrame, pointToProject, projectedPoint);

      expectedProjectedPoint.setX(0.15 / 1.05);
      expectedProjectedPoint.setY(0.15 / 1.05);
      expectedProjectedPoint.setZ(-1.0);
      expectedProjectedPoint.changeFrame(worldFrame);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test point barely above the plane
      pointToProject = new FramePoint3D(centerOfMassFrame, 0.15, 0.15, -0.99);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointOntoControlPlane(worldFrame, pointToProject, projectedPoint);

      expectedProjectedPoint.setX(0.15 / 0.99);
      expectedProjectedPoint.setY(0.15 / 0.99);
      expectedProjectedPoint.setZ(-1.0);
      expectedProjectedPoint.changeFrame(worldFrame);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test point barely below the plane
      pointToProject = new FramePoint3D(centerOfMassFrame, 0.15, 0.15, -1.01);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointOntoControlPlane(worldFrame, pointToProject, projectedPoint);

      expectedProjectedPoint.setX(1.0 / 1.01 * 0.15);
      expectedProjectedPoint.setY(1.0 / 1.01 * 0.15);
      expectedProjectedPoint.setZ(-1.0);
      expectedProjectedPoint.changeFrame(worldFrame);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test point far below the plane
      pointToProject = new FramePoint3D(centerOfMassFrame, 0.15, 0.15, -2.0);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointOntoControlPlane(worldFrame, pointToProject, projectedPoint);

      expectedProjectedPoint.setX(0.075);
      expectedProjectedPoint.setY(0.075);
      expectedProjectedPoint.setZ(-1.0);
      expectedProjectedPoint.changeFrame(worldFrame);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test point far above the plane
      pointToProject = new FramePoint3D(centerOfMassFrame, 0.15, 0.15, -0.05);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointOntoControlPlane(worldFrame, pointToProject, projectedPoint);

      expectedProjectedPoint.setX(20 * 0.15);
      expectedProjectedPoint.setY(20 * 0.15);
      expectedProjectedPoint.setZ(-1.0);
      expectedProjectedPoint.changeFrame(worldFrame);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test handling point slightly above the CoM
      pointToProject = new FramePoint3D(centerOfMassFrame, 0.15, 0.15, 0.05);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointOntoControlPlane(worldFrame, pointToProject, projectedPoint);

      expectedProjectedPoint.setX(-0.15 / 0.05);
      expectedProjectedPoint.setY(-0.15 / 0.05);
      expectedProjectedPoint.setZ(-1.0);
      expectedProjectedPoint.changeFrame(worldFrame);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test handling point well above the CoM
      pointToProject = new FramePoint3D(centerOfMassFrame, 0.15, 0.15, 0.5);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointOntoControlPlane(worldFrame, pointToProject, projectedPoint);

      expectedProjectedPoint.setX(-0.3);
      expectedProjectedPoint.setY(-0.3);
      expectedProjectedPoint.setZ(-1.0);
      expectedProjectedPoint.changeFrame(worldFrame);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);
   }

   /**
    * FIXME This test was never actually working but was passing before euclid 0.9.0.
    * <p>
    * Before that release, both the expected and actual points when transformed to world frame were
    * filled up with NaNs which caused the assertion to pass. With euclid 0.9.0, the transform are
    * slightly smarter in the sense that if their rotation part is equal to identity, no operation
    * is performed when rotating a point. This occurs as the center of mass frame and world frame
    * are aligned. As a result, the points are not rotated anymore (as it is not needed) and not
    * filled up with NaNs, now the test fails as it should have always been.
    * </p>
    */
   @Disabled
   @Test
   public void testProjectPointForwardAndLeftOntoPlaneEdgeCase()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      double gravity = 9.81;

      ReferenceFrame centerOfMassFrame = createCenterOfMassFrame(0.1, 0.1, 1.0);
      double planeHeightInCoMFrame = -1.0; //

      ICPControlPlane icpControlPlane = new ICPControlPlane(centerOfMassFrame, gravity, registry);
      icpControlPlane.setOmega0(Math.sqrt(-gravity / planeHeightInCoMFrame));


      // test plane height
      assertEquals(planeHeightInCoMFrame, icpControlPlane.getControlPlaneHeight(), 1e-10);

      // test handling point at CoM
      FramePoint3D pointToProject = new FramePoint3D(centerOfMassFrame, 0.15, 0.15, 0.0);
      pointToProject.changeFrame(worldFrame);

      FramePoint3D projectedPoint = new FramePoint3D(worldFrame);
      FramePoint3D expectedProjectedPoint = new FramePoint3D(centerOfMassFrame);

      icpControlPlane.projectPointOntoControlPlane(worldFrame, pointToProject, projectedPoint);

      expectedProjectedPoint.setX(Double.POSITIVE_INFINITY);
      expectedProjectedPoint.setY(Double.NEGATIVE_INFINITY);
      expectedProjectedPoint.setZ(-1.0);
      expectedProjectedPoint.changeFrame(worldFrame);

      assertEquals(expectedProjectedPoint.getX(), projectedPoint.getX(), 1e-10);
      assertEquals(expectedProjectedPoint.getY(), projectedPoint.getY(), 1e-10);
      assertEquals(expectedProjectedPoint.getZ(), projectedPoint.getZ(), 1e-10);
      expectedProjectedPoint.checkReferenceFrameMatch(projectedPoint);
   }

   @Test
   public void testRandomProjectOntoPlane()
   {
      Random random = new Random(12345);

      for (int iter = 0; iter < 1000; iter++)
      {
         YoVariableRegistry registry = new YoVariableRegistry("robert");
         double gravity = 9.81;

         double xCoMPosition = RandomNumbers.nextDouble(random, 10.0);
         double yCoMPosition = RandomNumbers.nextDouble(random, 10.0);
         double zCoMPosition = RandomNumbers.nextDouble(random, 10.0);
         ReferenceFrame centerOfMassFrame = createCenterOfMassFrame(xCoMPosition, yCoMPosition, zCoMPosition);
         double planeHeightInCoMFrame = RandomNumbers.nextDouble(random, -5.0, 0.001);

         ICPControlPlane icpControlPlane = new ICPControlPlane(centerOfMassFrame, gravity, registry);
         icpControlPlane.setOmega0(Math.sqrt(-gravity / planeHeightInCoMFrame));

         // test plane height
         assertEquals("iteration " + iter, planeHeightInCoMFrame, icpControlPlane.getControlPlaneHeight(), 1e-10);

         // test point a little below the plane
         double xPointPosition = RandomNumbers.nextDouble(random, 20.0);
         double yPointPosition = RandomNumbers.nextDouble(random, 20.0);
         double zPointPosition = RandomNumbers.nextDouble(random, 20.0);
         FramePoint3D pointToProject = new FramePoint3D(centerOfMassFrame, xPointPosition, yPointPosition, zPointPosition);
         pointToProject.changeFrame(worldFrame);

         FramePoint3D projectedPoint = new FramePoint3D(worldFrame);
         FramePoint3D expectedProjectedPoint = new FramePoint3D(centerOfMassFrame);

         icpControlPlane.projectPointOntoControlPlane(worldFrame, pointToProject, projectedPoint);

         expectedProjectedPoint.setX(xPointPosition * planeHeightInCoMFrame / zPointPosition);
         expectedProjectedPoint.setY(yPointPosition * planeHeightInCoMFrame / zPointPosition);
         expectedProjectedPoint.setZ(planeHeightInCoMFrame);
         expectedProjectedPoint.changeFrame(worldFrame);

         EuclidCoreTestTools.assertTuple3DEquals("iteration " + iter, expectedProjectedPoint, projectedPoint, 1e-10);
      }
   }

   @Test
   public void testProjectPointForwardAndLeftFromPlaneOntoSurface()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      double gravity = 9.81;

      ReferenceFrame centerOfMassFrame = createCenterOfMassFrame(0.1, 0.1, 1.0);
      double planeHeightInCoMFrame = -1.0; //

      ICPControlPlane icpControlPlane = new ICPControlPlane(centerOfMassFrame, gravity, registry);
      icpControlPlane.setOmega0(Math.sqrt(-gravity / planeHeightInCoMFrame));


      // test plane height
      assertEquals(planeHeightInCoMFrame, icpControlPlane.getControlPlaneHeight(), 1e-10);

      // test point a little below the plane
      FramePoint2D pointToProject = new FramePoint2D(centerOfMassFrame, 0.15, 0.15);
      pointToProject.changeFrame(worldFrame);

      FramePoint3D projectedPoint = new FramePoint3D(worldFrame);
      FramePoint3D expectedProjectedPoint = new FramePoint3D(centerOfMassFrame);

      icpControlPlane.projectPointFromPlaneOntoSurface(worldFrame, pointToProject, projectedPoint, -0.1);

      expectedProjectedPoint.setX(1.1 * 0.15);
      expectedProjectedPoint.setY(1.1 * 0.15);
      expectedProjectedPoint.changeFrame(worldFrame);
      expectedProjectedPoint.setZ(-0.1);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test point a little above the plane
      pointToProject = new FramePoint2D(centerOfMassFrame, 0.15, 0.15);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointFromPlaneOntoSurface(worldFrame, pointToProject, projectedPoint, 0.1);

      expectedProjectedPoint.setX(0.9 * 0.15);
      expectedProjectedPoint.setY(0.9 * 0.15);
      expectedProjectedPoint.changeFrame(worldFrame);
      expectedProjectedPoint.setZ(0.1);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test point on the plane
      pointToProject = new FramePoint2D(centerOfMassFrame, 0.15, 0.15);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointFromPlaneOntoSurface(worldFrame, pointToProject, projectedPoint, 0.0);

      expectedProjectedPoint.setX(0.15);
      expectedProjectedPoint.setY(0.15);
      expectedProjectedPoint.setZ(-1.0);
      expectedProjectedPoint.changeFrame(worldFrame);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test point slightly above the plane
      pointToProject = new FramePoint2D(centerOfMassFrame, 0.15, 0.15);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointFromPlaneOntoSurface(worldFrame, pointToProject, projectedPoint, 0.05);

      expectedProjectedPoint.setX(0.95 * 0.15);
      expectedProjectedPoint.setY(0.95 * 0.15);
      expectedProjectedPoint.changeFrame(worldFrame);
      expectedProjectedPoint.setZ(0.05);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);


      // test point slightly below the plane
      pointToProject = new FramePoint2D(centerOfMassFrame, 0.15, 0.15);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointFromPlaneOntoSurface(worldFrame, pointToProject, projectedPoint, -0.05);

      expectedProjectedPoint.setX(1.05 * 0.15);
      expectedProjectedPoint.setY(1.05 * 0.15);
      expectedProjectedPoint.changeFrame(worldFrame);
      expectedProjectedPoint.setZ(-0.05);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test point barely above the plane
      pointToProject = new FramePoint2D(centerOfMassFrame, 0.15, 0.15);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointFromPlaneOntoSurface(worldFrame, pointToProject, projectedPoint, 0.01);

      expectedProjectedPoint.setX(0.99 * 0.15);
      expectedProjectedPoint.setY(0.99 * 0.15);
      expectedProjectedPoint.changeFrame(worldFrame);
      expectedProjectedPoint.setZ(0.01);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test point barely below the plane
      pointToProject = new FramePoint2D(centerOfMassFrame, 0.15, 0.15);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointFromPlaneOntoSurface(worldFrame, pointToProject, projectedPoint, -0.01);

      expectedProjectedPoint.setX(1.01 * 0.15);
      expectedProjectedPoint.setY(1.01 * 0.15);
      expectedProjectedPoint.changeFrame(worldFrame);
      expectedProjectedPoint.setZ(-0.01);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test point far below the plane
      pointToProject = new FramePoint2D(centerOfMassFrame, 0.15, 0.15);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointFromPlaneOntoSurface(worldFrame, pointToProject, projectedPoint, -1.0);

      expectedProjectedPoint.setX(0.3);
      expectedProjectedPoint.setY(0.3);
      expectedProjectedPoint.changeFrame(worldFrame);
      expectedProjectedPoint.setZ(-1.0);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test point far above the plane
      pointToProject = new FramePoint2D(centerOfMassFrame, 0.15, 0.15);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointFromPlaneOntoSurface(worldFrame, pointToProject, projectedPoint, 0.95);

      expectedProjectedPoint.setX(0.05 * 0.15);
      expectedProjectedPoint.setY(0.05 * 0.15);
      expectedProjectedPoint.changeFrame(worldFrame);
      expectedProjectedPoint.setZ(0.95);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test handling point slightly above the CoM
      pointToProject = new FramePoint2D(centerOfMassFrame, 0.15, 0.15);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointFromPlaneOntoSurface(worldFrame, pointToProject, projectedPoint, 1.05);

      expectedProjectedPoint.setX(-0.05 * 0.15);
      expectedProjectedPoint.setY(-0.05 * 0.15);
      expectedProjectedPoint.changeFrame(worldFrame);
      expectedProjectedPoint.setZ(1.05);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);

      // test handling point well above the CoM
      pointToProject = new FramePoint2D(centerOfMassFrame, 0.15, 0.15);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointFromPlaneOntoSurface(worldFrame, pointToProject, projectedPoint, 1.5);

      expectedProjectedPoint.setX(-0.075);
      expectedProjectedPoint.setY(-0.075);
      expectedProjectedPoint.changeFrame(worldFrame);
      expectedProjectedPoint.setZ(1.5);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);
   }

   @Test
   public void testRandomProjectOntoSurface()
   {
      Random random = new Random(12345);

      for (int iter = 0; iter < 1000; iter++)
      {
         YoVariableRegistry registry = new YoVariableRegistry("robert");
         double gravity = 9.81;

         double xCoMPosition = RandomNumbers.nextDouble(random, 10.0);
         double yCoMPosition = RandomNumbers.nextDouble(random, 10.0);
         double zCoMPosition = RandomNumbers.nextDouble(random, 10.0);
         ReferenceFrame centerOfMassFrame = createCenterOfMassFrame(xCoMPosition, yCoMPosition, zCoMPosition);
         double planeHeightInCoMFrame = RandomNumbers.nextDouble(random, -5.0, 0.001);

         ICPControlPlane icpControlPlane = new ICPControlPlane(centerOfMassFrame, gravity, registry);
         icpControlPlane.setOmega0(Math.sqrt(-gravity / planeHeightInCoMFrame));

         // test plane height
         assertEquals("iteration " + iter, planeHeightInCoMFrame, icpControlPlane.getControlPlaneHeight(), 1e-10);

         // test point a little below the plane
         double xPointPosition = RandomNumbers.nextDouble(random, 20.0);
         double yPointPosition = RandomNumbers.nextDouble(random, 20.0);

         double zSurfacePosition = RandomNumbers.nextDouble(random, 20.0);
         FramePoint2D pointToProject = new FramePoint2D(centerOfMassFrame, xPointPosition, yPointPosition);
         pointToProject.changeFrame(worldFrame);

         FramePoint3D projectedPoint = new FramePoint3D(worldFrame);
         FramePoint3D expectedProjectedPoint = new FramePoint3D(centerOfMassFrame);

         icpControlPlane.projectPointFromPlaneOntoSurface(worldFrame, pointToProject, projectedPoint, zSurfacePosition);

         expectedProjectedPoint.setX(xPointPosition * (zSurfacePosition - zCoMPosition) / planeHeightInCoMFrame);
         expectedProjectedPoint.setY(yPointPosition * (zSurfacePosition - zCoMPosition) / planeHeightInCoMFrame);
         expectedProjectedPoint.changeFrame(worldFrame);
         expectedProjectedPoint.setZ(zSurfacePosition);

         EuclidCoreTestTools.assertTuple3DEquals("iteration " + iter, expectedProjectedPoint, projectedPoint, 1e-10);
      }
   }

   @Test
   public void testProjectPlanarRegionMostBasic()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      double gravity = 9.81;

      ReferenceFrame centerOfMassFrame = createCenterOfMassFrame(0.0, 0.0, 1.0);
      double planeHeightInCoMFrame = -1.0; //

      ICPControlPlane icpControlPlane = new ICPControlPlane(centerOfMassFrame, gravity, registry);
      icpControlPlane.setOmega0(Math.sqrt(-gravity / planeHeightInCoMFrame));

      TranslationReferenceFrame referenceFrame = new TranslationReferenceFrame("test", worldFrame);
      referenceFrame.updateTranslation(new Vector3D(0.0, 0.0, 0.1));

      ConvexPolygon2D convexPolygon2D = new ConvexPolygon2D();
      Point2D point1 = new Point2D(0.2, 0.2);
      Point2D point2 = new Point2D(0.2, -0.2);
      Point2D point3 = new Point2D(-0.2, -0.2);
      Point2D point4 = new Point2D(-0.2, 0.2);
      convexPolygon2D.addVertex(point1);
      convexPolygon2D.addVertex(point2);
      convexPolygon2D.addVertex(point3);
      convexPolygon2D.addVertex(point4);
      convexPolygon2D.update();

      PlanarRegion planarRegion = new PlanarRegion(referenceFrame.getTransformToWorldFrame(), convexPolygon2D);

      ConvexPolygon2D projectedConvexPolygon = new ConvexPolygon2D();
      icpControlPlane.projectPlanarRegionConvexHullOntoControlPlane(planarRegion, projectedConvexPolygon);

      Point2DReadOnly projectedPoint1 = convexPolygon2D.getVertex(0);
      Point2DReadOnly projectedPoint2 = convexPolygon2D.getVertex(1);
      Point2DReadOnly projectedPoint3 = convexPolygon2D.getVertex(2);
      Point2DReadOnly projectedPoint4 = convexPolygon2D.getVertex(3);

      Point2D predictedProjectPoint1 = new Point2D(projectedPoint1);
      Point2D predictedProjectPoint2 = new Point2D(projectedPoint2);
      Point2D predictedProjectPoint3 = new Point2D(projectedPoint3);
      Point2D predictedProjectPoint4 = new Point2D(projectedPoint4);

      predictedProjectPoint1.scale(1.0 / 0.9);
      predictedProjectPoint2.scale(1.0 / 0.9);
      predictedProjectPoint3.scale(1.0 / 0.9);
      predictedProjectPoint4.scale(1.0 / 0.9);


      EuclidCoreTestTools.assertTuple2DEquals(predictedProjectPoint1, projectedConvexPolygon.getVertex(0), 1e-10);
      EuclidCoreTestTools.assertTuple2DEquals(predictedProjectPoint2, projectedConvexPolygon.getVertex(1), 1e-10);
      EuclidCoreTestTools.assertTuple2DEquals(predictedProjectPoint3, projectedConvexPolygon.getVertex(2), 1e-10);
      EuclidCoreTestTools.assertTuple2DEquals(predictedProjectPoint4, projectedConvexPolygon.getVertex(3), 1e-10);
   }

   @Test
   public void testProjectPlanarRegion()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      double gravity = 9.81;

      ReferenceFrame centerOfMassFrame = createCenterOfMassFrame(0.1, 0.1, 1.0);
      double planeHeightInCoMFrame = -1.0; //

      ICPControlPlane icpControlPlane = new ICPControlPlane(centerOfMassFrame, gravity, registry);
      icpControlPlane.setOmega0(Math.sqrt(-gravity / planeHeightInCoMFrame));

      TranslationReferenceFrame referenceFrame = new TranslationReferenceFrame("test", worldFrame);
      referenceFrame.updateTranslation(new Vector3D(0.0, 0.0, 0.1));

      ConvexPolygon2D convexPolygon2D = new ConvexPolygon2D();
      Point2D point1 = new Point2D(0.2, 0.2);
      Point2D point2 = new Point2D(0.2, -0.2);
      Point2D point3 = new Point2D(-0.2, -0.2);
      Point2D point4 = new Point2D(-0.2, 0.2);
      convexPolygon2D.addVertex(point1);
      convexPolygon2D.addVertex(point2);
      convexPolygon2D.addVertex(point3);
      convexPolygon2D.addVertex(point4);
      convexPolygon2D.update();

      PlanarRegion planarRegion = new PlanarRegion(referenceFrame.getTransformToWorldFrame(), convexPolygon2D);

      ConvexPolygon2D projectedConvexPolygon = new ConvexPolygon2D();
      icpControlPlane.projectPlanarRegionConvexHullOntoControlPlane(planarRegion, projectedConvexPolygon);

      Point2DReadOnly projectedPoint1 = convexPolygon2D.getVertex(0);
      Point2DReadOnly projectedPoint2 = convexPolygon2D.getVertex(1);
      Point2DReadOnly projectedPoint3 = convexPolygon2D.getVertex(2);
      Point2DReadOnly projectedPoint4 = convexPolygon2D.getVertex(3);

      Point2D predictedProjectPoint1 = new Point2D(projectedPoint1);
      Point2D predictedProjectPoint2 = new Point2D(projectedPoint2);
      Point2D predictedProjectPoint3 = new Point2D(projectedPoint3);
      Point2D predictedProjectPoint4 = new Point2D(projectedPoint4);

      predictedProjectPoint1.sub(0.1, 0.1);
      predictedProjectPoint2.sub(0.1, 0.1);
      predictedProjectPoint3.sub(0.1, 0.1);
      predictedProjectPoint4.sub(0.1, 0.1);

      predictedProjectPoint1.scale(1.0 / 0.9);
      predictedProjectPoint2.scale(1.0 / 0.9);
      predictedProjectPoint3.scale(1.0 / 0.9);
      predictedProjectPoint4.scale(1.0 / 0.9);

      predictedProjectPoint1.add(0.1, 0.1);
      predictedProjectPoint2.add(0.1, 0.1);
      predictedProjectPoint3.add(0.1, 0.1);
      predictedProjectPoint4.add(0.1, 0.1);

      EuclidCoreTestTools.assertTuple2DEquals(predictedProjectPoint1, projectedConvexPolygon.getVertex(0), 1e-10);
      EuclidCoreTestTools.assertTuple2DEquals(predictedProjectPoint2, projectedConvexPolygon.getVertex(1), 1e-10);
      EuclidCoreTestTools.assertTuple2DEquals(predictedProjectPoint3, projectedConvexPolygon.getVertex(2), 1e-10);
      EuclidCoreTestTools.assertTuple2DEquals(predictedProjectPoint4, projectedConvexPolygon.getVertex(3), 1e-10);
   }

   private static ReferenceFrame createCenterOfMassFrame(double x, double y, double z)
   {
      TranslationReferenceFrame centerOfMassFrame = new TranslationReferenceFrame("centerOfMass", worldFrame);
      centerOfMassFrame.updateTranslation(new Vector3D(x, y, z));

      return centerOfMassFrame;
   }
}
