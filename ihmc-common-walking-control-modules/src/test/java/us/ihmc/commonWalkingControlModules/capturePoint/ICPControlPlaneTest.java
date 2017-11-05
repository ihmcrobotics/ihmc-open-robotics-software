package us.ihmc.commonWalkingControlModules.capturePoint;

import org.junit.Test;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.ICPControlPlane;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.referenceFrames.TranslationReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Random;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class ICPControlPlaneTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testProjectPointForwardAndLeftOntoPlane()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      YoDouble omega = new YoDouble("omega", registry);
      double gravity = 9.81;

      ReferenceFrame centerOfMassFrame = createCenterOfMassFrame(0.1, 0.1, 1.0);
      double planeHeightInCoMFrame = -1.0; //

      ICPControlPlane icpControlPlane = new ICPControlPlane(omega, centerOfMassFrame, gravity, registry);
      omega.set(Math.sqrt(-gravity / planeHeightInCoMFrame));


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

      // test handling point at CoM
      pointToProject = new FramePoint3D(centerOfMassFrame, 0.15, 0.15, 0.0);
      pointToProject.changeFrame(worldFrame);

      projectedPoint.setToZero(worldFrame);
      expectedProjectedPoint.setToZero(centerOfMassFrame);

      icpControlPlane.projectPointOntoControlPlane(worldFrame, pointToProject, projectedPoint);

      expectedProjectedPoint.setX(Double.POSITIVE_INFINITY);
      expectedProjectedPoint.setY(Double.NEGATIVE_INFINITY);
      expectedProjectedPoint.setZ(-1.0);
      expectedProjectedPoint.changeFrame(worldFrame);

      EuclidCoreTestTools.assertTuple3DEquals(expectedProjectedPoint, projectedPoint, 1e-10);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRandomProjectOntoPlane()
   {
      Random random = new Random(12345);

      for (int iter = 0; iter < 1000; iter++)
      {
         YoVariableRegistry registry = new YoVariableRegistry("robert");
         YoDouble omega = new YoDouble("omega", registry);
         double gravity = 9.81;

         double xCoMPosition = RandomNumbers.nextDouble(random, 10.0);
         double yCoMPosition = RandomNumbers.nextDouble(random, 10.0);
         double zCoMPosition = RandomNumbers.nextDouble(random, 10.0);
         ReferenceFrame centerOfMassFrame = createCenterOfMassFrame(xCoMPosition, yCoMPosition, zCoMPosition);
         double planeHeightInCoMFrame = RandomNumbers.nextDouble(random, -5.0, 0.001);

         ICPControlPlane icpControlPlane = new ICPControlPlane(omega, centerOfMassFrame, gravity, registry);
         omega.set(Math.sqrt(-gravity / planeHeightInCoMFrame));

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testProjectPointForwardAndLeftFromPlaneOntoSurface()
   {
      YoVariableRegistry registry = new YoVariableRegistry("robert");
      YoDouble omega = new YoDouble("omega", registry);
      double gravity = 9.81;

      ReferenceFrame centerOfMassFrame = createCenterOfMassFrame(0.1, 0.1, 1.0);
      double planeHeightInCoMFrame = -1.0; //

      ICPControlPlane icpControlPlane = new ICPControlPlane(omega, centerOfMassFrame, gravity, registry);
      omega.set(Math.sqrt(-gravity / planeHeightInCoMFrame));


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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRandomProjectOntoSurface()
   {
      Random random = new Random(12345);

      for (int iter = 0; iter < 1000; iter++)
      {
         YoVariableRegistry registry = new YoVariableRegistry("robert");
         YoDouble omega = new YoDouble("omega", registry);
         double gravity = 9.81;

         double xCoMPosition = RandomNumbers.nextDouble(random, 10.0);
         double yCoMPosition = RandomNumbers.nextDouble(random, 10.0);
         double zCoMPosition = RandomNumbers.nextDouble(random, 10.0);
         ReferenceFrame centerOfMassFrame = createCenterOfMassFrame(xCoMPosition, yCoMPosition, zCoMPosition);
         double planeHeightInCoMFrame = RandomNumbers.nextDouble(random, -5.0, 0.001);

         ICPControlPlane icpControlPlane = new ICPControlPlane(omega, centerOfMassFrame, gravity, registry);
         omega.set(Math.sqrt(-gravity / planeHeightInCoMFrame));

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

   private static ReferenceFrame createCenterOfMassFrame(double x, double y, double z)
   {
      TranslationReferenceFrame centerOfMassFrame = new TranslationReferenceFrame("centerOfMass", worldFrame);
      centerOfMassFrame.updateTranslation(new Vector3D(x, y, z));

      return centerOfMassFrame;
   }
}
