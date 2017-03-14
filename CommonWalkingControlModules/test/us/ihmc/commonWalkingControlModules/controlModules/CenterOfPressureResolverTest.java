package us.ihmc.commonWalkingControlModules.controlModules;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.tools.MemoryTools;

public class CenterOfPressureResolverTest
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.4)
   @Test(timeout = 30000)
   public void testCenterOfPressureResolverSimpleCaseWithNoTorque()
   {
      Point3D groundPoint = new Point3D();
      Vector3D groundNormal = new Vector3D(0.0, 0.0, 1.0);

      Point3D centerOfMassPoint = new Point3D(0.0, 0.0, 1.0);

      Vector3D centerOfMassForce = new Vector3D(1.0, 2.0, 1.0);
      Vector3D centerOfMassTorque = new Vector3D(0.0, 0.0, 0.0);

      Vector3D expectedCenterOfPressure = new Vector3D(-1.0, -2.0, 0.0);

      double expectedNormalTorque = 0.0;

      computeAndVerifyCenterOfPressureAndNormalTorque(groundPoint, groundNormal, centerOfMassPoint, centerOfMassForce, centerOfMassTorque,
            expectedCenterOfPressure, expectedNormalTorque);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCenterOfPressureResolverSimpleCaseWithVerticalForce()
   {
      Point3D groundPoint = new Point3D();
      Vector3D groundNormal = new Vector3D(0.0, 0.0, 1.0);

      Point3D centerOfMassPoint = new Point3D(0.0, 0.0, 1.0);

      Vector3D centerOfMassForce = new Vector3D(0.0, 0.0, 1.0);
      Vector3D centerOfMassTorque = new Vector3D(1.0, 2.0, 3.0);

      Vector3D expectedCenterOfPressure = new Vector3D(-2.0, 1.0, 0.0);

      double expectedNormalTorque = 3.0;

      computeAndVerifyCenterOfPressureAndNormalTorque(groundPoint, groundNormal, centerOfMassPoint, centerOfMassForce, centerOfMassTorque,
            expectedCenterOfPressure, expectedNormalTorque);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testCenterOfPressureResolverNoForceInZ()
   {
      Point3D groundPoint = new Point3D();
      Vector3D groundNormal = new Vector3D(0.0, 0.0, 1.0);

      Point3D centerOfMassPoint = new Point3D(0.0, 0.0, 1.0);

      Vector3D centerOfMassForce = new Vector3D(1.0, 0.0, 0.0);
      Vector3D centerOfMassTorque = new Vector3D(0.1, 0.2, 0.3);

      Vector3D expectedCenterOfPressure = new Vector3D(Double.NaN, Double.NaN, Double.NaN);

      double expectedNormalTorque = 0.3;

      computeAndVerifyCenterOfPressureAndNormalTorque(groundPoint, groundNormal, centerOfMassPoint, centerOfMassForce, centerOfMassTorque,
            expectedCenterOfPressure, expectedNormalTorque);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.4)
   @Test(timeout = 30000)
   public void testRandomExamples()
   {
      Random random = new Random(1776L);
      int numberOfTests = 10000;

      for (int i = 0; i < numberOfTests; i++)
      {
         Point3D groundPoint = RandomGeometry.nextPoint3D(random, 1.0, 1.0, 0.2);

         Vector3D groundNormal = RandomGeometry.nextVector3D(random);
         groundNormal.setZ(1.0);
         groundNormal.normalize();

         Point3D centerOfMassPoint = RandomGeometry.nextPoint3D(random, 1.0, 1.0, 0.2);
         centerOfMassPoint.setZ(1.2);

         Vector3D centerOfMassForce = RandomGeometry.nextVector3D(random, 100.0);
         centerOfMassForce.setZ(127.0);

         Vector3D centerOfMassTorque = new Vector3D(0.0, 0.0, 0.0);

         PoseReferenceFrame centerOfMassFrame = createTranslatedZUpFrame("centerOfMassFrame", centerOfMassPoint);
         SpatialForceVector spatialForceVector = new SpatialForceVector(centerOfMassFrame, centerOfMassForce, centerOfMassTorque);

         ImmutablePair<FramePoint, Double> centerOfPressureAndNormalTorque = computeCenterOfPressureAndNormalTorque(groundPoint, groundNormal,
               centerOfMassFrame, spatialForceVector);
         FramePoint centerOfPressure = centerOfPressureAndNormalTorque.getLeft();
         double normalTorqueResolvedAtCenterOfPressure = centerOfPressureAndNormalTorque.getRight();
         centerOfPressure.changeFrame(worldFrame);

         Point3D copPoint3d = new Point3D();
         centerOfPressure.get(copPoint3d);
         PoseReferenceFrame centerOfPressurePlaneFrame = createPlaneFrame("groundPlaneFrame", copPoint3d, groundNormal);
         spatialForceVector.changeFrame(centerOfPressurePlaneFrame);

         FrameVector forceResolvedInCenterOfPressureFrame = spatialForceVector.getLinearPartAsFrameVectorCopy();
         FrameVector torqueResolvedInCenterOfPressureFrame = spatialForceVector.getAngularPartAsFrameVectorCopy();

         forceResolvedInCenterOfPressureFrame.changeFrame(worldFrame);

         // Forces should be the same, after rotated into the same frame:
         EuclidCoreTestTools.assertTuple3DEquals(centerOfMassForce, forceResolvedInCenterOfPressureFrame.getVectorCopy(), 1e-7);

         // Torque should have no components in x and y resolved at the Center of Pressure:
         assertEquals(0.0, torqueResolvedInCenterOfPressureFrame.getX(), 1e-7);
         assertEquals(0.0, torqueResolvedInCenterOfPressureFrame.getY(), 1e-7);
         assertEquals(normalTorqueResolvedAtCenterOfPressure, torqueResolvedInCenterOfPressureFrame.getZ(), 1e-7);
      }
   }

   private static void computeAndVerifyCenterOfPressureAndNormalTorque(Point3D groundPoint, Vector3D groundNormal, Point3D centerOfMassPoint,
         Vector3D centerOfMassForce, Vector3D centerOfMassTorque, Vector3D expectedCenterOfPressure, double expectedNormalTorque)
   {
      PoseReferenceFrame centerOfMassFrame = createTranslatedZUpFrame("centerOfMassFrame", centerOfMassPoint);
      SpatialForceVector spatialForceVector = new SpatialForceVector(centerOfMassFrame, centerOfMassForce, centerOfMassTorque);

      ImmutablePair<FramePoint, Double> centerOfPressureAndNormalTorque = computeCenterOfPressureAndNormalTorque(groundPoint, groundNormal, centerOfMassFrame,
            spatialForceVector);
      FramePoint centerOfPressure = centerOfPressureAndNormalTorque.getLeft();
      double normalTorque = centerOfPressureAndNormalTorque.getRight();

      FramePoint expectedCenterOfPressureFramePoint = new FramePoint(worldFrame, expectedCenterOfPressure);

      if (Double.isNaN(expectedCenterOfPressureFramePoint.getX()) && Double.isNaN(expectedCenterOfPressureFramePoint.getY())
            && Double.isNaN(expectedCenterOfPressureFramePoint.getZ()))
         assertTrue(Double.isNaN(centerOfPressure.getX()) && Double.isNaN(centerOfPressure.getY()) && Double.isNaN(centerOfPressure.getZ()));
      else
         assertTrue("expectedCenterOfPressureFramePoint = " + expectedCenterOfPressureFramePoint + ", centerOfPressure = " + centerOfPressure,
               expectedCenterOfPressureFramePoint.epsilonEquals(centerOfPressure, 1e-7));

      assertEquals(expectedNormalTorque, normalTorque, 1e-7);
   }

   private static ImmutablePair<FramePoint, Double> computeCenterOfPressureAndNormalTorque(Point3D groundPoint, Vector3D groundNormal,
         ReferenceFrame centerOfMassFrame, SpatialForceVector spatialForceVector)
   {
      CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();

      PoseReferenceFrame groundPlaneFrame = createPlaneFrame("groundPlaneFrame", groundPoint, groundNormal);
      FramePoint2d centerOfPressure2d = new FramePoint2d(worldFrame);

      double normalTorqueAtCenterOfPressure = centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(centerOfPressure2d, spatialForceVector,
            groundPlaneFrame);
      FramePoint centerOfPressure = centerOfPressure2d.toFramePoint();
      centerOfPressure.changeFrame(worldFrame);
      return new ImmutablePair<FramePoint, Double>(centerOfPressure, normalTorqueAtCenterOfPressure);
   }

   private static PoseReferenceFrame createPlaneFrame(String name, Point3D planeReferencePoint, Vector3D planeSurfaceNormal)
   {
      PoseReferenceFrame planeFrame = new PoseReferenceFrame(name, ReferenceFrame.getWorldFrame());
      RotationMatrix rotationFromNormal = computeRotationFromNormal(planeSurfaceNormal);

      RigidBodyTransform transform3D = new RigidBodyTransform(rotationFromNormal, new Vector3D(planeReferencePoint));
      FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame(), transform3D);

      planeFrame.setPoseAndUpdate(framePose);
      planeFrame.update();
      return planeFrame;
   }

   private static PoseReferenceFrame createTranslatedZUpFrame(String name, Point3D frameCenterPoint)
   {
      PoseReferenceFrame translatedZUpFrame = new PoseReferenceFrame(name, ReferenceFrame.getWorldFrame());
      RigidBodyTransform transform3D = new RigidBodyTransform();
      transform3D.setTranslation(new Vector3D(frameCenterPoint));

      FramePose framePose = new FramePose(ReferenceFrame.getWorldFrame(), transform3D);
      translatedZUpFrame.setPoseAndUpdate(framePose);
      translatedZUpFrame.update();
      return translatedZUpFrame;
   }

   private static RotationMatrix computeRotationFromNormal(Vector3D normal)
   {
      // Won't work if normal is 1,0,0

      Vector3D zAxis = new Vector3D(normal);
      zAxis.normalize();

      Vector3D xAxis = new Vector3D(1.0, 0.0, 0.0);

      Vector3D yAxis = new Vector3D();
      yAxis.cross(normal, xAxis);
      yAxis.normalize();

      xAxis.cross(yAxis, zAxis);
      xAxis.normalize();

      zAxis.cross(xAxis, yAxis);
      zAxis.normalize();

      RotationMatrix rotation = new RotationMatrix(xAxis.getX(), xAxis.getY(), xAxis.getZ(), yAxis.getX(), yAxis.getY(), yAxis.getZ(), zAxis.getX(), zAxis.getY(),
            zAxis.getZ());
      return rotation;
   }
}
