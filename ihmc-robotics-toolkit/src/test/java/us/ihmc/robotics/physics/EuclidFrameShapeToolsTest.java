package us.ihmc.robotics.physics;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.shape.primitives.PointShape3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Vector3D;

public class EuclidFrameShapeToolsTest
{
   private static final int ITERATIONS = 5000;
   private static final double EPSILON = 1.0e-12;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testChangeFrame()
   {
      Random random = new Random(65491);

      for (int i = 0; i < ITERATIONS; i++)
      { // Test for Box3D
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);
         Box3D original = EuclidShapeRandomTools.nextBox3D(random);

         FramePose3D expectedPose = new FramePose3D(frameA, original.getPose());
         expectedPose.changeFrame(frameB);
         Vector3D expectedSize = new Vector3D(original.getSize());

         Box3D transformed = EuclidFrameShapeTools.changeFrame(original, frameA, frameB);
         EuclidGeometryTestTools.assertPose3DGeometricallyEquals(expectedPose, new Pose3D(transformed.getPose()), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedSize, transformed.getSize(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test for Capsule3D
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);
         Capsule3D original = EuclidShapeRandomTools.nextCapsule3D(random);

         FramePoint3D expectedPosition = new FramePoint3D(frameA, original.getPosition());
         expectedPosition.changeFrame(frameB);
         FrameVector3D expectedAxis = new FrameVector3D(frameA, original.getAxis());
         expectedAxis.changeFrame(frameB);

         Capsule3D transformed = EuclidFrameShapeTools.changeFrame(original, frameA, frameB);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPosition, transformed.getPosition(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedAxis, transformed.getAxis(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test for Cylinder3D
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);
         Cylinder3D original = EuclidShapeRandomTools.nextCylinder3D(random);

         FramePoint3D expectedPosition = new FramePoint3D(frameA, original.getPosition());
         expectedPosition.changeFrame(frameB);
         FrameVector3D expectedAxis = new FrameVector3D(frameA, original.getAxis());
         expectedAxis.changeFrame(frameB);

         Cylinder3D transformed = EuclidFrameShapeTools.changeFrame(original, frameA, frameB);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPosition, transformed.getPosition(), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedAxis, transformed.getAxis(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test for Ellipsoid3D
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);
         Ellipsoid3D original = EuclidShapeRandomTools.nextEllipsoid3D(random);

         FramePose3D expectedPose = new FramePose3D(frameA, original.getPose());
         expectedPose.changeFrame(frameB);
         Vector3D expectedRadii = new Vector3D(original.getRadii());

         Ellipsoid3D transformed = EuclidFrameShapeTools.changeFrame(original, frameA, frameB);
         EuclidGeometryTestTools.assertPose3DGeometricallyEquals(expectedPose, new Pose3D(transformed.getPose()), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedRadii, transformed.getRadii(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test for PointShape3D
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);
         PointShape3D original = EuclidShapeRandomTools.nextPointShape3D(random);

         FramePoint3D expectedPosition = new FramePoint3D(frameA, original);
         expectedPosition.changeFrame(frameB);

         PointShape3D transformed = EuclidFrameShapeTools.changeFrame(original, frameA, frameB);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPosition, transformed, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test for Ramp3D
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);
         Ramp3D original = EuclidShapeRandomTools.nextRamp3D(random);

         FramePose3D expectedPose = new FramePose3D(frameA, original.getPose());
         expectedPose.changeFrame(frameB);
         Vector3D expectedSize = new Vector3D(original.getSize());

         Ramp3D transformed = EuclidFrameShapeTools.changeFrame(original, frameA, frameB);
         EuclidGeometryTestTools.assertPose3DGeometricallyEquals(expectedPose, new Pose3D(transformed.getPose()), EPSILON);
         EuclidCoreTestTools.assertTuple3DEquals(expectedSize, transformed.getSize(), EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Test for Sphere3D
         ReferenceFrame frameA = EuclidFrameRandomTools.nextReferenceFrame("frameA", random, worldFrame);
         ReferenceFrame frameB = EuclidFrameRandomTools.nextReferenceFrame("frameB", random, worldFrame);
         Sphere3D original = EuclidShapeRandomTools.nextSphere3D(random);

         FramePoint3D expectedPosition = new FramePoint3D(frameA, original.getPosition());
         expectedPosition.changeFrame(frameB);

         Sphere3D transformed = EuclidFrameShapeTools.changeFrame(original, frameA, frameB);
         EuclidCoreTestTools.assertTuple3DEquals(expectedPosition, transformed.getPosition(), EPSILON);
      }
   }
}
