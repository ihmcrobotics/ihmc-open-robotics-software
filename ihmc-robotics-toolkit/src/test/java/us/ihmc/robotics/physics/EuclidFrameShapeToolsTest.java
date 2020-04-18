package us.ihmc.robotics.physics;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.shape.primitives.PointShape3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.tools.EuclidShapeRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class EuclidFrameShapeToolsTest
{
   private static final int ITERATIONS = 5000;
   private static final double EPSILON = 1.0e-12;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   @Test
   public void testBoundingBox3D()
   {
      Random random = new Random(5768787);

      for (int i = 0; i < ITERATIONS; i++)
      { // Box3D
         RigidBodyTransform frameTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         if (random.nextDouble() < 0.3)
            frameTransform.getRotation().setToZero();
         if (random.nextDouble() < 0.3)
            frameTransform.getTranslation().setToZero();
         ReferenceFrame referenceFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, frameTransform);
         Box3D boxInFrame = EuclidShapeRandomTools.nextBox3D(random);
         if (random.nextDouble() < 0.3)
            boxInFrame.getOrientation().setToZero();
         if (random.nextDouble() < 0.3)
            boxInFrame.getPosition().setToZero();
         Box3D boxInWorld = new Box3D(boxInFrame);
         referenceFrame.transformFromThisToDesiredFrame(worldFrame, boxInWorld);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         EuclidFrameShapeTools.boundingBox3D(worldFrame, boxInWorld, expected);
         EuclidFrameShapeTools.boundingBox3D(referenceFrame, boxInFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);

         boxInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Capsule3D
         RigidBodyTransform frameTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         if (random.nextDouble() < 0.3)
            frameTransform.getRotation().setToZero();
         if (random.nextDouble() < 0.3)
            frameTransform.getTranslation().setToZero();
         ReferenceFrame referenceFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, frameTransform);
         Capsule3D capsuleInFrame = EuclidShapeRandomTools.nextCapsule3D(random);
         Capsule3D capsuleInWorld = new Capsule3D(capsuleInFrame);
         referenceFrame.transformFromThisToDesiredFrame(worldFrame, capsuleInWorld);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         EuclidFrameShapeTools.boundingBox3D(worldFrame, capsuleInWorld, expected);
         EuclidFrameShapeTools.boundingBox3D(referenceFrame, capsuleInFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);

         capsuleInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Cylinder3D
         RigidBodyTransform frameTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         if (random.nextDouble() < 0.3)
            frameTransform.getRotation().setToZero();
         if (random.nextDouble() < 0.3)
            frameTransform.getTranslation().setToZero();
         ReferenceFrame referenceFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, frameTransform);
         Cylinder3D cylinderInFrame = EuclidShapeRandomTools.nextCylinder3D(random);
         Cylinder3D cylinderInWorld = new Cylinder3D(cylinderInFrame);
         referenceFrame.transformFromThisToDesiredFrame(worldFrame, cylinderInWorld);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         EuclidFrameShapeTools.boundingBox3D(worldFrame, cylinderInWorld, expected);
         EuclidFrameShapeTools.boundingBox3D(referenceFrame, cylinderInFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);

         cylinderInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Ellipsoid3D
         RigidBodyTransform frameTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         if (random.nextDouble() < 0.3)
            frameTransform.getRotation().setToZero();
         if (random.nextDouble() < 0.3)
            frameTransform.getTranslation().setToZero();
         ReferenceFrame referenceFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, frameTransform);
         Ellipsoid3D ellipsoidInFrame = EuclidShapeRandomTools.nextEllipsoid3D(random);
         if (random.nextDouble() < 0.3)
            ellipsoidInFrame.getOrientation().setToZero();
         if (random.nextDouble() < 0.3)
            ellipsoidInFrame.getPosition().setToZero();
         Ellipsoid3D ellipsoidInWorld = new Ellipsoid3D(ellipsoidInFrame);
         referenceFrame.transformFromThisToDesiredFrame(worldFrame, ellipsoidInWorld);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         EuclidFrameShapeTools.boundingBox3D(worldFrame, ellipsoidInWorld, expected);
         EuclidFrameShapeTools.boundingBox3D(referenceFrame, ellipsoidInFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);

         ellipsoidInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Ramp3D
         RigidBodyTransform frameTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         if (random.nextDouble() < 0.3)
            frameTransform.getRotation().setToZero();
         if (random.nextDouble() < 0.3)
            frameTransform.getTranslation().setToZero();
         ReferenceFrame referenceFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, frameTransform);
         Ramp3D rampInFrame = EuclidShapeRandomTools.nextRamp3D(random);
         if (random.nextDouble() < 0.3)
            rampInFrame.getOrientation().setToZero();
         if (random.nextDouble() < 0.3)
            rampInFrame.getPosition().setToZero();
         Ramp3D rampInWorld = new Ramp3D(rampInFrame);
         referenceFrame.transformFromThisToDesiredFrame(worldFrame, rampInWorld);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         EuclidFrameShapeTools.boundingBox3D(worldFrame, rampInWorld, expected);
         EuclidFrameShapeTools.boundingBox3D(referenceFrame, rampInFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);

         rampInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // Sphere3D
         RigidBodyTransform frameTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         if (random.nextDouble() < 0.3)
            frameTransform.getRotation().setToZero();
         if (random.nextDouble() < 0.3)
            frameTransform.getTranslation().setToZero();
         ReferenceFrame referenceFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, frameTransform);
         Sphere3D sphereInFrame = EuclidShapeRandomTools.nextSphere3D(random);
         Sphere3D sphereInWorld = new Sphere3D(sphereInFrame);
         referenceFrame.transformFromThisToDesiredFrame(worldFrame, sphereInWorld);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         EuclidFrameShapeTools.boundingBox3D(worldFrame, sphereInWorld, expected);
         EuclidFrameShapeTools.boundingBox3D(referenceFrame, sphereInFrame, actual);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);

         sphereInWorld.getBoundingBox(expected);
         EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);
      }

      for (int i = 0; i < ITERATIONS; i++)
      { // ConvexPolytope3D
         RigidBodyTransform frameTransform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         if (random.nextDouble() < 0.3)
            frameTransform.getRotation().setToZero();
         if (random.nextDouble() < 0.3)
            frameTransform.getTranslation().setToZero();
         ReferenceFrame referenceFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("shapeFrame", worldFrame, frameTransform);
         ConvexPolytope3D polytopeInFrame = EuclidShapeRandomTools.nextConvexPolytope3DWithEdgeCases(random);
         ConvexPolytope3D polytopeInWorld = new ConvexPolytope3D(polytopeInFrame);
         referenceFrame.transformFromThisToDesiredFrame(worldFrame, polytopeInWorld);
         BoundingBox3D expected = new BoundingBox3D();
         BoundingBox3D actual = new BoundingBox3D();
         EuclidFrameShapeTools.boundingBox3D(worldFrame, polytopeInWorld, expected);
         EuclidFrameShapeTools.boundingBox3D(referenceFrame, polytopeInFrame, actual);
         if (expected.containsNaN())
            assertTrue(actual.containsNaN());
         else
            EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);

         polytopeInWorld.getBoundingBox(expected);
         if (expected.containsNaN())
            assertTrue(actual.containsNaN());
         else
            EuclidGeometryTestTools.assertBoundingBox3DEquals(expected, actual, EPSILON);
      }
   }

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
