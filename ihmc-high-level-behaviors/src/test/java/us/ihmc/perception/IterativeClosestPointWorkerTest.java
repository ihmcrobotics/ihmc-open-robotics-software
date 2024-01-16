package us.ihmc.perception;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;

import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public class IterativeClosestPointWorkerTest
{
   private static final int objectSamples = 5000;

   @Test
   public void testFitWithBoxes()
   {
      Random random = new Random(1738L);
      int correspondences = 1000;
      IterativeClosestPointWorker icp = new IterativeClosestPointWorker(objectSamples, correspondences, random);

      Vector3D actualBoxDimensions = new Vector3D(0.4, 0.3, 0.5);

      icp.setDetectionShape(PrimitiveRigidBodyShape.BOX);
      icp.changeSize(actualBoxDimensions, new Vector3D(0.1, 0.1, 0.1), objectSamples);

      // test perfectly sized box with just translation error
      Vector3D translationError = new Vector3D(0.1, 0.0, 0.04);
//      testWithDifferentBoxes(icp, 1.0f, actualBoxDepth, actualBoxWidth, actualBoxHeight, new Pose3D(), translationError, new Quaternion(), random);

      // test slightly undersized box with just translation error
//      testWithDifferentBoxes(icp, 0.9f, actualBoxDepth, actualBoxWidth, actualBoxHeight, new Pose3D(), translationError, new Quaternion(), random);

      // test medium undersized box with just translation error
      testWithDifferentBoxes(icp, 0.75f, actualBoxDimensions.getX32(), actualBoxDimensions.getY32(), actualBoxDimensions.getZ32(), new Pose3D(), translationError, new Quaternion(), random);

      // test very undersized box with just translation error
      testWithDifferentBoxes(icp, 0.5f, actualBoxDimensions.getX32(), actualBoxDimensions.getY32(), actualBoxDimensions.getZ32(), new Pose3D(), translationError, new Quaternion(), random);
   }

   @Test
   public void testFitWithBoxesOfDifferentSizes()
   {
      Random random = new Random(1738L);
      int correspondences = 1000;
      IterativeClosestPointWorker icp = new IterativeClosestPointWorker(objectSamples, correspondences, random);

      float actualBoxWidth = 0.3f;
      float actualBoxDepth = 0.4f;
      float actualBoxHeight = 0.5f;

      icp.setDetectionShape(PrimitiveRigidBodyShape.BOX);
      icp.changeSize(new Vector3D(actualBoxDepth, actualBoxWidth, actualBoxHeight), new Vector3D(0.1f, 0.1f, 0.1f), objectSamples);

      // test perfect sized box
      testWithDifferentBoxesAndRandomError(icp, 1.0f, actualBoxDepth, actualBoxWidth, actualBoxHeight, random);

      // test slightly undersized point cloud
      testWithDifferentBoxesAndRandomError(icp, 0.9f, actualBoxDepth, actualBoxWidth, actualBoxHeight, random);

      // test with too small point cloud
      testWithDifferentBoxesAndRandomError(icp, 0.5f, actualBoxDepth, actualBoxWidth, actualBoxHeight, random);
   }

   private void testWithDifferentBoxesAndRandomError(IterativeClosestPointWorker icp,
                                                     float scaleFactor,
                                                     float actualBoxDepth,
                                                     float actualBoxWidth,
                                                     float actualBoxHeight,
                                                     Random random)
   {
      for (int i = 0; i < 5; i++)
      {
         Vector3D translationError = EuclidCoreRandomTools.nextVector3D(random, 0.05);
         Quaternion rotationError = EuclidCoreRandomTools.nextQuaternion(random, Math.toRadians(10.0));

         Pose3D actualBoxPose = EuclidGeometryRandomTools.nextPose3D(random, 0.5, Math.toRadians(90.0));

         testWithDifferentBoxes(icp, scaleFactor, actualBoxDepth, actualBoxWidth, actualBoxHeight, actualBoxPose, translationError, rotationError, random);
      }
   }

   private void testWithDifferentBoxes(IterativeClosestPointWorker icp,
                                       float scaleFactor,
                                       float actualBoxDepth,
                                       float actualBoxWidth,
                                       float actualBoxHeight,
                                       Pose3DReadOnly actualBoxPose,
                                       Vector3D translationError,
                                       Quaternion rotationError,
                                       Random random)
   {
      RigidBodyTransform errorTransform = new RigidBodyTransform(rotationError, translationError);

      Pose3D poseGuess = new Pose3D(actualBoxPose);
      errorTransform.transform(poseGuess);

      icp.setPoseGuess(poseGuess);

      Box3D box = new Box3D(poseGuess, actualBoxDepth, actualBoxWidth, actualBoxHeight);
      Box3D scaledBox = new Box3D(actualBoxPose, scaleFactor * actualBoxDepth, scaleFactor * actualBoxWidth, scaleFactor * actualBoxHeight);

      List<Point3D32> pointCloud = IterativeClosestPointTools.createBoxPointCloud(actualBoxPose,
                                                                                  scaleFactor * actualBoxDepth,
                                                                                  scaleFactor * actualBoxWidth,
                                                                                  scaleFactor * actualBoxHeight,
                                                                                  objectSamples,
                                                                                  random);

      icp.setEnvironmentPointCloud(pointCloud);
      icp.runICP(1);

      // we know the measurements were randomly made about the centroid. The corresponding measurement points come from these,  so the average of all of
      // these points should be the centroid. These points will be incorrectly sized compared to the actual box size, but they should still be centered
      // correctly.
      //         EuclidCoreTestTools.assertGeometricallyEquals(actualBoxPose.getPosition(), icp.getMeasurementCentroid(), 0.01);
      // The corresponding object points should all be projected onto the box oriented at the pose guess. This means that the centroid of all those points
      // should be located at the pose guess.
      //         EuclidCoreTestTools.assertGeometricallyEquals(poseGuess.getPosition(), icp.getObjectCentroid(), 0.01);

      for (Point3DReadOnly measurementPoint : icp.getCorrespondingMeasurementPoints())
      {
         // all of the measurement correspondence points projected onto the box should have zero distance to the box.
         assertEquals(0.0f,
                      IterativeClosestPointTools.distanceSquaredFromBox(actualBoxPose,
                                                                        measurementPoint,
                                                                        scaleFactor * actualBoxDepth,
                                                                        scaleFactor * actualBoxWidth,
                                                                        scaleFactor * actualBoxHeight),
                      5e-4);
         assertEquals(0.0f, scaledBox.signedDistance(measurementPoint), 5e-4);
      }
      for (Point3DReadOnly pointOnObject : icp.getCorrespondingObjectPoints())
      {
         // all of the object correspondence points projected onto the box should have zero distance to the box.
         assertEquals(0.0f, IterativeClosestPointTools.distanceSquaredFromBox(poseGuess, pointOnObject, actualBoxDepth, actualBoxWidth, actualBoxHeight), 5e-4);
         assertEquals(0.0f, box.signedDistance(pointOnObject), 5e-4);
      }

      // run some additional iterations to make sure things are right.
      icp.runICP(5);

      EuclidCoreTestTools.assertGeometricallyEquals(actualBoxPose, icp.getResultPose(), 5e-2);
   }
}
