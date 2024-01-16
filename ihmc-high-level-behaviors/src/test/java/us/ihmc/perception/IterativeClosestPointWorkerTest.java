package us.ihmc.perception;

import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
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
   @Test
   public void testWithTooSmallBox()
   {
      Random random = new Random(1738L);
      int objectSamples = 5000;
      int correspondences = 1000;
      IterativeClosestPointWorker icp = new IterativeClosestPointWorker(objectSamples, correspondences, random);

      Vector3D actualBoxDimensions = new Vector3D(0.4, 0.3, 0.5);

      icp.setDetectionShape(PrimitiveRigidBodyShape.BOX);
      icp.changeSize(actualBoxDimensions, new Vector3D(0.1, 0.1, 0.1), objectSamples);

      float scaleFactor = 0.5f;
      for (int i = 0; i < 5; i++)
      {
         Vector3D translationError = EuclidCoreRandomTools.nextVector3D(random, 0.05);
         Quaternion rotationError = EuclidCoreRandomTools.nextQuaternion(random, Math.toRadians(10.0));

         Pose3D actualBoxPose = EuclidGeometryRandomTools.nextPose3D(random, 0.5, Math.toRadians(90.0));
         RigidBodyTransform errorTransform = new RigidBodyTransform(rotationError, translationError);

         Pose3D poseGuess = new Pose3D(actualBoxPose);
         errorTransform.transform(poseGuess);

         icp.setPoseGuess(poseGuess);

         Vector3D scaledBoxDimensions = new Vector3D(actualBoxDimensions);
         scaledBoxDimensions.scale(scaleFactor);
         List<Point3D32> pointCloud = IterativeClosestPointTools.createBoxPointCloud(actualBoxPose,
                                                                                     scaledBoxDimensions.getX32(),
                                                                                     scaledBoxDimensions.getY32(),
                                                                                     scaledBoxDimensions.getZ32(),
                                                                                     objectSamples,
                                                                                     random);

         icp.setEnvironmentPointCloud(pointCloud);
         icp.runICP(1);

         // we know the measurements were randomly made about the centroid, so the average of all of these points should be the centroid.
         EuclidCoreTestTools.assertGeometricallyEquals(actualBoxPose.getPosition(), icp.getMeasurementCentroid(), 0.01);
         EuclidCoreTestTools.assertGeometricallyEquals(poseGuess.getPosition(), icp.getObjectCentroid(), 0.01);
         for (Point3DReadOnly pointOnObject : icp.getCorrespondingMeasurementPoints())
         {
            // all of the measurement correspondence points projected onto the box should have zero distance to the box.
            assertEquals(0.0f,
                         IterativeClosestPointTools.distanceSquaredFromBox(actualBoxPose,
                                                                           pointOnObject,
                                                                           scaledBoxDimensions.getX32(),
                                                                           scaledBoxDimensions.getY32(),
                                                                           scaledBoxDimensions.getZ32()),
                         5e-4);
         }
         for (Point3DReadOnly pointOnObject : icp.getCorrespondingObjectPoints())
         {
            // all of the object correspondence points projected onto the box should have zero distance to the box.
            assertEquals(0.0f,
                         IterativeClosestPointTools.distanceSquaredFromBox(poseGuess,
                                                                           pointOnObject,
                                                                           actualBoxDimensions.getX32(),
                                                                           actualBoxDimensions.getY32(),
                                                                           actualBoxDimensions.getZ32()),
                         5e-4);
         }

//         icp.runICP(5);

         EuclidCoreTestTools.assertEquals(actualBoxPose, icp.getResultPose(), 1e-4);
      }
   }
}
