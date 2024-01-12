package us.ihmc.perception;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.sceneGraph.rigidBody.primitive.PrimitiveRigidBodyShape;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class IterativeClosestPointTools
{
   public static double distanceSquaredFromShape(PrimitiveRigidBodyShape shape,
                                                 Pose3DReadOnly shapePose,
                                                 Point3DReadOnly pointQuery,
                                                 float xLength,
                                                 float yLength,
                                                 float zLength,
                                                 float xRadius,
                                                 float yRadius,
                                                 float zRadius,
                                                 boolean ignoreShapeType)
   {
      if (ignoreShapeType)
      {
         return pointQuery.distanceSquared(shapePose.getPosition());
      }

      double distanceSquared;
      float maxRadius = Math.max(xRadius, Math.max(yRadius, zRadius));
      double maxLength = Math.sqrt(2 * MathTools.square(Math.max(xLength, Math.max(yLength, zLength))));
      float biggestDistance = Math.max((float) maxLength, maxRadius);
      if (shape == null)
      {
         distanceSquared = distanceSquaredFromSphere(shapePose, pointQuery, biggestDistance);
      }
      else
      {
         switch (shape)
         {
            case BOX -> distanceSquared = distanceSquaredFromBox(shapePose, pointQuery, xLength, yLength, zLength);
            case CYLINDER -> distanceSquared = distanceSquaredFromCylinder(shapePose, pointQuery, zLength, xRadius);
            case ELLIPSOID -> distanceSquared = distanceSquaredFromEllipsoid(shapePose, pointQuery, xRadius, yRadius, zRadius);
            // case PRISM // TODO
            // case CONE // TODO
            default -> distanceSquared = distanceSquaredFromSphere(shapePose, pointQuery, biggestDistance);
         }
      }

      return distanceSquared;
   }

   public static double distanceSquaredFromBox(Pose3DReadOnly boxPose, Point3DReadOnly query, float xLength, float yLength, float zLength)
   {
      // first, transform the query to box frame.
      Point3D pointRelativeToBox = new Point3D(query);
      pointRelativeToBox.applyInverseTransform(boxPose);

      Vector3D boxSize = new Vector3D(xLength, yLength, zLength);
      return MathTools.square(Math.max(0.0, EuclidShapeTools.signedDistanceBetweenPoint3DAndBox3D(pointRelativeToBox, boxSize)));
   }

   public static double distanceSquaredFromSphere(Pose3DReadOnly spherePose, Point3DReadOnly query, float radius)
   {
      return MathTools.square(Math.max(0.0, EuclidShapeTools.signedDistanceBetweenPoint3DAndSphere3D(query, spherePose.getPosition(), radius)));
   }

   public static double distanceSquaredFromCylinder(Pose3DReadOnly cylinderPose, Point3DReadOnly query, float zLength, float radius)
   {
      // first, transform the query to box frame.
      Point3D pointRelativeToCylinder = new Point3D(query);
      pointRelativeToCylinder.applyInverseTransform(cylinderPose);

      Vector3D axis = new Vector3D(0.0, 0.0, 1.0);
      return MathTools.square(Math.max(EuclidShapeTools.signedDistanceBetweenPoint3DAndCylinder3D(pointRelativeToCylinder,
                                                                                                  new Point3D(),
                                                                                                  axis,
                                                                                                  zLength,
                                                                                                  radius), 0.0f));
   }

   public static double distanceSquaredFromEllipsoid(Pose3DReadOnly ellipsePose, Point3DReadOnly query, float xRadius, float yRadius, float zRadius)
   {
      Point3D pointRelativeToCylinder = new Point3D(query);
      pointRelativeToCylinder.applyInverseTransform(ellipsePose);

      return MathTools.square(Math.max(EuclidShapeTools.signedDistanceBetweenPoint3DAndEllipsoid3D(pointRelativeToCylinder,
                                                                                                   new Vector3D(xRadius, yRadius, zRadius)), 0.0));
   }

   public static boolean canComputeCorrespondencesOnShape(PrimitiveRigidBodyShape shape)
   {
      return switch (shape)
      {
         case BOX, CYLINDER, ELLIPSOID -> true;
         default -> false;
      };
   }

   public static void computeCorrespondencesOnShape(PrimitiveRigidBodyShape shape,
                                                    Pose3DReadOnly shapePose,
                                                    List<Point3D32> measurementPoints,
                                                    List<Point3D32> correspondingMeasurementPointsToPack,
                                                    List<Point3D32> correspondingObjectPointsToPack,
                                                    float xLength,
                                                    float yLength,
                                                    float zLength,
                                                    float xRadius,
                                                    float yRadius,
                                                    float zRadius,
                                                    int numberOfCorrespondences)
   {
      if (!canComputeCorrespondencesOnShape(shape))
         throw new RuntimeException("You can't compute shape correspondences for this shape type!");

      for (int i = 0; i < Math.min(measurementPoints.size(), numberOfCorrespondences); i++)
      {
         Point3D32 measurementPoint = measurementPoints.get(i);
         Point3D32 correspondingObjectPoint = computeCorrespondenceOnShape(shape,
                                                                           shapePose,
                                                                           measurementPoint,
                                                                           xLength,
                                                                           yLength,
                                                                           zLength,
                                                                           xRadius,
                                                                           yRadius,
                                                                           zRadius);
         // record
         correspondingMeasurementPointsToPack.add(measurementPoint);
         correspondingObjectPointsToPack.add(correspondingObjectPoint);
      }
   }

   private static Point3D32 computeCorrespondenceOnShape(PrimitiveRigidBodyShape shape,
                                                         Pose3DReadOnly shapePose,
                                                         Point3D32 point,
                                                         float xLength,
                                                         float yLength,
                                                         float zLength,
                                                         float xRadius,
                                                         float yRadius,
                                                         float zRadius)
   {
      Point3D32 correspondingPointOnShape;
      switch (shape)
      {
         case BOX -> correspondingPointOnShape = computeCorrespondingPointOnBox(shapePose, point, xLength, yLength, zLength);
         case CYLINDER -> correspondingPointOnShape = computeCorrespondingPointOnCylinder(shapePose, point, zLength, xRadius);
         case ELLIPSOID -> correspondingPointOnShape = computeCorrespondingPointOnEllipse(shapePose, point, xRadius, yRadius, zRadius);
         // case PRISM // TODO
         // case CONE // TODO
         default -> throw new RuntimeException("Invalid shape for computing a correspondence.");
      }

      return correspondingPointOnShape;
   }

   static Point3D32 computeCorrespondingPointOnBox(Pose3DReadOnly boxPose, Point3D32 query, float xLength, float yLength, float zLength)
   {
      Point3D pointRelativeToBox = new Point3D(query);
      pointRelativeToBox.applyInverseTransform(boxPose);

      Point3D32 correspondingPoint = new Point3D32();

      // Note that we can't use the EuclidShapeTools definition here. That approach doesn't project points onto the edge if they're inside.
      double halfSizeX = 0.5 * xLength;
      double halfSizeY = 0.5 * yLength;
      double halfSizeZ = 0.5 * zLength;

      double xLocal = pointRelativeToBox.getX();
      double yLocal = pointRelativeToBox.getY();
      double zLocal = pointRelativeToBox.getZ();

      double xLocalClamped = EuclidCoreTools.clamp(xLocal, halfSizeX);
      double yLocalClamped = EuclidCoreTools.clamp(yLocal, halfSizeY);
      double zLocalClamped = EuclidCoreTools.clamp(zLocal, halfSizeZ);

      correspondingPoint.set(xLocalClamped, yLocalClamped, zLocalClamped);
      correspondingPoint.applyTransform(boxPose);

      return correspondingPoint;
   }

   private static Point3D32 computeCorrespondingPointOnCylinder(Pose3DReadOnly cylinderPose, Point3D32 query, float zLength, float radius)
   {
      Point3D pointRelativeToCylinder = new Point3D(query);
      pointRelativeToCylinder.applyInverseTransform(cylinderPose);

      Vector3D axis = new Vector3D(0.0, 0.0, 1.0);
      Point3D32 correspondingPoint = new Point3D32();
      // FIXME this needs to be replaced, because we need the projection whether or not its inside.
      EuclidShapeTools.orthogonalProjectionOntoCylinder3D(pointRelativeToCylinder, new Point3D(), axis, zLength, radius, correspondingPoint);

      correspondingPoint.applyTransform(cylinderPose);

      return correspondingPoint;
   }

   private static Point3D32 computeCorrespondingPointOnEllipse(Pose3DReadOnly ellipsePose, Point3D32 query, float xRadius, float yRadius, float zRadius)
   {
      Point3D pointRelativeToCylinder = new Point3D(query);
      pointRelativeToCylinder.applyInverseTransform(ellipsePose);

      Point3D32 correspondingPoint = new Point3D32();
      // FIXME this needs to be replaced, because we need the projection whether or not its inside.
      EuclidShapeTools.orthogonalProjectionOntoEllipsoid3D(pointRelativeToCylinder, new Vector3D(xRadius, yRadius, zRadius), correspondingPoint);

      correspondingPoint.applyTransform(ellipsePose);

      return correspondingPoint;
   }

   public static List<Point3D32> createICPObjectPointCloud(PrimitiveRigidBodyShape shape,
                                                           Pose3DReadOnly shapePose,
                                                           float xLength,
                                                           float yLength,
                                                           float zLength,
                                                           float xRadius,
                                                           float yRadius,
                                                           float zRadius,
                                                           int numberOfICPObjectPoints,
                                                           Random random)
   {
      List<Point3D32> objectPointCloud;

      switch (shape)
      {
         case BOX -> objectPointCloud = IterativeClosestPointTools.createBoxPointCloud(shapePose, xLength, yLength, zLength, numberOfICPObjectPoints, random);
         case PRISM ->
               objectPointCloud = IterativeClosestPointTools.createPrismPointCloud(shapePose, xLength, yLength, zLength, numberOfICPObjectPoints, random);
         case CYLINDER -> objectPointCloud = IterativeClosestPointTools.createCylinderPointCloud(shapePose, zLength, xRadius, numberOfICPObjectPoints, random);
         case ELLIPSOID ->
               objectPointCloud = IterativeClosestPointTools.createEllipsoidPointCloud(shapePose, xRadius, yRadius, zRadius, numberOfICPObjectPoints, random);
         case CONE -> objectPointCloud = IterativeClosestPointTools.createConePointCloud(shapePose, zLength, xRadius, numberOfICPObjectPoints, random);
         default -> objectPointCloud = IterativeClosestPointTools.createDefaultBoxPointCloud(shapePose, numberOfICPObjectPoints, random);
      }

      return objectPointCloud;
   }

   public static List<Point3D32> createBoxPointCloud(Pose3DReadOnly boxPose, float xLength, float yLength, float zLength, int numberOfPoints, Random random)
   {
      List<Point3D32> boxObjectPointCloud = new ArrayList<>();
      Pose3D boxPointPose = new Pose3D();

      float halfBoxDepth = xLength / 2.0f;
      float halfBoxWidth = yLength / 2.0f;
      float halfBoxHeight = zLength / 2.0f;
      for (int i = 0; i < numberOfPoints; i++)
      {
         int j = random.nextInt(6);
         float x = random.nextFloat(-halfBoxDepth, halfBoxDepth);
         float y = random.nextFloat(-halfBoxWidth, halfBoxWidth);
         float z = random.nextFloat(-halfBoxHeight, halfBoxHeight);
         if (j == 0 | j == 1)
         {
            x = (-(j & 1) * halfBoxDepth * 2.0f) + halfBoxDepth;
         }
         if (j == 2 | j == 3)
         {
            y = (-(j & 1) * halfBoxWidth * 2.0f) + halfBoxWidth;
         }
         if (j == 4 | j == 5)
         {
            z = (-(j & 1) * halfBoxHeight * 2.0f) + halfBoxHeight;
         }

         boxPointPose.set(boxPose);
         boxPointPose.appendTranslation(x, y, z);

         boxObjectPointCloud.add(new Point3D32(boxPointPose.getPosition()));
      }

      return boxObjectPointCloud;
   }

   public static List<Point3D32> createPrismPointCloud(Pose3DReadOnly prismPose, float xLength, float yLength, float zLength, int numberOfPoints, Random random)
   {
      List<Point3D32> prismObjectPointCloud = new ArrayList<>();
      Pose3D prismPointPose = new Pose3D();

      float halfPrismDepth = xLength / 2.0f;
      float halfPrismWidth = yLength / 2.0f;

      for (int i = 0; i < numberOfPoints; i++)
      {
         int side = random.nextInt(0, 4);
         float x = random.nextFloat(-halfPrismDepth, halfPrismDepth);
         float y = random.nextFloat(-halfPrismWidth, halfPrismWidth);
         float z = random.nextFloat(0, zLength);
         if (side == 0 || side == 1) // triangular faces
         {
            x = (1.0f - (z / zLength)) * x;
            y = (-(side & 1) * halfPrismWidth * 2.0f) + halfPrismWidth;
         }
         else if (side == 2 || side == 3) // rectangular faces
         {
            x = (1.0f - (z / zLength)) * ((-(side & 1) * halfPrismDepth * 2.0f) + halfPrismDepth);
         }

         prismPointPose.set(prismPose);
         prismPointPose.appendTranslation(x, y, z);

         prismObjectPointCloud.add(new Point3D32(prismPointPose.getPosition()));
      }

      return prismObjectPointCloud;
   }

   public static List<Point3D32> createCylinderPointCloud(Pose3DReadOnly cylinderPose, float zLength, float xRadius, int numberOfPoints, Random random)
   {
      List<Point3D32> cylinderObjectPointCloud = new ArrayList<>();
      Pose3D cylinderPointPose = new Pose3D();

      float halfZLength = 0.5f * zLength;
      for (int i = 0; i < numberOfPoints; i++)
      {
         int j = random.nextInt(6);
         float z = random.nextFloat(-halfZLength, halfZLength);
         float r = xRadius;
         if (j == 0)
         {
            z = -halfZLength;
            r = random.nextFloat(0, xRadius);
         }
         if (j == 1)
         {
            z = halfZLength;
            r = random.nextFloat(0, xRadius);
         }
         double phi = random.nextDouble(0, 2 * Math.PI);
         float x = (float) Math.cos(phi) * r;
         float y = (float) Math.sin(phi) * r;

         cylinderPointPose.set(cylinderPose);
         cylinderPointPose.appendTranslation(x, y, z);

         cylinderObjectPointCloud.add(new Point3D32(cylinderPointPose.getPosition()));
      }

      return cylinderObjectPointCloud;
   }

   public static List<Point3D32> createEllipsoidPointCloud(Pose3DReadOnly ellipsePose,
                                                           float xRadius,
                                                           float yRadius,
                                                           float zRadius,
                                                           int numberOfPoints,
                                                           Random random)
   {
      List<Point3D32> ellipsoidObjectPointCloud = new ArrayList<>();
      Pose3D ellipsoidPointPose = new Pose3D();

      for (int i = 0; i < numberOfPoints; i++)
      {
         double phi = random.nextDouble(0, 2.0 * Math.PI);
         double theta = random.nextDouble(0, 2.0 * Math.PI);
         float x = (float) (Math.sin(phi) * Math.cos(theta) * xRadius);
         float y = (float) (Math.sin(phi) * Math.sin(theta) * yRadius);
         float z = (float) Math.cos(phi) * zRadius;

         ellipsoidPointPose.set(ellipsePose);
         ellipsoidPointPose.appendTranslation(x, y, z);

         ellipsoidObjectPointCloud.add(new Point3D32(ellipsoidPointPose.getPosition()));
      }

      return ellipsoidObjectPointCloud;
   }

   public static List<Point3D32> createConePointCloud(Pose3DReadOnly conePose, float zLength, float xRadius, int numberOfPoints, Random random)
   {
      List<Point3D32> coneObjectPointCloud = new ArrayList<>();
      Pose3D conePointPose = new Pose3D();

      for (int i = 0; i < numberOfPoints; i++)
      {
         float z = random.nextFloat(0, zLength);
         double phi = random.nextDouble(0, 2.0 * Math.PI);
         float x = (float) Math.cos(phi) * (zLength - z) * (xRadius / zLength);
         float y = (float) Math.sin(phi) * (zLength - z) * (xRadius / zLength);

         conePointPose.set(conePose);
         conePointPose.appendTranslation(x, y, z);

         coneObjectPointCloud.add(new Point3D32(conePointPose.getPosition()));
      }

      return coneObjectPointCloud;
   }

   public static List<Point3D32> createDefaultBoxPointCloud(Pose3DReadOnly boxPose, int numberOfPoints, Random random)
   {
      List<Point3D32> boxObjectPointCloud = new ArrayList<>();
      Pose3D boxPointPose = new Pose3D();

      float halfBoxWidth = 0.405f / 2.0f;
      float halfBoxDepth = 0.31f / 2.0f;
      float halfBoxHeight = 0.19f / 2.0f;
      for (int i = 0; i < numberOfPoints; i++)
      {
         int j = random.nextInt(6);
         float x = random.nextFloat(-halfBoxDepth, halfBoxDepth);
         float y = random.nextFloat(-halfBoxWidth, halfBoxWidth);
         float z = random.nextFloat(-halfBoxHeight, halfBoxHeight);
         if (j == 0 | j == 1)
         {
            x = (-(j & 1) * halfBoxDepth * 2.0f) + halfBoxDepth;
         }
         if (j == 2 | j == 3)
         {
            y = (-(j & 1) * halfBoxWidth * 2.0f) + halfBoxWidth;
         }
         if (j == 4 | j == 5)
         {
            z = (-(j & 1) * halfBoxHeight * 2.0f) + halfBoxHeight;
         }

         boxPointPose.set(boxPose);
         boxPointPose.appendTranslation(x, y, z);

         boxObjectPointCloud.add(new Point3D32(boxPointPose.getPosition()));
      }

      return boxObjectPointCloud;
   }
}
