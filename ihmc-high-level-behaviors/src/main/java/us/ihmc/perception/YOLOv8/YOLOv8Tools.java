package us.ihmc.perception.YOLOv8;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class YOLOv8Tools
{
   public static List<Point3DReadOnly> filterOutliers(List<? extends Point3DReadOnly> pointCloud, double zScoreThreshold, int numberOfSamples)
   {
      Point3D centroid = new Point3D();
      double standardDeviation = calculateStandardDeviationAndCentroid(pointCloud, numberOfSamples, true, centroid);

      return pointCloud.parallelStream().filter(point ->
      {
         Vector3D zVector = new Vector3D(point);
         zVector.sub(centroid);
         zVector.scale(1.0 / standardDeviation);
         return zVector.norm() < zScoreThreshold;
      }).collect(Collectors.toList());
   }

   public static double calculateStandardDeviationAndCentroid(List<? extends Point3DReadOnly> pointCloud, Point3DBasics centroidToPack)
   {
      return calculateStandardDeviationAndCentroid(pointCloud, pointCloud.size(), false, centroidToPack);
   }

   /**
    * Given a point cloud, computes the centroid and standard deviation of the points
    * @param pointCloud          The list of points used for calculations
    * @param maxNumberOfSamples  Maximum number of points to use for the computation. First N points in the list will be used.
    * @param shuffle             Whether to shuffle the point cloud before computations. Can b used to find approximate values with N points.
    * @param centroidToPack      Point object into which the centroid will be packed
    * @return The standard deviation of the points
    */
   public static double calculateStandardDeviationAndCentroid(List<? extends Point3DReadOnly> pointCloud,
                                                              int maxNumberOfSamples,
                                                              boolean shuffle,
                                                              Point3DBasics centroidToPack)
   {
      if (shuffle)
         Collections.shuffle(pointCloud);

      Vector3D sumVector = new Vector3D(0.0, 0.0, 0.0);
      Vector3D squaredSumVector = new Vector3D(0.0, 0.0, 0.0);
      int numberOfSamples = Math.min(pointCloud.size(), maxNumberOfSamples);

      pointCloud.parallelStream().limit(numberOfSamples).forEach(point ->
                                                                      {
                                                                         sumVector.add(point);
                                                                         squaredSumVector.add((point.getX() * point.getX()), (point.getY() * point.getY()), (point.getZ() * point.getZ()));
                                                                      });

      centroidToPack.set(sumVector);
      centroidToPack.scale(1.0 / numberOfSamples);

      Vector3D meanSquaredVector = new Vector3D(centroidToPack);
      meanSquaredVector.scale(meanSquaredVector.getX(), meanSquaredVector.getY(), meanSquaredVector.getZ());

      Vector3D varianceVector = new Vector3D(squaredSumVector);
      varianceVector.scale(1.0 / numberOfSamples);
      varianceVector.sub(meanSquaredVector);

      return Math.sqrt(varianceVector.getX() + varianceVector.getY() + varianceVector.getZ());
   }
}
