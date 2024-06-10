package us.ihmc.perception.YOLOv8;

import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.tools.io.WorkspaceResourceDirectory;
import us.ihmc.tools.io.WorkspaceResourceFile;

import java.io.BufferedReader;
import java.io.FileReader;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class YOLOv8Tools
{
   private static final WorkspaceResourceDirectory POINT_CLOUD_DIRECTORY = new WorkspaceResourceDirectory(YOLOv8DetectionClass.class, "/yoloICPPointClouds/");

   public static List<Point3D32> filterOutliers(List<Point3D32> pointCloud, double zScoreThreshold, int numberOfSamples)
   {
      Point3D32 centroid = new Point3D32();
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

   public static List<Point3D32> loadPointCloudFromFile(String fileName)
   {
      if (fileName == null)
         throw new NullPointerException("We can't run ICP on this object yet because we don't have a model point cloud file.");

      WorkspaceResourceFile pointCloudFile = new WorkspaceResourceFile(POINT_CLOUD_DIRECTORY, fileName);
      List<Point3D32> pointCloud;
      try (BufferedReader bufferedReader = new BufferedReader(new FileReader(pointCloudFile.getFilesystemFile().toFile())))
      {
         pointCloud = bufferedReader.lines().map(line ->
                                                 {
                                                    String[] xyzValues = line.split(",");
                                                    float x = Float.parseFloat(xyzValues[0]);
                                                    float y = Float.parseFloat(xyzValues[1]);
                                                    float z = Float.parseFloat(xyzValues[2]);
                                                    return new Point3D32(x, y, z);
                                                 }).collect(Collectors.toList());
      }
      catch (Exception e)
      {
         e.printStackTrace();
         throw new RuntimeException("Failed trying to load the file.");
         // Handle any I/O problems
      }

      return pointCloud;
   }

   public static Point3D32 computeCentroidOfPointCloud(List<Point3D32> pointCloud, int pointsToAverage)
   {
      int numberOfPointsToUse = Math.min(pointsToAverage, pointCloud.size());

      Point3D32 centroid = new Point3D32();
      for (int i = 0; i < numberOfPointsToUse; i++)
         centroid.add(pointCloud.get(i));
      centroid.scale(1.0 / numberOfPointsToUse);

      return centroid;
   }

   public static double detectionCentroidDistanceSquared(YOLOv8SegmentedDetection detectionA, YOLOv8SegmentedDetection detectionB)
   {
      return detectionA.getCentroid().distanceSquared(detectionB.getCentroid());
   }
}