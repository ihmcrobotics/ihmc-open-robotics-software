package us.ihmc.perception.gpuHeightMap;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

public class GPUPointCloudToolsTest
{
   @Disabled
   @Test
   public void testTransformPoints()
   {
      int maxPoints = 5000;
      GPUPointCloudTools tools = new GPUPointCloudTools(5000);

      Random random = new Random(1738L);

      Stopwatch gpuTimer = new Stopwatch();
      Stopwatch cpuTimer = new Stopwatch();
      Stopwatch parallelCpuTimer = new Stopwatch();

      gpuTimer.reset();
      cpuTimer.reset();
      parallelCpuTimer.reset();

      for (int iter = 0; iter < 500; iter++)
      {
         int numberOfPoints = RandomNumbers.nextInt(random, 1000, maxPoints);
         ArrayList<Point3D32> basePoints = new ArrayList<>();

         for (int i = 0; i < numberOfPoints; i++)
         {
            basePoints.add(EuclidCoreRandomTools.nextPoint3D32(random));
         }

         RigidBodyTransformReadOnly transform = EuclidCoreRandomTools.nextRigidBodyTransform(random);
         gpuTimer.resume();
         List<Point3D> transformedPoints = tools.transformPoints(basePoints, transform);
         gpuTimer.lap();
         gpuTimer.suspend();

         cpuTimer.resume();
         List<Point3D> transformedPointsExpected = basePoints.stream().map(point ->
         {
            Point3D transformedPoint = new Point3D();
            transform.transform(point, transformedPoint);
            return transformedPoint;
         }).collect(Collectors.toList());
         cpuTimer.lap();
         cpuTimer.suspend();

         parallelCpuTimer.resume();
         List<Point3D> transformedPointsExpectedAlt = basePoints.parallelStream().map(point ->
                                                                           {
                                                                              Point3D transformedPoint = new Point3D();
                                                                              transform.transform(point, transformedPoint);
                                                                              return transformedPoint;
                                                                           }).collect(Collectors.toList());
         parallelCpuTimer.lap();
         parallelCpuTimer.suspend();

         for (int i = 0; i < numberOfPoints; i++)
         {
            EuclidCoreTestTools.assertEquals(transformedPointsExpected.get(i), transformedPoints.get(i), 1e-5);
         }
      }

      LogTools.info("CPU average: " + cpuTimer.averageLap());
      LogTools.info("GPU average: " + gpuTimer.averageLap());
      LogTools.info("Parallel CPU average: " + parallelCpuTimer.averageLap());
   }
}
