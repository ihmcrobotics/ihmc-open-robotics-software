package us.ihmc.perception.gpuHeightMap;

import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.heightMap.HeightMapData;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;

public class SimpleGPUHeightMapUpdaterTest
{
   private static final int timingLaps = 1;
   private static final int pointsToInclude = 10000;

   @Test
   public void testSimpleHeightMap()
   {
      SimpleGPUHeightMapParameters parameters = new SimpleGPUHeightMapParameters();
      parameters.mapLength = 2.0;
      parameters.resolution = 0.04;
      SimpleGPUHeightMapUpdater gpuHeightMap = new SimpleGPUHeightMapUpdater(parameters);

      Random random = new Random(1738L);
      List<Point3D> pointsToAdd = new ArrayList<>();
      for (int i = 0; i < pointsToInclude; i++)
      {
         Point3D point = EuclidCoreRandomTools.nextPoint3D(random, 0.5 * parameters.mapLength);
         point.setZ(RandomNumbers.nextDouble(random, 0.1));
         pointsToAdd.add(point);
      }

      int numberOfCells = (int) Math.round(parameters.mapLength / parameters.resolution) + 2;

      DMatrixRMaj averageHeightAtPoint = null;
      DMatrixRMaj varianceAtPoint = null;
      DMatrixRMaj numberOfCellsAtPoint = null;

      Stopwatch naiveTime = new Stopwatch();
      naiveTime.start();

      for (int lap = 0; lap < timingLaps; lap++)
      {

         DMatrixRMaj cumulativeHeightAtPoint = new DMatrixRMaj(numberOfCells, numberOfCells);
         DMatrixRMaj cumulativeHeightSquaredAtPoint = new DMatrixRMaj(numberOfCells, numberOfCells);
         numberOfCellsAtPoint = new DMatrixRMaj(numberOfCells, numberOfCells);

         for (int i = 0; i < pointsToAdd.size(); i++)
         {
            Point3D point = pointsToAdd.get(i);
            int xIndex = SimpleGPUHeightMap.getIndex(point.getX(), 0.0, parameters.resolution, numberOfCells);
            int yIndex = SimpleGPUHeightMap.getIndex(point.getY(), 0.0, parameters.resolution, numberOfCells);

            cumulativeHeightAtPoint.add(xIndex, yIndex, point.getZ());
            cumulativeHeightSquaredAtPoint.add(xIndex, yIndex, MathTools.square(point.getZ()));
            numberOfCellsAtPoint.add(xIndex, yIndex, 1);
         }

         averageHeightAtPoint = new DMatrixRMaj(numberOfCells, numberOfCells);
         varianceAtPoint = new DMatrixRMaj(numberOfCells, numberOfCells);

         for (int i = 0; i < cumulativeHeightAtPoint.getNumElements(); i++)
         {
            double average = cumulativeHeightAtPoint.get(i) / numberOfCellsAtPoint.get(i);
            double variance = (cumulativeHeightSquaredAtPoint.get(i) - MathTools.square(cumulativeHeightAtPoint.get(i)) / numberOfCellsAtPoint.get(i))
                              / (numberOfCellsAtPoint.get(i) - 1);
            averageHeightAtPoint.set(i, average);
            varianceAtPoint.set(i, variance);
         }
         naiveTime.lap();
      }

      naiveTime.suspend();
      LogTools.info("Naive time = " + naiveTime.averageLap());


      Stopwatch stopwatch = new Stopwatch();
      stopwatch.start();
      for (int lap = 0; lap < timingLaps; lap++)
      {
         gpuHeightMap.input(pointsToAdd, new RigidBodyTransform());
         stopwatch.lap();
      }
      stopwatch.suspend();
      LogTools.info("GPU time = " + stopwatch.averageLap());

      for (int i = 0; i < pointsToAdd.size(); i++)
      {
         Point3D point = pointsToAdd.get(i);
         int xIndex = SimpleGPUHeightMap.getIndex(point.getX(), 0.0, parameters.resolution, numberOfCells);
         int yIndex = SimpleGPUHeightMap.getIndex(point.getY(), 0.0, parameters.resolution, numberOfCells);

         assertEquals(averageHeightAtPoint.get(xIndex, yIndex), gpuHeightMap.getHeightMap().getHeightAtPoint(point.getX(), point.getY()), 1e-5);
         assertEquals(numberOfCellsAtPoint.get(xIndex, yIndex), gpuHeightMap.getHeightMap().getPointsAtPoint(point.getX(), point.getY()), 1e-5);
         assertEquals(varianceAtPoint.get(xIndex, yIndex), gpuHeightMap.getHeightMap().getVarianceAtPoint(point.getX(), point.getY()), 1e-5);
      }
   }
}
