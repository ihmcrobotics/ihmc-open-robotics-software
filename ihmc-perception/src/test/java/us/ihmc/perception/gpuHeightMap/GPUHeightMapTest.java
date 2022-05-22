package us.ihmc.perception.gpuHeightMap;

import org.junit.jupiter.api.Test;
import org.ojalgo.random.RandomNumber;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.heightMap.HeightMapData;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class GPUHeightMapTest
{
   @Test
   public void testSimpleHeightMap()
   {
      GPUHeightMapParameters parameters = new GPUHeightMapParameters();
      parameters.mapLength = 2.0;
      GPUHeightMap gpuHeightMap = new GPUHeightMap(parameters);

      HeightMapData cpuHeightMap = new HeightMapData(parameters.resolution, parameters.mapLength, 0.0, 0.0);

      Random random = new Random(1738L);
      List<Point3D> pointsToAdd = new ArrayList<>();
      for (int i = 0; i < 100; i++)
      {
         Point3D point = EuclidCoreRandomTools.nextPoint3D(random, parameters.mapLength);
         pointsToAdd.add(point);
         cpuHeightMap.setHeightAt(point.getX(), point.getY(), point.getZ());
      }

      gpuHeightMap.input(pointsToAdd, new RigidBodyTransform());
   }
}
