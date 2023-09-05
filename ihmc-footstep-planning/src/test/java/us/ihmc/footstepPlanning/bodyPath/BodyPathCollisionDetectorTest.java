package us.ihmc.footstepPlanning.bodyPath;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.Random;

public class BodyPathCollisionDetectorTest
{
   private final Random random = new Random(2930);

   @Test
   public void testCollisionDetector()
   {
      BodyPathCollisionDetector collisionDetector = new BodyPathCollisionDetector();

      HeightMapData heightMapData = new HeightMapData(0.1, 2.0, 0.0, 0.0);

      double boxSizeX = 0.25;
      double boxSizeY = 0.45;
      collisionDetector.initialize(heightMapData.getGridResolutionXY(), boxSizeX, boxSizeY);

      double randomHeight = EuclidCoreRandomTools.nextDouble(random, 10.0);
      heightMapData.setHeightAt(0.0, 0.0, randomHeight + 0.1);
      boolean collision = collisionDetector.collisionDetected(heightMapData, new BodyPathLatticePoint(0.0, 0.0), 0, randomHeight, 0.2);
      Assertions.assertFalse(collision, "Body path collision detector failed");

      randomHeight = EuclidCoreRandomTools.nextDouble(random, 10.0);
      heightMapData.reset();
      heightMapData.setHeightAt(0.0, 0.0, randomHeight + 0.201);
      collision = collisionDetector.collisionDetected(heightMapData, new BodyPathLatticePoint(0.0, 0.0), 0, randomHeight, 0.2);
      Assertions.assertTrue(collision, "Body path collision detector failed");

      randomHeight = EuclidCoreRandomTools.nextDouble(random, 10.0);
      heightMapData.reset();
      heightMapData.setHeightAt(0.0, 0.0, randomHeight);
      heightMapData.setHeightAt(-0.2, 0.0, randomHeight + 0.201);
      heightMapData.setHeightAt(0.2, 0.0, randomHeight + 0.201);
      heightMapData.setHeightAt(0.0, -0.3, randomHeight + 0.201);
      heightMapData.setHeightAt(0.0,0.3, randomHeight + 0.201);
      collision = collisionDetector.collisionDetected(heightMapData, new BodyPathLatticePoint(0.0, 0.0), 0, randomHeight, 0.2);
      Assertions.assertFalse(collision, "Body path collision detector failed");

      collision = collisionDetector.collisionDetected(heightMapData, new BodyPathLatticePoint(0.0, 0.0), 4, 0.0, 0.2);
      Assertions.assertFalse(collision, "Body path collision detector failed");

      heightMapData.reset();
      collisionDetector.initialize(heightMapData.getGridResolutionXY(), 0.15, 0.3);

      heightMapData.reset();
      randomHeight = EuclidCoreRandomTools.nextDouble(random, 10.0);
      heightMapData.setHeightAt(0.0, 0.0, randomHeight + 0.201);
      collision = collisionDetector.collisionDetected(heightMapData, new BodyPathLatticePoint(0.0, 0.0), 1, randomHeight, 0.2);
      Assertions.assertTrue(collision, "Body path collision detector failed");

      Point2D[] collidingPoints = new Point2D[]{new Point2D(0.0, 0.0), new Point2D(0.1, 0.0), new Point2D(0.0, -0.1), new Point2D(0.0, 0.1), new Point2D(0.1, 0.1), new Point2D(-0.1, -0.1), new Point2D(-0.1, 0.0)};
      Point2D[] nonCollidingPoints = new Point2D[]{new Point2D(0.1, 0.1), new Point2D(-0.1, -0.1), new Point2D(0.2, 0.0)};

      for (int yawIndex : new int[]{2, 10})
      {
         heightMapData.reset();
         for (int i = 0; i < collidingPoints.length; i++)
         {
            heightMapData.setHeightAt(collidingPoints[i].getX(), collidingPoints[i].getY(), randomHeight + 0.201);
         }
         Assertions.assertTrue(collisionDetector.collisionDetected(heightMapData, new BodyPathLatticePoint(0.0, 0.0), yawIndex, randomHeight, 0.2), "Body collision detector failed");

         heightMapData.reset();
         for (int i = 0; i < nonCollidingPoints.length; i++)
         {
            heightMapData.setHeightAt(nonCollidingPoints[i].getX(), nonCollidingPoints[i].getY(), randomHeight + 0.201);
         }
         Assertions.assertFalse(collisionDetector.collisionDetected(heightMapData, new BodyPathLatticePoint(0.0, 0.0), yawIndex, randomHeight, 0.2), "Body collision detector failed");
      }

      for (int yawIndex : new int[]{6, 14})
      {
         heightMapData.reset();
         for (int i = 0; i < collidingPoints.length; i++)
         {
            heightMapData.setHeightAt(collidingPoints[i].getY(), -collidingPoints[i].getX(), randomHeight + 0.201);
         }
         Assertions.assertTrue(collisionDetector.collisionDetected(heightMapData, new BodyPathLatticePoint(0.0, 0.0), yawIndex, randomHeight, 0.2), "Body collision detector failed");
      }
   }

}
