package us.ihmc.sensorProcessing.heightMap;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class HeightMapManagerTest
{
   @Test
   public void testTranslateHeightMap()
   {
      HeightMapParameters parameters = new HeightMapParameters();
      double discretizationXY = 0.05;
      double sizeXY = 1.0;
      HeightMapManager heightMapManager = new HeightMapManager(parameters, discretizationXY, sizeXY);

      Point2D gridCenter = new Point2D(0.25, 0.25);
      heightMapManager.resetAtGridCenter(gridCenter.getX(), gridCenter.getY());
      int cellsToPopulate = 5;
      int pointsPerCell = 5;

      double farLeft = gridCenter.getX() - cellsToPopulate * discretizationXY;
      double farRight = gridCenter.getX() + cellsToPopulate * discretizationXY;;
      double farDown = gridCenter.getY() - cellsToPopulate * discretizationXY;
      double farUp = gridCenter.getY() + cellsToPopulate * discretizationXY;
      List<Point3D> points = new ArrayList<>();
      List<Point3D> locations = new ArrayList<>();
      double height = 0.2;
      for (double xPosition = farLeft; xPosition <= farRight; xPosition += discretizationXY)
      {
         for (double yPosition = farDown; yPosition <= farUp; yPosition += discretizationXY)
         {
            Point3D point = new Point3D(xPosition, yPosition, height);
            locations.add(point);
            for (int i = 0; i < pointsPerCell; i++)
               points.add(point);
         }
      }
      Point3D[] pointArray = new Point3D[points.size()];
      points.toArray(pointArray);
      heightMapManager.update(pointArray);

      // Check that the points added are at the right height
      for (int i = 0; i < locations.size(); i++)
      {
         Point3D location = locations.get(i);
         assertEquals(height, heightMapManager.getHeightAt(location.getX(), location.getY()), 1e-5);
      }

      Point2D newGridCenter = new Point2D(gridCenter);
      newGridCenter.add(4 * discretizationXY, -4 * discretizationXY);

      heightMapManager.translateToNewGridCenter(newGridCenter, parameters.getVarianceAddedWhenTranslating());


      // Check that the ptranslated points are at the right height
      for (int i = 0; i < locations.size(); i++)
      {
         Point3D location = locations.get(i);
         assertEquals(height, heightMapManager.getHeightAt(location.getX(), location.getY()), 1e-5);
      }
   }
}
