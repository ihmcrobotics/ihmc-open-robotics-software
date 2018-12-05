package us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.graphicsDescription.HeightMap;

public class HeightMapFootSnapper implements PointFootSnapper
{
   private final Point3D snappedStep = new Point3D();
   private final HeightMap heightMap;

   public HeightMapFootSnapper(HeightMap heightMap)
   {
      this.heightMap = heightMap;
   }

   @Override
   public Point3DReadOnly snapStep(double xPosition, double yPosition, double minimumZPosition)
   {
      snappedStep.setX(xPosition);
      snappedStep.setY(yPosition);
      snappedStep.setZ(heightMap.heightAt(xPosition, yPosition, 0.0));
      return snappedStep;
   }
}
