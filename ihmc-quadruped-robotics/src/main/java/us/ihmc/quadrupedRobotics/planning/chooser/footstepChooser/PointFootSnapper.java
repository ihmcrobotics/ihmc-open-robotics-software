package us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface PointFootSnapper
{
   /**
    * Projects the given xy position to the terrain.
    * If the given location cannot be snapped, this returns a point containing {@link Double#NaN}
    * @param xPosition world-frame x location of step
    * @param yPosition world-frame y location of step
    * @param minimumZPosition minimum world-frame z value to snap to
    * @return world-frame snapped location
    */
   Point3DReadOnly snapStep(double xPosition, double yPosition, double minimumZPosition);

   default Point3DReadOnly snapStep(double xPosition, double yPosition)
   {
      return snapStep(xPosition, yPosition, Double.NEGATIVE_INFINITY);
   }
}
