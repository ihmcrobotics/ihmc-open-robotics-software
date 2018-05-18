package us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface PointFootSnapper
{
   /**
    * Specifies the height of the terrain at the given world-frame xy position.
    * If the given location should not be stepped on (e.g. obstacle/low terrain)
    * this returns {@link Double#NaN}
    * @param xPosition world-frame x location of step
    * @param yPosition world-frame y location of step
    * @return world-frame z location, or {@link Double#NaN} if not a valid step location
    */
   Point3DReadOnly snapStep(double xPosition, double yPosition);
}
