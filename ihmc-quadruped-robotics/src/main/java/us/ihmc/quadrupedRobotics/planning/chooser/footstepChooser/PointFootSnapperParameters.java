package us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser;

public interface PointFootSnapperParameters
{
   /**
    * @return distance inside planar region to project snapped point
    */
   double distanceInsidePlanarRegion();

   /**
    * @return maximum angle between planar region and world z to consider
    */
   double maximumNormalAngleFromVertical();
}
