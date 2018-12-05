package us.ihmc.quadrupedRobotics.planning.chooser.footstepChooser;

public class DefaultPointFootSnapperParameters implements PointFootSnapperParameters
{
   @Override
   public double distanceInsidePlanarRegion()
   {
      return 0.08;
   }

   @Override
   public double maximumNormalAngleFromVertical()
   {
      return 0.3;
   }
}
