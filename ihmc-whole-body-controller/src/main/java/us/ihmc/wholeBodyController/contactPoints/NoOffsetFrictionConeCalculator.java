package us.ihmc.wholeBodyController.contactPoints;

import java.util.List;

import us.ihmc.euclid.tuple2D.Point2D;

public class NoOffsetFrictionConeCalculator implements FrictionConeRotationCalculator
{

   @Override
   public double computeYawOffset(List<Point2D> contactPoints, int contactIdx, int vectors, int vectorIdx)
   {
      return 0.0;
   }

}
