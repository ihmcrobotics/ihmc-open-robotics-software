package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.geometry.algorithms.FrameConvexPolygonWithLineIntersector2d;

public interface FootRotationCalculator extends SCS2YoGraphicHolder
{
   public void compute(FramePoint2DReadOnly desiredCoP, FramePoint2DReadOnly centerOfPressure);

   public boolean isFootRotating();

   public void getLineOfRotation(FrameLine2DBasics lineOfRotationToPack);

   public void reset();

   static boolean isIntersectionValid(FrameConvexPolygonWithLineIntersector2d intersector)
   {
      return intersector.getIntersectionResult() != FrameConvexPolygonWithLineIntersector2d.IntersectionResult.NO_INTERSECTION
            && intersector.getIntersectionResult() != FrameConvexPolygonWithLineIntersector2d.IntersectionResult.POINT_INTERSECTION
            && !intersector.getIntersectionPointOne().epsilonEquals(intersector.getIntersectionPointTwo(), 1e-3);
   }
}
