package us.ihmc.robotics.geometry.algorithms;

import us.ihmc.robotics.geometry.ConvexPolygon2dCalculator;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLine2d;
import us.ihmc.robotics.geometry.FramePoint2d;

public class FrameConvexPolygonWithLineIntersector2d
{
   private final FramePoint2d intersectionPointOne;
   private final FramePoint2d intersectionPointTwo;

   private IntersectionResult intersectionResult;

   public enum IntersectionResult
   {
      LINE_SEGMENT_INTERSECTION, POINT_INTERSECTION, NO_INTERSECTION;
   }

   public FrameConvexPolygonWithLineIntersector2d()
   {
      intersectionPointOne = new FramePoint2d();
      intersectionPointTwo = new FramePoint2d();

      intersectionResult = IntersectionResult.NO_INTERSECTION;
   }

   public void intersectWithLine(FrameConvexPolygon2d frameConvexPolygon2d, FrameLine2d frameLine2d)
   {
      int intersectionTypeInt = ConvexPolygon2dCalculator.intersectionWithLine(frameLine2d.getLine2d(), intersectionPointOne.getPoint(),
                                                                               intersectionPointTwo.getPoint(), frameConvexPolygon2d.getConvexPolygon2d());
      
      packIntersectionType(intersectionTypeInt);
   }
   
   /**
    * There is actually no ray class at the moment, so we use a FrameLine2d.
    * 
    * TODO: Make ray classes and use them. @dcalvert
    */
   public void intersectWithRay(FrameConvexPolygon2d frameConvexPolygon2d, FrameLine2d frameRay2d)
   {
      int intersectionTypeInt = ConvexPolygon2dCalculator.intersectionWithRay(frameRay2d.getLine2d(), intersectionPointOne.getPoint(),
                                                                               intersectionPointTwo.getPoint(), frameConvexPolygon2d.getConvexPolygon2d());
      
      packIntersectionType(intersectionTypeInt);
   }

   private void packIntersectionType(int intersectionTypeInt)
   {
      switch (intersectionTypeInt)
      {
      case 0:
         intersectionResult = IntersectionResult.NO_INTERSECTION;
         break;
      case 1:
         intersectionResult = IntersectionResult.POINT_INTERSECTION;
         break;
      case 2:
         intersectionResult = IntersectionResult.LINE_SEGMENT_INTERSECTION;
         break;
      }
   }
   
   public FramePoint2d getIntersectionPointOne()
   {
      return intersectionPointOne;
   }
   
   public FramePoint2d getIntersectionPointTwo()
   {
      return intersectionPointTwo;
   }

   public IntersectionResult getIntersectionResult()
   {
      return intersectionResult;
   }
}
