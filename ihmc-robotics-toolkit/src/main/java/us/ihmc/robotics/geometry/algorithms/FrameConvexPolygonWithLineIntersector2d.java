package us.ihmc.robotics.geometry.algorithms;

import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;

public class FrameConvexPolygonWithLineIntersector2d
{
   private final FramePoint2D intersectionPointOne;
   private final FramePoint2D intersectionPointTwo;

   private IntersectionResult intersectionResult;

   public enum IntersectionResult
   {
      LINE_SEGMENT_INTERSECTION, POINT_INTERSECTION, NO_INTERSECTION;
   }

   public FrameConvexPolygonWithLineIntersector2d()
   {
      intersectionPointOne = new FramePoint2D();
      intersectionPointTwo = new FramePoint2D();

      intersectionResult = IntersectionResult.NO_INTERSECTION;
   }

   public void intersectWithLine(FrameConvexPolygon2d frameConvexPolygon2d, FrameLine2D frameLine2d)
   {
      checkAndSetFrames(frameConvexPolygon2d, frameLine2d);
      int intersectionTypeInt = frameConvexPolygon2d.getConvexPolygon2d().intersectionWith(frameLine2d, intersectionPointOne,
                                                                                           intersectionPointTwo);

      packIntersectionType(intersectionTypeInt);
   }

   /**
    * There is actually no ray class at the moment, so we use a FrameLine2d.
    *
    * TODO: Make ray classes and use them. @dcalvert
    */
   public void intersectWithRay(FrameConvexPolygon2d frameConvexPolygon2d, FrameLine2D frameRay2d)
   {
      checkAndSetFrames(frameConvexPolygon2d, frameRay2d);
      int intersectionTypeInt = frameConvexPolygon2d.getConvexPolygon2d().intersectionWithRay(frameRay2d, intersectionPointOne,
                                                                                              intersectionPointTwo);

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

   private void checkAndSetFrames(ReferenceFrameHolder frameObject1, ReferenceFrameHolder frameObject2)
   {
      frameObject1.checkReferenceFrameMatch(frameObject2);
      intersectionPointOne.setToZero(frameObject1.getReferenceFrame());
      intersectionPointTwo.setToZero(frameObject1.getReferenceFrame());
   }

   public FramePoint2D getIntersectionPointOne()
   {
      return intersectionPointOne;
   }

   public FramePoint2D getIntersectionPointTwo()
   {
      return intersectionPointTwo;
   }

   public IntersectionResult getIntersectionResult()
   {
      return intersectionResult;
   }
}
