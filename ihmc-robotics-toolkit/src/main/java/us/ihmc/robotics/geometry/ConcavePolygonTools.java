package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

public class ConcavePolygonTools
{

   public static ConcavePolygonCropResult cropPolygonToAboveLine(ConvexPolygon2DReadOnly polygonToCrop,
                                                                 Line2DReadOnly cuttingLine,
                                                                 Vector2DReadOnly upDirection,
                                                                 ConvexPolygon2DBasics croppedPolygonToPack)
   {
      if (croppedPolygonToPack.getNumberOfVertices() < 5)
      {
         // must be convex, revert to convex polygon crop
         ConvexPolygonCropResult result = ConvexPolygonTools.cropPolygonToAboveLine(polygonToCrop,
                                                                                    cuttingLine,
                                                                                    upDirection,
                                                                                    croppedPolygonToPack);
         return ConcavePolygonCropResult.fromConvexPolygonCropResult(result);
      }



      return ConcavePolygonCropResult.CUT;
   }
}
