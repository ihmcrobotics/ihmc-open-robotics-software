package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class ConcavePolygonTools
{

   public static List<ConvexPolygon2D> cropPolygonToAboveLine(ConvexPolygon2DReadOnly polygonToCrop,
                                                              Line2DReadOnly cuttingLine,
                                                              Vector2DReadOnly upDirection)
   {
      ArrayList<ConvexPolygon2D> croppedPolygons = new ArrayList<>();
      if (polygonToCrop.getNumberOfVertices() < 5)
      {
         // must be convex, revert to convex polygon crop
         ConvexPolygon2D croppedPolygonToPack = new ConvexPolygon2D();
         ConvexPolygonCropResult result = ConvexPolygonTools.cropPolygonToAboveLine(polygonToCrop,
                                                                                    cuttingLine,
                                                                                    upDirection,
                                                                                    croppedPolygonToPack);
         if (result != ConvexPolygonCropResult.REMOVE_ALL)
         {
            croppedPolygons.add(croppedPolygonToPack);
         }
         return croppedPolygons;
      }



      return croppedPolygons;
   }
}
