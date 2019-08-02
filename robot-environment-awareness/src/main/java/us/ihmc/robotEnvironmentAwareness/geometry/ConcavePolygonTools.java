package us.ihmc.robotEnvironmentAwareness.geometry;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.geometry.ConvexPolygonCropResult;
import us.ihmc.robotics.geometry.ConvexPolygonTools;

import java.util.ArrayList;
import java.util.List;

public class ConcavePolygonTools
{

   public static List<ConcaveHull> cropPolygonToAboveLine(ConcaveHull polygonToCrop,
                                                          Line2DReadOnly cuttingLine,
                                                          Vector2DReadOnly upDirection)
   {
      ArrayList<ConcaveHull> resultingConcaveHulls = new ArrayList<>();
      if (polygonToCrop.getNumberOfVertices() < 5)
      {
         // must be convex, revert to convex polygon crop
         ConvexPolygon2D convexPolygonToCrop = new ConvexPolygon2D();
         for (Point2D concaveHullVertex : polygonToCrop.getConcaveHullVertices())
         {
            convexPolygonToCrop.addVertex(concaveHullVertex);
         }
         convexPolygonToCrop.update();
         ConvexPolygon2D croppedPolygonToPack = new ConvexPolygon2D();
         ConvexPolygonCropResult result = ConvexPolygonTools.cropPolygonToAboveLine(convexPolygonToCrop,
                                                                                    cuttingLine,
                                                                                    upDirection,
                                                                                    croppedPolygonToPack);
         if (result != ConvexPolygonCropResult.REMOVE_ALL)
         {
            ConcaveHull concaveHullToReturn = new ConcaveHull(croppedPolygonToPack.getVertexBufferView());
            resultingConcaveHulls.add(concaveHullToReturn);
         }
         return resultingConcaveHulls;
      }



      return resultingConcaveHulls;
   }
}
