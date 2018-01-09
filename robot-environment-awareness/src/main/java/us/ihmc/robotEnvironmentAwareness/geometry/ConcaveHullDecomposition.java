package us.ihmc.robotEnvironmentAwareness.geometry;

import static us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools.findClosestIntersectionWithRay;
import static us.ihmc.robotics.lists.ListWrappingIndexTools.next;
import static us.ihmc.robotics.lists.ListWrappingIndexTools.removeAllExclusive;
import static us.ihmc.robotics.lists.ListWrappingIndexTools.subListInclusive;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public class ConcaveHullDecomposition
{
   private static final boolean EXPORT_BAD_POLYGONS_TO_FILE = false;
   private static final String BAD_POLYGONS_FILE_NAME = "badPolygon";
   private static int badPolygonsFileIndex = 0;

   /**
    * Inspired from the SL-decomposition in the paper 
    * <a href="https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=2&cad=rja&uact=8&ved=0ahUKEwjZlOab96XPAhXBQD4KHcXeB4MQFggsMAE&url=https%3A%2F%2Fparasol.tamu.edu%2Fpublications%2Fdownload.php%3Ffile_id%3D390&usg=AFQjCNF3wXvuCxXNREhu4CW-oNyd1caa0A&sig2=X-zxaHykED7EuqkYhkfUgg">
    *  Approximate Convex Decomposition of Polygons</a>.
    *  @param concaveHullCollection [input] the collection of concave hulls to be decomposed into convex polygons.
    *  @param depthThreshold [input] the algorithm determines whether the polygon is to split or not by looking at the maximum depth of concave pockets in the concave hull.
    *  When a pocket is deeper than {@code depthThreshold} the concave hull will be split in two.
    *  Otherwise, the pocket vertices will be removed.
    *  @param convexPolygonsToPack [output] the convex polygons approximating the concave hull.
    */
   public static void recursiveApproximateDecomposition(ConcaveHullCollection concaveHullCollection, double depthThreshold, List<ConvexPolygon2D> convexPolygonsToPack)
   {
      for (ConcaveHull concaveHull : concaveHullCollection)
         recursiveApproximateDecomposition(concaveHull.getConcaveHullVertices(), depthThreshold, convexPolygonsToPack);
   }

   /**
    * Inspired from the SL-decomposition in the paper 
    * <a href="https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=2&cad=rja&uact=8&ved=0ahUKEwjZlOab96XPAhXBQD4KHcXeB4MQFggsMAE&url=https%3A%2F%2Fparasol.tamu.edu%2Fpublications%2Fdownload.php%3Ffile_id%3D390&usg=AFQjCNF3wXvuCxXNREhu4CW-oNyd1caa0A&sig2=X-zxaHykED7EuqkYhkfUgg">
    *  Approximate Convex Decomposition of Polygons</a>.
    *  @param concaveHull [input] the concave hull to be decomposed into convex polygons.
    *  @param depthThreshold [input] the algorithm determines whether the polygon is to split or not by looking at the maximum depth of concave pockets in the concave hull.
    *  When a pocket is deeper than {@code depthThreshold} the concave hull will be split in two.
    *  Otherwise, the pocket vertices will be removed.
    *  @param convexPolygonsToPack [output] the convex polygons approximating the concave hull.
    */
   public static void recursiveApproximateDecomposition(ConcaveHull concaveHull, double depthThreshold, List<ConvexPolygon2D> convexPolygonsToPack)
   {
      recursiveApproximateDecomposition(concaveHull.getConcaveHullVertices(), depthThreshold, convexPolygonsToPack);
   }

   /**
    * Inspired from the SL-decomposition in the paper 
    * <a href="https://www.google.com/url?sa=t&rct=j&q=&esrc=s&source=web&cd=2&cad=rja&uact=8&ved=0ahUKEwjZlOab96XPAhXBQD4KHcXeB4MQFggsMAE&url=https%3A%2F%2Fparasol.tamu.edu%2Fpublications%2Fdownload.php%3Ffile_id%3D390&usg=AFQjCNF3wXvuCxXNREhu4CW-oNyd1caa0A&sig2=X-zxaHykED7EuqkYhkfUgg">
    *  Approximate Convex Decomposition of Polygons</a>.
    *  @param concaveHullVertices [input] the concave hull to be decomposed into convex polygons.
    *  @param depthThreshold [input] the algorithm determines whether the polygon is to split or not by looking at the maximum depth of concave pockets in the concave hull.
    *  When a pocket is deeper than {@code depthThreshold} the concave hull will be split in two.
    *  Otherwise, the pocket vertices will be removed.
    *  @param convexPolygonsToPack [output] the convex polygons approximating the concave hull.
    */
   public static void recursiveApproximateDecomposition(List<? extends Point2DReadOnly> concaveHullVertices, double depthThreshold, List<ConvexPolygon2D> convexPolygonsToPack)
   {
      recursiveApproximateDecompositionInternal(new ArrayList<>(concaveHullVertices), depthThreshold, convexPolygonsToPack);
   }

   private static void recursiveApproximateDecompositionInternal(List<Point2DReadOnly> concaveHullVertices, double depthThreshold, List<ConvexPolygon2D> convexPolygonsToPack)
   {
      
      ConcaveHullPocket pocket = null;
      boolean hasFoundDeepPocket = false;

      while (!hasFoundDeepPocket)
      {
         if (concaveHullVertices.isEmpty())
         {
            PrintTools.warn("The concave hull is empty");
            return;
         }

         // The concave hull is actually convex, end of recursion
         if (ConcaveHullTools.isHullConvex(concaveHullVertices))
         {
            convexPolygonsToPack.add(new ConvexPolygon2D(concaveHullVertices));
            return;
         }

         pocket = ConcaveHullTools.findFirstConcaveHullPocket(concaveHullVertices);

         if (pocket == null)
         {
            PrintTools.error("Did not find any pocket.");
            if (EXPORT_BAD_POLYGONS_TO_FILE)
               ConcaveHullTools.exportVertexListToFile(concaveHullVertices, BAD_POLYGONS_FILE_NAME + (badPolygonsFileIndex++));
            return;
         }

         int startBridgeIndex = pocket.getStartBridgeIndex();
         int endBridgeIndex = pocket.getEndBridgeIndex();

         // The pocket is negligible, remove the vertices.
         if (pocket.getMaxDepth() < depthThreshold)
            removeAllExclusive(startBridgeIndex, endBridgeIndex, concaveHullVertices);
         else
            hasFoundDeepPocket = true;
      }

      int deepestVertexIndex = pocket.getDeepestVertexIndex();
      Vector2D cutDirection = new Vector2D();
      Point2DReadOnly endBridgeVertex = pocket.getEndBridgeVertex();
      Point2DReadOnly startBridgeVertex = pocket.getStartBridgeVertex();
      cutDirection.sub(endBridgeVertex, startBridgeVertex);
      cutDirection.normalize();
      cutDirection.set(cutDirection.getY(), -cutDirection.getX()); // Rotate 90 degrees to the right (inside polygon)

      Point2DReadOnly deepestVertex = pocket.getDeepestVertex();


      Point2D otherVertexForCutting = new Point2D();
      int startBridgeIndex = pocket.getStartBridgeIndex();
      int endBridgeIndex = pocket.getEndBridgeIndex();

      int otherVertexIndexForCutting = findClosestIntersectionWithRay(deepestVertex, cutDirection, endBridgeIndex, startBridgeIndex, concaveHullVertices, otherVertexForCutting);
      otherVertexIndexForCutting = next(otherVertexIndexForCutting, concaveHullVertices);
      
      if (otherVertexIndexForCutting == -1)
      {
         PrintTools.error("Something went wrong finding the other vertex for cutting. Pocket vertex: " + deepestVertex + ", bridge: start: "
               + startBridgeVertex + ", end: " + endBridgeVertex);
         if (EXPORT_BAD_POLYGONS_TO_FILE)
            ConcaveHullTools.exportVertexListToFile(concaveHullVertices, BAD_POLYGONS_FILE_NAME + (badPolygonsFileIndex++));
         return;
      }

      // It is easier to add a new vertex rather than using an existing one, it limits the number of edge cases.
      concaveHullVertices.add(otherVertexIndexForCutting, otherVertexForCutting);
      if (otherVertexIndexForCutting < deepestVertexIndex)
         deepestVertexIndex++;

      // decompose the two new polygons.
      int p1StartIndex = deepestVertexIndex;
      int p1EndIndex = otherVertexIndexForCutting;
      int p2StartIndex = otherVertexIndexForCutting;
      int p2EndIndex = deepestVertexIndex;

      List<Point2DReadOnly> p1 = subListInclusive(p1StartIndex, p1EndIndex, concaveHullVertices);
      List<Point2DReadOnly> p2 = subListInclusive(p2StartIndex, p2EndIndex, concaveHullVertices);

      if (p1.size() == concaveHullVertices.size() || p2.size() == concaveHullVertices.size())
      {
         if (EXPORT_BAD_POLYGONS_TO_FILE)
            ConcaveHullTools.exportVertexListToFile(concaveHullVertices, BAD_POLYGONS_FILE_NAME + (badPolygonsFileIndex++));
         PrintTools.error("Something went wrong splitting the polygon");
         return;
      }

      recursiveApproximateDecomposition(p1, depthThreshold, convexPolygonsToPack);
      recursiveApproximateDecomposition(p2, depthThreshold, convexPolygonsToPack);
   }
}
