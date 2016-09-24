package us.ihmc.geometry.polytope;

import javax.vecmath.Vector3d;

public class ExpandingPolytopeSilhouetteConstructor
{
   public static void computeSilhouetteFromW(ExpandingPolytopeEntry triangleEntrySeenByW, Vector3d w, ExpandingPolytopeEdgeList edgeListToPack)
   {
      triangleEntrySeenByW.setObsolete(); // This triangle is visible from w.

      edgeListToPack.clear();

      for (int triangleIndex = 0; triangleIndex < 3; triangleIndex++)
      {
         ExpandingPolytopeEntry adjacentTriangle = triangleEntrySeenByW.getAdjacentTriangle(triangleIndex);
         int adjacentTriangleEdgeIndex = triangleEntrySeenByW.getAdjacentTriangleEdgeIndex(triangleIndex);

         silhouette(adjacentTriangle, adjacentTriangleEdgeIndex, w, edgeListToPack);
      }
   }

   public static void silhouette(ExpandingPolytopeEntry entry, int i, Vector3d w, ExpandingPolytopeEdgeList edgeList)
   {
      if (!entry.isObsolete())
      {
         // Facet entry is visited for the first time.

         Vector3d closestPointToOrigin = entry.getClosestPointToOrigin();
         if (isSeeableFromW(closestPointToOrigin, w))
         {
            // Facet entry is not visible from w.
            edgeList.addEdge(entry, i);
         }
         else
         {
            // Mark entry visible, and search its neighbors.
            entry.setObsolete();
            int iPlusOne = (i + 1) % 3;
            int iPlusTwo = (i + 2) % 3;
            silhouette(entry.getAdjacentTriangle(iPlusOne), entry.getAdjacentTriangleEdgeIndex(iPlusOne), w, edgeList);
            silhouette(entry.getAdjacentTriangle(iPlusTwo), entry.getAdjacentTriangleEdgeIndex(iPlusTwo), w, edgeList);
         }
      }
   }

   public static boolean isSeeableFromW(Vector3d closestPointToOriginOnTriangle, Vector3d w)
   {
      return closestPointToOriginOnTriangle.dot(w) < closestPointToOriginOnTriangle.lengthSquared();
   }
}
