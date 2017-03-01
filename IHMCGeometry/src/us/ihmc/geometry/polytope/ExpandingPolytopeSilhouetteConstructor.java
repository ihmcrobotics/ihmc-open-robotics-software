package us.ihmc.geometry.polytope;

import us.ihmc.euclid.tuple3D.Vector3D;

public class ExpandingPolytopeSilhouetteConstructor
{
   public static void computeSilhouetteFromW(ExpandingPolytopeEntry triangleEntrySeenByW, Vector3D w, ExpandingPolytopeEdgeList edgeListToPack)
   {
      triangleEntrySeenByW.setObsolete(); // This triangle is visible from w.

      edgeListToPack.clear();

      for (int triangleIndex = 0; triangleIndex < 3; triangleIndex++)
      {
         ExpandingPolytopeEntry adjacentTriangle = triangleEntrySeenByW.getAdjacentTriangle(triangleIndex);
         int adjacentTriangleEdgeIndex = triangleEntrySeenByW.getAdjacentTriangleEdgeIndex(triangleIndex);

         if (adjacentTriangle != null)
         {            
            silhouette(adjacentTriangle, adjacentTriangleEdgeIndex, w, edgeListToPack);
         }
      }
   }

   public static void silhouette(ExpandingPolytopeEntry entry, int i, Vector3D w, ExpandingPolytopeEdgeList edgeList)
   {
      if ((entry != null) && !entry.isObsolete())
      {
         // Facet entry is visited for the first time.

         Vector3D closestPointToOrigin = entry.getClosestPointToOrigin();
         if (isNotVisibleFromW(closestPointToOrigin, w))
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

   public static boolean isNotVisibleFromW(Vector3D closestPointToOriginOnTriangle, Vector3D w)
   {
      return closestPointToOriginOnTriangle.dot(w) < closestPointToOriginOnTriangle.dot(closestPointToOriginOnTriangle);
   }
}
