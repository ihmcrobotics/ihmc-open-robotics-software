package us.ihmc.geometry.polytope;

import java.util.PriorityQueue;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class ExpandingPolytopeAlgorithm
{
   private final PriorityQueue<ExpandingPolytopeEntry> triangleEntryQueue = new PriorityQueue<ExpandingPolytopeEntry>();
   private final ExpandingPolytopeEdgeList edgeList = new ExpandingPolytopeEdgeList();

   private ConvexPolytope polytopeA;
   private ConvexPolytope polytopeB;

   private final double epsilonRelative;

   public ExpandingPolytopeAlgorithm(double epsilonRelative)
   {
      this.epsilonRelative = epsilonRelative;
   }

   public void setPolytopes(SimplexPolytope simplex, ConvexPolytope polytopeOne, ConvexPolytope polytopeTwo)
   {
      this.polytopeA = polytopeOne;
      this.polytopeB = polytopeTwo;

      triangleEntryQueue.clear();

      int numberOfPoints = simplex.getNumberOfPoints();
      if (numberOfPoints != 4)
         throw new RuntimeException("Implement for non tetrahedral simplex");

      Point3d pointOne = simplex.getPoint(0);
      Point3d pointTwo = simplex.getPoint(0);
      Point3d pointThree = simplex.getPoint(0);
      Point3d pointFour = simplex.getPoint(0);

      ExpandingPolytopeEntry entry123 = new ExpandingPolytopeEntry(pointOne, pointTwo, pointThree);
      ExpandingPolytopeEntry entry124 = new ExpandingPolytopeEntry(pointOne, pointTwo, pointFour);
      ExpandingPolytopeEntry entry134 = new ExpandingPolytopeEntry(pointOne, pointThree, pointFour);
      ExpandingPolytopeEntry entry234 = new ExpandingPolytopeEntry(pointThree, pointThree, pointFour);

      entry123.setAdjacentTriangle(0, entry124, 0);
      entry124.setAdjacentTriangle(0, entry123, 0);

      entry123.setAdjacentTriangle(1, entry234, 0);
      entry234.setAdjacentTriangle(0, entry123, 1);

      entry123.setAdjacentTriangle(2, entry134, 0);
      entry134.setAdjacentTriangle(0, entry123, 2);

      entry124.setAdjacentTriangle(1, entry234, 2);
      entry234.setAdjacentTriangle(2, entry124, 1);

      entry124.setAdjacentTriangle(2, entry134, 2);
      entry134.setAdjacentTriangle(2, entry124, 2);

      entry134.setAdjacentTriangle(1, entry234, 1);
      entry234.setAdjacentTriangle(1, entry134, 1);

      triangleEntryQueue.add(entry123);
      triangleEntryQueue.add(entry124);
      triangleEntryQueue.add(entry134);
      triangleEntryQueue.add(entry234);
   }

   private final Vector3d supportDirection = new Vector3d();
   //   private final Vector3d supportingVertexOne = new Vector3d();
   //   private final Vector3d supportingVertexTwo = new Vector3d();

   public void computeExpandedPolytope()
   {
      double mu = Double.POSITIVE_INFINITY; // Upper bound for the square penetration depth.

      while (true)
      {
         ExpandingPolytopeEntry triangleEntryToExpand = triangleEntryQueue.poll();
         boolean closeEnough = false;

         if (!triangleEntryToExpand.isObsolete())
         {
            Vector3d closestPointToOrigin = triangleEntryToExpand.getClosestPointToOrigin();

            supportDirection.set(closestPointToOrigin);

            PolytopeVertex supportingVertexA = polytopeA.getSupportingVertex(supportDirection);
            supportDirection.negate();
            PolytopeVertex supportingVertexB = polytopeB.getSupportingVertex(supportDirection);

            Vector3d w = new Vector3d();
            w.sub(supportingVertexA.getPosition(), supportingVertexB.getPosition());

            double vDotW = closestPointToOrigin.dot(w);
            double lengthSquared = closestPointToOrigin.lengthSquared();
            mu = Math.min(mu, vDotW * vDotW / lengthSquared);

            closeEnough = (mu <= (1.0 + epsilonRelative) * (1.0 + epsilonRelative) * lengthSquared);

            if (!closeEnough)
            {
               // Blow up the current polytope by adding vertex w.
               triangleEntryToExpand.setObsolete(); // This triangle is visible from w.
               edgeList.clear();

               for (int triangleIndex = 0; triangleIndex < 3; triangleIndex++)
               {
                  ExpandingPolytopeEntry adjacentTriangle = triangleEntryToExpand.getAdjacentTriangle(triangleIndex);
                  int adjacentTriangleEdgeIndex = triangleEntryToExpand.getAdjacentTriangleEdgeIndex(triangleIndex);

                  silhouette(adjacentTriangle, adjacentTriangleEdgeIndex, w, edgeList);
               }

               // edgeList now is the entire silhouette of the current polytope as seen from w.

               int numberOfEdges = edgeList.getNumberOfEdges();
               for (int edgeIndex = 0; edgeIndex < numberOfEdges; edgeIndex++)
               {
                  ExpandingPolytopeEdge edge = edgeList.getEdge(edgeIndex);

                  ExpandingPolytopeEntry sentry = edge.getEntry();
                  int sentryEdgeIndex = edge.getEdgeIndex();
                  int nextIndex = (sentryEdgeIndex + 1) % 3;
                  //TODO: Finish this after we get some vizes...
//                  ExpandingPolytopeEntry newEntry = new ExpandingPolytopeEntry(sentry.getVertex(nextIndex), sentry.getVertex(sentryEdgeIndex), w);
               }

            }

         }

         if (closeEnough)
            return;
         if (triangleEntryQueue.isEmpty())
            return;

         ExpandingPolytopeEntry bestRemainingTriangle = triangleEntryQueue.peek();
         Vector3d bestRemainingPointToOrigin = bestRemainingTriangle.getClosestPointToOrigin();
         if (bestRemainingPointToOrigin.lengthSquared() > mu)
            return;
      }
   }

   public static void silhouette(ExpandingPolytopeEntry entry, int i, Vector3d w, ExpandingPolytopeEdgeList edgeList)
   {
      if (!entry.isObsolete())
      {
         // Facet entry is visited for the first time.

         Vector3d closestPointToOrigin = entry.getClosestPointToOrigin();
         if (distanceSquared(closestPointToOrigin, w) < closestPointToOrigin.lengthSquared())
         {
            // Facet entry is not visible from w.
            edgeList.addEdge(entry, i);
         }
         else
         {
            // Mark entry visible, and search its neighbors.
            entry.setObsolete();
            int iPlusOne = (i + 1) % 3;
            int iPlusTwo = (i + 1) % 3;
            silhouette(entry.getAdjacentTriangle(iPlusOne), entry.getAdjacentTriangleEdgeIndex(iPlusOne), w, edgeList);
            silhouette(entry.getAdjacentTriangle(iPlusTwo), entry.getAdjacentTriangleEdgeIndex(iPlusTwo), w, edgeList);
         }
      }
   }

   private static double distanceSquared(Vector3d v, Vector3d w)
   {
      double xDiff = v.getX() - w.getX();
      double yDiff = v.getY() - w.getY();
      double zDiff = v.getZ() - w.getZ();

      return (xDiff * xDiff + yDiff * yDiff + zDiff * zDiff);
   }

}
