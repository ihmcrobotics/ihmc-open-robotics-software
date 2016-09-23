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

   private ExpandingPolytopeAlgorithmListener listener;

   public ExpandingPolytopeAlgorithm(double epsilonRelative)
   {
      this.epsilonRelative = epsilonRelative;
   }

   public void setExpandingPolytopeAlgorithmListener(ExpandingPolytopeAlgorithmListener listener)
   {
      this.listener = listener;
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
      Point3d pointTwo = simplex.getPoint(1);
      Point3d pointThree = simplex.getPoint(2);
      Point3d pointFour = simplex.getPoint(3);

      ExpandingPolytopeEntry entry123 = new ExpandingPolytopeEntry(pointOne, pointTwo, pointThree);
      ExpandingPolytopeEntry entry324 = new ExpandingPolytopeEntry(pointThree, pointTwo, pointFour);
      ExpandingPolytopeEntry entry421 = new ExpandingPolytopeEntry(pointFour, pointTwo, pointOne);
      ExpandingPolytopeEntry entry134 = new ExpandingPolytopeEntry(pointOne, pointThree, pointFour);

      entry123.setAdjacentTriangle(1, entry324, 0);
      entry324.setAdjacentTriangle(0, entry123, 1);

      entry123.setAdjacentTriangle(0, entry421, 1);
      entry421.setAdjacentTriangle(1, entry123, 0);

      entry123.setAdjacentTriangle(2, entry134, 0);
      entry134.setAdjacentTriangle(0, entry123, 2);

      entry324.setAdjacentTriangle(1, entry421, 0);
      entry421.setAdjacentTriangle(0, entry324, 1);

      entry324.setAdjacentTriangle(2, entry134, 1);
      entry134.setAdjacentTriangle(1, entry324, 2);

      entry421.setAdjacentTriangle(2, entry134, 2);
      entry134.setAdjacentTriangle(2, entry421, 2);

      if (entry123.closestIsInternal()) triangleEntryQueue.add(entry123);
      if (entry324.closestIsInternal()) triangleEntryQueue.add(entry324);
      if (entry421.closestIsInternal()) triangleEntryQueue.add(entry421);
      if (entry134.closestIsInternal()) triangleEntryQueue.add(entry134);

      if (listener != null)
      {
         listener.setPolytopes(simplex, polytopeOne, polytopeTwo, entry123);
      }
   }

   private final Vector3d supportDirection = new Vector3d();

   public Vector3d computeExpandedPolytope()
   {
      double mu = Double.POSITIVE_INFINITY; // Upper bound for the square penetration depth.
      Vector3d closestPointToOrigin = null;
      
      while (true)
      {
         ExpandingPolytopeEntry triangleEntryToExpand = triangleEntryQueue.poll();
         if (listener != null)
            listener.polledEntryToExpand(triangleEntryToExpand);

         boolean closeEnough = false;

         if (!triangleEntryToExpand.isObsolete())
         {
            closestPointToOrigin = triangleEntryToExpand.getClosestPointToOrigin();

            supportDirection.set(closestPointToOrigin);

            PolytopeVertex supportingVertexA = polytopeA.getSupportingVertex(supportDirection);
            supportDirection.negate();
            PolytopeVertex supportingVertexB = polytopeB.getSupportingVertex(supportDirection);

            Vector3d w = new Vector3d();
            w.sub(supportingVertexA.getPosition(), supportingVertexB.getPosition());

            if (listener != null)
            {
               listener.computedSupportingVertices(supportingVertexA, supportingVertexB, w);
            }

            double vDotW = closestPointToOrigin.dot(w);
            double lengthSquared = closestPointToOrigin.lengthSquared();
            mu = Math.min(mu, vDotW * vDotW / lengthSquared);
            closeEnough = (mu <= (1.0 + epsilonRelative) * (1.0 + epsilonRelative) * lengthSquared);

            if (listener != null)
            {
               listener.computedCloseEnough(vDotW, lengthSquared, mu, closeEnough);
            }

            if (!closeEnough)
            {
               // Blow up the current polytope by adding vertex w.
               computeSilhouetteFromW(triangleEntryToExpand, w, edgeList);

               if (listener != null)
               {
                  listener.computedSilhouetteFromW(edgeList);
               }

               // edgeList now is the entire silhouette of the current polytope as seen from w.

               int numberOfEdges = edgeList.getNumberOfEdges();
//               if (numberOfEdges < 3) throw new RuntimeException("Should have at least three edges, no?");
               
               ExpandingPolytopeEntry firstNewEntry = null;
               ExpandingPolytopeEntry previousEntry = null;
               Point3d wPoint = new Point3d(w);
               
               for (int edgeIndex = 0; edgeIndex < numberOfEdges; edgeIndex++)
               {
                  ExpandingPolytopeEdge edge = edgeList.getEdge(edgeIndex);

                  ExpandingPolytopeEntry sentry = edge.getEntry();
                  int sentryEdgeIndex = edge.getEdgeIndex();
                  int nextIndex = (sentryEdgeIndex + 1) % 3;

                  ExpandingPolytopeEntry newEntry = new ExpandingPolytopeEntry(sentry.getVertex(nextIndex), sentry.getVertex(sentryEdgeIndex), wPoint);
                  newEntry.setAdjacentTriangle(0, sentry, sentryEdgeIndex);
                  sentry.setAdjacentTriangle(sentryEdgeIndex, newEntry, 0);
                  
                  if (previousEntry != null) 
                  {
                     //TODO: Verify if these are correct:
                     newEntry.setAdjacentTriangle(1, previousEntry, 2);
                     newEntry.setAdjacentTriangle(2, previousEntry, 1);
                  }
                  
                  if (edgeIndex == 0) firstNewEntry = newEntry;
                  
                  if (listener != null)
                     listener.createdNewEntry(newEntry);
                  
                  if (newEntry.isAffinelyDependent())
                  {
                     return closestPointToOrigin;
                  }

                  double newEntryClosestDistanceSquared = newEntry.getClosestPointToOrigin().lengthSquared();
                  if ((newEntry.closestIsInternal()) && (closestPointToOrigin.lengthSquared() <= newEntryClosestDistanceSquared)
                        && (newEntryClosestDistanceSquared <= mu))
                  {
                     triangleEntryQueue.add(newEntry);
                     
                     if (listener != null)
                        listener.addedNewEntryToQueue(newEntry);
                  }
                  
                  previousEntry = newEntry;
               }
            }
         }

         if ((closeEnough) || (triangleEntryQueue.isEmpty()) || (triangleEntryQueue.peek().getClosestPointToOrigin().lengthSquared() > mu))
         {
            if (listener != null)
            {
               listener.foundMinimumPenetrationVector(closestPointToOrigin);
            }
            return closestPointToOrigin;
         }
      }
   }

   public static void computeSilhouetteFromW(ExpandingPolytopeEntry triangleEntrySeenByW, Vector3d w,  ExpandingPolytopeEdgeList edgeListToPack)
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
