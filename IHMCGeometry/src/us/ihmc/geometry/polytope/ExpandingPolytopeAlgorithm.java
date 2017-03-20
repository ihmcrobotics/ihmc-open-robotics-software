package us.ihmc.geometry.polytope;

import java.util.PriorityQueue;
import java.util.Set;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import gnu.trove.map.hash.THashMap;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class ExpandingPolytopeAlgorithm
{
   private final PriorityQueue<ExpandingPolytopeEntry> triangleEntryQueue = new PriorityQueue<ExpandingPolytopeEntry>();
   private final ExpandingPolytopeEdgeList edgeList = new ExpandingPolytopeEdgeList();

   private final THashMap<Point3D, Point3D> correspondingPointsOnA = new THashMap<>();
   private final THashMap<Point3D, Point3D> correspondingPointsOnB = new THashMap<>();

   private boolean polytopeIsRightHanded;
   private SupportingVertexHolder polytopeA;
   private SupportingVertexHolder polytopeB;

   private final double epsilonRelative;

   private ExpandingPolytopeAlgorithmListener listener;

   private final RecyclingArrayList<ExpandingPolytopeEntry> polytopeEntryPool = new RecyclingArrayList<>(ExpandingPolytopeEntry.class);

   private final Vector3D tempVector12 = new Vector3D();
   private final Vector3D tempVector13 = new Vector3D();
   private final Vector3D tempVector14 = new Vector3D();
   private final Vector3D tempVector12Cross13 = new Vector3D();

   public ExpandingPolytopeAlgorithm(double epsilonRelative)
   {
      this.epsilonRelative = epsilonRelative;
   }

   public void setExpandingPolytopeAlgorithmListener(ExpandingPolytopeAlgorithmListener listener)
   {
      this.listener = listener;
   }

   public void setPolytopes(SimplexPolytope simplex, SupportingVertexHolder polytopeOne, SupportingVertexHolder polytopeTwo)
   {
      polytopeEntryPool.clear();

      correspondingPointsOnA.clear();
      correspondingPointsOnB.clear();
      edgeList.clear();
      triangleEntryQueue.clear();

      this.polytopeA = polytopeOne;
      this.polytopeB = polytopeTwo;

      int numberOfPoints = simplex.getNumberOfPoints();
      if (numberOfPoints != 4)
         throw new RuntimeException("Implement for non tetrahedral simplex");

      Point3D pointOne = simplex.getPoint(0);
      Point3D pointTwo = simplex.getPoint(1);
      Point3D pointThree = simplex.getPoint(2);
      Point3D pointFour = simplex.getPoint(3);

      tempVector12.sub(pointTwo, pointOne);
      tempVector13.sub(pointThree, pointOne);
      tempVector14.sub(pointFour, pointOne);
      tempVector12Cross13.cross(tempVector12, tempVector13);
      double tripleProduct = tempVector12Cross13.dot(tempVector14);
      
      if (tripleProduct >= 0.0)
      {
         polytopeIsRightHanded = true;
      }
      else
      {
         polytopeIsRightHanded = false;
      }

      //TODO: Get rid of this if you can...
      if (Math.abs(tripleProduct) < 1e-10)
      {
         System.err.println("pointOne = " + pointOne);
         System.err.println("pointTwo = " + pointTwo);
         System.err.println("pointThree = " + pointThree);
         System.err.println("pointFour = " + pointFour);
         
         throw new RuntimeException("tripleProduct < 1e-10");
      }
      
      correspondingPointsOnA.put(pointOne, simplex.getCorrespondingPointOnPolytopeA(pointOne));
      correspondingPointsOnA.put(pointTwo, simplex.getCorrespondingPointOnPolytopeA(pointTwo));
      correspondingPointsOnA.put(pointThree, simplex.getCorrespondingPointOnPolytopeA(pointThree));
      correspondingPointsOnA.put(pointFour, simplex.getCorrespondingPointOnPolytopeA(pointFour));

      correspondingPointsOnB.put(pointOne, simplex.getCorrespondingPointOnPolytopeB(pointOne));
      correspondingPointsOnB.put(pointTwo, simplex.getCorrespondingPointOnPolytopeB(pointTwo));
      correspondingPointsOnB.put(pointThree, simplex.getCorrespondingPointOnPolytopeB(pointThree));
      correspondingPointsOnB.put(pointFour, simplex.getCorrespondingPointOnPolytopeB(pointFour));

      ExpandingPolytopeEntry entry123 = polytopeEntryPool.add();
      ExpandingPolytopeEntry entry324 = polytopeEntryPool.add();
      ExpandingPolytopeEntry entry421 = polytopeEntryPool.add();
      ExpandingPolytopeEntry entry134 = polytopeEntryPool.add();

      entry123.reset(pointOne, pointTwo, pointThree);
      entry324.reset(pointThree, pointTwo, pointFour);
      entry421.reset(pointFour, pointTwo, pointOne);
      entry134.reset(pointOne, pointThree, pointFour);

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

      if (entry123.closestIsInternal())
         triangleEntryQueue.add(entry123);
      if (entry324.closestIsInternal())
         triangleEntryQueue.add(entry324);
      if (entry421.closestIsInternal())
         triangleEntryQueue.add(entry421);
      if (entry134.closestIsInternal())
         triangleEntryQueue.add(entry134);

      if (listener != null)
      {
         listener.setPolytopes(simplex, polytopeOne, polytopeTwo, entry123);
      }
   }

   private final Vector3D supportDirection = new Vector3D();

   public void computeExpandedPolytope(Vector3D separatingVectorToPack, Point3D closestPointOnA, Point3D closestPointOnB)
   {
      double mu = Double.POSITIVE_INFINITY; // Upper bound for the square penetration depth.
      Vector3D closestPointToOrigin = null;
      ExpandingPolytopeEntry closestTriangleToOrigin = null;

      int numberOfIterations = 0;
      while (true)
      {
         //TODO: Stop the looping!
         numberOfIterations++;

         ExpandingPolytopeEntry triangleEntryToExpand = triangleEntryQueue.poll();
         if (listener != null)
            listener.polledEntryToExpand(triangleEntryToExpand);

         boolean closeEnough = false;

         if (!triangleEntryToExpand.isObsolete())
         {
            closestPointToOrigin = triangleEntryToExpand.getClosestPointToOrigin();
            closestTriangleToOrigin = triangleEntryToExpand;

            if (closestPointToOrigin.lengthSquared() < 1e-6)
            {
               // Create the support direction based on the normal of the triangle in case the closestPoint is slightly on the wrong side
               // to be robust to numerical round off errors.
               tempVector12.sub(triangleEntryToExpand.getVertex(1), triangleEntryToExpand.getVertex(0));
               tempVector13.sub(triangleEntryToExpand.getVertex(2), triangleEntryToExpand.getVertex(0));
               
               if (polytopeIsRightHanded) 
               {
                  supportDirection.cross(tempVector13, tempVector12);                  
               }
               else
               {
                  supportDirection.cross(tempVector12, tempVector13);                  
               }
            }
            else
            {
               supportDirection.set(closestPointToOrigin);
            }
            
            Point3D supportingVertexA = polytopeA.getSupportingVertex(supportDirection);
            supportDirection.negate();
            Point3D supportingVertexB = polytopeB.getSupportingVertex(supportDirection);

            Vector3D w = new Vector3D();
            w.sub(supportingVertexA, supportingVertexB);

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
               ExpandingPolytopeSilhouetteConstructor.computeSilhouetteFromW(triangleEntryToExpand, w, edgeList);

               if (listener != null)
               {
                  listener.computedSilhouetteFromW(edgeList);
               }

               // edgeList now is the entire silhouette of the current polytope as seen from w.

               ExpandingPolytopeEntry firstNewEntry = null;
               Point3D wPoint = new Point3D(w);
               correspondingPointsOnA.put(wPoint, supportingVertexA);
               correspondingPointsOnB.put(wPoint, supportingVertexB);

               int numberOfEdges = edgeList.getNumberOfEdges();

               //TODO: Recycle the trash here...
               THashMap<Point3D, ExpandingPolytopeEntry[]> mapFromStitchVertexToTriangles = new THashMap<>();

               for (int edgeIndex = 0; edgeIndex < numberOfEdges; edgeIndex++)
               {
                  ExpandingPolytopeEdge edge = edgeList.getEdge(edgeIndex);

                  ExpandingPolytopeEntry sentry = edge.getEntry();
                  int sentryEdgeIndex = edge.getEdgeIndex();
                  int nextIndex = (sentryEdgeIndex + 1) % 3;

                  Point3D sentryVertexOne = sentry.getVertex(sentryEdgeIndex);
                  Point3D sentryVertexTwo = sentry.getVertex(nextIndex);

//                  ExpandingPolytopeEntry newEntry = polytopeEntryPool.add();
//                  newEntry.reset(sentryVertexTwo, sentryVertexOne, wPoint);
                  
                  ExpandingPolytopeEntry newEntry = polytopeEntryPool.add();
                  newEntry.reset(sentryVertexTwo, sentryVertexOne, wPoint);
//                  ExpandingPolytopeEntry newEntry = new ExpandingPolytopeEntry(sentryVertexTwo, sentryVertexOne, wPoint);

                  if (newEntry.isAffinelyDependent())
                  {
                     computeClosestPointsOnAAndB(closestTriangleToOrigin, closestPointOnA, closestPointOnB);
                     if (listener != null)
                     {
                        listener.foundMinimumPenetrationVector(closestPointToOrigin, closestPointOnA, closestPointOnB);
                     }
                     separatingVectorToPack.set(closestPointToOrigin);
                     return;
                  }

                  ExpandingPolytopeEntry[] twoTriangles = getOrCreateTwoTriangleArray(mapFromStitchVertexToTriangles, sentryVertexOne);
                  storeNewEntry(newEntry, twoTriangles);
                  twoTriangles = getOrCreateTwoTriangleArray(mapFromStitchVertexToTriangles, sentryVertexTwo);
                  storeNewEntry(newEntry, twoTriangles);

                  newEntry.setAdjacentTriangle(0, sentry, sentryEdgeIndex);
                  sentry.setAdjacentTriangle(sentryEdgeIndex, newEntry, 0);

                  if (edgeIndex == 0)
                     firstNewEntry = newEntry;

                  if (listener != null)
                     listener.createdNewEntry(newEntry);

                  double newEntryClosestDistanceSquared = newEntry.getClosestPointToOrigin().lengthSquared();
                  if ((newEntry.closestIsInternal()) && (closestPointToOrigin.lengthSquared() <= newEntryClosestDistanceSquared)
                        && (newEntryClosestDistanceSquared <= mu))
                  {
                     triangleEntryQueue.add(newEntry);

                     if (listener != null)
                        listener.addedNewEntryToQueue(newEntry);
                  }
               }

               // Stich em up:
               Set<Point3D> keySet = mapFromStitchVertexToTriangles.keySet();

               for (Point3D stitchVertex : keySet)
               {
                  ExpandingPolytopeEntry[] trianglesToStitch = mapFromStitchVertexToTriangles.get(stitchVertex);
                  if ((trianglesToStitch[0] == null) || (trianglesToStitch[1] == null))
                  {
                     throw new RuntimeException("Stitch triangle is null");
                  }

                  ExpandingPolytopeEntry triangleOne = trianglesToStitch[0];
                  ExpandingPolytopeEntry triangleTwo = trianglesToStitch[1];

                  boolean stitchedTriangles = triangleOne.setAdjacentTriangleIfPossible(triangleTwo);
                  if (!stitchedTriangles)
                  {
                     throw new RuntimeException("Failed to stitch triangles!!");
                  }
               }

               if (listener != null)
               {
                  listener.expandedPolytope(firstNewEntry);
               }
            }
         }

         if ((numberOfIterations > 1000) || (closeEnough) || (triangleEntryQueue.isEmpty())
               || (triangleEntryQueue.peek().getClosestPointToOrigin().lengthSquared() > mu))
         {
            computeClosestPointsOnAAndB(closestTriangleToOrigin, closestPointOnA, closestPointOnB);

            if (listener != null)
            {
               listener.foundMinimumPenetrationVector(closestPointToOrigin, closestPointOnA, closestPointOnB);
            }
            separatingVectorToPack.set(closestPointToOrigin);
            return;
         }
      }
   }

   private void storeNewEntry(ExpandingPolytopeEntry newEntry, ExpandingPolytopeEntry[] twoTriangles)
   {
      if (twoTriangles[0] == null)
         twoTriangles[0] = newEntry;
      else if (twoTriangles[1] != null)
      {
         throw new RuntimeException("twoTriangles[1] != null");
      }
      else
      {
         twoTriangles[1] = newEntry;
      }
   }

   private ExpandingPolytopeEntry[] getOrCreateTwoTriangleArray(THashMap<Point3D, ExpandingPolytopeEntry[]> mapFromStitchVertexToTriangles,
         Point3D sentryVertexOne)
   {
      ExpandingPolytopeEntry[] twoTriangleArray = mapFromStitchVertexToTriangles.get(sentryVertexOne);
      if (twoTriangleArray == null)
      {
         twoTriangleArray = new ExpandingPolytopeEntry[2];
         mapFromStitchVertexToTriangles.put(sentryVertexOne, twoTriangleArray);
      }

      return twoTriangleArray;
   }

   private final Point3D tempPoint = new Point3D();

   private void computeClosestPointsOnAAndB(ExpandingPolytopeEntry closestTriangleToOrigin, Point3D closestPointOnA, Point3D closestPointOnB)
   {
      closestPointOnA.set(0.0, 0.0, 0.0);
      closestPointOnB.set(0.0, 0.0, 0.0);

      for (int i = 0; i < 3; i++)
      {
         Point3D vertex = closestTriangleToOrigin.getVertex(i);
         double lambda = closestTriangleToOrigin.getLambda(i);

         Point3D pointOnA = correspondingPointsOnA.get(vertex);
         Point3D pointOnB = correspondingPointsOnB.get(vertex);

         tempPoint.set(pointOnA);
         tempPoint.scale(lambda);
         closestPointOnA.add(tempPoint);

         tempPoint.set(pointOnB);
         tempPoint.scale(lambda);
         closestPointOnB.add(tempPoint);
      }

   }

}
