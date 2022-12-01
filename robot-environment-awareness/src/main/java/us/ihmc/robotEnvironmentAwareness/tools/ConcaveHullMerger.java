package us.ihmc.robotEnvironmentAwareness.tools;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * Class for merging two convex hulls. Goes around the outside, adding a point at a time. Keeps
 * track of what is the outside working hull and the inside resting hull. Does a pre-filter to
 * eliminate points on a hull that are too close together and another pre-filter to move the points
 * slightly if there are duplicates between the two hulls or points of the second are on the edge of
 * the first. Fills in holes that are formed when two hulls are merged.
 */
public class ConcaveHullMerger
{
   private static final double minimumDistanceBetweenHullPointsSquared = 0.005 * 0.005;
   private static final double minimumDistanceSquared = 0.002 * 0.002;
   private static final double amountToMove = 0.0025;
   private static final double maximumDotProductBetweenSkinnyEdges = Math.cos(Math.toRadians(0.5));
   private static final Random randomForWiggling = new Random(238949L);

   //TODO: What should this be set to? Should we make it be a parameter?
   private static final double depthThresholdForConcaveHullDecomposition = 0.001;

   //TODO: Check with Sylvain to see if this already exists somewhere else. If not, clean it up nicely and move it somewhere.
   //TODO: Maybe call this class PlanarRegionMergeTools
   public static ArrayList<PlanarRegion> mergePlanarRegions(PlanarRegion regionOne, PlanarRegion regionTwo, double maximumProjectionDistance)
   {
      return mergePlanarRegions(regionOne, regionTwo, maximumProjectionDistance, null);
   }

   /**
    * Merges two PlanarRegions and returns the merged PlanarRegion by projecting regionTwo onto the
    * plan of regionOne, finding the intersection of their concave hull on the plane of regionOne, and
    * then forming a new PlanarRegion using that new concave hull. If the concave hulls of the two
    * PlanarRegions do not intersect, returns null.
    * 
    * @param regionOne First PlanarRegion to merge.
    * @param regionTwo Second PlanarRegion to merge.
    * @return List of merged planarRegions. The list will be empty if the PlanarRegions did not
    *         intersect. Returns null if both PlanarRegions were invalid somehow.
    */
   public static ArrayList<PlanarRegion> mergePlanarRegions(PlanarRegion regionOne, PlanarRegion regionTwo, double maximumProjectionDistance,
                                                            ConcaveHullMergerListener listener)
   {
      RigidBodyTransform transformTwoToWorld = new RigidBodyTransform();
      regionTwo.getTransformToWorld(transformTwoToWorld);

      RigidBodyTransform transformWorldToOne = new RigidBodyTransform();
      regionOne.getTransformToLocal(transformWorldToOne);

      RigidBodyTransform transformTwoToOne = new RigidBodyTransform(transformWorldToOne);
      transformTwoToOne.multiply(transformTwoToWorld);

      List<? extends Point2DReadOnly> concaveHullTwoVertices = regionTwo.getConcaveHull();
      ArrayList<Point2D> concaveHullTwoVerticesTransformed = new ArrayList<Point2D>(concaveHullTwoVertices.size());

      for (int i = 0; i < concaveHullTwoVertices.size(); i++)
      {
         Point3D point3D = new Point3D(concaveHullTwoVertices.get(i));
         transformTwoToOne.transform(point3D);
         if (Math.abs(point3D.getZ()) > maximumProjectionDistance)
            return new ArrayList<PlanarRegion>();
         concaveHullTwoVerticesTransformed.add(new Point2D(point3D));
      }

      List<? extends Point2DReadOnly> concaveHullOneVertices = regionOne.getConcaveHull();

      List<Point2D> mergedConcaveHull = mergeConcaveHulls(concaveHullOneVertices, concaveHullTwoVerticesTransformed, listener);
      if (mergedConcaveHull == null)
         return null;

      if (mergedConcaveHull.isEmpty())
         return new ArrayList<PlanarRegion>();

      ArrayList<ConvexPolygon2D> newPolygonsFromConcaveHull = new ArrayList<ConvexPolygon2D>();

      ConcaveHullDecomposition.recursiveApproximateDecomposition(mergedConcaveHull, depthThresholdForConcaveHullDecomposition, newPolygonsFromConcaveHull);

      RigidBodyTransform transformOneToWorld = new RigidBodyTransform();
      regionOne.getTransformToWorld(transformOneToWorld);

      PlanarRegion planarRegion = new PlanarRegion(transformOneToWorld, mergedConcaveHull, newPolygonsFromConcaveHull);
      planarRegion.setRegionId(regionOne.getRegionId());

      ArrayList<PlanarRegion> planarRegions = new ArrayList<PlanarRegion>();
      planarRegions.add(planarRegion);

      return planarRegions;
   }

   /**
    * Merges two ConcaveHulls. ConcaveHulls are assumed to be in clockwise order. Will fill holes. If
    * the concave hulls do not intersect at all (including one fully inside the other) then returns
    * null. Preprocesses both hulls to remove slivers, nearly or fully colinear points, etc. If after
    * preprocessing, both hulls are invalid, returns null. If one hull is invalid, returns the other.
    * If there is no intersection of the preprocessed hulls, returns an empty list.
    * 
    * @param hullOne One hull to merge.
    * @param hullTwo The other hull to merge.
    * @return Merged hull. Returns null if they do not intersect.
    */
   public static List<Point2D> mergeConcaveHulls(List<? extends Point2DReadOnly> hullOneIn, List<? extends Point2DReadOnly> hullTwoIn, ConcaveHullMergerListener listener)
   {
      List<Point2D> hullOneInCopy = hullOneIn.stream().map(Point2D::new).collect(Collectors.toList());
      List<Point2D> hullTwoInCopy = hullTwoIn.stream().map(Point2D::new).collect(Collectors.toList());

      if (listener != null)
      {
         listener.originalHulls(hullOneInCopy, hullTwoInCopy);
      }



      // wiggle close points away from each other
      double minimumDistance = 1e-5;
      double infinitesimalDistance = 1e-14;
      Vector2D tempVector = null;
      for (Point2D a : hullOneInCopy)
      {
         for (Point2D b : hullTwoInCopy)
         {
            double distance = a.distance(b);
            if (distance < infinitesimalDistance)
            {
               double randomAngle = randomForWiggling.nextDouble() * 2.0 * Math.PI;
               double moveAmount = minimumDistance;
               b.addX(moveAmount * Math.cos(randomAngle));
               b.addY(moveAmount * Math.sin(randomAngle));
            }
            else if (distance < minimumDistance)
            {
               if (tempVector == null) tempVector = new Vector2D();

               double moveAmount = (minimumDistance - distance) / 2.0;

               tempVector.sub(a, b);  // moving the points directly away from each other the required amount
               tempVector.normalize();
               tempVector.scale(moveAmount);
               a.addX(tempVector.getX());
               a.addY(tempVector.getY());

               tempVector.negate();
               b.addX(tempVector.getX());
               b.addY(tempVector.getY());
            }
         }
      }

      List<Point2D> originalHullOne = hullOneInCopy;
      List<Point2D> originalHullTwo = hullTwoInCopy;

      List<Point2D> hullOneList = hullOneInCopy;
      List<Point2D> hullTwoList = hullTwoInCopy;

      hullOneList = preprocessHullByRemovingPoints(hullOneList);
      hullTwoList = preprocessHullByRemovingPoints(hullTwoList);

      boolean hullOneListIsValid = ((hullOneList != null) && (hullOneList.size() >= 3) && (!isConcaveHullSelfIntersecting(hullOneList)));
      boolean hullTwoListIsValid = ((hullTwoList != null) && (hullTwoList.size() >= 3) && (!isConcaveHullSelfIntersecting(hullTwoList)));

      if ((!hullOneListIsValid) && (!hullTwoListIsValid))
      {
         notifyListenerHullsAreInvalid(listener, hullOneList, hullTwoList);
         return null;
      }

      if (!hullTwoListIsValid)
      {
         notifyListenerHullsAreInvalid(listener, hullTwoList);
         return hullOneList;
      }

      if (!hullOneListIsValid)
      {
         notifyListenerHullsAreInvalid(listener, hullOneList);
         return hullTwoList;
      }

      hullTwoList = preprocessHullTwoToMoveDuplicatesOrOnEdges(hullOneList, hullTwoList);
      hullOneList = preprocessHullTwoToMoveDuplicatesOrOnEdges(hullTwoList, hullOneList);

      BoundingBox2D hullOneBoundingBox = createBoundingBox(hullOneList);
      BoundingBox2D hullTwoBoundingBox = createBoundingBox(hullTwoList);

      if (!hullOneBoundingBox.intersectsExclusive(hullTwoBoundingBox))
         return new ArrayList<Point2D>();

      BoundingBox2D unionOfOriginalBoundingBoxes = BoundingBox2D.union(hullOneBoundingBox, hullTwoBoundingBox);

      if (listener != null)
      {
         listener.preprocessedHull(hullOneList, hullTwoList);
      }

      // Find a point that is guaranteed to be on the outside by finding one with the lowest x. 
      // In case of ties, first one should be fine
      List<Point2D> workingHull = hullOneList;
      List<Point2D> restingHull = hullTwoList;
      double minX = Double.POSITIVE_INFINITY;
      Point2D startingVertex = null;
      int workingHullIndex = -1;

      for (int i = 0; i < hullOneList.size(); i++)
      {
         Point2D vertex = hullOneList.get(i);
         if (vertex.getX() < minX)
         {
            minX = vertex.getX();
            startingVertex = vertex;
            workingHullIndex = i;
         }
      }

      for (int i = 0; i < hullTwoList.size(); i++)
      {
         Point2D vertex = hullTwoList.get(i);
         if (vertex.getX() < minX)
         {
            minX = vertex.getX();
            startingVertex = vertex;
            workingHullIndex = i;
            workingHull = hullTwoList;
            restingHull = hullOneList;
         }
      }

      if (listener != null)
      {
         listener.foundStartingVertexAndWorkingHull(startingVertex, workingHull, (workingHull == hullOneList));
      }

      // Go in order until you hit an edge on the other convex hull or get to the end.
      boolean foundAnIntersection = false;
      ArrayList<Point2D> mergedVertices = new ArrayList<Point2D>();
      Point2D workingVertex = startingVertex;

      int nextIndex = (workingHullIndex + 1) % workingHull.size();

      int edgeStartIndexToSkip = -1;
      int count = 0;
      int veerticesInMergedHull = 0;
      boolean exitedWithoutALoop = false;

      int totalNumberOfVertices = hullOneList.size() + hullTwoList.size();
      while ((count <= totalNumberOfVertices) && (veerticesInMergedHull < 4 * totalNumberOfVertices))
      {
         mergedVertices.add(workingVertex);
         veerticesInMergedHull++;

         Point2D nextVertex = workingHull.get(nextIndex);
         LineSegment2D workingEdge = new LineSegment2D(workingVertex, nextVertex);

         if (listener != null)
         {
            listener.consideringWorkingEdge(workingEdge, (workingHull == hullOneList));
         }

         Pair<Integer, Point2D> intersection = findClosestIntersection(workingEdge, restingHull, edgeStartIndexToSkip);
         if (intersection == null)
         {
            workingVertex = nextVertex;

            if (workingVertex == startingVertex)
            {
               exitedWithoutALoop = true;
               break;
            }

            count++;
            nextIndex = (nextIndex + 1) % workingHull.size();
            edgeStartIndexToSkip = -1;
         }
         else
         {
            // If intersect an edge, take the path to the left.
            foundAnIntersection = true;
            Point2D intersectionPoint = intersection.getRight();

            if (listener != null)
            {
               listener.foundIntersectionPoint(intersectionPoint, (workingHull == hullOneList));
            }

            edgeStartIndexToSkip = nextIndex - 1;

            if (edgeStartIndexToSkip < 0)
            {
               edgeStartIndexToSkip = workingHull.size() - 1;
            }

            List<Point2D> temp = workingHull;
            workingHull = restingHull;
            restingHull = temp;

            nextIndex = intersection.getLeft();
            workingVertex = intersectionPoint;
         }
      }

      if (!exitedWithoutALoop)
      {
         LogTools.error("mergedVertices.size() > hullOne.length + hullTwo.length. Something got looped!");
         if (listener != null)
         {
            listener.hullGotLooped(originalHullOne, originalHullTwo, mergedVertices);
         }
      }

      BoundingBox2D finalBoundingBox = createBoundingBox(mergedVertices);
      if (boundingBoxShrunk(unionOfOriginalBoundingBoxes, finalBoundingBox))
         return new ArrayList<Point2D>();

      if (!foundAnIntersection)
      {
         if (unionOfOriginalBoundingBoxes.equals(finalBoundingBox))
         {

            // Check to make sure the smaller concave hull is inside the bigger one. 
            // At this point can do that by just finding one vertex that is either inside or one that is outside.
            // Since there were no crossings, they all must be inside or all outside.
            if ((!isPointInsideConcaveHull(hullTwoList.get(0), hullOneList)) && (!isPointInsideConcaveHull(hullOneList.get(0), hullTwoList)))
            {
               return new ArrayList<>();
            }
         }
      }

      return mergedVertices;
   }

   private static void notifyListenerHullsAreInvalid(ConcaveHullMergerListener listener, List<Point2D> hullList)
   {
      if (listener != null)
      {
         listener.hullIsInvalid(hullList);
      }
   }

   private static void notifyListenerHullsAreInvalid(ConcaveHullMergerListener listener, List<Point2D> hullListA, List<Point2D> hullListB)
   {
      if (listener != null)
      {
         listener.hullsAreInvalid(hullListA, hullListB);
      }
   }

   /**
    * Checks if a concave hull is self intersecting by seeing if any edge intersects any other edge,
    * except for itself and the one right before and right after it. We do not have to check those two
    * edges since we remove slivers before calling this method. Lists with 3 or fewer points or a null
    * list return false.
    * 
    * @param concaveHull Hull to check.
    * @return Whether or not the hull is self intersecting.
    */
   public static boolean isConcaveHullSelfIntersecting(List<Point2D> concaveHull)
   {
      if (concaveHull == null)
         return false;

      if (concaveHull.size() < 4)
         return false;

      for (int i = 0; i < concaveHull.size(); i++)
      {
         Point2D currentVertex = concaveHull.get(i);
         Point2D nextVertex = concaveHull.get(nextIndex(i, concaveHull));
         LineSegment2D edge = new LineSegment2D(currentVertex, nextVertex);

         int j = nextIndex(nextIndex(i, concaveHull), concaveHull);
         while (true)
         {
            Point2D firstVertexToCheck = concaveHull.get(j);
            Point2D secondVertexToCheck = concaveHull.get(nextIndex(j, concaveHull));

            if (secondVertexToCheck == currentVertex)
               break;

            LineSegment2D edgeToCheck = new LineSegment2D(firstVertexToCheck, secondVertexToCheck);

            boolean edgesIntersect = edge.intersectionWith(edgeToCheck, null);
            if (edgesIntersect)
               return true;

            j = nextIndex(j, concaveHull);
         }
      }

      return false;
   }

   public static List<Point2D> preprocessHullByRemovingPoints(List<Point2D> hull)
   {
      if ((hull == null) || (hull.size() < 3))
         return hull;

      int incomingHullSize = hull.size();

      hull = preprocessToRemovePointsThatAreTooCloseTogether(hull);
      hull = preprocessToRemoveSliversWithSmallAngles(hull);

      if (hull.size() < incomingHullSize)
         return preprocessHullByRemovingPoints(hull);

      hull = preprocessToRemoveColinearPoints(hull);

      return hull;
   }

   private static List<Point2D> preprocessToRemovePointsThatAreTooCloseTogether(List<Point2D> hull)
   {
      // TODO: Also remove ones that produce too little area by having a thin sliver. These will be problematic because 
      // other points that are close to the edges and get moved a little might get all tangled.
      if ((hull == null) || (hull.size() < 3))
         return null;

      ArrayList<Point2D> processedHull = new ArrayList<Point2D>();
      processedHull.add(hull.get(0));

      Point2D previousVertexAdded = hull.get(0);

      for (int i = 1; i < hull.size(); i++)
      {
         Point2D vertex = hull.get(i);

         if (previousVertexAdded.distanceSquared(vertex) >= minimumDistanceBetweenHullPointsSquared)
         {
            if (i == hull.size() - 1)
            {
               if (hull.get(0).distanceSquared(vertex) >= minimumDistanceBetweenHullPointsSquared)
               {
                  processedHull.add(vertex);
                  previousVertexAdded = vertex;
               }
            }
            else
            {
               processedHull.add(vertex);
               previousVertexAdded = vertex;
            }
         }
      }

      return processedHull;
   }

   private static List<Point2D> preprocessToRemoveSliversWithSmallAngles(List<Point2D> hull)
   {
      if ((hull == null) || (hull.size() < 3))
         return null;

      ArrayList<Point2D> processedHull = new ArrayList<Point2D>();

      for (int i = 0; i < hull.size(); i++)
      {
         Point2D vertex = hull.get(i);
         Point2D previousVertex = hull.get(previousIndex(i, hull));
         Point2D nextVertex = hull.get(nextIndex(i, hull));

         Vector2D v1 = new Vector2D();
         v1.sub(nextVertex, vertex);

         Vector2D v2 = new Vector2D();
         v2.sub(previousVertex, vertex);

         double dotProductBetweenUnitVectors = (v1.dot(v2) / (v1.length() * v2.length()));

         if (dotProductBetweenUnitVectors > maximumDotProductBetweenSkinnyEdges)
         {
            continue;
         }

         if (nextVertex.distanceSquared(previousVertex) < minimumDistanceBetweenHullPointsSquared)
         {
            continue;
         }

         processedHull.add(vertex);

      }

      return processedHull;
   }

   private static List<Point2D> preprocessToRemoveColinearPoints(List<Point2D> hull)
   {
      if ((hull == null) || (hull.size() < 3))
         return null;

      ArrayList<Point2D> processedHull = new ArrayList<Point2D>();

      for (int i = 0; i < hull.size(); i++)
      {
         Point2D vertex = hull.get(i);
         Point2D previousVertex = hull.get(previousIndex(i, hull));
         Point2D nextVertex = hull.get(nextIndex(i, hull));

         Vector2D v1 = new Vector2D();
         v1.sub(nextVertex, vertex);

         Vector2D v2 = new Vector2D();
         v2.sub(previousVertex, vertex);

         double dotProductBetweenUnitVectors = (v1.dot(v2) / (v1.length() * v2.length()));

         if (dotProductBetweenUnitVectors < -maximumDotProductBetweenSkinnyEdges)
         {
            continue;
         }

         processedHull.add(vertex);

      }

      return processedHull;
   }

   private static int nextIndex(int index, List<Point2D> hull)
   {
      return (index + 1) % hull.size();
   }

   private static int previousIndex(int index, List<Point2D> hull)
   {
      return (index - 1 + hull.size()) % hull.size();
   }

   private static boolean boundingBoxShrunk(BoundingBox2D unionOfOriginalBoundingBoxes, BoundingBox2D finalBoundingBox)
   {
      if (unionOfOriginalBoundingBoxes.getMinX() < finalBoundingBox.getMinX())
         return true;
      if (unionOfOriginalBoundingBoxes.getMinY() < finalBoundingBox.getMinY())
         return true;

      if (unionOfOriginalBoundingBoxes.getMaxX() > finalBoundingBox.getMaxX())
         return true;
      if (unionOfOriginalBoundingBoxes.getMaxY() > finalBoundingBox.getMaxY())
         return true;
      return false;
   }

   //TODO: This should be in a geometry tool somewhere or a constructor for BoundingBox2D.
   private static BoundingBox2D createBoundingBox(Point2D[] pointList)
   {
      double minX = Double.POSITIVE_INFINITY;
      double minY = Double.POSITIVE_INFINITY;
      double maxX = Double.NEGATIVE_INFINITY;
      double maxY = Double.NEGATIVE_INFINITY;

      for (Point2D point : pointList)
      {
         minX = Math.min(minX, point.getX());
         minY = Math.min(minY, point.getY());
         maxX = Math.max(maxX, point.getX());
         maxY = Math.max(maxY, point.getY());
      }

      return new BoundingBox2D(minX, minY, maxX, maxY);
   }

   private static BoundingBox2D createBoundingBox(Iterable<Point2D> pointList)
   {
      double minX = Double.POSITIVE_INFINITY;
      double minY = Double.POSITIVE_INFINITY;
      double maxX = Double.NEGATIVE_INFINITY;
      double maxY = Double.NEGATIVE_INFINITY;

      for (Point2D point : pointList)
      {
         minX = Math.min(minX, point.getX());
         minY = Math.min(minY, point.getY());
         maxX = Math.max(maxX, point.getX());
         maxY = Math.max(maxY, point.getY());
      }

      return new BoundingBox2D(minX, minY, maxX, maxY);
   }

   /**
    * Goes through hullTwo and moves any points that are either duplicates of those on hullOne or on
    * the edges of hullOne.
    * 
    * @param hullOne
    * @param hullTwo Hull to be processed.
    * @return Processed hull. Will be deep copy of hullTwo if there are no duplicates or points on the
    *         edges.
    */
   private static List<Point2D> preprocessHullTwoToMoveDuplicatesOrOnEdges(List<Point2D> hullOne, List<Point2D> hullTwo)
   {
      ArrayList<Point2D> newHull = new ArrayList<Point2D>();

      for (int i = 0; i < hullTwo.size(); i++)
      {
         newHull.add(preprocessHullPoint(hullOne, hullTwo.get(i)));
      }

      return newHull;
   }

   /**
    * Returns a moved version of the hullPoint if it is a copy of a vertex on the given hull or close
    * to the edge.
    * 
    * @param hullOne   Hull to check for closeness to.
    * @param hullPoint Point to move if necessary
    * @return Moved hull point.
    */
   private static Point2D preprocessHullPoint(List<Point2D> hullOne, Point2D hullPoint)
   {
      for (int i = 0; i < hullOne.size(); i++)
      {
         Point2D startVertex = hullOne.get(i);
         Point2D nextVertex = hullOne.get(nextIndex(i, hullOne));
         Point2D previousVertex = hullOne.get(previousIndex(i, hullOne));

         if (hullPoint.distanceSquared(startVertex) < minimumDistanceSquared)
         {
            Vector2D toPrevious = new Vector2D(previousVertex);
            toPrevious.sub(startVertex);
            toPrevious.normalize();
            toPrevious.set(-toPrevious.getY(), toPrevious.getX());
            toPrevious.scale(amountToMove);

            Vector2D toNext = new Vector2D(nextVertex);
            toNext.sub(startVertex);
            toNext.normalize();
            toNext.set(toNext.getY(), -toNext.getX());
            toNext.scale(amountToMove);

            Vector2D adjustment = new Vector2D(toPrevious);
            adjustment.add(toNext);

            Point2D adjustedPoint = new Point2D(hullPoint);
            adjustedPoint.add(adjustment);
            return adjustedPoint;
         }
      }

      for (int i = 0; i < hullOne.size(); i++)
      {
         Point2D startVertex = hullOne.get(i);
         Point2D nextVertex = hullOne.get(nextIndex(i, hullOne));

         LineSegment2D edge = new LineSegment2D(startVertex, nextVertex);
         if (edge.distanceSquared(hullPoint) < minimumDistanceSquared)
         {
            Vector2D adjustment = new Vector2D(nextVertex);
            adjustment.sub(startVertex);
            adjustment.normalize();

            adjustment.set(adjustment.getY(), -adjustment.getX());
            adjustment.scale(amountToMove);

            Point2D adjustedPoint = new Point2D(hullPoint);
            adjustedPoint.add(adjustment);
            return adjustedPoint;
         }
      }

      return new Point2D(hullPoint);
   }

   public static void printHull(String name, List<Point2D> hullPoints)
   {
      System.out.print("double[][]" + name + " = new double[][]{");

      for (int i = 0; i < hullPoints.size(); i++)
      {
         Point2D point = hullPoints.get(i);

         System.out.print("{" + point.getX() + ", " + point.getY() + "}");
         if (i == hullPoints.size() - 1)
         {
            System.out.println();
         }
         else
         {
            System.out.println(",");
         }
      }
      System.out.println("};");
   }

   /**
    * Finds the intersection in a concave hull as an edge crosses from outside to inside the hull. If
    * there are multiple intersections, returns the one that is closest to the edges start point. If
    * there is no intersection, returns null.
    * 
    * @param edge        Edge to check for crossing.
    * @param concaveHull Concave Hull to check if edge crosses it.
    * @return Point2D where the hull crosses and the second index of the concave hull edge that it
    *         intersects.
    */
   public static Pair<Integer, Point2D> findClosestIntersection(LineSegment2D edge, List<Point2D> concaveHull, int edgeStartIndexToSkip)
   {
      int previousIndex = concaveHull.size() - 1;
      int nextIndex = 0;

      int bestIndex = -1;
      Point2DBasics bestIntersection = null;
      double bestDistanceSquared = Double.POSITIVE_INFINITY;

      while (nextIndex < concaveHull.size())
      {
         if (previousIndex != edgeStartIndexToSkip)
         {
            Point2D previousVertex = concaveHull.get(previousIndex);
            Point2D nextVertex = concaveHull.get(nextIndex);

            LineSegment2D hullEdge = new LineSegment2D(previousVertex, nextVertex);

            Point2DBasics intersection = edge.intersectionWith(hullEdge);

            if (intersection != null)
            {
               double distanceSquared = edge.getFirstEndpoint().distanceSquared(intersection);

               if (distanceSquared < bestDistanceSquared)
               {
                  bestDistanceSquared = distanceSquared;
                  bestIntersection = intersection;
                  bestIndex = nextIndex;
               }
            }
         }

         previousIndex = nextIndex;
         nextIndex++;
      }

      if (bestIntersection == null)
         return null;

      return new ImmutablePair<Integer, Point2D>(bestIndex, new Point2D(bestIntersection));
   }

   /**
    * Checks if a point is inside a concave hull defined by a list of points. Does so by forming a ray
    * from the point and making sure it crosses the hull an odd number of times. Since a previous step
    * pushed points away from corners or vertices of the other hull, we don't need to worry too much
    * about edge cases here or small epsilon issues. The one place we do need to worry is if a ray
    * perfectly crosses the vertex of two edges to escape. In this case it might be counted 0, 1, or 2
    * times. So to avoid this problem, we use three different rays and let them vote on it. Not very
    * elegant, but should give a 6 sigma solution unless an adversary wants to throw us off. But for
    * our uses here, it is good enough.
    * 
    * @param pointToCheck Point2D to check if inside the concaveHull
    * @param concaveHull  ArrayList of Point2Ds that define the concaveHull.
    * @return true if the point is inside the concaveHull.
    */
   private static boolean isPointInsideConcaveHull(Point2D pointToCheck, List<Point2D> concaveHull)
   {
      //TODO: Check with Sylvain to see if he has well tested, robust versions of this method.
      //TODO: Add some failing test cases for the adversary edge cases and figure out a good fix.
      int previousIndex = concaveHull.size() - 1;
      int nextIndex = 0;

      Vector2D rayDirectionOne = new Vector2D(1.01, 0.02);
      Vector2D rayDirectionTwo = new Vector2D(-1.05, 2.09);
      Vector2D rayDirectionThree = new Vector2D(-1.07, -1.03);

      int crossingCountOne = countNumberOfCrossings(pointToCheck, concaveHull, previousIndex, nextIndex, rayDirectionOne);
      int crossingCountTwo = countNumberOfCrossings(pointToCheck, concaveHull, previousIndex, nextIndex, rayDirectionTwo);
      int crossingCountThree = countNumberOfCrossings(pointToCheck, concaveHull, previousIndex, nextIndex, rayDirectionThree);

      boolean isOddNumberOne = isOddNumber(crossingCountOne);
      boolean isOddNumberTwo = isOddNumber(crossingCountTwo);
      boolean isOddNumberThree = isOddNumber(crossingCountThree);

      return (voteOnBooleans(isOddNumberOne, isOddNumberTwo, isOddNumberThree));
   }

   private static boolean voteOnBooleans(boolean... votes)
   {
      int numberFor = 0;
      int numberAgainst = 0;

      for (boolean vote : votes)
      {
         if (vote)
            numberFor++;
         else
            numberAgainst++;
      }

      return (numberFor > numberAgainst);
   }

   private static int countNumberOfCrossings(Point2D pointToCheck, List<Point2D> concaveHull, int previousIndex, int nextIndex, Vector2D rayDirection)
   {
      int crossingCount = 0;
      while (nextIndex < concaveHull.size())
      {
         Point2D previousVertex = concaveHull.get(previousIndex);
         Point2D nextVertex = concaveHull.get(nextIndex);

         boolean rayIntersects = EuclidGeometryTools.doRay2DAndLineSegment2DIntersect(pointToCheck, rayDirection, previousVertex, nextVertex);

         if (rayIntersects)
         {
            crossingCount++;
         }

         previousIndex = nextIndex;
         nextIndex++;
      }
      return crossingCount;
   }

   private static boolean isOddNumber(int number)
   {
      return (number % 2 != 0);
   }

}
