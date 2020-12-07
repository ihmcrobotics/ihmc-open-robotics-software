package us.ihmc.robotEnvironmentAwareness.tools;

import java.util.List;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.tuple2D.Point2D;

public interface ConcaveHullMergerListener
{

   public abstract void originalHulls(List<Point2D> hullOne, List<Point2D> hullTwo);

   public abstract void preprocessedHull(List<Point2D> hullOne, List<Point2D> hullTwo);

   public abstract void foundStartingVertexAndWorkingHull(Point2D startingVertex, List<Point2D> workingHull, boolean workingHullIsOne);

   public abstract void consideringWorkingEdge(LineSegment2D workingEdge, boolean workingHullIsOne);

   public abstract void foundIntersectionPoint(Point2D intersectionPoint, boolean workingHullIsOne);

   public abstract void hullGotLooped(List<Point2D> hullOne, List<Point2D> hullTwo, List<Point2D> mergedVertices);

   public abstract void hullIsInvalid(List<Point2D> invalidHull);

   public abstract void hullsAreInvalid(List<Point2D> invalidHullA, List<Point2D> invalidHullB);


}