package us.ihmc.robotEnvironmentAwareness.tools;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.tuple2D.Point2D;

import java.util.List;

public interface ConcaveHullMergerListenerAdapter extends ConcaveHullMergerListener
{
   @Override
   public default void originalHulls(List<Point2D> hullOne, List<Point2D> hullTwo)
   {

   }

   @Override
   public default void preprocessedHull(List<Point2D> hullOne, List<Point2D> hullTwo)
   {

   }

   @Override
   public default void foundStartingVertexAndWorkingHull(Point2D startingVertex, List<Point2D> workingHull, boolean workingHullIsOne)
   {

   }

   @Override
   public default void consideringWorkingEdge(LineSegment2D workingEdge, boolean workingHullIsOne)
   {

   }

   @Override
   public default void foundIntersectionPoint(Point2D intersectionPoint, boolean workingHullIsOne)
   {

   }

   @Override
   public default void hullGotLooped(List<Point2D> hullOne, List<Point2D> hullTwo, List<Point2D> mergedVertices)
   {

   }

   @Override
   public default void hullIsInvalid(List<Point2D> invalidHull)
   {

   }

   @Override
   public default void hullsAreInvalid(List<Point2D> invalidHullA, List<Point2D> invalidHullB)
   {

   }
}