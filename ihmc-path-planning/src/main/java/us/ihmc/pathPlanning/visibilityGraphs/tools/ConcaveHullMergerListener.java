package us.ihmc.pathPlanning.visibilityGraphs.tools;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.tuple2D.Point2D;

public interface ConcaveHullMergerListener
{

   public abstract void originalHulls(ArrayList<Point2D> hullOne, ArrayList<Point2D> hullTwo);

   public abstract void preprocessedHull(ArrayList<Point2D> hullOne, ArrayList<Point2D> hullTwo);

   public abstract void foundStartingVertexAndWorkingHull(Point2D startingVertex, ArrayList<Point2D> workingHull, boolean workingHullIsOne);

   public abstract void consideringWorkingEdge(LineSegment2D workingEdge, boolean workingHullIsOne);

   public abstract void foundIntersectionPoint(Point2D intersectionPoint, boolean workingHullIsOne);

   public abstract void hullGotLooped(ArrayList<Point2D> hullOne, ArrayList<Point2D> hullTwo, ArrayList<Point2D> mergedVertices);


}