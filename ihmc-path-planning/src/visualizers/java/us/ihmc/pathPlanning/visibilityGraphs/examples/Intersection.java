package us.ihmc.pathPlanning.visibilityGraphs.examples;

import us.ihmc.euclid.tuple2D.Point2D;

public class Intersection
{
   int firstIndex = 0;
   int secondIndex = 0;
   Point2D intersection = null;

   public Intersection(int firstIndex, int secondIndex, Point2D intersection)
   {
      this.firstIndex = firstIndex;
      this.secondIndex = secondIndex;
      this.intersection = intersection;
   }

   public int getFirstIndex()
   {
      return firstIndex;
   }

   public int getSecondIndex()
   {
      return secondIndex;
   }

   public Point2D getIntersection()
   {
      return intersection;
   }
}
