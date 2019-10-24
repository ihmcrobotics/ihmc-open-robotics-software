package us.ihmc.robotEnvironmentAwareness.geometry;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public class ConcaveHullCutPoint
{
   private final Tuple2DReadOnly point;
   private int nextVertex; // this field exists to align intersections up with hull vertices
   private final boolean isIntersection;
   private boolean visited = false;

   public ConcaveHullCutPoint(Tuple2DReadOnly point)
   {
      this.point = point;
      this.isIntersection = false;
   }

   public ConcaveHullCutPoint(FramePoint2D frameIntersection, int nextVertex)
   {
      this.point = frameIntersection;
      this.nextVertex = nextVertex;
      this.isIntersection = true;
   }

   public int getNextVertex()
   {
      return nextVertex;
   }

   public void setVisited(boolean visited)
   {
      this.visited = visited;
   }

   public boolean getVisited()
   {
      return visited;
   }

   public Tuple2DReadOnly getPoint()
   {
      return point;
   }

   public double getX()
   {
      return point.getX();
   }

   public FramePoint2D getAsFramePoint()
   {
      return (FramePoint2D) point;
   }

   public boolean isIntersection()
   {
      return isIntersection;
   }
}
