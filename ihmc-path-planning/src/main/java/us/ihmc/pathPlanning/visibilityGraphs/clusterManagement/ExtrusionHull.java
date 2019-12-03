package us.ihmc.pathPlanning.visibilityGraphs.clusterManagement;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Stream;

public class ExtrusionHull
{
   private final List<Point2DReadOnly> pointsInHull = new ArrayList<>();

   public ExtrusionHull()
   {
   }

   public ExtrusionHull(ExtrusionHull other)
   {
      this(other.getPoints());
   }

   public ExtrusionHull(List<? extends Point2DReadOnly> points)
   {
      addAllPoints(points);
   }

   public ExtrusionHull(ExtrusionHull other, Function<ExtrusionHull, List<Point2DReadOnly>> function)
   {
      pointsInHull.addAll(function.apply(other));
   }

   public void clear()
   {
      pointsInHull.clear();
   }

   public boolean isEmpty()
   {
      return pointsInHull.isEmpty();
   }

   public int size()
   {
      return pointsInHull.size();
   }

   public Point2DReadOnly get(int i)
   {
      return pointsInHull.get(i);
   }

   public void addAllPoints(ExtrusionHull other)
   {
      addAllPoints(other.getPoints());
   }

   public void addAllPoints(List<? extends Point2DReadOnly> points)
   {
      points.forEach(this::addPoint);
   }

   public void addPoint(Point2DReadOnly point)
   {
      pointsInHull.add(new Point2D(point));
   }

   public void addPoint(Point3DReadOnly point)
   {
      pointsInHull.add(new Point2D(point));
   }

   public Stream<Point2DReadOnly> stream()
   {
      return pointsInHull.stream();
   }

   public List<Point2DReadOnly> getPoints()
   {
      return pointsInHull;
   }

   public ExtrusionHull copy()
   {
      return new ExtrusionHull(this);
   }

   public ExtrusionHull copy(Function<ExtrusionHull, List<Point2DReadOnly>> function)
   {
      return new ExtrusionHull(this, function);
   }
}
