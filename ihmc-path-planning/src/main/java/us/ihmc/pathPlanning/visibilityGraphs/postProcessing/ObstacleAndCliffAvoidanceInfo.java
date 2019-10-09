package us.ihmc.pathPlanning.visibilityGraphs.postProcessing;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

import java.util.ArrayList;
import java.util.List;

public class ObstacleAndCliffAvoidanceInfo
{
   private final Point3D originalPosition = new Point3D();
   private final Vector2D shiftToAvoidObstacles = new Vector2D();
   private final Vector2D shiftToAvoidObstaclesAndCliff = new Vector2D();

   private final List<Point2DReadOnly> closestObstacleClusterPoints = new ArrayList<>();
   private final List<Point2DReadOnly> closestCliffClusterPoints = new ArrayList<>();

   private boolean wasIntroduced = false;

   public void setWasIntroduced(boolean wasIntroduced)
   {
      this.wasIntroduced = wasIntroduced;
   }

   public void setOriginalPosition(Point3DReadOnly position)
   {
      this.originalPosition.set(position);
   }

   public void setShiftToAvoidObstacles(Vector2DReadOnly shift)
   {
      this.shiftToAvoidObstacles.set(shift);
   }

   public void setClosestObstacleClusterPoints(List<Point2DReadOnly> closestObstacleClusterPoints)
   {
      for (Point2DReadOnly point : closestObstacleClusterPoints)
      {
         this.closestObstacleClusterPoints.add(new Point2D(point));
      }
   }

   public void setShiftToAvoidObstaclesAndCliffs(Vector2DReadOnly shift)
   {
      this.shiftToAvoidObstaclesAndCliff.set(shift);
   }

   public void setClosestCliffClusterPoints(List<Point2DReadOnly> closestCliffClusterPoints)
   {
      for (Point2DReadOnly point : closestCliffClusterPoints)
      {
         this.closestCliffClusterPoints.add(new Point2D(point));
      }
   }

}
