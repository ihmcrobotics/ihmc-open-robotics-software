package us.ihmc.perception.tools;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class ActiveMappingTools
{
   public static void getStraightGoalFootPoses(Pose3D leftSolePose, Pose3D rightSolePose, Pose3D leftGoalPose, Pose3D rightGoalPose, float distance)
   {
      leftSolePose.setZ(0);
      rightSolePose.setZ(0);

      leftGoalPose.set(leftSolePose);
      leftGoalPose.appendTranslation(distance, 0.0, 0.0);

      rightGoalPose.set(rightSolePose);
      rightGoalPose.appendTranslation(distance, 0.0, 0.0);
   }

   public static Pose2D getNearestUnexploredNode(PlanarRegionsList planarRegionMap, Point2DReadOnly gridOrigin, Pose2D robotPose, int gridSize, float resolution)
   {
      Pose2D goalPose = new Pose2D(gridOrigin, 0.0);
      Point2D robotLocation = new Point2D(robotPose.getX(), robotPose.getY());
      float robotYaw = (float) robotPose.getYaw();
      float nearestDistance = Float.MAX_VALUE;
      for (int i = 0; i < gridSize; i++)
      {
         for (int j = 0; j < gridSize; j++)
         {
            Point3D point = new Point3D(gridOrigin.getX() + resolution * i, gridOrigin.getY() + resolution * j, 0.0);

            boolean explored = false;
            for (PlanarRegion region : planarRegionMap.getPlanarRegionsAsList())
            {
               Point3D projectedPoint = PlanarRegionTools.projectInZToPlanarRegion(point, region);
               explored |= PlanarRegionTools.isPointInWorldInsidePlanarRegion(region, projectedPoint);
            }

            if (!explored)
            {
               Point2D currentPoint = new Point2D(point.getX(), point.getY());
               float distanceToNode = (float) currentPoint.distance(robotLocation);
               float yawRobotToGoal = (float) Math.atan2(currentPoint.getY() - robotLocation.getY(), currentPoint.getX() - robotLocation.getX());

               if (distanceToNode < nearestDistance && distanceToNode > 0.3 && Math.abs(yawRobotToGoal - robotYaw) < Math.PI / 2.5f)
               {
                  goalPose.set(point.getX(), point.getY(), yawRobotToGoal);
                  nearestDistance = distanceToNode;
               }
            }
         }
      }

      return goalPose;
   }

   public static int getIndexFromCoordinates(double coordinate, float resolution, int offset)
   {
      return (int) (coordinate * resolution + offset);
   }

   public static double getCoordinateFromIndex(int index, double resolution, int offset)
   {
      return (index - offset) / resolution;
   }
}
