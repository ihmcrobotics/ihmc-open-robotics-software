package us.ihmc.perception.tools;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ActiveMappingTools
{
   public static void setRandomizedStraightGoalPoses(SideDependentList<FramePose3D> originalPoseToPlanFrom,
                                                     SideDependentList<FramePose3D> startPose,
                                                     SideDependentList<FramePose3D> goalPose,
                                                     float xDistance,
                                                     float zDistance)
   {
      float offsetX = (float) (Math.random() * 0.1 - 0.05);
      float offsetY = (float) (Math.random() * 0.1 - 0.05);

      for (RobotSide side : RobotSide.values)
      {
         goalPose.get(side).getPosition().set(startPose.get(side).getPosition());
         goalPose.get(side).getOrientation().set(originalPoseToPlanFrom.get(side).getOrientation());
         goalPose.get(side).getTranslation().setX(startPose.get(side).getPosition().getX() + xDistance + offsetX);
         goalPose.get(side).getTranslation().setY(originalPoseToPlanFrom.get(side).getPosition().getY() + offsetY);
         goalPose.get(side).getTranslation().setZ(startPose.get(side).getPosition().getZ() + zDistance);

         goalPose.get(side).changeFrame(ReferenceFrame.getWorldFrame());
      }
   }

   public static void setStraightGoalPoses(SideDependentList<FramePose3D> originalPoseToPlanFrom,
                                           SideDependentList<FramePose3D> startPose,
                                           SideDependentList<FramePose3D> goalPose,
                                           float xDistance,
                                           float zDistance)
   {
      for (RobotSide side : RobotSide.values)
      {
         goalPose.get(side).getPosition().set(startPose.get(side).getPosition());
         goalPose.get(side).getOrientation().set(originalPoseToPlanFrom.get(side).getOrientation());
         goalPose.get(side).getTranslation().setX(startPose.get(side).getPosition().getX() + xDistance);
         goalPose.get(side).getTranslation().setY(originalPoseToPlanFrom.get(side).getPosition().getY());
         goalPose.get(side).getTranslation().setZ(startPose.get(side).getPosition().getZ() + zDistance);

         goalPose.get(side).changeFrame(ReferenceFrame.getWorldFrame());
      }
   }

   public static Pose2D getNearestUnexploredNode(PlanarRegionsList planarRegionMap,
                                                 Point2DReadOnly gridOrigin,
                                                 Pose2D robotPose,
                                                 int gridSize,
                                                 float resolution)
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
