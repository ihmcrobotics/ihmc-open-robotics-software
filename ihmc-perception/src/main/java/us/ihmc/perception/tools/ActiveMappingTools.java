package us.ihmc.perception.tools;

import org.bytedeco.javacv.Frame;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FixedReferenceFrame;
import us.ihmc.euclid.referenceFrame.FrameMatrix3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ActiveMappingTools
{
   public static void setStraightGoalPoses(FixedReferenceFrame originalReferenceFrame, int segment, SideDependentList<FramePose3D> originalPoseToPlanFrom, SideDependentList<FramePose3D> startPose, SideDependentList<FramePose3D> goalPose, float distance)
   {
      SideDependentList<FramePose3D> tempPose = new SideDependentList<>(new FramePose3D(), new FramePose3D());

      for (RobotSide side : RobotSide.values)
      {

         tempPose.get(side).getPosition().set(originalPoseToPlanFrom.get(side).getPosition());
         tempPose.get(side).getOrientation().set(originalPoseToPlanFrom.get(side).getOrientation());
         tempPose.get(side).changeFrame(originalReferenceFrame);
         tempPose.get(side).getTranslation().addX(distance * segment);
         tempPose.get(side).getTranslation().addZ(startPose.get(side).getPosition().getZ() + 0.1);
         tempPose.get(side).changeFrame(ReferenceFrame.getWorldFrame());

         goalPose.get(side).set(tempPose.get(side));
      }
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
