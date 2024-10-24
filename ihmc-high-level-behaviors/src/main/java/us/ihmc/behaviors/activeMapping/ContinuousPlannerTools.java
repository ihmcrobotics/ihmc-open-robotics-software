package us.ihmc.behaviors.activeMapping;

import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.Random;

public class ContinuousPlannerTools
{
   public static double getDistanceFromRobotToGoalPoseOnXYPlane(Point3DReadOnly robotPositionInWorld, SideDependentList<FramePose3D> goalPoses)
   {
      FramePose3D leftGoalPose = goalPoses.get(RobotSide.LEFT);
      FramePose3D rightGoalPose = goalPoses.get(RobotSide.RIGHT);

      // Get point halfway between the left and right goal poses
      Point3D middleDistanceBetweenGoalPoses = new Point3D();
      middleDistanceBetweenGoalPoses.interpolate(leftGoalPose.getPosition(), rightGoalPose.getPosition(), 0.5);

      return middleDistanceBetweenGoalPoses.distanceXY(robotPositionInWorld);
   }

   public static SideDependentList<FramePose3D> setGoalPoseBasedOnLateralJoystickValue(ReferenceFrame referenceFrame,
                                                                                       SideDependentList<FramePose3D> stancePose,
                                                                                       double lateralValueInRadians,
                                                                                       float xDistance,
                                                                                       float zDistance,
                                                                                       float nominalStanceWidth)
   {
      // Get the robot's current rotation (yaw) and translation (position) from the reference frame
      RigidBodyTransform currentRobotLocation = new RigidBodyTransform();
      currentRobotLocation.getRotation().set(referenceFrame.getTransformToWorldFrame().getRotation());
      currentRobotLocation.getTranslation().set(referenceFrame.getTransformToWorldFrame().getTranslation());

      // Extract rotation (yaw) from the reference frame's transform
      double robotYaw = currentRobotLocation.getRotation().getYaw();

      // Extract the robot's current position from the reference frame's transform
      Point3D robotLocation = new Point3D();
      robotLocation.set(currentRobotLocation.getTranslation());

      // Compute the yaw for the goal by adding the lateralValueInRadians (1 radian to the right)
      double goalYaw = robotYaw + lateralValueInRadians;

      // Goal pose list for both sides
      SideDependentList<FramePose3D> goalPose = new SideDependentList<>();

      // Iterate over robot sides
      for (RobotSide side : RobotSide.values)
      {
         // Create a new goal pose for each side
         goalPose.put(side, new FramePose3D());

         // Calculate the goal position relative to the robot's location, keeping a fixed radius
         double goalX = robotLocation.getX() + xDistance * Math.cos(goalYaw);
         double goalY = robotLocation.getY() + xDistance * Math.sin(goalYaw);
         double goalZ = robotLocation.getZ() + zDistance; // Adjust for height if necessary

         // Set the goal position in the pose
         goalPose.get(side).getPosition().set(goalX, goalY, goalZ);

         // Set the orientation so that the yaw matches the robot's facing direction plus the offset
         goalPose.get(side).getOrientation().setToYawOrientation(goalYaw);
      }

      // Adjust for nominal stance width (side-to-side foot distance)
      goalPose.get(RobotSide.LEFT).appendTranslation(0.0, nominalStanceWidth / 2.0, 0.0);
      goalPose.get(RobotSide.RIGHT).appendTranslation(0.0, -nominalStanceWidth / 2.0, 0.0);

      return goalPose;
   }

   public static SideDependentList<FramePose3D> setRandomizedStraightGoalPoses(FramePose3D walkingStartPose,
                                                                               SideDependentList<FramePose3D> stancePose,
                                                                               float xDistance,
                                                                               float xRandomMargin,
                                                                               float zDistance,
                                                                               float nominalStanceWidth)
   {
      float offsetX = (float) (Math.random() * xRandomMargin - xRandomMargin / 2.0f);

      FramePose3D stanceMidPose = new FramePose3D();
      stanceMidPose.interpolate(stancePose.get(RobotSide.LEFT), stancePose.get(RobotSide.RIGHT), 0.5);

      SideDependentList<FramePose3D> goalPose = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         goalPose.put(side, new FramePose3D());
         RigidBodyTransform stanceToWalkingFrameTransform = new RigidBodyTransform();
         RigidBodyTransform worldToWalkingFrameTransform = new RigidBodyTransform();

         stanceToWalkingFrameTransform.set(stanceMidPose);
         worldToWalkingFrameTransform.set(walkingStartPose);
         worldToWalkingFrameTransform.invert();
         stanceToWalkingFrameTransform.multiply(worldToWalkingFrameTransform);

         double xWalkDistance = stanceToWalkingFrameTransform.getTranslation().norm();
         goalPose.get(side).getPosition().set(walkingStartPose.getPosition());
         goalPose.get(side).getOrientation().set(walkingStartPose.getOrientation());
         goalPose.get(side).appendTranslation(xWalkDistance + xDistance + offsetX, 0, stanceMidPose.getZ() + zDistance - walkingStartPose.getZ());
      }

      // These are done after because of the ( - ) or ( + ) for the nominal stance
      goalPose.get(RobotSide.LEFT).appendTranslation(0.0, nominalStanceWidth / 2.0f, 0.0);
      goalPose.get(RobotSide.RIGHT).appendTranslation(0.0, -nominalStanceWidth / 2.0f, 0.0);

      return goalPose;
   }

   public static void generateSensorZUpToStraightGoalFootPoses(HeightMapData latestHeightMapData,
                                                               RigidBodyTransform sensorZUpToWorldTransform,
                                                               SideDependentList<FramePose3D> startPoseToPack,
                                                               SideDependentList<FramePose3D> goalPoseToPack,
                                                               Random random,
                                                               double xDistance,
                                                               double xMargin,
                                                               double yBound,
                                                               double zOffset,
                                                               double nominalStanceWidth)
   {
      double heightAtStartPose = latestHeightMapData.getHeightAt(sensorZUpToWorldTransform.getTranslation().getX(),
                                                                 sensorZUpToWorldTransform.getTranslation().getY());
      double heightAtGoalPose = latestHeightMapData.getHeightAt(sensorZUpToWorldTransform.getTranslation().getX() + xDistance + xMargin / 2.0,
                                                                sensorZUpToWorldTransform.getTranslation().getY());

      if (heightAtStartPose == Double.NaN || heightAtGoalPose == Double.NaN)
      {
         LogTools.error("Height at start or goal pose is NaN");
      }
      else
      {
         // set start pose to be below the camera
         startPoseToPack.get(RobotSide.LEFT).set(sensorZUpToWorldTransform);
         startPoseToPack.get(RobotSide.LEFT).appendTranslation(0.0, nominalStanceWidth / 2.0f, heightAtStartPose + zOffset);

         startPoseToPack.get(RobotSide.RIGHT).set(sensorZUpToWorldTransform);
         startPoseToPack.get(RobotSide.RIGHT).appendTranslation(0.0, -nominalStanceWidth / 2.0f, heightAtStartPose + zOffset);

         // set goal pose to be 1.65m in front of the camera
         goalPoseToPack.get(RobotSide.LEFT).set(sensorZUpToWorldTransform);

         if (random != null)
            goalPoseToPack.get(RobotSide.LEFT)
                          .appendTranslation(random.nextDouble(xDistance - xMargin / 2.0f, xDistance + xMargin / 2.0f),
                                             random.nextDouble(-yBound, yBound),
                                             heightAtGoalPose + zOffset);
         else
            goalPoseToPack.get(RobotSide.LEFT).appendTranslation(xDistance, nominalStanceWidth / 2.0f, heightAtGoalPose + zOffset);

         goalPoseToPack.get(RobotSide.RIGHT).set(goalPoseToPack.get(RobotSide.LEFT));
         goalPoseToPack.get(RobotSide.RIGHT).appendTranslation(0.0, -nominalStanceWidth / 2.0f, 0.0);
      }
   }

   public static Pose2D getNearestUnexploredNode(PlanarRegionsList planarRegionMap,
                                                 Point2DReadOnly gridOrigin,
                                                 Pose2D robotPose,
                                                 int gridSize,
                                                 float resolution,
                                                 float minimumDistanceToNode,
                                                 float maximumYawDifference)
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

               // Math.PI / 2.5 should be the maximum yaw difference between the robot and the goal
               if (distanceToNode < nearestDistance && distanceToNode > minimumDistanceToNode && Math.abs(yawRobotToGoal - robotYaw) < maximumYawDifference)
               {
                  goalPose.set(point.getX(), point.getY(), yawRobotToGoal);
                  nearestDistance = distanceToNode;
               }
            }
         }
      }

      return goalPose;
   }
}
