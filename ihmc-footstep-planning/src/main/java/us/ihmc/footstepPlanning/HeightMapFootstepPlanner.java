package us.ihmc.footstepPlanning;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

public class HeightMapFootstepPlanner
{
   public static FootstepPlan plan(Pose3DReadOnly start,
                                   Pose3DReadOnly goal,
                                   FootstepPlannerParametersReadOnly parameters,
                                   SideDependentList<ConvexPolygon2D> footPolygons,
                                   HeightMapData heightMap)
   {
      LogTools.info("Starting to plan with height map");

      FootstepPlan footstepPlan = new FootstepPlan();
      HeightMapPolygonSnapper snapper = new HeightMapPolygonSnapper();

      List<Pose2D> poses = generateTurnWalkTurnPoses(start, goal, parameters);
      RobotSide stepSide = RobotSide.LEFT;
      double idealStepWidth = parameters.getIdealFootstepWidth();

      for (int i = 0; i < poses.size(); i++)
      {
         Pose2D pose = poses.get(i);
         pose.appendTranslation(0.0, stepSide.negateIfRightSide(0.5 * idealStepWidth));
         RigidBodyTransform footstepTransform = new RigidBodyTransform();
         footstepTransform.getTranslation().set(pose.getPosition());
         footstepTransform.getRotation().setToYawOrientation(pose.getYaw());

         ConvexPolygon2D footPolygon = new ConvexPolygon2D(footPolygons.get(stepSide));
         footPolygon.applyTransform(footstepTransform);

         RigidBodyTransform snapTransform = snapper.snapPolygonToHeightMap(footPolygon, heightMap);
         if (snapTransform == null)
         {
//            break;
         }
         else
         {
            snapTransform.transform(footstepTransform);
         }

         FramePose3D step = new FramePose3D();
         step.set(footstepTransform);
         footstepPlan.addFootstep(stepSide, step);
         stepSide = stepSide.getOppositeSide();
      }

      LogTools.info("Computed path with " + footstepPlan.getNumberOfSteps() + " steps");

      return footstepPlan;
   }

   private static List<Pose2D> generateTurnWalkTurnPoses(Pose3DReadOnly start, Pose3DReadOnly goal, FootstepPlannerParametersReadOnly parameters)
   {
      List<Pose2D> poses = new ArrayList<>();

      double walkHeading = Math.atan2(goal.getY() - start.getY(), goal.getX() - start.getX());
      double yawPerStep = Math.abs(parameters.getMinimumStepYaw());

      // initial turn
      double deltaTurn = EuclidCoreTools.angleDifferenceMinusPiToPi(walkHeading, start.getYaw());
      int turnSteps = (int) Math.abs((deltaTurn) / yawPerStep);
      for (int i = 1; i <= turnSteps; i++)
      {
         Pose2D pose = new Pose2D(start);
         pose.appendRotation(yawPerStep * i);
         poses.add(pose);
      }

      Pose2D startOfWalkPose = new Pose2D(start);
      startOfWalkPose.setYaw(walkHeading);
      poses.add(startOfWalkPose);

      // walk
      double walkDistance = goal.getPosition().distanceXY(start.getPosition());
      int walkSteps = (int) (walkDistance / parameters.getIdealFootstepLength());
      Vector2D walkDirection = new Vector2D();
      walkDirection.set(goal.getX() - start.getX(), goal.getY() - start.getY());
      walkDirection.normalize();

      for (int i = 1; i <= walkSteps; i++)
      {
         Pose2D pose = new Pose2D(startOfWalkPose);
         pose.getPosition().add(i * walkDirection.getX() * parameters.getIdealFootstepLength(), i * walkDirection.getY() * parameters.getIdealFootstepLength());
         poses.add(pose);
      }

      // final turn
      Pose2D startOfTurnPose = new Pose2D(goal);
      startOfTurnPose.setYaw(walkHeading);
      poses.add(startOfTurnPose);

      deltaTurn = EuclidCoreTools.angleDifferenceMinusPiToPi(goal.getYaw(), walkHeading);
      turnSteps = (int) Math.abs((deltaTurn) / yawPerStep);
      for (int i = 1; i <= turnSteps; i++)
      {
         Pose2D pose = new Pose2D(startOfTurnPose);
         pose.appendRotation(yawPerStep * i);
         poses.add(pose);
      }

      poses.add(new Pose2D(goal));
      return poses;
   }
}
