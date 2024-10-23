package us.ihmc.footstepPlanning.simplePlanners;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.polygonSnapping.HeightMapPolygonSnapper;
import us.ihmc.log.LogTools;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.commons.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.ArrayList;
import java.util.List;

public class HeightMapFootstepPlanner
{
   public static FootstepPlan debug(SideDependentList<ConvexPolygon2D> footPolygons, HeightMapData heightMap)
   {
      List<Point3D> stepsToDebug = new ArrayList<>();
      stepsToDebug.add(new Point3D(1.3, 0.0, 0.0));

      FootstepPlan footstepPlan = new FootstepPlan();
      HeightMapPolygonSnapper snapper = new HeightMapPolygonSnapper();
      FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
      environmentHandler.setHeightMap(heightMap);

      for (int i = 0; i < stepsToDebug.size(); i++)
      {
         FramePose3D pose = new FramePose3D();
         pose.getPosition().set(stepsToDebug.get(i));

         //         footstepPlan.addFootstep(RobotSide.LEFT, pose);

         RigidBodyTransform footstepTransform = new RigidBodyTransform();
         footstepTransform.getTranslation().set(pose.getPosition());
         footstepTransform.getTranslation().setZ(0.0);
         footstepTransform.getRotation().setToYawOrientation(pose.getYaw());

         FootstepSnapData snapData = snapper.computeSnapData(pose.getX(),
                                                             pose.getY(),
                                                             pose.getYaw(),
                                                             footPolygons.get(RobotSide.LEFT),
                                                             environmentHandler,
                                                             0.06,
                                                             Math.toRadians(45.0));

         RigidBodyTransform snapTransform = snapData.getSnapTransform();

         if (snapTransform != null)
         {
            FramePose3D step = new FramePose3D(ReferenceFrame.getWorldFrame(), footstepTransform);
            step.applyTransform(snapTransform);

            System.out.println("step translation: " + step.getPosition());
            System.out.println("step ypr: " + step.getOrientation().getYaw() + ", " + step.getOrientation().getPitch() + ", " + step.getRoll());

//            double zOnPlane = snapper.getBestFitPlane().getZOnPlane(pose.getX(), pose.getY());
//            step.getPosition().set(pose.getX(), pose.getY(), zOnPlane);
//
//            EuclidGeometryTools.orientation3DFromZUpToVector3D(snapper.getBestFitPlane().getNormal(), step.getOrientation());
            footstepPlan.addFootstep(RobotSide.LEFT, step);
         }
      }

      return footstepPlan;
   }

   public static FootstepPlan plan(Pose3DReadOnly start,
                                   Pose3DReadOnly goal,
                                   DefaultFootstepPlannerParametersReadOnly parameters,
                                   SideDependentList<ConvexPolygon2D> footPolygons,
                                   HeightMapData heightMap)
   {
      LogTools.info("Starting to plan with height map");

      FootstepPlan footstepPlan = new FootstepPlan();
      HeightMapPolygonSnapper snapper = new HeightMapPolygonSnapper();
      FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
      environmentHandler.setHeightMap(heightMap);

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

         FootstepSnapData snapData = snapper.computeSnapData(pose.getX(),
                                                             pose.getY(),
                                                             pose.getYaw(),
                                                             footPolygons.get(RobotSide.LEFT),
                                                             environmentHandler,
                                                             parameters.getHeightMapSnapThreshold(),
                                                             parameters.getMinSurfaceIncline());

         FramePose3D step = new FramePose3D(ReferenceFrame.getWorldFrame(), footstepTransform);
         if (snapData.getSnapTransform() != null)
         {
            step.applyTransform(snapData.getSnapTransform());
         }
         footstepPlan.addFootstep(stepSide, step);
         stepSide = stepSide.getOppositeSide();
      }

      LogTools.info("Computed path with " + footstepPlan.getNumberOfSteps() + " steps");

      return footstepPlan;
   }

   private static List<Pose2D> generateTurnWalkTurnPoses(Pose3DReadOnly start, Pose3DReadOnly goal, DefaultFootstepPlannerParametersReadOnly parameters)
   {
      List<Pose2D> poses = new ArrayList<>();

      double walkHeading = Math.atan2(goal.getY() - start.getY(), goal.getX() - start.getX());
      double yawPerStep = Math.abs(parameters.getMinStepYaw());

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
