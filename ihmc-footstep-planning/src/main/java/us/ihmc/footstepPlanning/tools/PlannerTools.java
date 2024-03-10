package us.ihmc.footstepPlanning.tools;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.graphSearch.collision.BodyCollisionData;
import us.ihmc.footstepPlanning.graphSearch.collision.BoundingBoxCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloFootstepNode;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloFootstepPlannerRequest;
import us.ihmc.footstepPlanning.monteCarloPlanning.MonteCarloPlannerTools;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class PlannerTools
{
   private static final MonteCarloFootstepPlannerParameters monteCarloFootstepPlannerParameters = new MonteCarloFootstepPlannerParameters();
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static final double footLength = 0.2;
   public static final double footWidth = 0.1;

   public static ConvexPolygon2D createFootPolygon(double footLength, double heelWidth, double toeWidth)
   {
      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(footLength / 2.0, toeWidth / 2.0);
      footPolygon.addVertex(footLength / 2.0, -toeWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, heelWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, -heelWidth / 2.0);
      footPolygon.update();

      return footPolygon;
   }

   public static ArrayList<Point2D> createFootContactPoints(double footLength, double heelWidth, double toeWidth)
   {
      ArrayList<Point2D> contactPoints = new ArrayList<>();
      contactPoints.add(new Point2D(footLength / 2.0, toeWidth / 2.0));
      contactPoints.add(new Point2D(footLength / 2.0, -toeWidth / 2.0));
      contactPoints.add(new Point2D(-footLength / 2.0, heelWidth / 2.0));
      contactPoints.add(new Point2D(-footLength / 2.0, -heelWidth / 2.0));

      return contactPoints;
   }

   public static ConvexPolygon2D createFootPolygon(double footLength, double footWidth)
   {
      return createFootPolygon(footLength, footWidth, footWidth);
   }

   public static ConvexPolygon2D createDefaultFootPolygon()
   {
      return createFootPolygon(footLength, footWidth);
   }

   public static SideDependentList<ConvexPolygon2D> createDefaultFootPolygons()
   {
      return createFootPolygons(footLength, footWidth);
   }

   public static SideDependentList<ConvexPolygon2D> createFootPolygons(double footLength, double footWidth)
   {
      return createFootPolygons(footLength, footWidth, footWidth);
   }

   public static SideDependentList<ConvexPolygon2D> createFootPolygons(double footLength, double heelWidth, double toeWidth)
   {
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
         footPolygons.put(side, createFootPolygon(footLength, heelWidth, toeWidth));
      return footPolygons;
   }

   public static SideDependentList<Pose3D> createSquaredUpFootsteps(Point3DReadOnly midFootPosition, double midFootYaw, double stanceWidth)
   {
      Pose3D midFootPose = new Pose3D(midFootPosition, new Quaternion(midFootYaw, 0.0, 0.0));
      return createSquaredUpFootsteps(midFootPose, stanceWidth);
   }

   public static SideDependentList<Pose3D> createSquaredUpFootsteps(Pose3DReadOnly midFootPose, double stanceWidth)
   {
      return new SideDependentList<>(side ->
                                     {
                                        Pose3D footstepPose = new Pose3D(midFootPose);
                                        footstepPose.appendTranslation(0.0, 0.5 * side.negateIfRightSide(stanceWidth), 0.0);
                                        return footstepPose;
                                     });
   }

   public static void addGoalViz(FramePose3D goalPose, YoRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      YoFramePoint3D yoGoal = new YoFramePoint3D("GoalPosition", worldFrame, registry);
      yoGoal.set(goalPose.getPosition());
      graphicsListRegistry.registerYoGraphic("viz", new YoGraphicPosition("GoalViz", yoGoal, 0.05, YoAppearance.Yellow()));
      YoFramePoint3D yoStart = new YoFramePoint3D("StartPosition", worldFrame, registry);
      graphicsListRegistry.registerYoGraphic("viz", new YoGraphicPosition("StartViz", yoStart, 0.05, YoAppearance.Blue()));
      PoseReferenceFrame goalFrame = new PoseReferenceFrame("GoalFrame", goalPose);
      FrameVector3D goalOrientation = new FrameVector3D(goalFrame, 0.5, 0.0, 0.0);
      goalOrientation.changeFrame(worldFrame);
      YoFrameVector3D yoGoalOrientation = new YoFrameVector3D("GoalVector", worldFrame, registry);
      yoGoalOrientation.set(goalOrientation);
      //      graphicsListRegistry.registerYoGraphic("vizOrientation", new YoGraphicVector("GoalOrientationViz", yoGoal, yoGoalOrientation, 1.0, YoAppearance.White()));
   }

   public static boolean isGoalNextToLastStep(FramePose3D goalPose, FootstepPlan footstepPlan)
   {
      return isGoalNextToLastStep(goalPose, footstepPlan, 0.5);
   }

   public static boolean isGoalNextToLastStep(FramePose3D goalPose, FootstepPlan footstepPlan, double epsilon)
   {
      int steps = footstepPlan.getNumberOfSteps();
      if (steps < 1)
         throw new RuntimeException("Did not get enough footsteps to check if goal is within feet.");

      PlannedFootstep footstep = footstepPlan.getFootstep(steps - 1);
      FramePose3D stepPose = new FramePose3D();
      footstep.getFootstepPose(stepPose);
      RobotSide stepSide = footstep.getRobotSide();

      double midFeetOffset = stepSide.negateIfLeftSide(0.125);
      Vector3D goalOffset = new Vector3D(0.0, midFeetOffset, 0.0);
      RigidBodyTransform soleToWorld = new RigidBodyTransform();
      stepPose.get(soleToWorld);
      soleToWorld.transform(goalOffset);

      FramePose3D achievedGoal = new FramePose3D(stepPose);
      Point3D goalPosition = new Point3D(achievedGoal.getPosition());
      goalPosition.add(goalOffset);
      achievedGoal.getPosition().set(goalPosition);

      if (achievedGoal.epsilonEquals(goalPose, epsilon))
         return true;
      else
         return false;
   }

   public static boolean isGoalNextToLastStep(Point3D goalPosition, FootstepPlan footstepPlan)
   {
      return isGoalNextToLastStep(goalPosition, footstepPlan, 0.5);
   }

   public static boolean isGoalNextToLastStep(Point3D desiredPosition, FootstepPlan footstepPlan, double epsilon)
   {
      Point3D goalPosition = getEndPosition(footstepPlan);

      if (goalPosition.epsilonEquals(desiredPosition, epsilon))
         return true;
      else
         return false;
   }

   public static Point3D getEndPosition(FootstepPlan footstepPlan)
   {
      int steps = footstepPlan.getNumberOfSteps();
      if (steps < 1)
         throw new RuntimeException("Did not get enough footsteps to get end position.");

      PlannedFootstep footstep = footstepPlan.getFootstep(steps - 1);
      FramePose3D stepPose = new FramePose3D();
      footstep.getFootstepPose(stepPose);
      RobotSide stepSide = footstep.getRobotSide();

      double midFeetOffset = stepSide.negateIfLeftSide(0.125);
      Vector3D goalOffset = new Vector3D(0.0, midFeetOffset, 0.0);
      RigidBodyTransform soleToWorld = new RigidBodyTransform();
      stepPose.get(soleToWorld);
      soleToWorld.transform(goalOffset);

      FramePose3D achievedGoal = new FramePose3D(stepPose);
      Point3D goalPosition = new Point3D(achievedGoal.getPosition());
      goalPosition.add(goalOffset);

      return goalPosition;
   }

   /**
    * Checks the supplied body path for body collisions. The bounding box size and vertical offset are
    * supplied by the parameters.
    */
   public static boolean doesPathContainBodyCollisions(Pose3DReadOnly robotPose,
                                                       List<? extends Pose3DReadOnly> bodyPathWaypoints,
                                                       PlanarRegionsList planarRegionsList,
                                                       FootstepPlannerParametersReadOnly parameters,
                                                       double horizonDistanceToCheck)
   {
      BodyCollisionData collisionData = detectCollisionsAlongBodyPath(robotPose, bodyPathWaypoints, planarRegionsList, parameters, horizonDistanceToCheck);
      return collisionData != null && collisionData.isCollisionDetected();
   }

   public static BodyCollisionData detectCollisionsAlongBodyPath(Pose3DReadOnly robotPose,
                                                                 List<? extends Pose3DReadOnly> bodyPathWaypoints,
                                                                 PlanarRegionsList planarRegionsList,
                                                                 FootstepPlannerParametersReadOnly parameters,
                                                                 double horizonDistanceToCheck)
   {
      WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder = new WaypointDefinedBodyPathPlanHolder();
      bodyPathPlanHolder.setPoseWaypoints(bodyPathWaypoints);

      BoundingBoxCollisionDetector collisionDetector = new BoundingBoxCollisionDetector();
      collisionDetector.setBoxDimensions(parameters.getBodyBoxDepth(), parameters.getBodyBoxWidth(), parameters.getBodyBoxHeight());
      collisionDetector.setPlanarRegionsList(planarRegionsList);

      double distanceAlongPathPerCheck = 0.15;
      double totalPathLength = bodyPathPlanHolder.computePathLength(0.0);
      double deltaAlphaPerCheck = distanceAlongPathPerCheck / totalPathLength;

      Pose3D pathWaypoint = new Pose3D();
      Pose3D pathLookAhead = new Pose3D();
      Pose3D pathLookBehind = new Pose3D();

      double alpha = bodyPathPlanHolder.getClosestPoint(new Point2D(robotPose.getX(), robotPose.getY()), new Pose3D());
      double startLength = alpha * totalPathLength;

      while (true)
      {
         if (alpha > 1.0 || alpha * totalPathLength > startLength + horizonDistanceToCheck)
         {
            break;
         }

         bodyPathPlanHolder.getPointAlongPath(alpha, pathWaypoint);
         bodyPathPlanHolder.getPointAlongPath(MathTools.clamp(alpha - 0.01, 0.0, 1.0), pathLookBehind);
         bodyPathPlanHolder.getPointAlongPath(MathTools.clamp(alpha + 0.01, 0.0, 1.0), pathLookAhead);
         double yaw = Math.atan2(pathLookAhead.getY() - pathLookBehind.getY(), pathLookAhead.getX() - pathLookBehind.getX());

         collisionDetector.setBoxPose(pathWaypoint.getX(), pathWaypoint.getY(), pathWaypoint.getZ() + parameters.getBodyBoxBaseZ(), yaw);
         BodyCollisionData bodyCollisionData = collisionDetector.checkForCollision();
         if (bodyCollisionData.isCollisionDetected())
         {
            return bodyCollisionData;
         }

         alpha += deltaAlphaPerCheck;
      }

      return null;
   }

   public static void extrapolatePose(Pose3DReadOnly robotPose,
                                      List<? extends Pose3DReadOnly> bodyPathWaypoints,
                                      double horizonDistanceToCheck,
                                      Pose3D poseToPack)
   {
      WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder = new WaypointDefinedBodyPathPlanHolder();
      bodyPathPlanHolder.setPoseWaypoints(bodyPathWaypoints);

      double alpha = bodyPathPlanHolder.getClosestPoint(new Point2D(robotPose.getX(), robotPose.getY()), new Pose3D());
      double totalPathLength = bodyPathPlanHolder.computePathLength(0.0);
      double alphaToQuery = MathTools.clamp(alpha + horizonDistanceToCheck / totalPathLength, 0.0, 1.0);
      bodyPathPlanHolder.getPointAlongPath(alphaToQuery, poseToPack);
   }

   /**
    * Calculates the total expected duration of a footstep plan, which depends
    * on parameters outside of {@link FootstepPlannerParametersReadOnly}.
    */
   public static double calculateNominalTotalPlanExecutionDuration(FootstepPlan footstepPlan,
                                                                   double defaultSwingDuration,
                                                                   double defaultInitialTransferDuration,
                                                                   double defaultTransferDuration,
                                                                   double finalTransferDuration)
   {
      return calculateFootstepCompletionTime(footstepPlan,
                                             defaultSwingDuration,
                                             defaultInitialTransferDuration,
                                             defaultTransferDuration,
                                             finalTransferDuration,
                                             footstepPlan.getNumberOfSteps());
   }

   /**
    * Calculates as per {@link #calculateNominalTotalPlanExecutionDuration},
    * however only adds up the durations of the first n footsteps as specified
    * by the numberOfFootstepsToAddUp parameter.
    */
   public static double calculateFootstepCompletionTime(FootstepPlan footstepPlan,
                                                        double defaultSwingDuration,
                                                        double defaultInitialTransferDuration,
                                                        double defaultTransferDuration,
                                                        double finalTransferDuration,
                                                        int numberOfFootstepsToAddUp)
   {
      int numberOfSteps = footstepPlan.getNumberOfSteps();
      double planExecutionTime = 0.0;
      for (int i = 0; i < numberOfFootstepsToAddUp; i++)
      {
         PlannedFootstep footstep = footstepPlan.getFootstep(i);
         double transferDuration = footstep.getTransferDuration();
         double swingDuration = footstep.getSwingDuration();
         if (transferDuration < 0.0)
         {
            if (i == 0)
            {
               planExecutionTime += defaultInitialTransferDuration;
            }
            else
            {
               planExecutionTime += defaultTransferDuration;
            }
         }
         else
         {
            planExecutionTime += transferDuration;
         }

         if (swingDuration < 0.0)
         {
            planExecutionTime += defaultSwingDuration;
         }
         else
         {
            planExecutionTime += swingDuration;
         }
      }
      planExecutionTime += finalTransferDuration;
      return planExecutionTime;
   }

   public static double computeTotalScore(FootstepPlannerRequest aStarRequest, List<FootstepGraphNode> nodes, TerrainMapData terrainMapData)
   {
      MonteCarloFootstepPlannerRequest request = new MonteCarloFootstepPlannerRequest();
      request.setStartFootPose(RobotSide.LEFT, aStarRequest.getStartFootPoses().get(RobotSide.LEFT));
      request.setStartFootPose(RobotSide.RIGHT, aStarRequest.getStartFootPoses().get(RobotSide.RIGHT));
      request.setGoalFootPose(RobotSide.LEFT, aStarRequest.getGoalFootPoses().get(RobotSide.LEFT));
      request.setGoalFootPose(RobotSide.RIGHT, aStarRequest.getGoalFootPoses().get(RobotSide.RIGHT));
      request.setTerrainMapData(terrainMapData);

      double totalScore = 0.0;

      for (FootstepGraphNode node : nodes)
      {
         MonteCarloFootstepNode previousNode = new MonteCarloFootstepNode(new Point3D(node.getFirstStep().getX() * 50,
                                                                                      node.getFirstStep().getY() * 50,
                                                                                      node.getFirstStep().getYaw()), null, node.getFirstStepSide(), 0);
         MonteCarloFootstepNode footstepNode = new MonteCarloFootstepNode(new Point3D(node.getSecondStep().getX() * 50,
                                                                                      node.getSecondStep().getY() * 50,
                                                                                      node.getSecondStep().getYaw()), null, node.getSecondStepSide(), 0);
         double score = MonteCarloPlannerTools.scoreFootstepNode(previousNode, footstepNode, request, monteCarloFootstepPlannerParameters, false);
         totalScore += score;
      }

      return totalScore;
   }
}
