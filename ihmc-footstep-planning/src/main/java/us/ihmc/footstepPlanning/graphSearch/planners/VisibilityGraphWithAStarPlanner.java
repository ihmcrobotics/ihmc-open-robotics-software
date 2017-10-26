package us.ihmc.footstepPlanning.graphSearch.planners;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.heuristics.BodyPathHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.stepCost.DistanceAndYawBasedCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlan;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class VisibilityGraphWithAStarPlanner implements FootstepPlanner
{
   private static final double defaultHeuristicWeight = 15.0;
   private static final double planningHorizon = 1.0;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble timeout = new  YoDouble("timeout", registry);

   private final YoDouble timeSpentBeforeFootstepPlanner = new  YoDouble("timeSpentBeforeFootstepPlanner", registry);
   private final YoDouble timeSpentInFootstepPlanner = new  YoDouble("timeSpentInFootstepPlanner", registry);
   private final YoEnum<FootstepPlanningResult> yoResult = new YoEnum<>("planningResult", registry, FootstepPlanningResult.class);

   private final FootstepPlannerParameters parameters;
   private final WaypointDefinedBodyPathPlan bodyPath;
   private final BodyPathHeuristics heuristics;
   private final FootstepPlanner footstepPlanner;

   private PlanarRegionsList planarRegionsList;
   private final FramePose bodyStartPose = new FramePose();
   private final FramePose bodyGoalPose = new FramePose();
   private final List<Point2D> waypoints = new ArrayList<>();

   private final boolean visualizing;
   private static final int bodyPathPointsForVisualization = 100;
   private final List<YoFramePoint> bodyPathPoints = new ArrayList<>();

   public VisibilityGraphWithAStarPlanner(FootstepPlannerParameters parameters, SideDependentList<ConvexPolygon2D> footPolygons,
                                          YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
      this.parameters = parameters;
      bodyPath = new WaypointDefinedBodyPathPlan();
      heuristics = new BodyPathHeuristics(registry, parameters, bodyPath);

      FootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      FootstepNodeChecker nodeChecker = new SnapBasedNodeChecker(parameters, footPolygons, snapper);
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);
      FootstepCost stepCostCalculator = new DistanceAndYawBasedCost(parameters);
      FootstepNodeSnapper postProcessingSnapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters, null);

      heuristics.setWeight(defaultHeuristicWeight);
      footstepPlanner = new AStarFootstepPlanner(parameters, nodeChecker, heuristics, expansion, stepCostCalculator, postProcessingSnapper, registry);

      visualizing = graphicsListRegistry != null;
      if (visualizing)
      {
         setupVisualization(graphicsListRegistry, registry);
      }
   }

   private void setupVisualization(YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry registry)
   {
      for (int i = 0; i < bodyPathPointsForVisualization; i++)
      {
         YoFramePoint point = new YoFramePoint("BodyPathPoint" + i, ReferenceFrame.getWorldFrame(), registry);
         point.setToNaN();
         bodyPathPoints.add(point);
         YoGraphicPosition pointVisualization = new YoGraphicPosition("BodyPathPoint" + i, point, 0.02, YoAppearance.Yellow());
         graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), pointVisualization);
      }
   }

   @Override
   public void setInitialStanceFoot(FramePose stanceFootPose, RobotSide side)
   {
      double defaultStepWidth = parameters.getIdealFootstepWidth();
      ReferenceFrame stanceFrame = new PoseReferenceFrame("stanceFrame", stanceFootPose);
      bodyStartPose.setToZero(stanceFrame);
      bodyStartPose.setY(side.negateIfLeftSide(defaultStepWidth / 2.0));
      bodyStartPose.changeFrame(ReferenceFrame.getWorldFrame());

      footstepPlanner.setInitialStanceFoot(stanceFootPose, side);
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      AStarFootstepPlanner.checkGoalType(goal);
      bodyGoalPose.setIncludingFrame(goal.getGoalPoseBetweenFeet());
   }

   @Override
   public void setTimeout(double timeout)
   {
      this.timeout.set(timeout);
   }

   @Override
   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      footstepPlanner.setPlanarRegions(planarRegionsList);
      this.planarRegionsList = planarRegionsList;
   }

   @Override
   public FootstepPlanningResult plan()
   {
      long startTime = System.currentTimeMillis();
      waypoints.clear();

      if (planarRegionsList == null)
      {
         waypoints.add(new Point2D(bodyStartPose.getX(), bodyStartPose.getY()));
         waypoints.add(new Point2D(bodyGoalPose.getX(), bodyGoalPose.getY()));
      }
      else
      {
         NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(planarRegionsList.getPlanarRegionsAsList());
         Point3D startPos = PlanarRegionTools.projectPointToPlanesVertically(bodyStartPose.getPosition(), planarRegionsList);
         Point3D goalPos = PlanarRegionTools.projectPointToPlanesVertically(bodyGoalPose.getPosition(), planarRegionsList);
         List<Point3D> path = navigableRegionsManager.calculateBodyPath(startPos, goalPos);

         if (path.size() < 2)
         {
            double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
            timeSpentBeforeFootstepPlanner.set(seconds);
            timeSpentInFootstepPlanner.set(0.0);
            yoResult.set(FootstepPlanningResult.PLANNER_FAILED);
//            PointCloudTools.savePlanarRegionsToFile(planarRegionsList, startPos, goalPos);
            return yoResult.getEnumValue();
         }

         for (Point3D waypoint3d : path)
         {
            waypoints.add(new Point2D(waypoint3d.getX(), waypoint3d.getY()));
         }
      }

      bodyPath.setWaypoints(waypoints);
      bodyPath.compute(null, null);

      if (visualizing)
      {
         updateBodyPathVisualization();
      }

      Pose2D goalPose2d = new Pose2D();
      double pathLength = bodyPath.computePathLength(0.0);
      double alpha = MathTools.clamp(planningHorizon / pathLength, 0.0, 1.0);
      bodyPath.getPointAlongPath(alpha, goalPose2d);
      heuristics.setGoalAlpha(alpha);

      FramePose footstepPlannerGoal = new FramePose();
      footstepPlannerGoal.setPosition(goalPose2d.getX(), goalPose2d.getY(), 0.0);
      footstepPlannerGoal.setYawPitchRoll(goalPose2d.getYaw(), 0.0, 0.0);

      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      goal.setGoalPoseBetweenFeet(footstepPlannerGoal);
      footstepPlanner.setGoal(goal);

      double seconds = (System.currentTimeMillis() - startTime) / 1000.0;
      timeSpentBeforeFootstepPlanner.set(seconds);
      footstepPlanner.setTimeout(timeout.getDoubleValue() - seconds);

      startTime = System.currentTimeMillis();
      yoResult.set(footstepPlanner.plan());
      seconds = (System.currentTimeMillis() - startTime) / 1000.0;
      timeSpentInFootstepPlanner.set(seconds);

      return yoResult.getEnumValue();
   }

   private void updateBodyPathVisualization()
   {
      Pose2D tempPose = new Pose2D();
      for (int i = 0; i < bodyPathPointsForVisualization; i++)
      {
         double percent = (double) i / (double) (bodyPathPointsForVisualization - 1);
         bodyPath.getPointAlongPath(percent, tempPose);
         Point3D position = new Point3D();
         position.set(tempPose.getPosition());
         bodyPathPoints.get(i).set(PlanarRegionTools.projectPointToPlanesVertically(position, planarRegionsList));
      }
   }

   @Override
   public FootstepPlan getPlan()
   {
      return footstepPlanner.getPlan();
   }

}
