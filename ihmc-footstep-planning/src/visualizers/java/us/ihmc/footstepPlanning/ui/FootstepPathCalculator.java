package us.ihmc.footstepPlanning.ui;

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.SnapAndWiggleBasedNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.planners.BodyPathBasedFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.planners.DepthFirstFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.planners.VisibilityGraphWithAStarPlanner;
import us.ihmc.footstepPlanning.graphSearch.stepCost.ConstantFootstepCost;
import us.ihmc.footstepPlanning.simplePlanners.PlanThenSnapPlanner;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
import us.ihmc.javaFXToolkit.messager.Messager;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI.*;

public class FootstepPathCalculator
{
   private static final boolean VERBOSE = true;

   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final AtomicReference<PlanarRegionsList> planarRegionsReference;
   private final AtomicReference<Point3D> startPositionReference;
   private final AtomicReference<Point3D> goalPositionReference;
   private final AtomicReference<Double> startOrientationReference;
   private final AtomicReference<Double> goalOrientationReference;
   private final AtomicReference<FootstepPlannerType> footstepPlannerTypeReference;

   private final AtomicReference<FootstepPlannerParameters> parameters;

   private final Messager messager;

   public FootstepPathCalculator(Messager messager)
   {
      this.messager = messager;

      planarRegionsReference = messager.createInput(PlanarRegionDataTopic);
      startPositionReference = messager.createInput(StartPositionTopic);
      goalPositionReference = messager.createInput(GoalPositionTopic);
      startOrientationReference = messager.createInput(StartOrientationTopic, 0.0);
      goalOrientationReference = messager.createInput(GoalOrientationTopic, 0.0);
      parameters = messager.createInput(PlannerParametersTopic, new DefaultFootstepPlanningParameters());
      footstepPlannerTypeReference = messager.createInput(PlannerTypeTopic, FootstepPlannerType.A_STAR);

      messager.registerTopicListener(ComputePathTopic, request -> computePathOnThread());
   }

   public void clear()
   {
      planarRegionsReference.set(null);
      startPositionReference.set(null);
      goalPositionReference.set(null);
      startOrientationReference.set(null);
      goalOrientationReference.set(null);
   }

   public void start()
   {
   }

   public void stop()
   {
      executorService.shutdownNow();
   }

   private void computePathOnThread()
   {
      executorService.submit(this::computePath);
   }

   private void computePath()
   {
      if (VERBOSE)
      {
         PrintTools.info(this, "Starting to compute path...");
      }

      PlanarRegionsList planarRegionsList = planarRegionsReference.get();

      if (planarRegionsList == null)
         return;

      Point3D start = startPositionReference.get();

      if (start == null)
         return;

      Point3D goal = goalPositionReference.get();

      if (goal == null)
         return;

      if (VERBOSE)
         PrintTools.info(this, "Computing footstep path.");

      try
      {
         FootstepPlanner planner = createPlanner();

         planner.setPlanarRegions(planarRegionsList);
         planner.setTimeout(5.0);

         AxisAngle startRotation = new AxisAngle(startOrientationReference.get(), 0.0, 0.0);
         AxisAngle goalRotation = new AxisAngle(goalOrientationReference.get(), 0.0, 0.0);

         planner.setInitialStanceFoot(new FramePose3D(ReferenceFrame.getWorldFrame(), start, startRotation), RobotSide.LEFT);

         FootstepPlannerGoal plannerGoal = new FootstepPlannerGoal();
         plannerGoal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
         plannerGoal.setGoalPoseBetweenFeet(new FramePose3D(ReferenceFrame.getWorldFrame(), goal, goalRotation));
         planner.setGoal(plannerGoal);

         FootstepPlanningResult planningResult = planner.plan();

         if (VERBOSE)
         {
            PrintTools.info(this, "Planner result: " + planningResult);
            if (planningResult.validForExecution())
               PrintTools.info(this, "Planner result: " + planner.getPlan().getNumberOfSteps());
         }

         messager.submitMessage(PlanningResultTopic, planningResult);

         if (planningResult.validForExecution())
            messager.submitMessage(FootstepPlanTopic, planner.getPlan());
      }
      catch (Exception e)
      {
         PrintTools.error(this, e.getMessage());
         e.printStackTrace();
      }

   }

   private FootstepPlanner createPlanner()
   {
      SideDependentList<ConvexPolygon2D> contactPointsInSoleFrame = PlannerTools.createDefaultFootPolygons();
      YoVariableRegistry registry = new YoVariableRegistry("visualizerRegistry");

      switch (footstepPlannerTypeReference.get())
      {
      case PLANAR_REGION_BIPEDAL:
         return createPlanarRegionBipedalPlanner(contactPointsInSoleFrame, registry);
      case PLAN_THEN_SNAP:
         return new PlanThenSnapPlanner(new TurnWalkTurnPlanner(), contactPointsInSoleFrame);
      case A_STAR:
         return createAStarPlanner(contactPointsInSoleFrame, registry);
      case SIMPLE_BODY_PATH:
         return new BodyPathBasedFootstepPlanner(parameters.get(), contactPointsInSoleFrame, registry);
      case VIS_GRAPH_WITH_A_STAR:
         return new VisibilityGraphWithAStarPlanner(parameters.get(), contactPointsInSoleFrame, null, registry);
      default:
         throw new RuntimeException("Planner type " + footstepPlannerTypeReference.get() + " is not valid!");
      }
   }

   private FootstepPlanner createAStarPlanner(SideDependentList<ConvexPolygon2D> footPolygons, YoVariableRegistry registry)
   {
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters.get());
      return AStarFootstepPlanner.createRoughTerrainPlanner(parameters.get(), null, footPolygons, expansion, registry);
   }

   private FootstepPlanner createPlanarRegionBipedalPlanner(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, YoVariableRegistry registry)
   {
      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygonsInSoleFrame, parameters.get(), null);
      SnapAndWiggleBasedNodeChecker nodeChecker = new SnapAndWiggleBasedNodeChecker(footPolygonsInSoleFrame, null, parameters.get());
      ConstantFootstepCost stepCostCalculator = new ConstantFootstepCost(1.0);

      DepthFirstFootstepPlanner footstepPlanner = new DepthFirstFootstepPlanner(parameters.get(), snapper, nodeChecker, stepCostCalculator, registry);
      footstepPlanner.setFeetPolygons(footPolygonsInSoleFrame, footPolygonsInSoleFrame);
      footstepPlanner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      footstepPlanner.setExitAfterInitialSolution(false);

      return footstepPlanner;
   }
}