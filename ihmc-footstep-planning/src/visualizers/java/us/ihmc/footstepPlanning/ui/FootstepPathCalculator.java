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
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
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
         SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
         FootstepPlannerParameters parameters = this.parameters.get();
         ParameterBasedNodeExpansion expansion = new ParameterBasedNodeExpansion(parameters);
         FootstepPlanner planner = AStarFootstepPlanner
               .createRoughTerrainPlanner(parameters, null, footPolygons, expansion, new YoVariableRegistry("visualizerRegistry"));

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

         messager.submitMessage(PlanTopic, planner.getPlan());

      }
      catch (Exception e)
      {
         PrintTools.error(this, e.getMessage());
         e.printStackTrace();
      }

   }

}