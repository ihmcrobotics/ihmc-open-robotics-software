package us.ihmc.footstepPlanning.ui.components;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.BodyPathData;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ComputePath;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.GoalOrientation;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.GoalPosition;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.GoalVisibilityMap;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.InitialSupportSide;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.InterRegionVisibilityMap;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.LowLevelGoalOrientation;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.LowLevelGoalPosition;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlanarRegionData;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlannerHorizonLength;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlannerParameters;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlannerStatus;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlannerTimeout;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlannerType;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlanningResult;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.RequestPlannerStatistics;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.StartOrientation;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.StartPosition;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.StartVisibilityMap;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.VisibilityGraphsParameters;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.VisibilityMapWithNavigableRegionData;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlannerStatus;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.tools.statistics.GraphSearchStatistics;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.pathPlanning.statistics.ListOfStatistics;
import us.ihmc.pathPlanning.statistics.PlannerStatistics;
import us.ihmc.pathPlanning.statistics.VisibilityGraphStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.InterRegionVisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

public class FootstepPathCalculatorModule
{
   private static final boolean VERBOSE = true;

   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final AtomicReference<PlanarRegionsList> planarRegionsReference;
   private final AtomicReference<Point3D> startPositionReference;
   private final AtomicReference<Quaternion> startOrientationReference;
   private final AtomicReference<RobotSide> initialStanceSideReference;
   private final AtomicReference<Point3D> goalPositionReference;
   private final AtomicReference<Quaternion> goalOrientationReference;
   private final AtomicReference<FootstepPlannerType> footstepPlannerTypeReference;

   private final AtomicReference<Double> plannerTimeoutReference;
   private final AtomicReference<Double> plannerBestEffortTimeoutReference;
   private final AtomicReference<Double> plannerHorizonLengthReference;

   private final AtomicReference<FootstepPlannerParametersReadOnly> parameters;
   private final AtomicReference<VisibilityGraphsParametersReadOnly> visibilityGraphsParameters;

   private final Messager messager;
   private final FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());

   public FootstepPathCalculatorModule(Messager messager)
   {
      this.messager = messager;

      planarRegionsReference = messager.createInput(PlanarRegionData);
      startPositionReference = messager.createInput(StartPosition);
      startOrientationReference = messager.createInput(StartOrientation, new Quaternion());
      initialStanceSideReference = messager.createInput(InitialSupportSide, RobotSide.LEFT);
      goalPositionReference = messager.createInput(GoalPosition);
      goalOrientationReference = messager.createInput(GoalOrientation, new Quaternion());

      parameters = messager.createInput(PlannerParameters, new DefaultFootstepPlannerParameters());
      visibilityGraphsParameters = messager.createInput(VisibilityGraphsParameters, new DefaultVisibilityGraphParameters());
      footstepPlannerTypeReference = messager.createInput(PlannerType, FootstepPlannerType.A_STAR);
      plannerTimeoutReference = messager.createInput(PlannerTimeout, 5.0);
      plannerBestEffortTimeoutReference = messager.createInput(PlannerBestEffortTimeout, 0.0);
      plannerHorizonLengthReference = messager.createInput(PlannerHorizonLength, 1.0);

      messager.registerTopicListener(ComputePath, request -> computePathOnThread());
      messager.registerTopicListener(RequestPlannerStatistics, request -> sendPlannerStatistics());
   }

   public void clear()
   {
      planarRegionsReference.set(null);
      startPositionReference.set(null);
      startOrientationReference.set(null);
      initialStanceSideReference.set(null);
      goalPositionReference.set(null);
      goalOrientationReference.set(null);
      plannerTimeoutReference.set(null);
      plannerBestEffortTimeoutReference.set(null);
      plannerHorizonLengthReference.set(null);
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
         LogTools.info("Starting to compute path...");
      }

      PlanarRegionsList planarRegionsList = planarRegionsReference.get();

      if (planarRegionsList == null)
         return;

      Point3D startPosition = startPositionReference.get();

      if (startPosition == null)
         return;

      Point3D goal = goalPositionReference.get();

      if (goal == null)
         return;

      if (VERBOSE)
         LogTools.info("Computing footstep path.");

      try
      {
         FootstepPlannerRequest request = new FootstepPlannerRequest();
         request.setPlanarRegionsList(planarRegionsList);
         request.setTimeout(plannerTimeoutReference.get());
         request.setHorizonLength(plannerHorizonLengthReference.get());
         request.getStanceFootPose().set(startPosition, startOrientationReference.get());
         request.setInitialStanceSide(initialStanceSideReference.get());
         request.setGoalPose(goal, goalOrientationReference.get());
         request.setPlanBodyPath(footstepPlannerTypeReference.get().plansPath());

         planningModule.getFootstepPlannerParameters().set(parameters.get());
         planningModule.getVisibilityGraphParameters().set(visibilityGraphsParameters.get());

         messager.submitMessage(PlannerStatus, FootstepPlannerStatus.PLANNING_PATH);

         planningModule.addBodyPathPlanCallback(bodyPathMessage ->
                                                {
                                                   if (FootstepPlanningResult.fromByte(bodyPathMessage.getFootstepPlanningResult()).validForExecution())
                                                   {
                                                      messager.submitMessage(PlannerStatus, FootstepPlannerStatus.PLANNING_STEPS);
                                                      messager.submitMessage(BodyPathData, new ArrayList<>(bodyPathMessage.getBodyPath()));
                                                   }
                                                });
         planningModule.addStatusCallback(status -> messager.submitMessage(PlanningResult, status.getResult()));

         FootstepPlannerOutput output = planningModule.handleRequest(request);

         messager.submitMessage(PlanningResult, output.getResult());
         messager.submitMessage(PlannerStatus, FootstepPlannerStatus.IDLE);

         if (output.getResult().validForExecution())
         {
            messager.submitMessage(FootstepPlanResponse, FootstepDataMessageConverter.createFootstepDataListFromPlan(output.getFootstepPlan(), -1.0, -1.0, ExecutionMode.OVERRIDE));
            if (!output.getLowLevelGoal().containsNaN())
            {
               messager.submitMessage(LowLevelGoalPosition, new Point3D(output.getLowLevelGoal().getPosition()));
               messager.submitMessage(LowLevelGoalOrientation, new Quaternion(output.getLowLevelGoal().getOrientation()));
            }
         }
      }
      catch (Exception e)
      {
         LogTools.error(e.getMessage());
         e.printStackTrace();
      }
   }

   private void sendPlannerStatistics()
   {
      if (footstepPlannerTypeReference.get().plansPath())
      {
         sendVisibilityGraphStatisticsMessages(planningModule.getBodyPathPlanner().getPlannerStatistics());
      }

      GraphSearchStatistics graphSearchStatistics = new GraphSearchStatistics();
      graphSearchStatistics.set(planningModule);
      sendGraphSearchPlannerStatisticsMessage(graphSearchStatistics);
   }

   private void sendPlannerStatisticsMessages(PlannerStatistics plannerStatistics)
   {
      switch (plannerStatistics.getStatisticsType())
      {
      case LIST:
         sendListOfStatisticsMessages((ListOfStatistics) plannerStatistics);
         break;
      case VISIBILITY_GRAPH:
         sendVisibilityGraphStatisticsMessages((VisibilityGraphStatistics) plannerStatistics);
         break;
      case GRAPH_SEARCH:
         sendGraphSearchPlannerStatisticsMessage((GraphSearchStatistics) plannerStatistics);
         break;
      }
   }

   private void sendListOfStatisticsMessages(ListOfStatistics listOfStatistics)
   {
      while (listOfStatistics.getNumberOfStatistics() > 0)
         sendPlannerStatisticsMessages(listOfStatistics.pollStatistics());
   }

   private void sendVisibilityGraphStatisticsMessages(VisibilityGraphStatistics statistics)
   {
      VisibilityMapHolder startMap = new VisibilityMapHolder()
      {
         @Override
         public int getMapId()
         {
            return statistics.getStartMapId();
         }

         @Override
         public VisibilityMap getVisibilityMapInLocal()
         {
            return statistics.getStartVisibilityMap();
         }

         @Override
         public VisibilityMap getVisibilityMapInWorld()
         {
            return statistics.getStartVisibilityMap();
         }
      };
      VisibilityMapHolder goalMap = new VisibilityMapHolder()
      {
         @Override
         public int getMapId()
         {
            return statistics.getGoalMapId();
         }

         @Override
         public VisibilityMap getVisibilityMapInLocal()
         {
            return statistics.getGoalVisibilityMap();
         }

         @Override
         public VisibilityMap getVisibilityMapInWorld()
         {
            return statistics.getGoalVisibilityMap();
         }
      };
      InterRegionVisibilityMap interRegionVisibilityMap = new InterRegionVisibilityMap();
      interRegionVisibilityMap.addConnections(statistics.getInterRegionsVisibilityMap().getConnections());

      List<VisibilityMapWithNavigableRegion> navigableRegionList = new ArrayList<>();
      for (int i = 0; i < statistics.getNumberOfNavigableRegions(); i++)
         navigableRegionList.add(statistics.getNavigableRegion(i));

      messager.submitMessage(StartVisibilityMap, startMap);
      messager.submitMessage(GoalVisibilityMap, goalMap);
      messager.submitMessage(VisibilityMapWithNavigableRegionData, navigableRegionList);
      messager.submitMessage(InterRegionVisibilityMap, interRegionVisibilityMap);
   }

   private void sendGraphSearchPlannerStatisticsMessage(GraphSearchStatistics graphSearchStatistics)
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.ExpandedNodesMap, graphSearchStatistics.getExpandedNodes());
      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepGraphPart, graphSearchStatistics.getFullGraph());
   }

   public static FootstepPathCalculatorModule createMessagerModule(SharedMemoryMessager messager)
   {
      return new FootstepPathCalculatorModule(messager);
   }
}