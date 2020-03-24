package us.ihmc.footstepPlanning.ui.components;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.BodyPathData;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ComputePath;
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
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.PlanningResult;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.RequestPlannerStatistics;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.StartVisibilityMap;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.VisibilityGraphsParameters;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.VisibilityMapWithNavigableRegionData;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlannerStatus;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.pathPlanning.statistics.ListOfStatistics;
import us.ihmc.pathPlanning.statistics.PlannerStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphHolder;
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
   private final AtomicReference<RobotSide> initialStanceSideReference;
   private final AtomicReference<Pose3DReadOnly> leftFootStartPose;
   private final AtomicReference<Pose3DReadOnly> rightFootStartPose;
   private final AtomicReference<Pose3DReadOnly> leftFootGoalPose;
   private final AtomicReference<Pose3DReadOnly> rightFootGoalPose;
   private final AtomicReference<Boolean> performAStarSearch;
   private final AtomicReference<Boolean> planBodyPath;

   private final AtomicReference<Double> plannerTimeoutReference;
   private final AtomicReference<Integer> plannerMaxIterationsReference;
   private final AtomicReference<Double> plannerHorizonLengthReference;

   private final AtomicReference<Boolean> snapGoalSteps;
   private final AtomicReference<Boolean> abortIfGoalStepSnapFails;

   private final AtomicReference<FootstepPlannerParametersReadOnly> parameters;
   private final AtomicReference<VisibilityGraphsParametersReadOnly> visibilityGraphsParameters;

   private final Messager messager;
   private final FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());

   public FootstepPathCalculatorModule(Messager messager)
   {
      this.messager = messager;

      planarRegionsReference = messager.createInput(PlanarRegionData);
      initialStanceSideReference = messager.createInput(InitialSupportSide, RobotSide.LEFT);
      leftFootStartPose = messager.createInput(LeftFootPose);
      rightFootStartPose = messager.createInput(RightFootPose);
      leftFootGoalPose = messager.createInput(LeftFootGoalPose);
      rightFootGoalPose = messager.createInput(RightFootGoalPose);

      parameters = messager.createInput(PlannerParameters, new DefaultFootstepPlannerParameters());
      visibilityGraphsParameters = messager.createInput(VisibilityGraphsParameters, new DefaultVisibilityGraphParameters());
      performAStarSearch = messager.createInput(PerformAStarSearch, false);
      planBodyPath = messager.createInput(PlanBodyPath, true);
      plannerTimeoutReference = messager.createInput(PlannerTimeout, 5.0);
      plannerHorizonLengthReference = messager.createInput(PlannerHorizonLength, 1.0);

      plannerMaxIterationsReference = messager.createInput(MaxIterations, -1);
      snapGoalSteps = messager.createInput(SnapGoalSteps, false);
      abortIfGoalStepSnapFails = messager.createInput(AbortIfGoalStepSnapFails, false);

      messager.registerTopicListener(ComputePath, request -> computePathOnThread());
      messager.registerTopicListener(RequestPlannerStatistics, request -> sendVisibilityGraph());

      new FootPoseFromMidFootUpdater(messager).start();
   }

   public void clear()
   {
      planarRegionsReference.set(null);
      initialStanceSideReference.set(null);
      leftFootStartPose.set(null);
      rightFootStartPose.set(null);
      leftFootGoalPose.set(null);
      rightFootGoalPose.set(null);
      plannerTimeoutReference.set(null);
      plannerMaxIterationsReference.set(null);
      plannerHorizonLengthReference.set(null);
      snapGoalSteps.set(null);
      abortIfGoalStepSnapFails.set(null);
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

      if (leftFootStartPose.get() == null || rightFootStartPose.get() == null)
         return;

      if (leftFootGoalPose.get() == null || rightFootGoalPose.get() == null)
         return;

      if (VERBOSE)
         LogTools.info("Computing footstep path.");

      try
      {
         FootstepPlannerRequest request = new FootstepPlannerRequest();
         request.setPlanarRegionsList(planarRegionsList);
         request.setTimeout(plannerTimeoutReference.get());
         request.setMaximumIterations(plannerMaxIterationsReference.get());
         request.setHorizonLength(plannerHorizonLengthReference.get());
         request.setRequestedInitialStanceSide(initialStanceSideReference.get());
         request.setStartFootPoses(leftFootStartPose.get(), rightFootStartPose.get());
         request.setGoalFootPoses(leftFootGoalPose.get(), rightFootGoalPose.get());
         request.setPlanBodyPath(planBodyPath.get());
         request.setPerformAStarSearch(performAStarSearch.get());
         request.setSnapGoalSteps(snapGoalSteps.get());
         request.setAbortIfGoalStepSnappingFails(abortIfGoalStepSnapFails.get());

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

   public FootstepPlanningModule getPlanningModule()
   {
      return planningModule;
   }

   private void sendVisibilityGraph()
   {
      VisibilityGraphHolder visibilityGraphHolder = planningModule.getBodyPathPlanner().getVisibilityGraphHolder();
      if (visibilityGraphHolder.getStartVisibilityMap().isEmpty())
      {
         return;
      }

      VisibilityMapHolder startMap = new VisibilityMapHolder()
      {
         @Override
         public int getMapId()
         {
            return visibilityGraphHolder.getStartMapId();
         }

         @Override
         public VisibilityMap getVisibilityMapInLocal()
         {
            return visibilityGraphHolder.getStartVisibilityMap();
         }

         @Override
         public VisibilityMap getVisibilityMapInWorld()
         {
            return visibilityGraphHolder.getStartVisibilityMap();
         }
      };
      VisibilityMapHolder goalMap = new VisibilityMapHolder()
      {
         @Override
         public int getMapId()
         {
            return visibilityGraphHolder.getGoalMapId();
         }

         @Override
         public VisibilityMap getVisibilityMapInLocal()
         {
            return visibilityGraphHolder.getGoalVisibilityMap();
         }

         @Override
         public VisibilityMap getVisibilityMapInWorld()
         {
            return visibilityGraphHolder.getGoalVisibilityMap();
         }
      };
      InterRegionVisibilityMap interRegionVisibilityMap = new InterRegionVisibilityMap();
      interRegionVisibilityMap.addConnections(visibilityGraphHolder.getInterRegionsVisibilityMap().getConnections());

      List<VisibilityMapWithNavigableRegion> navigableRegionList = new ArrayList<>();
      for (int i = 0; i < visibilityGraphHolder.getNumberOfNavigableRegions(); i++)
         navigableRegionList.add(visibilityGraphHolder.getNavigableRegion(i));

      messager.submitMessage(StartVisibilityMap, startMap);
      messager.submitMessage(GoalVisibilityMap, goalMap);
      messager.submitMessage(VisibilityMapWithNavigableRegionData, navigableRegionList);
      messager.submitMessage(InterRegionVisibilityMap, interRegionVisibilityMap);
   }

   public static FootstepPathCalculatorModule createMessagerModule(SharedMemoryMessager messager)
   {
      return new FootstepPathCalculatorModule(messager);
   }
}