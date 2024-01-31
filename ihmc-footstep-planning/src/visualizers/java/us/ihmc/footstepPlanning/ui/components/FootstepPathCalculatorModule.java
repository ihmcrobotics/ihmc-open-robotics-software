package us.ihmc.footstepPlanning.ui.components;

import perception_msgs.msg.dds.HeightMapMessage;
import toolbox_msgs.msg.dds.FootstepPlanningTimingsMessage;
import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.tools.FootstepPlannerRejectionReasonReport;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

public class FootstepPathCalculatorModule
{
   private static final boolean VERBOSE = true;

   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final AtomicReference<PlanarRegionsList> planarRegionsReference;
   private final AtomicReference<HeightMapMessage> heightMapReference;
   private final AtomicReference<RobotSide> initialStanceSideReference;
   private final AtomicReference<Pose3DReadOnly> leftFootStartPose;
   private final AtomicReference<Pose3DReadOnly> rightFootStartPose;
   private final AtomicReference<Pose3DReadOnly> leftFootGoalPose;
   private final AtomicReference<Pose3DReadOnly> rightFootGoalPose;
   private final AtomicReference<Boolean> performAStarSearch;
   private final AtomicReference<Boolean> planBodyPath;
   private final AtomicReference<Boolean> planNarrowPassage;

   private final AtomicReference<Double> plannerTimeoutReference;
   private final AtomicReference<Integer> plannerMaxIterationsReference;
   private final AtomicReference<Double> plannerHorizonLengthReference;

   private final AtomicReference<Boolean> snapGoalSteps;
   private final AtomicReference<Boolean> abortIfGoalStepSnapFails;

   private final AtomicReference<FootstepPlannerParametersReadOnly> parameters;
   private final AtomicReference<AStarBodyPathPlannerParametersReadOnly> pathPlannerParameters;

   private final Messager messager;
   private final FootstepPlanningModule planningModule = new FootstepPlanningModule(getClass().getSimpleName());

   public FootstepPathCalculatorModule(Messager messager)
   {
      this.messager = messager;

      planarRegionsReference = messager.createInput(PlanarRegionData);
      heightMapReference = messager.createInput(HeightMapData);
      initialStanceSideReference = messager.createInput(InitialSupportSide, RobotSide.LEFT);
      leftFootStartPose = messager.createInput(LeftFootPose);
      rightFootStartPose = messager.createInput(RightFootPose);
      leftFootGoalPose = messager.createInput(LeftFootGoalPose);
      rightFootGoalPose = messager.createInput(RightFootGoalPose);

      parameters = messager.createInput(PlannerParameters, new DefaultFootstepPlannerParameters());
      pathPlannerParameters = messager.createInput(AStarBodyPathPlannerParameters, new AStarBodyPathPlannerParameters());
      performAStarSearch = messager.createInput(PerformAStarSearch, false);
      planBodyPath = messager.createInput(PlanBodyPath, true);
      planNarrowPassage = messager.createInput(PlanNarrowPassage, true);
      plannerTimeoutReference = messager.createInput(PlannerTimeout, 5.0);
      plannerHorizonLengthReference = messager.createInput(PlannerHorizonLength, 1.0);

      plannerMaxIterationsReference = messager.createInput(MaxIterations, -1);
      snapGoalSteps = messager.createInput(SnapGoalSteps, false);
      abortIfGoalStepSnapFails = messager.createInput(AbortIfGoalStepSnapFails, false);

      messager.addTopicListener(ComputePath, request -> computePathOnThread());
      new FootPoseFromMidFootUpdater(messager).start();
      new FootstepCompletionListener(messager).start();

      messager.addTopicListener(HaltPlanning, halt -> planningModule.halt());
   }

   public void clear()
   {
      planarRegionsReference.set(null);
      heightMapReference.set(null);
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

      HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(heightMapReference.get());

      if (heightMapData == null)
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
         request.setHeightMapData(heightMapData);
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
         planningModule.getAStarBodyPathPlannerParameters().set(pathPlannerParameters.get());

         planningModule.addStatusCallback(status -> messager.submitMessage(FootstepPlanningResultTopic, status.getFootstepPlanningResult()));

         FootstepPlannerOutput output = planningModule.handleRequest(request);

         LogTools.info("Footstep planner completed with {}, {} step(s)",
                       output.getFootstepPlanningResult(),
                       output.getFootstepPlan().getNumberOfSteps());
         if (output.getFootstepPlan().getNumberOfSteps() < 1) // failed
         {
            FootstepPlannerRejectionReasonReport rejectionReasonReport = new FootstepPlannerRejectionReasonReport(planningModule);
            rejectionReasonReport.update();
            for (BipedalFootstepPlannerNodeRejectionReason reason : rejectionReasonReport.getSortedReasons())
            {
               double rejectionPercentage = rejectionReasonReport.getRejectionReasonPercentage(reason);
               LogTools.info("Rejection {}%: {}", FormattingTools.getFormattedToSignificantFigures(rejectionPercentage, 3), reason);
            }
         }

         messager.submitMessage(BodyPathPlanningResultTopic, output.getBodyPathPlanningResult());
         messager.submitMessage(FootstepPlanningResultTopic, output.getFootstepPlanningResult());

         messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanResponse,
                                FootstepDataMessageConverter.createFootstepDataListFromPlan(output.getFootstepPlan(), -1.0, -1.0));
         messager.submitMessage(FootstepPlannerMessagerAPI.ReceivedPlanId, output.getRequestId());
         messager.submitMessage(FootstepPlannerMessagerAPI.BodyPathData, Pair.of(output.getBodyPath(), output.getBodyPathUnsmoothed()));

         if (output.getGoalPose() != null)
         {
            messager.submitMessage(FootstepPlannerMessagerAPI.LowLevelGoalPosition, output.getGoalPose().getPosition());
            messager.submitMessage(FootstepPlannerMessagerAPI.LowLevelGoalOrientation, output.getGoalPose().getOrientation());
         }
         if (output.getFootstepPlanningResult() == FootstepPlanningResult.EXCEPTION)
         {
            StringBuilder stackTrace = new StringBuilder();
            StackTraceElement[] stackTraceArray = output.getException().getStackTrace();
            for (int i = 0; i < Math.min(stackTraceArray.length, 20); i++)
            {
               stackTrace.append(stackTraceArray[i].toString());
            }
            messager.submitMessage(FootstepPlannerMessagerAPI.PlannerExceptionStackTrace, stackTrace.toString());
         }
         else
         {
            messager.submitMessage(FootstepPlannerMessagerAPI.PlannerExceptionStackTrace,
                                   "No stack trace available, planner status wasn't " + FootstepPlanningResult.EXCEPTION + ", it was: " + output.getFootstepPlanningResult());
         }

         FootstepPlanningTimingsMessage timingsMessage = new FootstepPlanningTimingsMessage();
         output.getPlannerTimings().setPacket(timingsMessage);
         messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimings, timingsMessage);

         // broadcast log data
         messager.submitMessage(FootstepPlannerMessagerAPI.GraphData,
                                Triple.of(planningModule.getEdgeDataMap(), planningModule.getIterationData(), planningModule.getFootstepPlanVariableDescriptors()));
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

   public static FootstepPathCalculatorModule createMessagerModule(SharedMemoryMessager messager)
   {
      return new FootstepPathCalculatorModule(messager);
   }
}