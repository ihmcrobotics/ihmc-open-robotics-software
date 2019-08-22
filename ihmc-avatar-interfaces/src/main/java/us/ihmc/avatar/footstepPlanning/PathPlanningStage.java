package us.ihmc.avatar.footstepPlanning;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.pathPlanners.SplinePathPlanner;
import us.ihmc.footstepPlanning.graphSearch.pathPlanners.VisibilityGraphPathPlanner;
import us.ihmc.footstepPlanning.graphSearch.pathPlanners.WaypointsForFootstepsPlanner;
import us.ihmc.pathPlanning.statistics.PlannerStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.providers.EnumProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class PathPlanningStage implements WaypointsForFootstepsPlanner
{
   private static final boolean debug = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private FootstepPlanningResult pathPlanResult = null;

   private final YoInteger sequenceId;
   private final YoBoolean initialize;

   private final EnumMap<FootstepPlannerType, WaypointsForFootstepsPlanner> plannerMap = new EnumMap<>(FootstepPlannerType.class);
   private final EnumProvider<FootstepPlannerType> activePlannerEnum;

   private final AtomicReference<List<Pose3DReadOnly>> waypoints = new AtomicReference<>();

   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>();
   private final AtomicReference<FootstepPlannerGoal> goal = new AtomicReference<>();
   private final AtomicReference<FramePose3D> stanceFootPose = new AtomicReference<>();
   private final AtomicReference<RobotSide> stanceFootSide = new AtomicReference<>();
   private final AtomicDouble timeout = new AtomicDouble();

   private final List<PlannerCompletionCallback> completionCallbackList = new ArrayList<>();

   private final int stageId;

   private Runnable stageRunnable;
   private IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher;


   public PathPlanningStage(int stageId, FootstepPlannerParametersReadOnly parameters, VisibilityGraphsParametersReadOnly visibilityGraphsParameters,
                            EnumProvider<FootstepPlannerType> activePlanner)
   {
      this.stageId = stageId;
      this.activePlannerEnum = activePlanner;

      String prefix = stageId + "_Path_";

      initialize = new YoBoolean(prefix + "Initialize" + registry.getName(), registry);
      sequenceId = new YoInteger(prefix + "PlanningSequenceId", registry);

      plannerMap.put(FootstepPlannerType.SIMPLE_BODY_PATH, new SplinePathPlanner(parameters, registry));
      plannerMap.put(FootstepPlannerType.VIS_GRAPH_WITH_A_STAR, new VisibilityGraphPathPlanner(prefix, parameters, visibilityGraphsParameters, registry));

      initialize.set(true);
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   private WaypointsForFootstepsPlanner getPlanner()
   {
      return plannerMap.get(activePlannerEnum.getValue());
   }

   public int getStageId()
   {
      return stageId;
   }

   public int getPlanSequenceId()
   {
      return sequenceId.getIntegerValue();
   }

   public void setPlanSequenceId(int sequenceId)
   {
      this.sequenceId.set(sequenceId);
   }


   public void addCompletionCallback(PlannerCompletionCallback completionCallback)
   {
      completionCallbackList.add(completionCallback);
   }

   @Override
   public void setInitialStanceFoot(FramePose3D stanceFootPose, RobotSide side)
   {
      this.stanceFootPose.set(stanceFootPose);
      this.stanceFootSide.set(side);
   }

   @Override
   public void setGoal(FootstepPlannerGoal goal)
   {
      this.goal.set(goal);
   }

   @Override
   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      this.planarRegionsList.set(planarRegionsList == null ? null : planarRegionsList.copy());
   }

   @Override
   public void setTimeout(double timeout)
   {
      this.timeout.set(timeout);
   }

   @Override
   public void computeBestEffortPlan(double horizonLength)
   {
      getPlanner().computeBestEffortPlan(horizonLength);
   }

   @Override
   public FootstepPlanningResult planWaypoints()
   {
      return getPlanner().planWaypoints();
   }

   @Override
   public List<Pose3DReadOnly> getWaypoints()
   {
      return waypoints.getAndSet(null);
   }

   @Override
   public PlannerStatistics<?> getPlannerStatistics()
   {
      return getPlanner().getPlannerStatistics();
   }

   public void requestInitialize()
   {
      initialize.set(true);
   }

   public void update()
   {
      if (initialize.getBooleanValue())
      {
         if (!initialize())
            return;
         initialize.set(false);
      }

      updateInternal();
   }

   public boolean initialize()
   {
      pathPlanResult = null;

      getPlanner().setInitialStanceFoot(stanceFootPose.get(), stanceFootSide.get());
      getPlanner().setGoal(goal.get());
      getPlanner().setPlanarRegionsList(planarRegionsList.get());
      getPlanner().setTimeout(timeout.get());

      return true;
   }

   @Override
   public void cancelPlanning()
   {
      getPlanner().cancelPlanning();
      pathPlanResult = null;
      waypoints.set(null);
      for (PlannerCompletionCallback completionCallback : completionCallbackList)
      {
         completionCallback.pathPlanningIsComplete(pathPlanResult, this);
      }
   }


   public void updateInternal()
   {
      sendMessageToUI(
            "Starting To Plan Path, using type " + activePlannerEnum.getValue().toString());

      if (debug)
         PrintTools.info("Planning path.");

      pathPlanResult = planWaypoints();
      if (pathPlanResult.validForExecution())
         waypoints.set(getPlanner().getWaypoints());

      for (PlannerCompletionCallback completionCallback : completionCallbackList)
         completionCallback.pathPlanningIsComplete(pathPlanResult, this);
   }


   public Runnable createStageRunnable()
   {
      if (stageRunnable != null)
      {
         if (debug)
            PrintTools.error(this, "stageRunnable is not null.");
         return null;
      }

      stageRunnable = new Runnable()
      {
         @Override
         public void run()
         {
            if (Thread.interrupted())
               return;

            update();
         }
      };

      return stageRunnable;
   }

   public void destroyStageRunnable()
   {
      stageRunnable = null;
   }

   public void setTextToSpeechPublisher(IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher)
   {
      this.textToSpeechPublisher = textToSpeechPublisher;
   }

   private void sendMessageToUI(String message)
   {
      textToSpeechPublisher.publish(MessageTools.createTextToSpeechPacket(message));
   }
}
