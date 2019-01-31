package us.ihmc.quadrupedCommunication.networkProcessing.footstepPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.visibilityGraphs.YoVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedCommunication.networkProcessing.OutputManager;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedRobotDataReceiver;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlannerType;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.QuadrupedBodyPathAndFootstepPlanner;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.QuadrupedFootstepPlannerGoal;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.QuadrupedFootstepPlannerStart;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.QuadrupedAStarFootstepPlanner;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.turnWalkTurn.QuadrupedSplineWithTurnWalkTurnPlanner;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.turnWalkTurn.QuadrupedVisGraphWithTurnWalkTurnPlanner;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedFootstepPlanningController extends QuadrupedToolboxController
{
   private final YoEnum<FootstepPlannerType> activePlanner = new YoEnum<>("activePlanner", registry, FootstepPlannerType.class);
   private final EnumMap<FootstepPlannerType, QuadrupedBodyPathAndFootstepPlanner> plannerMap = new EnumMap<>(FootstepPlannerType.class);

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();

   private final AtomicReference<QuadrupedFootstepPlanningRequestPacket> latestRequestReference = new AtomicReference<>(null);
   private Optional<PlanarRegionsList> planarRegionsList = Optional.empty();

   private final YoQuadrupedXGaitSettings xGaitSettings;
   private final YoVisibilityGraphParameters visibilityGraphParameters;
   private final AtomicLong robotTimestampNanos = new AtomicLong();
   private final YoDouble robotTimestamp = new YoDouble("robotTimestamp", registry);
   private final YoBoolean isDone = new YoBoolean("isDone", registry);
   private final YoDouble timeout = new YoDouble("toolboxTimeout", registry);

   public QuadrupedFootstepPlanningController(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, VisibilityGraphsParameters visibilityGraphParameters,
                                              FootstepPlannerParameters footstepPlannerParameters, PointFootSnapperParameters pointFootSnapperParameters,
                                              OutputManager statusOutputManager, QuadrupedRobotDataReceiver robotDataReceiver,
                                              YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry, long tickTimeMs)
   {
      super(robotDataReceiver, statusOutputManager, parentRegistry);

      xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);
      this.visibilityGraphParameters = new YoVisibilityGraphParameters(visibilityGraphParameters, registry);

      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(footstepPlannerParameters, xGaitSettings);
      plannerMap.put(FootstepPlannerType.SIMPLE_PATH_TURN_WALK_TURN,
                     new QuadrupedSplineWithTurnWalkTurnPlanner(xGaitSettings, robotTimestamp, pointFootSnapperParameters,
                                                                robotDataReceiver.getReferenceFrames(), null, registry));
      plannerMap.put(FootstepPlannerType.VIS_GRAPH_WITH_TURN_WALK_TURN,
                     new QuadrupedVisGraphWithTurnWalkTurnPlanner(xGaitSettings, this.visibilityGraphParameters, robotTimestamp, pointFootSnapperParameters,
                                                                  robotDataReceiver.getReferenceFrames(), null, registry));
      plannerMap.put(FootstepPlannerType.A_STAR,
                     QuadrupedAStarFootstepPlanner.createPlanner(footstepPlannerParameters, defaultXGaitSettings, null, expansion, registry));
      activePlanner.set(FootstepPlannerType.SIMPLE_PATH_TURN_WALK_TURN);
   }

   public void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      controllerStateChangeMessage.set(message);
   }

   public void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      steppingStateChangeMessage.set(message);
   }

   public void processPlanarRegionsListMessage(PlanarRegionsListMessage message)
   {
      for (FootstepPlannerType plannerKey : plannerMap.keySet())
      {
         QuadrupedBodyPathAndFootstepPlanner planner = plannerMap.get(plannerKey);
         planner.setPlanarRegionsList(PlanarRegionMessageConverter.convertToPlanarRegionsList(message));
      }
   }

   public void processGroundPlaneMessage(QuadrupedGroundPlaneMessage message)
   {
      for (FootstepPlannerType plannerKey : plannerMap.keySet())
      {
         QuadrupedBodyPathAndFootstepPlanner planner = plannerMap.get(plannerKey);
         planner.setGroundPlane(message);
      }
   }

   public void processXGaitSettingsPacket(QuadrupedXGaitSettingsPacket packet)
   {
      xGaitSettings.set(packet);
   }

   public void processRobotTimestamp(long timestampInNanos)
   {
      this.robotTimestampNanos.set(timestampInNanos);
   }

   public void processFootstepPlanningRequest(QuadrupedFootstepPlanningRequestPacket footstepPlanningRequestPacket)
   {
      latestRequestReference.set(footstepPlanningRequestPacket);
   }

   @Override
   public boolean initializeInternal()
   {
      robotTimestamp.set(Conversions.nanosecondsToSeconds(robotTimestampNanos.get()));

      isDone.set(false);

      QuadrupedFootstepPlanningRequestPacket request = latestRequestReference.getAndSet(null);
      if (request == null)
         return false;

      activePlanner.set(FootstepPlannerType.fromByte(request.getRequestedFootstepPlannerType()));

      PlanarRegionsListMessage planarRegionsListMessage = request.getPlanarRegionsListMessage();
      if (planarRegionsListMessage == null)
      {
         this.planarRegionsList = Optional.empty();
      }
      else
      {
         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
         if (planarRegionsList.isEmpty())
            this.planarRegionsList = Optional.empty();
         else
            this.planarRegionsList = Optional.of(planarRegionsList);
      }

      QuadrupedBodyPathAndFootstepPlanner planner = plannerMap.get(activePlanner.getEnumValue());

      FramePose3D initialPose = new FramePose3D(ReferenceFrame.getWorldFrame(), request.getBodyPositionInWorld(), request.getBodyOrientationInWorld());
      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame(), request.getGoalPositionInWorld(), request.getGoalOrientationInWorld());

      QuadrupedFootstepPlannerStart start = new QuadrupedFootstepPlannerStart();
      QuadrupedFootstepPlannerGoal goal = new QuadrupedFootstepPlannerGoal();
      start.setStartPose(initialPose);
      goal.setGoalPose(goalPose);

      planner.setStart(start);
      planner.setGoal(goal);

      return true;
   }

   @Override
   public void updateInternal()
   {
      robotTimestamp.set(Conversions.nanosecondsToSeconds(robotTimestampNanos.get()));

      QuadrupedBodyPathAndFootstepPlanner planner = plannerMap.get(activePlanner.getEnumValue());

      if (planarRegionsList.isPresent())
      {
         planner.setPlanarRegionsList(planarRegionsList.get());
      }
      else
      {
         planner.setPlanarRegionsList(null);
      }

      planner.planPath();
      planner.plan();

      reportMessage(convertToMessage(planner.getSteps()));
      isDone.set(true);
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   private static QuadrupedTimedStepListMessage convertToMessage(List<? extends QuadrupedTimedStep> steps)
   {
      if (steps == null)
         return null;

      List<QuadrupedTimedStepMessage> stepMessages = new ArrayList<>();
      for (int i = 0; i < steps.size(); i++)
      {
         stepMessages.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(steps.get(i)));
      }

      return QuadrupedMessageTools.createQuadrupedTimedStepListMessage(stepMessages, false);
   }
}
