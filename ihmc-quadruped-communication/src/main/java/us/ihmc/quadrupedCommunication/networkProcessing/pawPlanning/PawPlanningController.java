package us.ihmc.quadrupedCommunication.networkProcessing.pawPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.visibilityGraphs.YoVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.quadrupedCommunication.QuadrupedMessageTools;
import us.ihmc.quadrupedCommunication.networkProcessing.OutputManager;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedRobotDataReceiver;
import us.ihmc.quadrupedCommunication.networkProcessing.QuadrupedToolboxController;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.*;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.AStarPawPlanner;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.VisibilityGraphWithAStarPawPlanner;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.YoPawPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.turnWalkTurn.QuadrupedSplineWithTurnWalkTurnPlanner;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.turnWalkTurn.QuadrupedVisGraphWithTurnWalkTurnPlanner;
import us.ihmc.quadrupedFootstepPlanning.pathPlanning.WaypointsForPawPlanner;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.atomic.AtomicReference;

public class PawPlanningController extends QuadrupedToolboxController
{
   private final YoEnum<PawPlannerType> activePlanner = new YoEnum<>("activePlanner", registry, PawPlannerType.class);
   private final EnumMap<PawPlannerType, BodyPathAndPawPlanner> plannerMap = new EnumMap<>(PawPlannerType.class);

   private final AtomicReference<HighLevelStateChangeStatusMessage> controllerStateChangeMessage = new AtomicReference<>();
   private final AtomicReference<QuadrupedSteppingStateChangeMessage> steppingStateChangeMessage = new AtomicReference<>();

   private final AtomicReference<QuadrupedFootstepPlanningRequestPacket> latestRequestReference = new AtomicReference<>(null);
   private Optional<PlanarRegionsList> planarRegionsList = Optional.empty();

   private final YoQuadrupedXGaitSettings xGaitSettings;
   private final YoVisibilityGraphParameters visibilityGraphParameters;
   private final YoPawPlannerParameters footstepPlannerParameters;
   private final AtomicLong robotTimestampNanos = new AtomicLong();
   private final YoDouble robotTimestamp = new YoDouble("robotTimestamp", registry);
   private final YoBoolean isDone = new YoBoolean("isDone", registry);
   private final YoDouble timeout = new YoDouble("toolboxTimeout", registry);
   private final YoInteger planId = new YoInteger("planId", registry);


   public PawPlanningController(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, VisibilityGraphsParameters defaultVisibilityGraphParameters,
                                PawPlannerParameters defaultPawPlannerParameters, PointFootSnapperParameters pointFootSnapperParameters,
                                OutputManager statusOutputManager, QuadrupedRobotDataReceiver robotDataReceiver,
                                YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry, long tickTimeMs)
   {
      super(robotDataReceiver, statusOutputManager, parentRegistry);

      xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);
      visibilityGraphParameters = new YoVisibilityGraphParameters(defaultVisibilityGraphParameters, registry);
      footstepPlannerParameters = new YoPawPlannerParameters(defaultPawPlannerParameters, registry);

      if (robotDataReceiver != null)
      {
         plannerMap.put(PawPlannerType.SIMPLE_PATH_TURN_WALK_TURN,
                        new QuadrupedSplineWithTurnWalkTurnPlanner(xGaitSettings, robotTimestamp, pointFootSnapperParameters, robotDataReceiver.getReferenceFrames(), null, registry));
         plannerMap.put(PawPlannerType.VIS_GRAPH_WITH_TURN_WALK_TURN,
                        new QuadrupedVisGraphWithTurnWalkTurnPlanner(xGaitSettings, visibilityGraphParameters, robotTimestamp, pointFootSnapperParameters,
                                                                     robotDataReceiver.getReferenceFrames(), null, registry));
      }
      plannerMap.put(PawPlannerType.A_STAR,
                     AStarPawPlanner.createPlanner(footstepPlannerParameters, xGaitSettings, null, registry));
      plannerMap.put(PawPlannerType.VIS_GRAPH_WITH_A_STAR, new VisibilityGraphWithAStarPawPlanner(footstepPlannerParameters, xGaitSettings,
                                                                                                  visibilityGraphParameters, graphicsListRegistry, registry));
      activePlanner.set(PawPlannerType.SIMPLE_PATH_TURN_WALK_TURN);

      planId.set(FootstepPlanningRequestPacket.NO_PLAN_ID);
   }

   public void processHighLevelStateChangeMessage(HighLevelStateChangeStatusMessage message)
   {
      controllerStateChangeMessage.set(message);
   }

   public void processSteppingStateChangeMessage(QuadrupedSteppingStateChangeMessage message)
   {
      steppingStateChangeMessage.set(message);
   }

   public void processGroundPlaneMessage(QuadrupedGroundPlaneMessage message)
   {
      for (PawPlannerType plannerKey : plannerMap.keySet())
      {
         BodyPathAndPawPlanner planner = plannerMap.get(plannerKey);
         planner.setGroundPlane(message);
      }
   }

   public void processSupportRegionParameters(QuadrupedSupportPlanarRegionParametersMessage message)
   {
      for (PawPlannerType plannerKey : plannerMap.keySet())
      {
         WaypointsForPawPlanner planner = plannerMap.get(plannerKey).getWaypointPathPlanner();
         if (planner != null)
            planner.setFallbackRegionSize(message.getInsideSupportRegionSize());
      }
   }

   public void processFootstepPlannerParametersPacket(QuadrupedPawPlannerParametersPacket packet)
   {
      footstepPlannerParameters.set(packet);
   }

   public void processVisibilityGraphParametersPacket(VisibilityGraphsParametersPacket packet)
   {
      visibilityGraphParameters.set(packet);
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

      planId.set(request.getPlannerRequestId());
      if (request.getRequestedFootstepPlannerType() >= 0)
         activePlanner.set(PawPlannerType.fromByte(request.getRequestedFootstepPlannerType()));

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

      BodyPathAndPawPlanner planner = plannerMap.get(activePlanner.getEnumValue());

      if (planarRegionsList.isPresent())
      {
         planner.setPlanarRegionsList(planarRegionsList.get());
      }
      else
      {
         planner.setPlanarRegionsList(null);
      }

      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame(), request.getGoalPositionInWorld(), request.getGoalOrientationInWorld());

      PawPlannerStart start = new PawPlannerStart();
      PawPlannerGoal goal = new PawPlannerGoal();

      PawPlannerTargetType targetType = PawPlannerTargetType.fromByte(request.getStartTargetType());
      if (targetType == PawPlannerTargetType.POSE_BETWEEN_FEET)
      {
         FramePose3D initialPose = new FramePose3D(ReferenceFrame.getWorldFrame(), request.getBodyPositionInWorld(), request.getBodyOrientationInWorld());
         start.setStartPose(initialPose);
         start.setStartType(targetType);
      }
      else
      {
         start.setPawStartPosition(RobotQuadrant.FRONT_LEFT, new FramePoint3D(ReferenceFrame.getWorldFrame(), request.getFrontLeftPositionInWorld()));
         start.setPawStartPosition(RobotQuadrant.FRONT_RIGHT, new FramePoint3D(ReferenceFrame.getWorldFrame(), request.getFrontRightPositionInWorld()));
         start.setPawStartPosition(RobotQuadrant.HIND_LEFT, new FramePoint3D(ReferenceFrame.getWorldFrame(), request.getHindLeftPositionInWorld()));
         start.setPawStartPosition(RobotQuadrant.HIND_RIGHT, new FramePoint3D(ReferenceFrame.getWorldFrame(), request.getHindRightPositionInWorld()));
         start.setStartType(PawPlannerTargetType.FOOTSTEPS);
      }

      start.setInitialQuadrant(RobotQuadrant.fromByte(request.getInitialStepRobotQuadrant()));
      goal.setGoalPose(goalPose);

      double horizonLength = request.getHorizonLength();
      if (horizonLength > 0.0 && Double.isFinite(horizonLength))
         planner.setPlanningHorizonLength(horizonLength);

      double timeout = request.getTimeout();
      if (timeout > 0.0 && Double.isFinite(timeout))
      {
         this.timeout.set(timeout);
         planner.setTimeout(timeout);
      }
      else
      {
         planner.setTimeout(Double.POSITIVE_INFINITY);
      }

      double bestEffortTimeout = request.getBestEffortTimeout();
      if (bestEffortTimeout > 0.0 && Double.isFinite(bestEffortTimeout))
      {
         planner.setBestEffortTimeout(bestEffortTimeout);
      }
      else
      {
         planner.setBestEffortTimeout(0.0);
      }

      planner.setStart(start);
      planner.setGoal(goal);

      return true;
   }

   @Override
   public void updateInternal()
   {
      robotTimestamp.set(Conversions.nanosecondsToSeconds(robotTimestampNanos.get()));

      BodyPathAndPawPlanner planner = plannerMap.get(activePlanner.getEnumValue());

      reportMessage(packStatus(PawPlannerStatus.PLANNING_PATH));

      PawPlanningResult status = planner.planPath();

      BodyPathPlan bodyPathPlan = null;
      if (status.validForExecution())
      {
         bodyPathPlan = planner.getPathPlan();
         reportMessage(packStatus(PawPlannerStatus.PLANNING_STEPS));
         reportMessage(packPathResult(bodyPathPlan, status));

         status = planner.plan();
      }

      PawPlan pawPlan;
      if (status.validForExecution())
         pawPlan = planner.getPlan();
      else
         pawPlan = null;

      reportMessage(packStepResult(pawPlan, bodyPathPlan, status));

      finishUp();
   }

   public void finishUp()
   {
      if (DEBUG)
         PrintTools.info("Finishing up the planner");
      plannerMap.get(activePlanner.getEnumValue()).cancelPlanning();
      reportMessage(packStatus(PawPlannerStatus.IDLE));
      isDone.set(true);
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   private FootstepPlannerStatusMessage packStatus(PawPlannerStatus status)
   {
      FootstepPlannerStatusMessage message = new FootstepPlannerStatusMessage();
      message.setFootstepPlannerStatus(status.toByte());

      return message;
   }

   private BodyPathPlanMessage packPathResult(BodyPathPlan bodyPathPlan, PawPlanningResult status)
   {
      if (DEBUG)
      {
         PrintTools.info("Finished planning path. Result: " + status);
      }

      BodyPathPlanMessage result = new BodyPathPlanMessage();
      if (bodyPathPlan != null)
      {
         for (int i = 0; i < bodyPathPlan.getNumberOfWaypoints(); i++)
            result.getBodyPath().add().set(bodyPathPlan.getWaypoint(i));

         result.getPathPlannerStartPose().set(bodyPathPlan.getStartPose());
         result.getPathPlannerGoalPose().set(bodyPathPlan.getGoalPose());
      }

      planarRegionsList.ifPresent(regions -> result.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regions)));
      result.setPlanId(planId.getIntegerValue());
      result.setFootstepPlanningResult(status.toByte());
      return result;
   }

   private QuadrupedFootstepPlanningToolboxOutputStatus packStepResult(PawPlan pawPlan, BodyPathPlan bodyPathPlan, PawPlanningResult status)
   {
      QuadrupedFootstepPlanningToolboxOutputStatus result = new QuadrupedFootstepPlanningToolboxOutputStatus();
      if (pawPlan == null)
      {
         result.getFootstepDataList().set(new QuadrupedTimedStepListMessage());
      }
      else
      {
         result.getFootstepDataList().set(convertToTimedStepListMessage(pawPlan));
         result.getLowLevelPlannerGoal().set(pawPlan.getLowLevelPlanGoal());
      }

      if (bodyPathPlan != null)
      {
         for (int i = 0; i < bodyPathPlan.getNumberOfWaypoints(); i++)
            result.getBodyPath().add().set(bodyPathPlan.getWaypoint(i));
      }

      planarRegionsList.ifPresent(regions -> result.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regions)));
      result.setPlanId(planId.getIntegerValue());
      result.setFootstepPlanningResult(status.toByte());
      result.setTimeTaken(plannerMap.get(activePlanner.getEnumValue()).getPlanningDuration());

      return result;
   }

   private static QuadrupedTimedStepListMessage convertToTimedStepListMessage(PawPlan pawPlan)
   {
      if (pawPlan == null)
         return null;

      List<QuadrupedTimedStepMessage> stepMessages = new ArrayList<>();
      for (int i = 0; i < pawPlan.getNumberOfSteps(); i++)
      {
         stepMessages.add(QuadrupedMessageTools.createQuadrupedTimedStepMessage(pawPlan.getPawStep(i)));
      }

      return QuadrupedMessageTools.createQuadrupedTimedStepListMessage(stepMessages, false);
   }
}
