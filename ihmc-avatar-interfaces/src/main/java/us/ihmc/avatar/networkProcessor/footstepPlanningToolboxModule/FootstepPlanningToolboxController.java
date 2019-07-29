package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.BodyPathPlanMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlannerParametersPacket;
import controller_msgs.msg.dds.FootstepPlannerStatusMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.TextToSpeechPacket;
import controller_msgs.msg.dds.VisibilityGraphsParametersPacket;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.RosBasedPlannerListener;
import us.ihmc.footstepPlanning.graphSearch.heuristics.DistanceAndYawBasedHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.*;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.graphSearch.parameters.YoFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.planners.DepthFirstFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.planners.SplinePathWithAStarPlanner;
import us.ihmc.footstepPlanning.graphSearch.planners.VisibilityGraphWithAStarPlanner;
import us.ihmc.footstepPlanning.graphSearch.stepCost.ConstantFootstepCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCostBuilder;
import us.ihmc.footstepPlanning.simplePlanners.PlanThenSnapPlanner;
import us.ihmc.footstepPlanning.simplePlanners.TurnWalkTurnPlanner;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.statistics.ListOfStatistics;
import us.ihmc.pathPlanning.statistics.PlannerStatistics;
import us.ihmc.pathPlanning.statistics.VisibilityGraphStatistics;
import us.ihmc.pathPlanning.visibilityGraphs.VisibilityGraphMessagesConverter;
import us.ihmc.pathPlanning.visibilityGraphs.YoVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityGraphsParameters;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.graphics.YoGraphicPlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class FootstepPlanningToolboxController extends ToolboxController
{
   private final YoEnum<FootstepPlannerType> activePlanner = new YoEnum<>("activePlanner", registry, FootstepPlannerType.class);
   private final EnumMap<FootstepPlannerType, BodyPathAndFootstepPlanner> plannerMap = new EnumMap<>(FootstepPlannerType.class);

   private final AtomicReference<FootstepPlanningRequestPacket> latestRequestReference = new AtomicReference<>(null);
   private final AtomicReference<FootstepPlannerParametersPacket> latestFootstepPlannerParametersReference = new AtomicReference<>(null);
   private final AtomicReference<VisibilityGraphsParametersPacket> latestVisibilityGraphsParametersReference = new AtomicReference<>(null);
   private Optional<PlanarRegionsList> planarRegionsList = Optional.empty();

   private final YoBoolean isDone = new YoBoolean("isDone", registry);
   private final YoBoolean requestedPlanarRegions = new YoBoolean("RequestedPlanarRegions", registry);
   private final YoDouble toolboxTime = new YoDouble("ToolboxTime", registry);
   private final YoDouble timeout = new YoDouble("ToolboxTimeout", registry);
   private final YoInteger planId = new YoInteger("planId", registry);

   private final YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList;

   private double dt;

   private final YoFootstepPlannerParameters footstepPlanningParameters;
   private final YoVisibilityGraphParameters visibilityGraphsParameters;
   private IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher;

   public FootstepPlanningToolboxController(RobotContactPointParameters<RobotSide> contactPointParameters, FootstepPlannerParametersReadOnly footstepPlannerParameters,
                                            VisibilityGraphsParameters visibilityGraphsParameters, StatusMessageOutputManager statusOutputManager,
                                            YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry, double dt)
   {
      super(statusOutputManager, parentRegistry);
      this.dt = dt;
      this.yoGraphicPlanarRegionsList = new YoGraphicPlanarRegionsList("FootstepPlannerToolboxPlanarRegions", 200, 30, registry);

      SideDependentList<ConvexPolygon2D> contactPointsInSoleFrame;
      if (contactPointParameters == null)
         contactPointsInSoleFrame = PlannerTools.createDefaultFootPolygons();
      else
         contactPointsInSoleFrame = createFootPolygonsFromContactPoints(contactPointParameters);

      footstepPlanningParameters = new YoFootstepPlannerParameters(registry, footstepPlannerParameters);
      this.visibilityGraphsParameters = new YoVisibilityGraphParameters(visibilityGraphsParameters, registry);

      plannerMap.put(FootstepPlannerType.PLANAR_REGION_BIPEDAL, createPlanarRegionBipedalPlanner(contactPointsInSoleFrame));
      plannerMap.put(FootstepPlannerType.PLAN_THEN_SNAP, new PlanThenSnapPlanner(new TurnWalkTurnPlanner(footstepPlanningParameters), contactPointsInSoleFrame));
      plannerMap.put(FootstepPlannerType.A_STAR, createAStarPlanner(contactPointsInSoleFrame));
      plannerMap
            .put(FootstepPlannerType.SIMPLE_BODY_PATH, new SplinePathWithAStarPlanner(footstepPlanningParameters, contactPointsInSoleFrame, parentRegistry, null));
      plannerMap.put(FootstepPlannerType.VIS_GRAPH_WITH_A_STAR,
                     new VisibilityGraphWithAStarPlanner(footstepPlanningParameters, this.visibilityGraphsParameters, contactPointsInSoleFrame,
                                                         graphicsListRegistry, parentRegistry));
      activePlanner.set(FootstepPlannerType.PLANAR_REGION_BIPEDAL);

      graphicsListRegistry.registerYoGraphic("footstepPlanningToolbox", yoGraphicPlanarRegionsList);
      isDone.set(true);
      planId.set(FootstepPlanningRequestPacket.NO_PLAN_ID);
   }

   private AStarFootstepPlanner createAStarPlanner(SideDependentList<ConvexPolygon2D> footPolygons)
   {
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(footstepPlanningParameters);

      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      FootstepNodeSnapAndWiggler postProcessingSnapper = new FootstepNodeSnapAndWiggler(footPolygons, footstepPlanningParameters);
      FootstepNodeBodyCollisionDetector collisionDetector = new FootstepNodeBodyCollisionDetector(footstepPlanningParameters);

      SnapBasedNodeChecker snapBasedNodeChecker = new SnapBasedNodeChecker(footstepPlanningParameters, footPolygons, snapper);
      BodyCollisionNodeChecker bodyCollisionNodeChecker = new BodyCollisionNodeChecker(collisionDetector, footstepPlanningParameters, snapper);
      PlanarRegionBaseOfCliffAvoider cliffAvoider = new PlanarRegionBaseOfCliffAvoider(footstepPlanningParameters, snapper, footPolygons);

      DistanceAndYawBasedHeuristics heuristics = new DistanceAndYawBasedHeuristics(footstepPlanningParameters.getCostParameters().getAStarHeuristicsWeight(), footstepPlanningParameters);

      FootstepNodeChecker nodeChecker = new FootstepNodeCheckerOfCheckers(Arrays.asList(snapBasedNodeChecker, bodyCollisionNodeChecker, cliffAvoider));
//      nodeChecker.addPlannerListener(nu);

      FootstepCostBuilder costBuilder = new FootstepCostBuilder();
      costBuilder.setFootstepPlannerParameters(footstepPlanningParameters);
      costBuilder.setSnapper(snapper);
      costBuilder.setIncludeHeightCost(true);
      costBuilder.setIncludeHeightCost(true);
      costBuilder.setIncludePitchAndRollCost(true);

      FootstepCost footstepCost = costBuilder.buildCost();

      long updateFrequency = 1000;
      RosBasedPlannerListener plannerListener = new RosBasedPlannerListener(statusOutputManager, snapper, updateFrequency);

      snapBasedNodeChecker.addPlannerListener(plannerListener);
      bodyCollisionNodeChecker.addPlannerListener(plannerListener);

      AStarFootstepPlanner planner = new AStarFootstepPlanner(footstepPlanningParameters, nodeChecker, heuristics, expansion, footstepCost,
                                                              postProcessingSnapper, plannerListener, footPolygons, registry);

      return planner;
   }

   private DepthFirstFootstepPlanner createPlanarRegionBipedalPlanner(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame)
   {
      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygonsInSoleFrame, footstepPlanningParameters);
      SnapAndWiggleBasedNodeChecker nodeChecker = new SnapAndWiggleBasedNodeChecker(footPolygonsInSoleFrame, footstepPlanningParameters);
      ConstantFootstepCost stepCostCalculator = new ConstantFootstepCost(1.0);

      DepthFirstFootstepPlanner footstepPlanner = new DepthFirstFootstepPlanner(footstepPlanningParameters, snapper, nodeChecker, stepCostCalculator, registry);
      footstepPlanner.setFeetPolygons(footPolygonsInSoleFrame, footPolygonsInSoleFrame);
      footstepPlanner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      footstepPlanner.setExitAfterInitialSolution(false);

      return footstepPlanner;
   }

   private FootstepPlannerStatusMessage packStatus(FootstepPlannerStatus status)
   {
      FootstepPlannerStatusMessage message = new FootstepPlannerStatusMessage();
      message.setFootstepPlannerStatus(status.toByte());

      return message;
   }

   @Override
   public void updateInternal()
   {
      toolboxTime.add(dt);
      if (toolboxTime.getDoubleValue() > 20.0)
      {
         if (DEBUG)
            LogTools.info("Hard timeout at " + toolboxTime.getDoubleValue());
         reportMessage(packStepResult(null, null, FootstepPlanningResult.TIMED_OUT_BEFORE_SOLUTION));
         isDone.set(true);
         return;
      }

      BodyPathAndFootstepPlanner planner = plannerMap.get(activePlanner.getEnumValue());

      if (planarRegionsList.isPresent())
      {
         planner.setPlanarRegions(planarRegionsList.get());
         yoGraphicPlanarRegionsList.submitPlanarRegionsListToRender(planarRegionsList.get());
         yoGraphicPlanarRegionsList.processPlanarRegionsListQueue();
      }
      else
      {
         planner.setPlanarRegions(null);
         yoGraphicPlanarRegionsList.clear();
      }

      sendMessageToUI("Starting To Plan: " + planId.getIntegerValue() + ", " + activePlanner.getEnumValue().toString());

      reportMessage(packStatus(FootstepPlannerStatus.PLANNING_PATH));

      FootstepPlanningResult status = planner.planPath();

      BodyPathPlan bodyPathPlan = null;
      if (status.validForExecution())
      {
         bodyPathPlan = planner.getPathPlan();
         reportMessage(packStatus(FootstepPlannerStatus.PLANNING_STEPS));
         reportMessage(packPathResult(bodyPathPlan, status));

         status = planner.plan();
      }

      FootstepPlan footstepPlan = planner.getPlan();

      sendMessageToUI("Result: " + planId.getIntegerValue() + ", " + status.toString());

      reportMessage(packStepResult(footstepPlan, bodyPathPlan, status));

      finishUp();
   }

   public void finishUp()
   {
      if (DEBUG)
         LogTools.info("Finishing up the planner");
      plannerMap.get(activePlanner.getEnumValue()).cancelPlanning();
      reportMessage(packStatus(FootstepPlannerStatus.IDLE));
      isDone.set(true);
   }

   @Override
   public boolean initialize()
   {
      isDone.set(false);
      requestedPlanarRegions.set(false);
      toolboxTime.set(0.0);

      FootstepPlanningRequestPacket request = latestRequestReference.getAndSet(null);
      if (request == null)
         return false;

      planId.set(request.getPlannerRequestId());
      FootstepPlannerType requestedPlannerType = FootstepPlannerType.fromByte(request.getRequestedFootstepPlannerType());

      FootstepPlannerParametersPacket footstepPlannerParameters = latestFootstepPlannerParametersReference.getAndSet(null);
      if (footstepPlannerParameters != null)
         footstepPlanningParameters.set(footstepPlannerParameters);

      VisibilityGraphsParametersPacket visibilityGraphParameters = latestVisibilityGraphsParametersReference.getAndSet(null);
      if (visibilityGraphParameters != null)
         this.visibilityGraphsParameters.set(visibilityGraphParameters);

      if (DEBUG)
      {
         LogTools.info("Starting to plan. Plan id: " + request.getPlannerRequestId() + ". Timeout: " + request.getTimeout());
      }

      if (requestedPlannerType != null)
      {
         activePlanner.set(requestedPlannerType);
      }

      PlanarRegionsListMessage planarRegionsListMessage = request.getPlanarRegionsListMessage();
      if (planarRegionsListMessage == null)
      {
         this.planarRegionsList = Optional.empty();
      }
      else
      {
         PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage);
         this.planarRegionsList = Optional.of(planarRegionsList);
      }

      FramePose3D initialStancePose = new FramePose3D(ReferenceFrame.getWorldFrame());
      initialStancePose.setPosition(new Point3D(request.getStanceFootPositionInWorld()));
      initialStancePose.setOrientation(new Quaternion(request.getStanceFootOrientationInWorld()));

      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame());
      goalPose.setPosition(new Point3D(request.getGoalPositionInWorld()));
      goalPose.setOrientation(new Quaternion(request.getGoalOrientationInWorld()));

      FootstepPlanner planner = plannerMap.get(activePlanner.getEnumValue());
      planner.setInitialStanceFoot(initialStancePose, RobotSide.fromByte(request.getInitialStanceRobotSide()));

      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      goal.setGoalPoseBetweenFeet(goalPose);
      planner.setGoal(goal);

      double horizonLength = request.getHorizonLength();
      if (horizonLength > 0 && Double.isFinite(horizonLength))
         planner.setPlanningHorizonLength(horizonLength);

      double timeout = request.getTimeout();
      if (timeout > 0.0 && Double.isFinite(timeout))
      {
         planner.setTimeout(timeout);
         this.timeout.set(timeout);

         if (DEBUG)
         {
            LogTools.info("Setting timeout to " + timeout);
         }
      }
      else
      {
         planner.setTimeout(Double.POSITIVE_INFINITY);
      }

      return true;
   }

   private void sendMessageToUI(String message)
   {
      textToSpeechPublisher.publish(MessageTools.createTextToSpeechPacket(message));
   }

   @Override
   public boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   private BodyPathPlanMessage packPathResult(BodyPathPlan bodyPathPlan, FootstepPlanningResult status)
   {
      if (DEBUG)
      {
         LogTools.info("Finished planning path. Result: " + status);
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

   private FootstepPlanningToolboxOutputStatus packStepResult(FootstepPlan footstepPlan, BodyPathPlan bodyPathPlan, FootstepPlanningResult status)
   {
      if (DEBUG)
      {
         LogTools.info("Finished planning. Result: " + status);
      }

      FootstepPlanningToolboxOutputStatus result = new FootstepPlanningToolboxOutputStatus();
      if (footstepPlan == null)
      {
         result.getFootstepDataList().set(new FootstepDataListMessage());
      }
      else
      {
         result.getFootstepDataList().set(FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan, 0.0, 0.0, ExecutionMode.OVERRIDE));

         if (footstepPlan.hasLowLevelPlanGoal())
         {
            result.getLowLevelPlannerGoal().set(footstepPlan.getLowLevelPlanGoal());
         }
      }

      if (bodyPathPlan != null)
      {
         for (int i = 0; i < bodyPathPlan.getNumberOfWaypoints(); i++)
            result.getBodyPath().add().set(bodyPathPlan.getWaypoint(i));
      }

      planarRegionsList.ifPresent(regions -> result.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regions)));
      result.setPlanId(planId.getIntegerValue());
      result.setFootstepPlanningResult(status.toByte());
      result.getFootstepPlanningStatistics().setTimeTaken(plannerMap.get(activePlanner.getEnumValue()).getPlanningDuration());
      return result;
   }

   private static SideDependentList<ConvexPolygon2D> createFootPolygonsFromContactPoints(RobotContactPointParameters<RobotSide> contactPointParameters)
   {
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(side);
         ConvexPolygon2D scaledFoot = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints));
         footPolygons.set(side, scaledFoot);
      }

      return footPolygons;
   }

   public void processRequest(FootstepPlanningRequestPacket request)
   {
      latestRequestReference.set(request);
   }

   public void processFootstepPlannerParameters(FootstepPlannerParametersPacket parameters)
   {
      latestFootstepPlannerParametersReference.set(parameters);
   }

   public void processVisibilityGraphsParameters(VisibilityGraphsParametersPacket parameters)
   {
      latestVisibilityGraphsParametersReference.set(parameters);
   }

   public void processPlanningStatisticsRequest()
   {
      FootstepPlanner planner = plannerMap.get(activePlanner.getEnumValue());
      sendPlannerStatistics(planner.getPlannerStatistics());
   }

   private void sendPlannerStatistics(PlannerStatistics plannerStatistics)
   {
      switch (plannerStatistics.getStatisticsType())
      {
      case LIST:
         sendListOfStatistics((ListOfStatistics) plannerStatistics);
         break;
      case VISIBILITY_GRAPH:
         reportMessage(VisibilityGraphMessagesConverter
                             .convertToBodyPathPlanStatisticsMessage(planId.getIntegerValue(), (VisibilityGraphStatistics) plannerStatistics));
         break;
      }
   }

   private void sendListOfStatistics(ListOfStatistics listOfStatistics)
   {
      while (listOfStatistics.getNumberOfStatistics() > 0)
         sendPlannerStatistics(listOfStatistics.pollStatistics());
   }

   public void setTextToSpeechPublisher(IHMCRealtimeROS2Publisher<TextToSpeechPacket> publisher)
   {
      textToSpeechPublisher = publisher;
   }
}
