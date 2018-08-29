package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.TextToSpeechPacket;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commons.PrintTools;
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
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.YoFootstepPlannerParameters;
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
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessageConverter;
import us.ihmc.robotModels.FullHumanoidRobotModel;
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
   private static final boolean debug = true;

   private final YoEnum<FootstepPlannerType> activePlanner = new YoEnum<>("activePlanner", registry, FootstepPlannerType.class);
   private final EnumMap<FootstepPlannerType, FootstepPlanner> plannerMap = new EnumMap<>(FootstepPlannerType.class);

   private final AtomicReference<FootstepPlanningRequestPacket> latestRequestReference = new AtomicReference<FootstepPlanningRequestPacket>(null);
   private Optional<PlanarRegionsList> planarRegionsList = Optional.empty();

   private final YoBoolean isDone = new YoBoolean("isDone", registry);
   private final YoBoolean requestedPlanarRegions = new YoBoolean("RequestedPlanarRegions", registry);
   private final YoDouble toolboxTime = new YoDouble("ToolboxTime", registry);
   private final YoInteger planId = new YoInteger("planId", registry);

   private final RobotContactPointParameters<RobotSide> contactPointParameters;
   private final YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList;

   private double dt;

   private final YoFootstepPlannerParameters footstepPlanningParameters;
   private IHMCRealtimeROS2Publisher<TextToSpeechPacket> textToSpeechPublisher;

   public FootstepPlanningToolboxController(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullHumanoidRobotModel,
                                            StatusMessageOutputManager statusOutputManager, YoVariableRegistry parentRegistry,
                                            YoGraphicsListRegistry graphicsListRegistry, double dt)
   {
      super(statusOutputManager, parentRegistry);
      this.contactPointParameters = drcRobotModel.getContactPointParameters();
      this.dt = dt;
      this.yoGraphicPlanarRegionsList = new YoGraphicPlanarRegionsList("FootstepPlannerToolboxPlanarRegions", 200, 30, registry);

      SideDependentList<ConvexPolygon2D> contactPointsInSoleFrame = createFootPolygonsFromContactPoints(contactPointParameters);

      footstepPlanningParameters = new YoFootstepPlannerParameters(registry, drcRobotModel.getFootstepPlannerParameters());

      plannerMap.put(FootstepPlannerType.PLANAR_REGION_BIPEDAL, createPlanarRegionBipedalPlanner(contactPointsInSoleFrame));
      plannerMap.put(FootstepPlannerType.PLAN_THEN_SNAP, new PlanThenSnapPlanner(new TurnWalkTurnPlanner(), contactPointsInSoleFrame));
      plannerMap.put(FootstepPlannerType.A_STAR, createAStarPlanner(contactPointsInSoleFrame));
      plannerMap.put(FootstepPlannerType.SIMPLE_BODY_PATH,
                     new BodyPathBasedFootstepPlanner(footstepPlanningParameters, contactPointsInSoleFrame, parentRegistry));
      plannerMap.put(FootstepPlannerType.VIS_GRAPH_WITH_A_STAR,
                     new VisibilityGraphWithAStarPlanner(footstepPlanningParameters, contactPointsInSoleFrame, graphicsListRegistry, parentRegistry));
      activePlanner.set(FootstepPlannerType.PLANAR_REGION_BIPEDAL);

      graphicsListRegistry.registerYoGraphic("footstepPlanningToolbox", yoGraphicPlanarRegionsList);
      isDone.set(true);
      planId.set(FootstepPlanningRequestPacket.NO_PLAN_ID);
   }

   private AStarFootstepPlanner createAStarPlanner(SideDependentList<ConvexPolygon2D> footPolygons)
   {
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(footstepPlanningParameters);
      AStarFootstepPlanner planner = AStarFootstepPlanner.createRoughTerrainPlanner(footstepPlanningParameters, null, footPolygons, expansion, registry);
      return planner;
   }

   private DepthFirstFootstepPlanner createPlanarRegionBipedalPlanner(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame)
   {
      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygonsInSoleFrame, footstepPlanningParameters, null);
      SnapAndWiggleBasedNodeChecker nodeChecker = new SnapAndWiggleBasedNodeChecker(footPolygonsInSoleFrame, null, footstepPlanningParameters);
      ConstantFootstepCost stepCostCalculator = new ConstantFootstepCost(1.0);

      DepthFirstFootstepPlanner footstepPlanner = new DepthFirstFootstepPlanner(footstepPlanningParameters, snapper, nodeChecker, stepCostCalculator, registry);
      footstepPlanner.setFeetPolygons(footPolygonsInSoleFrame, footPolygonsInSoleFrame);
      footstepPlanner.setMaximumNumberOfNodesToExpand(Integer.MAX_VALUE);
      footstepPlanner.setExitAfterInitialSolution(false);

      return footstepPlanner;
   }

   @Override
   protected void updateInternal()
   {
      toolboxTime.add(dt);
      if (toolboxTime.getDoubleValue() > 20.0)
      {
         reportMessage(packResult(null, FootstepPlanningResult.TIMED_OUT_BEFORE_SOLUTION));
         isDone.set(true);
         return;
      }

      FootstepPlanner planner = plannerMap.get(activePlanner.getEnumValue());

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

      FootstepPlanningResult status = planner.plan();
      FootstepPlan footstepPlan = planner.getPlan();

      sendMessageToUI("Result: " + planId.getIntegerValue() + ", " + status.toString());

      reportMessage(packResult(footstepPlan, status));
      isDone.set(true);
   }

   @Override
   protected boolean initialize()
   {
      isDone.set(false);
      requestedPlanarRegions.set(false);
      toolboxTime.set(0.0);

      FootstepPlanningRequestPacket request = latestRequestReference.getAndSet(null);
      if (request == null)
         return false;

      planId.set(request.getPlannerRequestId());
      FootstepPlannerType requestedPlannerType = FootstepPlannerType.fromByte(request.getRequestedFootstepPlannerType());

      if (debug)
      {
         PrintTools.info("Starting to plan. Plan id: " + request.getPlannerRequestId() + ". Timeout: " + request.getTimeout());
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

      double timeout = request.getTimeout();
      if (timeout > 0.0 && Double.isFinite(timeout))
      {
         planner.setTimeout(timeout);

         if (debug)
         {
            PrintTools.info("Setting timeout to " + timeout);
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
   protected boolean isDone()
   {
      return isDone.getBooleanValue();
   }

   private FootstepPlanningToolboxOutputStatus packResult(FootstepPlan footstepPlan, FootstepPlanningResult status)
   {
      if (debug)
      {
         PrintTools.info("Finished planning. Result: " + status);
      }

      FootstepPlanningToolboxOutputStatus result = new FootstepPlanningToolboxOutputStatus();
      if (footstepPlan == null)
      {
         result.getFootstepDataList().set(new FootstepDataListMessage());
      }
      else
      {
         result.getFootstepDataList().set(FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan, 0.0, 0.0, ExecutionMode.OVERRIDE));
      }

      planarRegionsList.ifPresent(regions -> result.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(regions)));
      result.setPlanId(planId.getIntegerValue());
      result.setFootstepPlanningResult(status.toByte());
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

   public void setTextToSpeechPublisher(IHMCRealtimeROS2Publisher<TextToSpeechPacket> publisher)
   {
      textToSpeechPublisher = publisher;
   }
}
