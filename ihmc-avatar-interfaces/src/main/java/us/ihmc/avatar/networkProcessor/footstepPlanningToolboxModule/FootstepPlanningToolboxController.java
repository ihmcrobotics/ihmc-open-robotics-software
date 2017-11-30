package us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.modules.ToolboxController;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.*;
import us.ihmc.communication.packets.RequestPlanarRegionsListMessage.RequestType;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.YoFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.PlanarRegionBipedalFootstepPlannerVisualizer;
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
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPlanarRegionsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.roughTerrain.PlanarRegionBipedalFootstepPlannerVisualizerFactory;
import us.ihmc.humanoidRobotics.communication.packets.walking.*;
import us.ihmc.humanoidRobotics.communication.subscribers.HumanoidRobotDataReceiver;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.multicastLogDataProtocol.modelLoaders.LogModelProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.sensors.ForceSensorDataHolder;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.wholeBodyController.RobotContactPointParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.*;
import java.util.concurrent.atomic.AtomicReference;

public class FootstepPlanningToolboxController extends ToolboxController
{
   private static final boolean debug = true;
   private final boolean visualize = true;
   private HumanoidRobotDataReceiver robotDataReceiver;

   private final YoEnum<FootstepPlanningRequestPacket.FootstepPlannerType> activePlanner = new YoEnum<>("activePlanner", registry,
                                                                                                        FootstepPlanningRequestPacket.FootstepPlannerType.class);
   private final EnumMap<FootstepPlanningRequestPacket.FootstepPlannerType, FootstepPlanner> plannerMap = new EnumMap<>(FootstepPlanningRequestPacket.FootstepPlannerType.class);

   private final AtomicReference<FootstepPlanningRequestPacket> latestRequestReference = new AtomicReference<FootstepPlanningRequestPacket>(null);
   private final AtomicReference<PlanarRegionsListMessage> latestPlanarRegionsReference = new AtomicReference<PlanarRegionsListMessage>(null);
   private Optional<PlanarRegionsList> planarRegionsList = Optional.empty();

   private final YoBoolean usePlanarRegions = new YoBoolean("usePlanarRegions", registry);
   private final YoBoolean isDone = new YoBoolean("isDone", registry);
   private final YoBoolean requestedPlanarRegions = new YoBoolean("RequestedPlanarRegions", registry);
   private final YoDouble toolboxTime = new YoDouble("ToolboxTime", registry);
   private final YoDouble timeReceivedPlanarRegion = new YoDouble("timeReceivedPlanarRegion", registry);
   private final YoInteger planId = new YoInteger("planId", registry);

   private final HumanoidReferenceFrames humanoidReferenceFrames;
   private final RobotContactPointParameters contactPointParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private final LogModelProvider logModelProvider;
   private final FootstepDataListWithSwingOverTrajectoriesAssembler footstepDataListWithSwingOverTrajectoriesAssembler;
   private final YoGraphicPlanarRegionsList yoGraphicPlanarRegionsList;

   private final double collisionSphereRadius = 0.2;
   private final PacketCommunicator packetCommunicator;
   private double dt;

   private final YoFootstepPlannerParameters footstepPlanningParameters;

   public FootstepPlanningToolboxController(DRCRobotModel drcRobotModel, FullHumanoidRobotModel fullHumanoidRobotModel,
                                            StatusMessageOutputManager statusOutputManager, PacketCommunicator packetCommunicator,
                                            YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry, double dt)
   {
      super(statusOutputManager, parentRegistry);
      this.packetCommunicator = packetCommunicator;
      this.contactPointParameters = drcRobotModel.getContactPointParameters();
      this.walkingControllerParameters = drcRobotModel.getWalkingControllerParameters();
      this.logModelProvider = drcRobotModel.getLogModelProvider();
      this.dt = dt;
      this.yoGraphicPlanarRegionsList = new YoGraphicPlanarRegionsList("FootstepPlannerToolboxPlanarRegions", 200, 30, registry);
      packetCommunicator.attachListener(PlanarRegionsListMessage.class, createPlanarRegionsConsumer());

      SideDependentList<ConvexPolygon2D> contactPointsInSoleFrame = createFootPolygonsFromContactPoints(contactPointParameters);

      humanoidReferenceFrames = createHumanoidReferenceFrames(fullHumanoidRobotModel);
      footstepDataListWithSwingOverTrajectoriesAssembler = new FootstepDataListWithSwingOverTrajectoriesAssembler(humanoidReferenceFrames,
                                                                                                                  walkingControllerParameters, parentRegistry,
                                                                                                                 graphicsListRegistry);
      footstepDataListWithSwingOverTrajectoriesAssembler.setCollisionSphereRadius(collisionSphereRadius);

      footstepPlanningParameters = new YoFootstepPlannerParameters(registry, drcRobotModel.getFootstepPlannerParameters());

      plannerMap.put(FootstepPlanningRequestPacket.FootstepPlannerType.PLANAR_REGION_BIPEDAL,
                     createPlanarRegionBipedalPlanner(contactPointsInSoleFrame, fullHumanoidRobotModel));
      plannerMap.put(FootstepPlanningRequestPacket.FootstepPlannerType.PLAN_THEN_SNAP,
                     new PlanThenSnapPlanner(new TurnWalkTurnPlanner(), contactPointsInSoleFrame));
      plannerMap.put(FootstepPlanningRequestPacket.FootstepPlannerType.A_STAR, createAStarPlanner(contactPointsInSoleFrame, drcRobotModel));
      plannerMap.put(FootstepPlanningRequestPacket.FootstepPlannerType.SIMPLE_BODY_PATH,
                     new BodyPathBasedFootstepPlanner(footstepPlanningParameters, contactPointsInSoleFrame, parentRegistry));
      plannerMap.put(FootstepPlanningRequestPacket.FootstepPlannerType.VIS_GRAPH_WITH_A_STAR,
                     new VisibilityGraphWithAStarPlanner(footstepPlanningParameters, contactPointsInSoleFrame, graphicsListRegistry, parentRegistry));
      activePlanner.set(FootstepPlanningRequestPacket.FootstepPlannerType.PLANAR_REGION_BIPEDAL);

      graphicsListRegistry.registerYoGraphic("footstepPlanningToolbox", yoGraphicPlanarRegionsList);
      usePlanarRegions.set(true);
      isDone.set(true);
      planId.set(FootstepPlanningRequestPacket.NO_PLAN_ID);
   }

   private AStarFootstepPlanner createAStarPlanner(SideDependentList<ConvexPolygon2D> footPolygons, DRCRobotModel robotModel)
   {
      /**
       * A robot specific node expansion can be achieved with this.
       * Currently only supported in A-star planner for Atlas and Valkyrie.
       * Use SimpleSideBasedExpansion ( defaults to Atlas) if using other robots or add custom footstep expansion class.
       * */
      //      FootstepNodeExpansion expansion = robotModel.getPlanarRegionFootstepPlannerParameters().getReachableFootstepExpansion();
      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(footstepPlanningParameters);
      AStarFootstepPlanner planner = AStarFootstepPlanner.createRoughTerrainPlanner(footstepPlanningParameters, null, footPolygons, expansion, registry);
      return planner;
   }

   private DepthFirstFootstepPlanner createPlanarRegionBipedalPlanner(SideDependentList<ConvexPolygon2D> footPolygonsInSoleFrame, FullRobotModel fullRobotModel)
   {
      PlanarRegionBipedalFootstepPlannerVisualizer listener = null;
      if (visualize)
      {
         listener = PlanarRegionBipedalFootstepPlannerVisualizerFactory.createWithYoVariableServer(0.01, fullRobotModel, logModelProvider,
                                                                                                   footPolygonsInSoleFrame, "Toolbox_");
      }

      FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(footPolygonsInSoleFrame, footstepPlanningParameters, listener);
      SnapAndWiggleBasedNodeChecker nodeChecker = new SnapAndWiggleBasedNodeChecker(footPolygonsInSoleFrame, listener, footstepPlanningParameters, null);
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
      robotDataReceiver.updateRobotModel();
      toolboxTime.add(dt);
      if (toolboxTime.getDoubleValue() > 20.0)
      {
         reportMessage(packResult(null, FootstepPlanningResult.TIMED_OUT_BEFORE_SOLUTION));
         isDone.set(true);
         return;
      }

      FootstepPlanner planner = plannerMap.get(activePlanner.getEnumValue());

      if (usePlanarRegions.getBooleanValue())
      {
         yoGraphicPlanarRegionsList.processPlanarRegionsListQueue();
         if (!requestAndWaitForPlanarRegions(planner))
            return;
      }
      else
      {
         planner.setPlanarRegions(null);
         planarRegionsList = Optional.empty();
         yoGraphicPlanarRegionsList.clear();
      }

      sendMessageToUI("Starting To Plan: " + planId.getIntegerValue() + ", " + activePlanner.getEnumValue().toString());

      FootstepPlanningResult status = planner.plan();
      FootstepPlan footstepPlan = planner.getPlan();

      sendMessageToUI("Result: " + planId.getIntegerValue() + ", " + status.toString());

      reportMessage(packResult(footstepPlan, status));
      isDone.set(true);
   }

   private boolean requestAndWaitForPlanarRegions(FootstepPlanner planner)
   {
      if (!requestedPlanarRegions.getBooleanValue())
         requestPlanarRegions();

      PlanarRegionsListMessage planarRegionsMessage = latestPlanarRegionsReference.getAndSet(null);
      if (planarRegionsMessage == null)
         return false;

      timeReceivedPlanarRegion.set(toolboxTime.getDoubleValue());
      PlanarRegionsList planarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsMessage);
      planner.setPlanarRegions(planarRegions);
      planarRegionsList = Optional.of(planarRegions);
      yoGraphicPlanarRegionsList.submitPlanarRegionsListToRender(planarRegions);

      return true;
   }

   private void requestPlanarRegions()
   {
      RequestPlanarRegionsListMessage requestPlanarRegionsListMessage = new RequestPlanarRegionsListMessage(RequestType.SINGLE_UPDATE);
      requestPlanarRegionsListMessage.setDestination(PacketDestination.REA_MODULE);
      packetCommunicator.send(requestPlanarRegionsListMessage);
      latestPlanarRegionsReference.set(null);
      requestedPlanarRegions.set(true);
   }

   @Override
   protected boolean initialize()
   {
      isDone.set(false);
      requestedPlanarRegions.set(false);
      toolboxTime.set(0.0);
      timeReceivedPlanarRegion.set(0.0);

      FootstepPlanningRequestPacket request = latestRequestReference.getAndSet(null);
      if (request == null)
         return false;

      planId.set(request.planId);
      FootstepPlanningRequestPacket.FootstepPlannerType requestedPlannerType = request.requestedPlannerType;

      if (debug)
      {
         PrintTools.info("Starting to plan. Plan id: " + request.planId + ". Timeout: " + request.timeout);
      }

      if (requestedPlannerType != null)
      {
         activePlanner.set(requestedPlannerType);
      }

      usePlanarRegions.set(!request.assumeFlatGround);

      FramePose initialStancePose = new FramePose(ReferenceFrame.getWorldFrame());
      initialStancePose.setPosition(new Point3D(request.stanceFootPositionInWorld));
      initialStancePose.setOrientation(new Quaternion(request.stanceFootOrientationInWorld));

      FramePose goalPose = new FramePose(ReferenceFrame.getWorldFrame());
      goalPose.setPosition(new Point3D(request.goalPositionInWorld));
      goalPose.setOrientation(new Quaternion(request.goalOrientationInWorld));

      FootstepPlanner planner = plannerMap.get(activePlanner.getEnumValue());
      planner.setInitialStanceFoot(initialStancePose, request.initialStanceSide);

      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      goal.setGoalPoseBetweenFeet(goalPose);
      planner.setGoal(goal);

      double timeout = request.getTimeout();
      if (timeout > 0.0 && Double.isFinite(timeout))
      {
         planner.setTimeout(timeout);

         if(debug)
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

   public PacketConsumer<FootstepPlanningRequestPacket> createRequestConsumer()
   {
      return new PacketConsumer<FootstepPlanningRequestPacket>()
      {
         @Override
         public void receivedPacket(FootstepPlanningRequestPacket packet)
         {
            if (packet == null)
               return;
            latestRequestReference.set(packet);
         }
      };
   }

   public PacketConsumer<PlanarRegionsListMessage> createPlanarRegionsConsumer()
   {
      return new PacketConsumer<PlanarRegionsListMessage>()
      {
         @Override
         public void receivedPacket(PlanarRegionsListMessage packet)
         {
            if (packet == null)
               return;
            latestPlanarRegionsReference.set(packet);
         }
      };
   }

   public HumanoidReferenceFrames createHumanoidReferenceFrames(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      ForceSensorDataHolder forceSensorDataHolder = new ForceSensorDataHolder(Arrays.asList(fullHumanoidRobotModel.getForceSensorDefinitions()));
      robotDataReceiver = new HumanoidRobotDataReceiver(fullHumanoidRobotModel, forceSensorDataHolder);

      packetCommunicator.attachListener(RobotConfigurationData.class, robotDataReceiver);
      return robotDataReceiver.getReferenceFrames();
   }

   private void sendMessageToUI(String message)
   {
      TextToSpeechPacket packet = new TextToSpeechPacket(message);
      packet.setDestination(PacketDestination.UI);
      packetCommunicator.send(packet);
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
         result.footstepDataList = new FootstepDataListMessage();
      }
      else
      {
         result.footstepDataList = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan, 0.0, 0.0, ExecutionMode.OVERRIDE);
      }

      if(activePlanner.getEnumValue().equals(FootstepPlanningRequestPacket.FootstepPlannerType.VIS_GRAPH_WITH_A_STAR))
      {
         setOutputStatusOfVisibilityGraph(result);
      }

      planarRegionsList.ifPresent(result::setPlanarRegionsList);
      result.setPlanId(planId.getIntegerValue());
      result.planningResult = status;
      return result;
   }

   private void setOutputStatusOfVisibilityGraph(FootstepPlanningToolboxOutputStatus result)
   {
      VisibilityGraphWithAStarPlanner visibilityGraphWithAStarPlanner = (VisibilityGraphWithAStarPlanner) plannerMap.get(FootstepPlanningRequestPacket.FootstepPlannerType.VIS_GRAPH_WITH_A_STAR);
      result.navigableExtrusions = visibilityGraphWithAStarPlanner.getNavigableRegions();
      result.lowLevelPlannerGoal = visibilityGraphWithAStarPlanner.getLowLevelPlannerGoal();

      List<Point2D> waypointList = visibilityGraphWithAStarPlanner.getBodyPathWaypoints();
      Point2D[] waypoints = new Point2D[waypointList.size()];
      for (int i = 0; i < waypointList.size(); i++)
      {
         waypoints[i] = waypointList.get(i);
      }
      result.bodyPath = waypoints;
   }

   private static SideDependentList<ConvexPolygon2D> createFootPolygonsFromContactPoints(RobotContactPointParameters contactPointParameters)
   {
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
      {
         ArrayList<Point2D> footPoints = contactPointParameters.getFootContactPoints().get(side);
         ConvexPolygon2D scaledFoot = new ConvexPolygon2D(footPoints);
         footPolygons.set(side, scaledFoot);
      }

      return footPolygons;
   }

}
