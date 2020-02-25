package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.*;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnappingTools;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.heuristics.BodyPathHeuristics;
import us.ihmc.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.footstepPlanning.graphSearch.heuristics.DistanceAndYawBasedHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.*;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.pathPlanners.VisibilityGraphPathPlanner;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCostBuilder;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.pathPlanning.graph.search.AStarIterationData;
import us.ihmc.pathPlanning.graph.search.AStarPathPlanner;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.BodyPathPostProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class FootstepPlanningModule implements CloseableAndDisposable
{
   private static final double defaultStatusPublishPeriod = 1.0;

   private final String name;
   private Ros2Node ros2Node;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final VisibilityGraphsParametersBasics visibilityGraphParameters;
   private final AStarPathPlanner<FootstepNode> footstepPlanner;
   private final SimplePlanarRegionFootstepNodeSnapper snapper;
   private final FootstepNodeSnapAndWiggler snapAndWiggler;
   private final FootstepNodeChecker checker;
   private final CostToGoHeuristics distanceAndYawHeuristics;
   private final FootstepCost stepCost;

   private final VisibilityGraphPathPlanner bodyPathPlanner;
   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder = new WaypointDefinedBodyPathPlanHolder();
   private final BodyPathHeuristics bodyPathHeuristics;

   private final AtomicBoolean isPlanning = new AtomicBoolean();
   private final AtomicBoolean haltRequested = new AtomicBoolean();
   private final FootstepPlanningRequestPacket requestPacket = new FootstepPlanningRequestPacket();
   private final FootstepPlanningToolboxOutputStatus outputStatus = new FootstepPlanningToolboxOutputStatus();
   private final Stopwatch stopwatch = new Stopwatch();
   private final double statusPublishPeriod;

   private FootstepPlanningResult result = null;
   private FootstepNode endNode = null;
   private double endNodeCost;

   private final BooleanSupplier bodyPathPlanRequested = () -> FootstepPlannerType.fromByte(requestPacket.getRequestedFootstepPlannerType()).plansPath();
   private final Supplier<CostToGoHeuristics> heuristicsSupplier;

   private Consumer<FootstepPlanningRequestPacket> requestCallback = request -> {};
   private Consumer<AStarIterationData<FootstepNode>> iterationCallback = iterationData -> {};
   private Consumer<BodyPathPlanMessage> bodyPathResultCallback = bodyPathPlanMessage -> {};
   private Consumer<FootstepPlanningToolboxOutputStatus> statusCallback = result -> {};

   public FootstepPlanningModule(String name)
   {
      this(name, new DefaultFootstepPlannerParameters(), new DefaultVisibilityGraphParameters(), PlannerTools.createDefaultFootPolygons(), defaultStatusPublishPeriod);
   }

   public FootstepPlanningModule(String name,
                                 FootstepPlannerParametersBasics footstepPlannerParameters,
                                 VisibilityGraphsParametersBasics visibilityGraphParameters,
                                 SideDependentList<ConvexPolygon2D> footPolygons,
                                 double statusPublishPeriod)
   {
      this.name = name;
      this.statusPublishPeriod = statusPublishPeriod;
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.visibilityGraphParameters = visibilityGraphParameters;

      this.snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      this.snapAndWiggler  = new FootstepNodeSnapAndWiggler(footPolygons, footstepPlannerParameters);

      BodyPathPostProcessor pathPostProcessor = new ObstacleAvoidanceProcessor(visibilityGraphParameters);
      this.bodyPathHeuristics = new BodyPathHeuristics(footstepPlannerParameters.getAStarHeuristicsWeight(), footstepPlannerParameters, snapper, bodyPathPlanHolder);
      this.bodyPathPlanner = new VisibilityGraphPathPlanner(footstepPlannerParameters, visibilityGraphParameters, pathPostProcessor, registry);

      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(footstepPlannerParameters);
      SnapBasedNodeChecker snapBasedNodeChecker = new SnapBasedNodeChecker(footstepPlannerParameters, footPolygons, snapper);
      FootstepNodeBodyCollisionDetector collisionDetector = new FootstepNodeBodyCollisionDetector(footstepPlannerParameters);
      BodyCollisionNodeChecker bodyCollisionNodeChecker = new BodyCollisionNodeChecker(collisionDetector, footstepPlannerParameters, snapper);
      PlanarRegionBaseOfCliffAvoider cliffAvoider = new PlanarRegionBaseOfCliffAvoider(footstepPlannerParameters, snapper, footPolygons);
      this.checker = new FootstepNodeCheckerOfCheckers(Arrays.asList(snapBasedNodeChecker, bodyCollisionNodeChecker, cliffAvoider));

      FootstepCostBuilder costBuilder = new FootstepCostBuilder();
      costBuilder.setFootstepPlannerParameters(footstepPlannerParameters);
      costBuilder.setSnapper(snapper);
      costBuilder.setFootPolygons(footPolygons);
      costBuilder.setIncludeHeightCost(true);
      costBuilder.setIncludePitchAndRollCost(true);
      costBuilder.setIncludeAreaCost(true);
      this.stepCost = costBuilder.buildCost();
      this.distanceAndYawHeuristics = new DistanceAndYawBasedHeuristics(snapper, footstepPlannerParameters.getAStarHeuristicsWeight(), footstepPlannerParameters);

      this.heuristicsSupplier = () -> bodyPathPlanRequested.getAsBoolean() ? bodyPathHeuristics : distanceAndYawHeuristics;
      this.footstepPlanner = new AStarPathPlanner<>(expansion::expandNode, checker::isNodeValid, stepCost::compute, node -> heuristicsSupplier.get().compute(node));
      checker.addFootstepGraph(footstepPlanner.getGraph());
   }

   public void handleRequestPacket(FootstepPlanningRequestPacket requestPacket)
   {
      if (isPlanning.get())
      {
         LogTools.info("Received planning request packet but planner is currently running");
         return;
      }

      this.requestPacket.set(requestPacket);
      isPlanning.set(true);
      haltRequested.set(false);
      result = FootstepPlanningResult.SOLUTION_DOES_NOT_REACH_GOAL;
      bodyPathPlanHolder.getPlan().clear();
      requestCallback.accept(requestPacket);

      // Set goal pose
      FramePose3D goalPose = new FramePose3D(ReferenceFrame.getWorldFrame(), requestPacket.getGoalPositionInWorld(), requestPacket.getGoalOrientationInWorld());
      heuristicsSupplier.get().setGoalPose(goalPose);

      // Update planar regions
      PlanarRegionsList planarRegionsList = null;
      if (!requestPacket.getAssumeFlatGround() && !requestPacket.getPlanarRegionsListMessage().getRegionId().isEmpty())
      {
         planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(requestPacket.getPlanarRegionsListMessage());
      }

      snapper.setPlanarRegions(planarRegionsList);
      snapAndWiggler.setPlanarRegions(planarRegionsList);
      checker.setPlanarRegions(planarRegionsList);
      bodyPathPlanner.setPlanarRegionsList(planarRegionsList);

      // Setup planner
      FootstepNode startNode = createStartNode(requestPacket);
      endNode = startNode;
      addStartPoseToSnapper(requestPacket, startNode);
      footstepPlanner.initialize(startNode);
      SideDependentList<FootstepNode> goalNodes = createGoalNodes(requestPacket);

      // Start timer
      stopwatch.start();

      // Plan body path if requested
      if (bodyPathPlanRequested.getAsBoolean())
      {
         bodyPathHeuristics.setGoalAlpha(1.0);
         FramePose3D stanceFootPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                      requestPacket.getStanceFootPositionInWorld(),
                                                      requestPacket.getStanceFootOrientationInWorld());
         RobotSide stanceFootSide = RobotSide.fromByte(requestPacket.getInitialStanceRobotSide());
         bodyPathPlanner.setInitialStanceFoot(stanceFootPose, stanceFootSide);
         FootstepPlannerGoal footstepPlannerGoal = new FootstepPlannerGoal();
         footstepPlannerGoal.setGoalPoseBetweenFeet(goalPose);
         footstepPlannerGoal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
         bodyPathPlanner.setGoal(footstepPlannerGoal);

         FootstepPlanningResult bodyPathPlannerResult = bodyPathPlanner.planWaypoints();
         if (!bodyPathPlannerResult.validForExecution())
         {
            result = bodyPathPlannerResult;
            isPlanning.set(false);
            reportStatus();
            return;
         }

         List<Pose3DReadOnly> waypoints = bodyPathPlanner.getWaypoints();
         if (waypoints.size() < 2)
         {
            if (footstepPlannerParameters.getReturnBestEffortPlan())
            {
               double horizonLength = Double.POSITIVE_INFINITY;
               bodyPathPlanner.computeBestEffortPlan(horizonLength);
            }
            else
            {
               result = FootstepPlanningResult.PLANNER_FAILED;
               isPlanning.set(false);
               reportStatus();
               return;
            }
         }

         bodyPathPlanHolder.setPoseWaypoints(waypoints);
         reportBodyPathPlan();
      }

      // Calculate end node cost
      endNodeCost = heuristicsSupplier.get().compute(endNode);

      // Check valid goal
      if (!bodyPathPlanRequested.getAsBoolean() && !footstepPlannerParameters.getReturnBestEffortPlan() && !validGoal(goalNodes))
      {
         result = FootstepPlanningResult.INVALID_GOAL;
         isPlanning.set(false);
         reportStatus();
         return;
      }

      // Start planning loop
      while (true)
      {
         if (stopwatch.totalElapsed() >= requestPacket.getTimeout() || haltRequested.get())
         {
            result = FootstepPlanningResult.TIMED_OUT_BEFORE_SOLUTION;
            break;
         }

         AStarIterationData<FootstepNode> iterationData = footstepPlanner.doPlanningIteration();
         iterationCallback.accept(iterationData);

         if (iterationData.getParentNode() == null)
         {
            result = FootstepPlanningResult.NO_PATH_EXISTS;
            break;
         }
         if (checkIfGoalIsReached(goalNodes, iterationData))
         {
            result = FootstepPlanningResult.SUB_OPTIMAL_SOLUTION;
            break;
         }
         if (stopwatch.lapElapsed() > statusPublishPeriod && !MathTools.epsilonEquals(stopwatch.totalElapsed(), requestPacket.getTimeout(), 0.1))
         {
            reportStatus();
            stopwatch.lap();
         }
      }

      reportStatus();
      isPlanning.set(false);
   }

   private void reportStatus()
   {
      outputStatus.setPlanId(requestPacket.getPlannerRequestId());
      outputStatus.setFootstepPlanningResult(result.toByte());

      // Pack solution path
      FootstepDataListMessage footstepDataList = outputStatus.getFootstepDataList();
      footstepDataList.setOffsetFootstepsHeightWithExecutionError(true);
      footstepDataList.getFootstepDataList().clear();
      List<FootstepNode> path = footstepPlanner.getGraph().getPathFromStart(endNode);
      for (int i = 1; i < path.size(); i++)
      {
         FootstepDataMessage footstepDataMessage = footstepDataList.getFootstepDataList().add();

         footstepDataMessage.setRobotSide(path.get(i).getRobotSide().toByte());

         RigidBodyTransform footstepPose = new RigidBodyTransform();
         footstepPose.setRotationYawAndZeroTranslation(path.get(i).getYaw());
         footstepPose.setTranslationX(path.get(i).getX());
         footstepPose.setTranslationY(path.get(i).getY());

         FootstepNodeSnapData snapData = snapAndWiggler.snapFootstepNode(path.get(i));
         RigidBodyTransform snapTransform = snapData.getSnapTransform();
         snapTransform.transform(footstepPose);
         footstepDataMessage.getLocation().set(footstepPose.getTranslation());
         footstepDataMessage.getOrientation().set(footstepPose.getRotation());

         if (requestPacket.getAssumeFlatGround() || requestPacket.getPlanarRegionsListMessage().getRegionId().isEmpty())
         {
            double flatGroundHeight = requestPacket.getStanceFootPositionInWorld().getZ();
            footstepDataMessage.getLocation().setZ(flatGroundHeight);
         }

         ConvexPolygon2D foothold = snapData.getCroppedFoothold();
         if (!foothold.isEmpty())
         {
            footstepDataMessage.getPredictedContactPoints2d().clear();
            for (int j = 0; j < foothold.getNumberOfVertices(); j++)
            {
               footstepDataMessage.getPredictedContactPoints2d().add().set(foothold.getVertex(j));
            }
         }
      }

      BodyPathPlan bodyPathPlan = bodyPathPlanHolder.getPlan();
      if (bodyPathPlan.getNumberOfWaypoints() > 0)
      {
         outputStatus.getBodyPath().clear();
         for (int i = 0; i < bodyPathPlan.getNumberOfWaypoints(); i++)
         {
            outputStatus.getBodyPath().add().set(bodyPathPlan.getWaypoint(i));
         }
      }

      outputStatus.getPlanarRegionsList().set(requestPacket.getPlanarRegionsListMessage());
      outputStatus.getFootstepPlanningStatistics().setTimeTaken(stopwatch.totalElapsed());
      statusCallback.accept(outputStatus);
   }

   private void reportBodyPathPlan()
   {
      BodyPathPlanMessage bodyPathPlanMessage = new BodyPathPlanMessage();
      BodyPathPlan bodyPathPlan = bodyPathPlanHolder.getPlan();
      for (int i = 0; i < bodyPathPlan.getNumberOfWaypoints(); i++)
      {
         bodyPathPlanMessage.getBodyPath().add().set(bodyPathPlan.getWaypoint(i));
      }

      bodyPathPlanMessage.getPathPlannerStartPose().set(bodyPathPlan.getStartPose());
      bodyPathPlanMessage.getPathPlannerGoalPose().set(bodyPathPlan.getGoalPose());
      bodyPathPlanMessage.setPlanId(requestPacket.getPlannerRequestId());
      bodyPathPlanMessage.setFootstepPlanningResult(result.toByte());
      bodyPathPlanMessage.getPlanarRegionsList().set(requestPacket.getPlanarRegionsListMessage());
      bodyPathResultCallback.accept(bodyPathPlanMessage);
   }

   private boolean validGoal(SideDependentList<FootstepNode> goalNodes)
   {
      for (RobotSide side : RobotSide.values)
      {
         if (!checker.isNodeValid(goalNodes.get(side), null))
            return false;
      }

      return true;
   }

   public void addRequestCallback(Consumer<FootstepPlanningRequestPacket> callback)
   {
      requestCallback = requestCallback.andThen(callback);
   }

   public void addIterationCallback(Consumer<AStarIterationData<FootstepNode>> callback)
   {
      iterationCallback = iterationCallback.andThen(callback);
   }

   public void addBodyPathPlanCallback(Consumer<BodyPathPlanMessage> callback)
   {
      bodyPathResultCallback = bodyPathResultCallback.andThen(callback);
   }

   public void addStatusCallback(Consumer<FootstepPlanningToolboxOutputStatus> callback)
   {
      statusCallback = statusCallback.andThen(callback);
   }

   public boolean registerRosNode(Ros2Node ros2Node)
   {
      if (this.ros2Node != null)
         return false;
      this.ros2Node = ros2Node;
      return true;
   }

   public void halt()
   {
      haltRequested.set(true);
   }

   public String getName()
   {
      return name;
   }

   public FootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return footstepPlannerParameters;
   }

   public VisibilityGraphsParametersBasics getVisibilityGraphParameters()
   {
      return visibilityGraphParameters;
   }

   public SimplePlanarRegionFootstepNodeSnapper getSnapper()
   {
      return snapper;
   }

   public FootstepNodeChecker getChecker()
   {
      return checker;
   }

   @Override
   public void closeAndDispose()
   {
      if (ros2Node != null)
      {
         ros2Node.destroy();
         ros2Node = null;
      }
   }

   private boolean checkIfGoalIsReached(SideDependentList<FootstepNode> goalNodes, AStarIterationData<FootstepNode> iterationData)
   {
      for (int i = 0; i < iterationData.getValidChildNodes().size(); i++)
      {
         FootstepNode childNode = iterationData.getValidChildNodes().get(i);
         if (childNode.equals(goalNodes.get(childNode.getRobotSide())))
         {
            endNode = goalNodes.get(childNode.getRobotSide().getOppositeSide());
            footstepPlanner.getGraph().checkAndSetEdge(childNode, endNode, 0.0);
            return true;
         }

         double cost = footstepPlanner.getGraph().getCostFromStart(childNode) + distanceAndYawHeuristics.compute(childNode);
         if(cost < endNodeCost)
         {
            endNode = childNode;
            endNodeCost = cost;
         }
      }
      return false;
   }

   private void addStartPoseToSnapper(FootstepPlanningRequestPacket requestPacket, FootstepNode startNode)
   {
      Pose3D footPose = new Pose3D(requestPacket.getStanceFootPositionInWorld(), new Quaternion(requestPacket.getStanceFootOrientationInWorld()));
      RigidBodyTransform snapTransform = FootstepNodeSnappingTools.computeSnapTransform(startNode, footPose);
      snapper.addSnapData(startNode, new FootstepNodeSnapData(snapTransform));
   }

   private static FootstepNode createStartNode(FootstepPlanningRequestPacket requestPacket)
   {
      Point3D stancePosition = requestPacket.getStanceFootPositionInWorld();
      double stanceYaw = requestPacket.getStanceFootOrientationInWorld().getYaw();
      RobotSide robotSide = RobotSide.fromByte(requestPacket.getInitialStanceRobotSide());
      return new FootstepNode(stancePosition.getX(), stancePosition.getY(), stanceYaw, robotSide);
   }

   private SideDependentList<FootstepNode> createGoalNodes(FootstepPlanningRequestPacket requestPacket)
   {
      return new SideDependentList<>(side ->
                                     {
                                        Pose3D goalPose = new Pose3D(requestPacket.getGoalPositionInWorld(), requestPacket.getGoalOrientationInWorld());
                                        goalPose.appendTranslation(0.0, 0.5 * side.negateIfRightSide(footstepPlannerParameters.getIdealFootstepWidth()), 0.0);
                                        return new FootstepNode(goalPose.getX(), goalPose.getY(), goalPose.getYaw(), side);
                                     });
   }
}
