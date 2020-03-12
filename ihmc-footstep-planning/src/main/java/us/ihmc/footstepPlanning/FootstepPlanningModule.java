package us.ihmc.footstepPlanning;

import controller_msgs.msg.dds.BodyPathPlanMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnappingTools;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.footstepPlanning.graphSearch.heuristics.DistanceAndYawBasedHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.*;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.IdealStepCalculator;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.pathPlanners.VisibilityGraphPathPlanner;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCostCalculator;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.footstepPlanning.log.FootstepPlannerIterationData;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.pathPlanning.graph.search.AStarIterationData;
import us.ihmc.pathPlanning.graph.search.AStarPathPlanner;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.BodyPathPostProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

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
   private final IdealStepCalculator idealStepCalculator;
   private final FootstepCostCalculator stepCostCalculator;

   private final VisibilityGraphPathPlanner bodyPathPlanner;
   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder = new WaypointDefinedBodyPathPlanHolder();

   private final AtomicBoolean isPlanning = new AtomicBoolean();
   private final AtomicBoolean haltRequested = new AtomicBoolean();
   private final FootstepPlannerRequest request = new FootstepPlannerRequest();
   private final FootstepPlannerOutput output = new FootstepPlannerOutput();
   private final Stopwatch stopwatch = new Stopwatch();
   private double statusPublishPeriod = defaultStatusPublishPeriod;

   private FootstepPlanningResult result = null;
   private FootstepNode endNode = null;
   private double endNodeCost;
   private final FramePose3D startMidFootPose = new FramePose3D();
   private final FramePose3D goalMidFootPose = new FramePose3D();

   private final BooleanSupplier bodyPathPlanRequested = request::getPlanBodyPath;

   private Consumer<FootstepPlannerRequest> requestCallback = request -> {};
   private Consumer<AStarIterationData<FootstepNode>> iterationCallback = iterationData -> {};
   private Consumer<BodyPathPlanMessage> bodyPathResultCallback = bodyPathPlanMessage -> {};
   private Consumer<FootstepPlannerOutput> statusCallback = result -> {};

   private final FootstepPlannerEdgeData edgeData = new FootstepPlannerEdgeData();
   private final HashMap<GraphEdge<FootstepNode>, FootstepPlannerEdgeData> edgeDataMap = new HashMap<>();
   private final List<FootstepPlannerIterationData> iterationData = new ArrayList<>();

   public FootstepPlanningModule(String name)
   {
      this(name, new DefaultFootstepPlannerParameters(), new DefaultVisibilityGraphParameters(), PlannerTools.createDefaultFootPolygons());
   }

   public FootstepPlanningModule(String name,
                                 FootstepPlannerParametersBasics footstepPlannerParameters,
                                 VisibilityGraphsParametersBasics visibilityGraphParameters,
                                 SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.name = name;
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.visibilityGraphParameters = visibilityGraphParameters;

      this.snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      this.snapAndWiggler  = new FootstepNodeSnapAndWiggler(footPolygons, footstepPlannerParameters);

      BodyPathPostProcessor pathPostProcessor = new ObstacleAvoidanceProcessor(visibilityGraphParameters);
      this.bodyPathPlanner = new VisibilityGraphPathPlanner(footstepPlannerParameters, visibilityGraphParameters, pathPostProcessor, registry);

      FootstepNodeExpansion expansion = new ParameterBasedNodeExpansion(footstepPlannerParameters);
      this.checker = new FootstepNodeChecker(footstepPlannerParameters, footPolygons, snapper, edgeData);
      this.idealStepCalculator = new IdealStepCalculator(footstepPlannerParameters, checker::isNodeValid, bodyPathPlanHolder);

      this.distanceAndYawHeuristics = new DistanceAndYawBasedHeuristics(snapper, footstepPlannerParameters.getAStarHeuristicsWeight(), footstepPlannerParameters, bodyPathPlanHolder, edgeData);
      this.stepCostCalculator = new FootstepCostCalculator(footstepPlannerParameters, snapper, idealStepCalculator::computeIdealStep, distanceAndYawHeuristics::compute, footPolygons, edgeData);

      this.footstepPlanner = new AStarPathPlanner<>(expansion::expandNode, checker::isNodeValid, stepCostCalculator::computeCost, distanceAndYawHeuristics::compute);
      checker.setParentNodeSupplier(node -> footstepPlanner.getGraph().getParentNode(node));
      footstepPlanner.getGraph().setGraphExpansionCallback(edge ->
                                                           {
                                                              edgeData.setCostFromStart(footstepPlanner.getGraph().getCostFromStart(edge.getEndNode()));
                                                              edgeDataMap.put(edge, edgeData.getCopyAndClear());
                                                           });
   }

   public FootstepPlannerOutput handleRequest(FootstepPlannerRequest request)
   {
      if (isPlanning.get())
      {
         LogTools.info("Received planning request packet but planner is currently running");
         return null;
      }

      try
      {
         return handleRequestInternal(request);
      }
      catch (Exception exception)
      {
         exception.printStackTrace();

         output.clear();
         output.setPlanId(request.getRequestId());
         output.setResult(FootstepPlanningResult.EXCEPTION);
         output.setException(exception);
         statusCallback.accept(output);
         return output;
      }
   }

   private FootstepPlannerOutput handleRequestInternal(FootstepPlannerRequest request)
   {
      // Start timer
      stopwatch.start();

      this.request.set(request);
      isPlanning.set(true);
      haltRequested.set(false);
      result = FootstepPlanningResult.SOLUTION_DOES_NOT_REACH_GOAL;
      bodyPathPlanHolder.getPlan().clear();
      startMidFootPose.interpolate(request.getStartFootPoses().get(RobotSide.LEFT), request.getStartFootPoses().get(RobotSide.RIGHT), 0.5);
      goalMidFootPose.interpolate(request.getGoalFootPoses().get(RobotSide.LEFT), request.getGoalFootPoses().get(RobotSide.RIGHT), 0.5);
      requestCallback.accept(request);

      // Reset logged variables
      edgeData.clear();
      edgeDataMap.clear();
      iterationData.clear();

      // Update planar regions
      PlanarRegionsList planarRegionsList = null;
      if (!request.getAssumeFlatGround() && request.getPlanarRegionsList() != null && !request.getPlanarRegionsList().isEmpty())
      {
         planarRegionsList = request.getPlanarRegionsList();
      }

      snapper.setPlanarRegions(planarRegionsList);
      snapAndWiggler.setPlanarRegions(planarRegionsList);
      checker.setPlanarRegions(planarRegionsList);
      bodyPathPlanner.setPlanarRegionsList(planarRegionsList);

      if (bodyPathPlanRequested.getAsBoolean())
      {
         bodyPathPlanner.setStanceFootPoses(request.getStartFootPoses().get(RobotSide.LEFT), request.getStartFootPoses().get(RobotSide.RIGHT));
         bodyPathPlanner.setGoal(goalMidFootPose);

         FootstepPlanningResult bodyPathPlannerResult = bodyPathPlanner.planWaypoints();
         if (!bodyPathPlannerResult.validForExecution())
         {
            result = bodyPathPlannerResult;
            isPlanning.set(false);
            reportStatus();
            return output;
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
               return output;
            }
         }

         bodyPathPlanHolder.setPoseWaypoints(waypoints);
         double pathLength = bodyPathPlanHolder.computePathLength(0.0);
         if (MathTools.intervalContains(request.getHorizonLength(), 0.0, pathLength))
         {
            double alphaIntermediateGoal = request.getHorizonLength() / pathLength;
            bodyPathPlanHolder.getPointAlongPath(alphaIntermediateGoal, goalMidFootPose);
         }
         reportBodyPathPlan();
      }
      else
      {
         List<Pose3DReadOnly> waypoints = new ArrayList<>();
         waypoints.add(startMidFootPose);
         waypoints.addAll(request.getBodyPathWaypoints());
         waypoints.add(goalMidFootPose);

         bodyPathPlanHolder.setPoseWaypoints(waypoints);
      }

      // Setup footstep planner
      FootstepNode startNode = createStartNode(request);
      endNode = startNode;
      addFootPosesToSnapper(request);
      footstepPlanner.initialize(startNode);
      SideDependentList<FootstepNode> goalNodes = createGoalNodes(request);
      distanceAndYawHeuristics.setGoalPose(goalMidFootPose);
      idealStepCalculator.initialize(goalNodes);

      // Calculate end node cost
      endNodeCost = distanceAndYawHeuristics.compute(endNode);

      // Check valid goal
      if (!bodyPathPlanRequested.getAsBoolean() && !footstepPlannerParameters.getReturnBestEffortPlan() && !validGoal(goalNodes))
      {
         result = FootstepPlanningResult.INVALID_GOAL;
         isPlanning.set(false);
         reportStatus();
         return output;
      }

      // Start planning loop
      while (true)
      {
         if (stopwatch.totalElapsed() >= request.getTimeout() || haltRequested.get())
         {
            result = FootstepPlanningResult.TIMED_OUT_BEFORE_SOLUTION;
            break;
         }

         AStarIterationData<FootstepNode> iterationData = footstepPlanner.doPlanningIteration();
         recordIterationData(iterationData);
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
         if (stopwatch.lapElapsed() > statusPublishPeriod && !MathTools.epsilonEquals(stopwatch.totalElapsed(), request.getTimeout(), 0.1))
         {
            reportStatus();
            stopwatch.lap();
         }
      }

      reportStatus();
      markSolutionEdges();
      isPlanning.set(false);
      return output;
   }

   private void reportStatus()
   {
      output.setPlanId(request.getRequestId());
      output.setResult(result);

      // Pack solution path
      output.getFootstepPlan().clear();
      List<FootstepNode> path = footstepPlanner.getGraph().getPathFromStart(endNode);
      for (int i = 1; i < path.size(); i++)
      {
         SimpleFootstep footstep = new SimpleFootstep();

         footstep.setRobotSide(path.get(i).getRobotSide());

         RigidBodyTransform footstepPose = new RigidBodyTransform();
         footstepPose.setRotationYawAndZeroTranslation(path.get(i).getYaw());
         footstepPose.setTranslationX(path.get(i).getX());
         footstepPose.setTranslationY(path.get(i).getY());

         FootstepNodeSnapData snapData = snapper.snapFootstepNode(path.get(i));
         RigidBodyTransform snapTransform = snapData.getSnapTransform();
         snapTransform.transform(footstepPose);
         footstep.getSoleFramePose().set(footstepPose);

         if (request.getAssumeFlatGround() || request.getPlanarRegionsList() == null || request.getPlanarRegionsList().isEmpty())
         {
            double flatGroundHeight = startMidFootPose.getZ();
            footstep.getSoleFramePose().setZ(flatGroundHeight);
         }

         footstep.setFoothold(snapData.getCroppedFoothold());
         output.getFootstepPlan().addFootstep(footstep);
      }

      BodyPathPlan bodyPathPlan = bodyPathPlanHolder.getPlan();
      if (bodyPathPlan.getNumberOfWaypoints() > 0)
      {
         output.getBodyPath().clear();
         for (int i = 0; i < bodyPathPlan.getNumberOfWaypoints(); i++)
         {
            output.getBodyPath().add(new Pose3D(bodyPathPlan.getWaypoint(i)));
         }

         output.getLowLevelGoal().set(goalMidFootPose);
      }

      output.setPlanarRegionsList(request.getPlanarRegionsList());
      statusCallback.accept(output);
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
      bodyPathPlanMessage.setPlanId(request.getRequestId());
      bodyPathPlanMessage.setFootstepPlanningResult(result.toByte());
      bodyPathPlanMessage.getPlanarRegionsList().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(request.getPlanarRegionsList()));
      bodyPathResultCallback.accept(bodyPathPlanMessage);
   }

   private void markSolutionEdges()
   {
      List<FootstepNode> path = footstepPlanner.getGraph().getPathFromStart(endNode);
      for (int i = 1; i < path.size(); i++)
      {
         edgeDataMap.get(new GraphEdge<>(path.get(i - 1), path.get(i))).setSolutionEdge(true);
      }
   }

   private void recordIterationData(AStarIterationData<FootstepNode> iterationData)
   {
      if (iterationData.getParentNode() == null)
      {
         return;
      }

      FootstepPlannerIterationData loggedData = new FootstepPlannerIterationData();
      loggedData.setStanceNode(iterationData.getParentNode());
      iterationData.getValidChildNodes().forEach(loggedData::addChildNode);
      iterationData.getInvalidChildNodes().forEach(loggedData::addChildNode);
      loggedData.setIdealStep(idealStepCalculator.computeIdealStep(iterationData.getParentNode()));
      loggedData.setStanceNodeSnapData(snapper.getSnapData(iterationData.getParentNode()));
      this.iterationData.add(loggedData);
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

   public void addRequestCallback(Consumer<FootstepPlannerRequest> callback)
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

   public void addStatusCallback(Consumer<FootstepPlannerOutput> callback)
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

   public boolean isPlanning()
   {
      return isPlanning.get();
   }

   public FootstepPlannerRequest getRequest()
   {
      return request;
   }

   public FootstepPlannerOutput getOutput()
   {
      return output;
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

   public AStarPathPlanner<FootstepNode> getFootstepPlanner()
   {
      return footstepPlanner;
   }

   public VisibilityGraphPathPlanner getBodyPathPlanner()
   {
      return bodyPathPlanner;
   }

   public FootstepNode getEndNode()
   {
      return endNode;
   }

   public HashMap<GraphEdge<FootstepNode>, FootstepPlannerEdgeData> getEdgeDataMap()
   {
      return edgeDataMap;
   }

   public List<FootstepPlannerIterationData> getIterationData()
   {
      return iterationData;
   }

   public void setStatusPublishPeriod(double statusPublishPeriod)
   {
      this.statusPublishPeriod = statusPublishPeriod;
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
      double distanceProximity = request.getGoalDistanceProximity();
      double yawProximity = request.getGoalYawProximity();

      for (int i = 0; i < iterationData.getValidChildNodes().size(); i++)
      {
         FootstepNode childNode = iterationData.getValidChildNodes().get(i);
         FootstepNode goalNode = goalNodes.get(childNode.getRobotSide());
         boolean validProximityToGoal = isValidProximityToGoal(distanceProximity, yawProximity, childNode, goalNode);

         if (validProximityToGoal)
         {
            boolean proximityMode = distanceProximity > 0.0 || yawProximity > 0.0;
            if (proximityMode && isValidProximityToGoal(distanceProximity, yawProximity, footstepPlanner.getGraph().getParentNode(childNode), goalNode))
            {
               endNode = childNode;
               return true;
            }
            else if (!proximityMode)
            {
               endNode = goalNodes.get(childNode.getRobotSide().getOppositeSide());
               footstepPlanner.getGraph().checkAndSetEdge(childNode, endNode, 0.0);
               return true;
            }
         }

         double cost = footstepPlanner.getGraph().getCostFromStart(childNode) + distanceAndYawHeuristics.compute(childNode);
         if (cost < endNodeCost)
         {
            endNode = childNode;
            endNodeCost = cost;
         }
      }

      return false;
   }

   private boolean isValidProximityToGoal(double distanceProximity, double yawProximity, FootstepNode childNode, FootstepNode goalNode)
   {
      boolean validXYDistanceToGoal, validYawDistanceToGoal;

      // check distance
      if(distanceProximity > 0.0)
      {
         validXYDistanceToGoal = goalNode.euclideanDistanceSquared(childNode) < MathTools.square(distanceProximity);
      }
      else
      {
         validXYDistanceToGoal = childNode.equalPosition(goalNode);
      }
      // check yaw
      if(yawProximity > 0.0)
      {
         validYawDistanceToGoal = Math.abs(AngleTools.computeAngleDifferenceMinusPiToPi(goalNode.getYaw(), childNode.getYaw())) < yawProximity;
      }
      else
      {
         validYawDistanceToGoal = childNode.getYawIndex() == goalNode.getYawIndex();
      }

      return validXYDistanceToGoal && validYawDistanceToGoal;
   }

   private void addFootPosesToSnapper(FootstepPlannerRequest request)
   {
      addSnapData(request.getStartFootPoses().get(RobotSide.LEFT), RobotSide.LEFT);
      addSnapData(request.getStartFootPoses().get(RobotSide.RIGHT), RobotSide.RIGHT);
      addSnapData(request.getGoalFootPoses().get(RobotSide.LEFT), RobotSide.LEFT);
      addSnapData(request.getGoalFootPoses().get(RobotSide.RIGHT), RobotSide.RIGHT);
   }

   private void addSnapData(Pose3D footstepPose, RobotSide side)
   {
      FootstepNode footstepNode = new FootstepNode(footstepPose.getX(), footstepPose.getY(), footstepPose.getYaw(), side);
      FootstepNodeSnapData snapData = new FootstepNodeSnapData(FootstepNodeSnappingTools.computeSnapTransform(footstepNode, footstepPose));
      snapper.addSnapData(footstepNode, snapData);
      snapAndWiggler.addSnapData(footstepNode, snapData);
   }

   private static FootstepNode createStartNode(FootstepPlannerRequest request)
   {
      RobotSide robotSide = request.getRequestedInitialStanceSide();
      Pose3D startFootPose = request.getStartFootPoses().get(robotSide);
      return new FootstepNode(startFootPose.getX(), startFootPose.getY(), startFootPose.getYaw(), robotSide);
   }

   private SideDependentList<FootstepNode> createGoalNodes(FootstepPlannerRequest request)
   {
      return new SideDependentList<>(side ->
                                     {
                                        Pose3D goalFootPose = request.getGoalFootPoses().get(side);
                                        return new FootstepNode(goalFootPose.getX(), goalFootPose.getY(), goalFootPose.getYaw(), side);
                                     });
   }
}
