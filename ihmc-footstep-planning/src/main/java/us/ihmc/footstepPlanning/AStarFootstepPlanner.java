package us.ihmc.footstepPlanning;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.graphSearch.AStarFootstepPlannerIterationConductor;
import us.ihmc.footstepPlanning.graphSearch.AStarIterationData;
import us.ihmc.footstepPlanning.graphSearch.FootstepCostCalculator;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerHeuristicCalculator;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnappingTools;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstanceNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.IdealStepCalculator;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.footstepPlanning.log.FootstepPlannerIterationData;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Function;

public class AStarFootstepPlanner
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final AStarFootstepPlannerIterationConductor footstepPlanner;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final FootstepNodeSnapAndWiggler snapper;
   private final ParameterBasedNodeExpansion expansion;
   private final FootstepNodeChecker checker;
   private final FootstepPlannerHeuristicCalculator distanceAndYawHeuristics;
   private final IdealStepCalculator idealStepCalculator;
   private final FootstepPlannerCompletionChecker completionChecker;
   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder;

   private final FootstepPlannerEdgeData edgeData;
   private final HashMap<GraphEdge<FootstepNode>, FootstepPlannerEdgeData> edgeDataMap = new HashMap<>();
   private final List<FootstepPlannerIterationData> iterationData = new ArrayList<>();
   private final List<FootstepPlannerTerminationCondition> customTerminationConditions = new ArrayList<>();

   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final FramePose3D goalMidFootPose = new FramePose3D();
   private final AtomicBoolean haltRequested = new AtomicBoolean();

   private Consumer<Pair<FootstepPlannerRequest, FootstepPlannerOutput>> postProcessorCallback = null;
   private Consumer<AStarIterationData<FootstanceNode>> iterationCallback = iterationData -> {};

   private final Stopwatch stopwatch = new Stopwatch();
   private int iterations = 0;
   private FootstepPlanningResult result = null;

   public AStarFootstepPlanner(FootstepPlannerParametersBasics footstepPlannerParameters,
                               SideDependentList<ConvexPolygon2D> footPolygons,
                               WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder)
   {
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.bodyPathPlanHolder = bodyPathPlanHolder;
      this.footPolygons = footPolygons;
      this.snapper = new FootstepNodeSnapAndWiggler(footPolygons, footstepPlannerParameters);

      this.checker = new FootstepNodeChecker(footstepPlannerParameters, footPolygons, snapper, registry);
      this.idealStepCalculator = new IdealStepCalculator(footstepPlannerParameters, checker::isNodeValid, bodyPathPlanHolder, registry);
      this.expansion = new ParameterBasedNodeExpansion(footstepPlannerParameters, idealStepCalculator::computeIdealNode, footPolygons);

      this.distanceAndYawHeuristics = new FootstepPlannerHeuristicCalculator(snapper, footstepPlannerParameters, bodyPathPlanHolder, registry);
      FootstepCostCalculator stepCostCalculator = new FootstepCostCalculator(footstepPlannerParameters, snapper, idealStepCalculator::computeIdealNode, distanceAndYawHeuristics::compute, footPolygons, registry);

      this.footstepPlanner = new AStarFootstepPlannerIterationConductor(expansion, checker::isNodeValid, stepCostCalculator::computeCost, distanceAndYawHeuristics::compute);
      this.completionChecker = new FootstepPlannerCompletionChecker(footstepPlannerParameters, footstepPlanner, distanceAndYawHeuristics);

      List<YoVariable> allVariables = registry.collectSubtreeVariables();
      this.edgeData = new FootstepPlannerEdgeData(allVariables.size());
      footstepPlanner.getGraph().setGraphExpansionCallback(edge ->
                                                           {
                                                              for (int i = 0; i < allVariables.size(); i++)
                                                              {
                                                                 edgeData.setData(i, allVariables.get(i).getValueAsLongBits());
                                                              }

                                                              // TODO
//                                                              edgeData.setStanceNode(edge.getStartNode());
//                                                              edgeData.setCandidateNode(edge.getEndNode());
//                                                              edgeData.getCandidateNodeSnapData().set(snapper.snapFootstepNode(edge.getEndNode()));

//                                                              edgeDataMap.put(edge, edgeData.getCopyAndClear());
                                                              stepCostCalculator.resetLoggedVariables();
                                                           });
   }

   public void handleRequest(FootstepPlannerRequest request, FootstepPlannerOutput outputToPack)
   {
      Objects.requireNonNull(postProcessorCallback);
      iterations = 0;
      stopwatch.start();

      // Reset logged variables
      edgeData.clear();
      edgeDataMap.clear();
      iterationData.clear();

      haltRequested.set(false);
      result = FootstepPlanningResult.PLANNING;
      outputToPack.setRequestId(request.getRequestId());

      // Update planar regions
      boolean flatGroundMode = request.getAssumeFlatGround() || request.getPlanarRegionsList() == null || request.getPlanarRegionsList().isEmpty();
      PlanarRegionsList planarRegionsList = flatGroundMode ? null : request.getPlanarRegionsList();

      snapper.setPlanarRegions(planarRegionsList);
      checker.setPlanarRegions(planarRegionsList);
      idealStepCalculator.setPlanarRegionsList(planarRegionsList);

      double pathLength = bodyPathPlanHolder.computePathLength(0.0);
      boolean imposeHorizonLength =
            request.getPlanBodyPath() && request.getHorizonLength() > 0.0 && !MathTools.intervalContains(pathLength, 0.0, request.getHorizonLength());
      SideDependentList<FootstepNode> goalNodes;
      if (imposeHorizonLength)
      {
         bodyPathPlanHolder.getPointAlongPath(request.getHorizonLength() / pathLength, goalMidFootPose);
         SideDependentList<Pose3D> goalSteps = PlannerTools.createSquaredUpFootsteps(goalMidFootPose, footstepPlannerParameters.getIdealFootstepWidth());
         goalNodes = createGoalNodes(goalSteps::get);
      }
      else
      {
         goalMidFootPose.interpolate(request.getGoalFootPoses().get(RobotSide.LEFT), request.getGoalFootPoses().get(RobotSide.RIGHT), 0.5);
         goalNodes = createGoalNodes(request.getGoalFootPoses()::get);
      }

      // Setup footstep planner
      FootstanceNode startNode = createStartNode(request);
      addFootPosesToSnapper(request);
      footstepPlanner.initialize(startNode);
      distanceAndYawHeuristics.initialize(goalMidFootPose, request.getDesiredHeading());
      idealStepCalculator.initialize(goalNodes, request.getDesiredHeading());
      completionChecker.initialize(startNode, goalNodes, request.getGoalDistanceProximity(), request.getGoalYawProximity());
      expansion.initialize();
      snapper.initialize();

      // Check valid goal
      if (!snapAndCheckGoalNodes(goalNodes, imposeHorizonLength, request))
      {
         result = FootstepPlanningResult.INVALID_GOAL;
         reportStatus(request, outputToPack);
         return;
      }

      // Start planning loop
      while (true)
      {
         iterations++;
         outputToPack.getPlannerTimings().setStepPlanningIterations(iterations);

         if (stopwatch.totalElapsed() >= request.getTimeout())
         {
            result = FootstepPlanningResult.TIMED_OUT_BEFORE_SOLUTION;
            break;
         }
         if (haltRequested.get() || checkCustomTerminationConditions())
         {
            result = FootstepPlanningResult.HALTED;
            break;
         }
         if (request.getMaximumIterations() > 0 && iterations > request.getMaximumIterations())
         {
            result = FootstepPlanningResult.MAXIMUM_ITERATIONS_REACHED;
            break;
         }

         FootstanceNode nodeToExpand = footstepPlanner.getNextNode();
         if (nodeToExpand == null)
         {
            result = FootstepPlanningResult.NO_PATH_EXISTS;
            break;
         }

         AStarIterationData<FootstanceNode> iterationData = footstepPlanner.doPlanningIteration(nodeToExpand, true);
//         recordIterationData(iterationData);
         iterationCallback.accept(iterationData);

         FootstanceNode achievedGoalNode = completionChecker.checkIfGoalIsReached(iterationData);
         if (achievedGoalNode != null)
         {
            // the final graph expansion is handled manually
            AStarIterationData<FootstanceNode> finalIterationData = new AStarIterationData<>();
            finalIterationData.setParentNode(achievedGoalNode);
            finalIterationData.getValidChildNodes().add(completionChecker.getEndNode());
//            recordIterationData(finalIterationData);
            iterationCallback.accept(finalIterationData);

            result = FootstepPlanningResult.FOUND_SOLUTION;
            break;
         }
         if (publishStatus(request))
         {
            reportStatus(request, outputToPack);
            stopwatch.lap();
         }
      }

//      markSolutionEdges();
      reportStatus(request, outputToPack);
   }

   private boolean publishStatus(FootstepPlannerRequest request)
   {
      double statusPublishPeriod = request.getStatusPublishPeriod();
      if (statusPublishPeriod <= 0.0)
      {
         return false;
      }

      return stopwatch.lapElapsed() > statusPublishPeriod && !MathTools.epsilonEquals(stopwatch.totalElapsed(), request.getTimeout(), 0.1);
   }

   private void reportStatus(FootstepPlannerRequest request, FootstepPlannerOutput outputToPack)
   {
      outputToPack.setRequestId(request.getRequestId());
      outputToPack.setFootstepPlanningResult(result);

      // Pack solution path
      outputToPack.getFootstepPlan().clear();
      List<FootstanceNode> path = footstepPlanner.getGraph().getPathFromStart(completionChecker.getEndNode());
      for (int i = 1; i < path.size(); i++)
      {
         FootstanceNode footstepNode = path.get(i);
         FootstepNodeSnapData snapData = snapper.snapFootstepNode(footstepNode.getStanceNode(), footstepNode.getSwingNode(), true);
         PlannedFootstep footstep = new PlannedFootstep(footstepNode.getStanceSide());
         footstep.getFootstepPose().set(snapData.getSnappedNodeTransform(footstepNode.getStanceNode()));

         if (request.getAssumeFlatGround() || request.getPlanarRegionsList() == null || request.getPlanarRegionsList().isEmpty())
         {
            double flatGroundHeight = 0.5 * (request.getStartFootPoses().get(RobotSide.LEFT).getZ() + request.getStartFootPoses().get(RobotSide.RIGHT).getZ());
            footstep.getFootstepPose().setZ(flatGroundHeight);
         }

         if (!footstepPlannerParameters.getWiggleWhilePlanning())
         {
            // log wiggle transform if not yet computed
//            edgeDataMap.get(new GraphEdge<>(path.get(i - 1).getStanceNode(), path.get(i).getStanceNode())).getCandidateNodeSnapData().set(snapData);
         }

         footstep.getFoothold().set(snapData.getCroppedFoothold());
         outputToPack.getFootstepPlan().addFootstep(footstep);
      }

      outputToPack.setPlanarRegionsList(request.getPlanarRegionsList());
      postProcessorCallback.accept(Pair.of(request, outputToPack));
   }

//   private void markSolutionEdges()
//   {
//      edgeDataMap.values().forEach(data -> data.setSolutionEdge(false));
//
//      List<FootstanceNode> path = footstepPlanner.getGraph().getPathFromStart(completionChecker.getEndNode());
//      for (int i = 1; i < path.size(); i++)
//      {
//         edgeDataMap.get(new GraphEdge<>(path.get(i - 1).getStanceNode(), path.get(i).getStanceNode())).setSolutionEdge(true);
//      }
//   }

//   private void recordIterationData(AStarIterationData<FootstanceNode> iterationData)
//   {
//      if (iterationData.getParentNode() == null)
//      {
//         return;
//      }
//
//      FootstepPlannerIterationData loggedData = null;
//      for (int i = 0; i < this.iterationData.size(); i++)
//      {
//         if (this.iterationData.get(i).getStanceNode().equals(iterationData.getParentNode()))
//         {
//            loggedData = this.iterationData.get(i);
//            break;
//         }
//      }
//
//      if (loggedData == null)
//      {
//         loggedData = new FootstepPlannerIterationData();
//         loggedData.setStanceNode(iterationData.getParentNode());
//         loggedData.setIdealStep(idealStepCalculator.computeIdealNode(iterationData.getParentNode()));
//         loggedData.setStanceNodeSnapData(snapper.snapFootstepNode(iterationData.getParentNode()));
//         this.iterationData.add(loggedData);
//      }
//
//      iterationData.getValidChildNodes().forEach(loggedData::addChildNode);
//      iterationData.getInvalidChildNodes().forEach(loggedData::addChildNode);
//   }

   private final Pose2D endNodePose = new Pose2D();

   public boolean checkCustomTerminationConditions()
   {
      if (customTerminationConditions.isEmpty())
      {
         return false;
      }

      FootstanceNode endNode = completionChecker.getEndNode();
      endNodePose.set(endNode.getStanceNode().getX(), endNode.getStanceNode().getY(), endNode.getStanceNode().getYaw());
      int endNodePathSize = completionChecker.getEndNodePathSize();

      for (int i = 0; i < customTerminationConditions.size(); i++)
      {
         boolean terminatePlanner = customTerminationConditions.get(i).terminatePlanner(stopwatch.totalElapsed(), iterations, endNodePose, endNodePathSize);
         if (terminatePlanner)
         {
            return true;
         }
      }

      return false;
   }

   public void setPostProcessorCallback(Consumer<Pair<FootstepPlannerRequest, FootstepPlannerOutput>> postProcessorCallback)
   {
      this.postProcessorCallback = postProcessorCallback;
   }

   public void addIterationCallback(Consumer<AStarIterationData<FootstanceNode>> callback)
   {
      iterationCallback = iterationCallback.andThen(callback);
   }

   public void addCustomTerminationCondition(FootstepPlannerTerminationCondition plannerTerminationCondition)
   {
      customTerminationConditions.add(plannerTerminationCondition);
   }

   public void clearCustomTerminationConditions()
   {
      customTerminationConditions.clear();
   }

   private boolean snapAndCheckGoalNodes(SideDependentList<FootstepNode> goalNodes, boolean horizonLengthImposed, FootstepPlannerRequest request)
   {
      FootstepNodeSnapData leftGoalStepSnapData, rightGoalStepSnapData;
      boolean snapGoalSteps = horizonLengthImposed || request.getSnapGoalSteps();

      if (snapGoalSteps)
      {
         leftGoalStepSnapData = snapper.snapFootstepNode(goalNodes.get(RobotSide.LEFT));
         rightGoalStepSnapData = snapper.snapFootstepNode(goalNodes.get(RobotSide.RIGHT));
      }
      else
      {
         addSnapData(request.getGoalFootPoses().get(RobotSide.LEFT), RobotSide.LEFT);
         addSnapData(request.getGoalFootPoses().get(RobotSide.RIGHT), RobotSide.RIGHT);
         return true;
      }

      if (request.getAbortIfGoalStepSnappingFails())
      {
         return !leftGoalStepSnapData.getSnapTransform().containsNaN() && !rightGoalStepSnapData.getSnapTransform().containsNaN();
      }
      else
      {
         return true;
      }
   }

   public void halt()
   {
      haltRequested.set(true);
   }

   private void addFootPosesToSnapper(FootstepPlannerRequest request)
   {
      addSnapData(request.getStartFootPoses().get(RobotSide.LEFT), RobotSide.LEFT);
      addSnapData(request.getStartFootPoses().get(RobotSide.RIGHT), RobotSide.RIGHT);
   }

   private void addSnapData(Pose3D footstepPose, RobotSide side)
   {
      FootstepNode footstepNode = new FootstepNode(footstepPose.getX(), footstepPose.getY(), footstepPose.getYaw(), side);
      FootstepNodeSnapData snapData = new FootstepNodeSnapData(FootstepNodeSnappingTools.computeSnapTransform(footstepNode, footstepPose));
      snapData.getCroppedFoothold().set(footPolygons.get(side));
      snapData.getWiggleTransformInWorld().setIdentity();
      snapper.addSnapData(footstepNode, snapData);
   }

   private static FootstanceNode createStartNode(FootstepPlannerRequest request)
   {
      RobotSide initialStanceSide = request.getRequestedInitialStanceSide();

      Pose3D initialStancePose = request.getStartFootPoses().get(initialStanceSide);
      Pose3D initialSwingPose = request.getStartFootPoses().get(initialStanceSide.getOppositeSide());

      FootstepNode initialStanceNode = new FootstepNode(initialStancePose.getX(), initialStancePose.getY(), initialStancePose.getYaw(), initialStanceSide);
      FootstepNode initialSwingNode = new FootstepNode(initialSwingPose.getX(), initialSwingPose.getY(), initialSwingPose.getYaw(), initialStanceSide.getOppositeSide());
      return new FootstanceNode(initialStanceNode, initialSwingNode);
   }

   private static SideDependentList<FootstepNode> createGoalNodes(Function<RobotSide, Pose3D> poses)
   {
      return new SideDependentList<>(side ->
                                     {
                                        Pose3DReadOnly goalFootPose = poses.apply(side);
                                        return new FootstepNode(goalFootPose.getX(), goalFootPose.getY(), goalFootPose.getYaw(), side);
                                     });
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }

   public SideDependentList<ConvexPolygon2D> getFootPolygons()
   {
      return footPolygons;
   }

   public FootstepNodeSnapAndWiggler getSnapper()
   {
      return snapper;
   }

   public FootstepNodeChecker getChecker()
   {
      return checker;
   }

   public AStarFootstepPlannerIterationConductor getLowLevelStepPlanner()
   {
      return footstepPlanner;
   }

   public FootstanceNode getEndNode()
   {
      return completionChecker.getEndNode();
   }

   public HashMap<GraphEdge<FootstepNode>, FootstepPlannerEdgeData> getEdgeDataMap()
   {
      return edgeDataMap;
   }

   public List<FootstepPlannerIterationData> getIterationData()
   {
      return iterationData;
   }

   public int getIterations()
   {
      return iterations;
   }
}
