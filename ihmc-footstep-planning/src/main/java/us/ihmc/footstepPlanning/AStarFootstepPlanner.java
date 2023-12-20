package us.ihmc.footstepPlanning;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.graphSearch.AStarFootstepPlannerIterationConductor;
import us.ihmc.footstepPlanning.graphSearch.AStarIterationData;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerHeuristicCalculator;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnappingTools;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepChecker;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCostCalculator;
import us.ihmc.footstepPlanning.graphSearch.stepExpansion.IdealStepCalculator;
import us.ihmc.footstepPlanning.graphSearch.stepExpansion.ParameterBasedStepExpansion;
import us.ihmc.footstepPlanning.graphSearch.stepExpansion.ReferenceBasedIdealStepCalculator;
import us.ihmc.footstepPlanning.graphSearch.stepExpansion.ReferenceBasedStepExpansion;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.footstepPlanning.log.FootstepPlannerIterationData;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Function;

public class AStarFootstepPlanner
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final AStarFootstepPlannerIterationConductor iterationConductor;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final FootstepPlannerEnvironmentHandler plannerEnvironmentHandler;
   private final FootstepSnapAndWiggler snapper;
   private final ParameterBasedStepExpansion nominalExpansion;
   private final ReferenceBasedStepExpansion referenceBasedExpansion;
   private final FootstepChecker checker;
   private final FootstepPlannerHeuristicCalculator distanceAndYawHeuristics;
   private final IdealStepCalculator idealStepCalculator;
   private final ReferenceBasedIdealStepCalculator referenceBasedIdealStepCalculator;
   private final FootstepPlannerCompletionChecker completionChecker;
   private final FootstepCostCalculator stepCostCalculator;
   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder;

   private final FootstepPlannerEdgeData edgeData;
   private final HashMap<GraphEdge<FootstepGraphNode>, FootstepPlannerEdgeData> edgeDataMap = new HashMap<>();
   private final List<FootstepPlannerIterationData> iterationData = new ArrayList<>();
   private final List<FootstepPlannerTerminationCondition> customTerminationConditions = new ArrayList<>();

   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final FramePose3D goalMidFootPose = new FramePose3D();
   private final AtomicBoolean haltRequested = new AtomicBoolean();

   private final SwingPlanningModule swingPlanningModule;

   /** Called each iteration. Should be very lightweight, mainly used for variable copying for the logger */
   private List<Consumer<AStarIterationData<FootstepGraphNode>>> iterationCallbacks = new ArrayList<>();
   /** Called at the status publish frequency. Post-processes the plan and publishes it */
   private final List<Consumer<FootstepPlannerOutput>> statusCallbacks;

   private double planningStartTime;
   private final Stopwatch stopwatch;
   private int iterations = 0;
   private FootstepPlanningResult result = null;

   public AStarFootstepPlanner(FootstepPlannerParametersBasics footstepPlannerParameters,
                               SideDependentList<ConvexPolygon2D> footPolygons,
                               WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder,
                               SwingPlannerParametersBasics swingPlannerParameters,
                               WalkingControllerParameters walkingControllerParameters,
                               StepReachabilityData stepReachabilityData,
                               Stopwatch stopwatch,
                               List<Consumer<FootstepPlannerOutput>> statusCallbacks)
   {
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.bodyPathPlanHolder = bodyPathPlanHolder;
      this.footPolygons = footPolygons;
      this.plannerEnvironmentHandler = new FootstepPlannerEnvironmentHandler(footPolygons);
      this.snapper = new FootstepSnapAndWiggler(footPolygons, footstepPlannerParameters, plannerEnvironmentHandler);
      this.stopwatch = stopwatch;
      this.statusCallbacks = statusCallbacks;

      this.checker = new FootstepChecker(footstepPlannerParameters, footPolygons, snapper, stepReachabilityData, registry);
      this.idealStepCalculator = new IdealStepCalculator(footstepPlannerParameters, checker, bodyPathPlanHolder, plannerEnvironmentHandler, registry);
      this.referenceBasedIdealStepCalculator = new ReferenceBasedIdealStepCalculator(footstepPlannerParameters, idealStepCalculator, registry);

      this.nominalExpansion = new ParameterBasedStepExpansion(footstepPlannerParameters, referenceBasedIdealStepCalculator, footPolygons);
      this.referenceBasedExpansion = new ReferenceBasedStepExpansion(referenceBasedIdealStepCalculator, nominalExpansion);

      this.distanceAndYawHeuristics = new FootstepPlannerHeuristicCalculator(footstepPlannerParameters, bodyPathPlanHolder, registry);
      stepCostCalculator = new FootstepCostCalculator(footstepPlannerParameters, snapper, referenceBasedIdealStepCalculator, distanceAndYawHeuristics::compute, footPolygons, registry);

      this.iterationConductor = new AStarFootstepPlannerIterationConductor(referenceBasedExpansion, checker, stepCostCalculator, distanceAndYawHeuristics::compute);
      this.completionChecker = new FootstepPlannerCompletionChecker(footstepPlannerParameters, iterationConductor, distanceAndYawHeuristics, snapper);

      referenceBasedIdealStepCalculator.setFootstepGraph(iterationConductor.getGraph());

      List<YoVariable> allVariables = registry.collectSubtreeVariables();
      this.edgeData = new FootstepPlannerEdgeData(allVariables.size());
      iterationConductor.getGraph().setGraphExpansionCallback(edge ->
                                                           {
                                                              for (int i = 0; i < allVariables.size(); i++)
                                                              {
                                                                 edgeData.setData(i, allVariables.get(i).getValueAsLongBits());
                                                              }

                                                              edgeData.setParentNode(edge.getStartNode());
                                                              edgeData.setChildNode(edge.getEndNode());
                                                              edgeData.getEndStepSnapData().set(snapper.snapFootstep(edge.getEndNode().getSecondStep()));

                                                              edgeDataMap.put(edge, edgeData.getCopyAndClear());
                                                              stepCostCalculator.resetLoggedVariables();
                                                           });

      this.swingPlanningModule = new SwingPlanningModule(footstepPlannerParameters,
                                                         swingPlannerParameters,
                                                         walkingControllerParameters,
                                                         footPolygons);
   }

   public void handleRequest(FootstepPlannerRequest request, FootstepPlannerOutput outputToPack)
   {
      iterations = 0;
      planningStartTime = stopwatch.totalElapsed();
      stopwatch.lap();

      // Reset logged variables
      edgeData.clear();
      edgeDataMap.clear();
      iterationData.clear();

      haltRequested.set(false);
      result = FootstepPlanningResult.PLANNING;

      // Update planar regions
      boolean hasHeightMap = request.getHeightMapData() != null && !request.getHeightMapData().isEmpty();
      boolean flatGroundMode = request.getAssumeFlatGround() || !hasHeightMap;

      HeightMapData heightMapData = flatGroundMode ? null : request.getHeightMapData();

      if (flatGroundMode)
      {
         double flatGroundHeight = 0.5 * (request.getStartFootPoses().get(RobotSide.LEFT).getZ() + request.getStartFootPoses().get(RobotSide.RIGHT).getZ());
         snapper.setFlatGroundHeight(flatGroundHeight);
      }

      snapper.clearSnapData();
      plannerEnvironmentHandler.setFallbackHeightMap(heightMapData);

      checker.setHeightMapData(heightMapData);
      stepCostCalculator.setHeightMapData(heightMapData);

      double pathLength = bodyPathPlanHolder.computePathLength(0.0);
      boolean imposeHorizonLength = request.getPlanBodyPath() && request.getHorizonLength() > 0.0 && !MathTools.intervalContains(pathLength, 0.0, request.getHorizonLength());
      SideDependentList<DiscreteFootstep> goalSteps;
      if (imposeHorizonLength)
      {
         bodyPathPlanHolder.getPointAlongPath(request.getHorizonLength() / pathLength, goalMidFootPose);
         SideDependentList<Pose3D> goalStepPoses = PlannerTools.createSquaredUpFootsteps(goalMidFootPose, footstepPlannerParameters.getIdealFootstepWidth());
         goalSteps = createGoalSteps(goalStepPoses::get);
      }
      else
      {
         goalMidFootPose.interpolate(request.getGoalFootPoses().get(RobotSide.LEFT), request.getGoalFootPoses().get(RobotSide.RIGHT), 0.5);
         goalSteps = createGoalSteps(request.getGoalFootPoses()::get);
      }

      // Setup footstep planner
      FootstepGraphNode startNode = createStartNode(request);
      addFootPosesToSnapper(request);
      iterationConductor.initialize(startNode);
      distanceAndYawHeuristics.initialize(goalMidFootPose);
      idealStepCalculator.initialize(goalSteps);
      completionChecker.initialize(startNode, goalSteps, request.getGoalDistanceProximity(), request.getGoalYawProximity());
      nominalExpansion.initialize();
      snapper.initialize();

      // Check valid goal
      if (!snapAndCheckGoalNodes(goalSteps, imposeHorizonLength, request))
      {
         result = FootstepPlanningResult.INVALID_GOAL;
         reportStatus(request, outputToPack);
         return;
      }

      // Start planning loop
      if (request.getReferencePlan() == null)
      {
         referenceBasedIdealStepCalculator.clearReferencePlan();
      }
      else
      {
         referenceBasedIdealStepCalculator.setReferenceFootstepPlan(request.getReferencePlan());
      }

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

         FootstepGraphNode nodeToExpand = iterationConductor.getNextNode();
         if (nodeToExpand == null)
         {
            result = FootstepPlanningResult.NO_PATH_EXISTS;
            break;
         }

         boolean performIterativeExpansion = false;
         AStarIterationData<FootstepGraphNode> iterationData = iterationConductor.doPlanningIteration(nodeToExpand, performIterativeExpansion);
         recordIterationData(iterationData);
         iterationCallbacks.forEach(callback -> callback.accept(iterationData));

         FootstepGraphNode achievedGoalNode = completionChecker.checkIfGoalIsReached(iterationData);
         if (achievedGoalNode != null)
         {
            // the final graph expansion is handled manually when not in proximity mode
            if (!completionChecker.isProximityModeEnabled())
            {
               AStarIterationData<FootstepGraphNode> finalIterationData = new AStarIterationData<>();
               finalIterationData.setParentNode(achievedGoalNode);
               finalIterationData.getValidChildNodes().add(completionChecker.getEndNode());
               recordIterationData(finalIterationData);
               iterationCallbacks.forEach(callback -> callback.accept(iterationData));
            }

            result = FootstepPlanningResult.FOUND_SOLUTION;
            break;
         }
         if (publishStatus(request))
         {
            reportStatus(request, outputToPack);
            stopwatch.lap();
         }
      }

      markSolutionEdges();
      reportStatus(request, outputToPack);
   }

   private boolean publishStatus(FootstepPlannerRequest request)
   {
      double statusPublishPeriod = request.getStatusPublishPeriod();
      if (statusPublishPeriod <= 0.0)
      {
         return false;
      }

      return stopwatch.lapElapsed() > statusPublishPeriod && !MathTools.epsilonEquals(stopwatch.totalElapsed(), request.getTimeout(), 0.8 * request.getStatusPublishPeriod());
   }

   private void reportStatus(FootstepPlannerRequest request, FootstepPlannerOutput outputToPack)
   {
      outputToPack.setFootstepPlanningResult(result);

      // Pack solution path
      outputToPack.getFootstepPlan().clear();
      List<FootstepGraphNode> path = iterationConductor.getGraph().getPathFromStart(completionChecker.getEndNode());
      for (int i = 1; i < path.size(); i++)
      {
         FootstepGraphNode footstepNode = path.get(i);
         FootstepSnapData snapData = snapper.snapFootstep(footstepNode.getSecondStep(), footstepNode.getFirstStep(), true);
         PlannedFootstep footstep = new PlannedFootstep(footstepNode.getSecondStepSide());
         footstep.getFootstepPose().set(snapData.getSnappedStepTransform(footstepNode.getSecondStep()));

         if (!footstepPlannerParameters.getWiggleWhilePlanning())
         {
            // log wiggle transform if not yet computed
            edgeDataMap.get(new GraphEdge<>(path.get(i - 1), path.get(i))).getEndStepSnapData().set(snapData);
         }

         outputToPack.getFootstepPlan().addFootstep(footstep);
      }

      if (!request.getAssumeFlatGround())
      {
         swingPlanningModule.computeSwingWaypoints(request.getPlanarRegionsList(),
                                                   request.getHeightMapData(),
                                                   outputToPack.getFootstepPlan(),
                                                   request.getStartFootPoses(),
                                                   request.getSwingPlannerType());
      }

      outputToPack.getPlannerTimings().setTimePlanningStepsSeconds(stopwatch.totalElapsed() - planningStartTime);
      outputToPack.getPlannerTimings().setTotalElapsedSeconds(stopwatch.totalElapsed());
      statusCallbacks.forEach(callback -> callback.accept(outputToPack));
   }

   public void clearLoggedData()
   {
      edgeDataMap.clear();
      iterationData.clear();
   }

   private void markSolutionEdges()
   {
      edgeDataMap.values().forEach(data -> data.setSolutionEdge(false));

      List<FootstepGraphNode> path = iterationConductor.getGraph().getPathFromStart(completionChecker.getEndNode());
      for (int i = 1; i < path.size(); i++)
      {
         edgeDataMap.get(new GraphEdge<>(path.get(i - 1), path.get(i))).setSolutionEdge(true);
      }
   }

   private void recordIterationData(AStarIterationData<FootstepGraphNode> iterationData)
   {
      if (iterationData.getParentNode() == null)
      {
         return;
      }

      /* Check if this node has already been expanded, since a max branch factor can be imposed, if so append that log data */
      FootstepPlannerIterationData loggedData = null;
      for (int i = 0; i < this.iterationData.size(); i++)
      {
         if (this.iterationData.get(i).getParentNode().equals(iterationData.getParentNode()))
         {
            loggedData = this.iterationData.get(i);
            break;
         }
      }

      if (loggedData == null)
      {
         loggedData = new FootstepPlannerIterationData();
         loggedData.setParentNode(iterationData.getParentNode());
         referenceBasedIdealStepCalculator.setFootstepGraph(iterationConductor.getGraph());
         DiscreteFootstep idealFootstep = referenceBasedIdealStepCalculator.computeIdealStep(iterationData.getParentNode().getSecondStep(),
                                                                                             iterationData.getParentNode().getFirstStep());
         DiscreteFootstep nominalIdealStep = referenceBasedIdealStepCalculator.getNominalIdealStep();
         loggedData.setIdealChildNode(new FootstepGraphNode(iterationData.getParentNode().getSecondStep(), idealFootstep));
         loggedData.setNominalIdealChildNode(new FootstepGraphNode(iterationData.getParentNode().getSecondStep(), nominalIdealStep));
         loggedData.setParentEndSnapData(snapper.snapFootstep(iterationData.getParentNode().getSecondStep()));
         loggedData.setParentStartSnapData(snapper.snapFootstep(iterationData.getParentNode().getFirstStep()));
         this.iterationData.add(loggedData);
      }

      iterationData.getValidChildNodes().forEach(loggedData::addChildNode);
      iterationData.getInvalidChildNodes().forEach(loggedData::addChildNode);
   }

   public boolean checkCustomTerminationConditions()
   {
      if (customTerminationConditions.isEmpty())
      {
         return false;
      }

      int endNodePathSize = completionChecker.getEndNodePathSize();
      for (int i = 0; i < customTerminationConditions.size(); i++)
      {
         boolean terminatePlanner = customTerminationConditions.get(i)
                                                               .terminatePlanner(stopwatch.totalElapsed(),
                                                                                 iterations,
                                                                                 completionChecker.getEndNodeEndStepTransform(),
                                                                                 completionChecker.getEndNodeStartStepTransform(),
                                                                                 endNodePathSize);
         if (terminatePlanner)
         {
            return true;
         }
      }

      return false;
   }

   public void addIterationCallback(Consumer<AStarIterationData<FootstepGraphNode>> callback)
   {
      iterationCallbacks.add(callback);
   }

   public void addCustomTerminationCondition(FootstepPlannerTerminationCondition plannerTerminationCondition)
   {
      customTerminationConditions.add(plannerTerminationCondition);
   }

   public void clearCustomTerminationConditions()
   {
      customTerminationConditions.clear();
   }

   private boolean snapAndCheckGoalNodes(SideDependentList<DiscreteFootstep> goalSteps, boolean horizonLengthImposed, FootstepPlannerRequest request)
   {
      FootstepSnapData leftGoalStepSnapData, rightGoalStepSnapData;
      boolean snapGoalSteps = horizonLengthImposed || request.getSnapGoalSteps();

      if (snapGoalSteps)
      {
         leftGoalStepSnapData = snapper.snapFootstep(goalSteps.get(RobotSide.LEFT));
         rightGoalStepSnapData = snapper.snapFootstep(goalSteps.get(RobotSide.RIGHT));
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
      DiscreteFootstep footstep = new DiscreteFootstep(footstepPose.getX(), footstepPose.getY(), footstepPose.getYaw(), side);
      FootstepSnapData snapData = new FootstepSnapData(FootstepSnappingTools.computeSnapTransform(footstep, footstepPose));
      snapData.getCroppedFoothold().set(footPolygons.get(side));
      snapData.getWiggleTransformInWorld().setIdentity();
      snapper.addSnapData(footstep, snapData);
   }

   private static FootstepGraphNode createStartNode(FootstepPlannerRequest request)
   {
      RobotSide initialStanceSide = request.getRequestedInitialStanceSide();

      Pose3D initialStancePose = request.getStartFootPoses().get(initialStanceSide);
      Pose3D initialSwingPose = request.getStartFootPoses().get(initialStanceSide.getOppositeSide());

      DiscreteFootstep initialStanceStep = new DiscreteFootstep(initialStancePose.getX(), initialStancePose.getY(), initialStancePose.getYaw(), initialStanceSide);
      DiscreteFootstep initialStartOfSwing = new DiscreteFootstep(initialSwingPose.getX(), initialSwingPose.getY(), initialSwingPose.getYaw(), initialStanceSide.getOppositeSide());
      return new FootstepGraphNode(initialStartOfSwing, initialStanceStep);
   }

   private static SideDependentList<DiscreteFootstep> createGoalSteps(Function<RobotSide, Pose3D> poses)
   {
      return new SideDependentList<>(side ->
                                     {
                                        Pose3DReadOnly goalFootPose = poses.apply(side);
                                        return new DiscreteFootstep(goalFootPose.getX(), goalFootPose.getY(), goalFootPose.getYaw(), side);
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

   public FootstepSnapAndWiggler getSnapper()
   {
      return snapper;
   }

   public FootstepChecker getChecker()
   {
      return checker;
   }

   public AStarFootstepPlannerIterationConductor getIterationConductor()
   {
      return iterationConductor;
   }

   public FootstepGraphNode getEndNode()
   {
      return completionChecker.getEndNode();
   }

   public HashMap<GraphEdge<FootstepGraphNode>, FootstepPlannerEdgeData> getEdgeDataMap()
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

   public SwingPlanningModule getSwingPlanningModule()
   {
      return swingPlanningModule;
   }

   public ReferenceBasedIdealStepCalculator getReferenceBasedIdealStepCalculator()
   {
      return referenceBasedIdealStepCalculator;
   }
}
