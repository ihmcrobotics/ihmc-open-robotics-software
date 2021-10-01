package us.ihmc.footstepPlanning;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Collectors;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.graphSearch.AStarIterationData;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepChecker;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.footstepPlanning.log.FootstepPlannerIterationData;
import us.ihmc.footstepPlanning.log.VariableDescriptor;
import us.ihmc.footstepPlanning.narrowPassage.NarrowPassageBodyPathOptimizer;
import us.ihmc.footstepPlanning.simplePlanners.PlanThenSnapPlanner;
import us.ihmc.footstepPlanning.swing.AdaptiveSwingTrajectoryCalculator;
import us.ihmc.footstepPlanning.swing.DefaultSwingPlannerParameters;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.BodyPathPostProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.yoVariables.variable.YoVariableType;

public class FootstepPlanningModule implements CloseableAndDisposable
{
   private final String name;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private ROS2NodeInterface ros2Node;
   private boolean manageROS2Node = false;
   private final VisibilityGraphsParametersBasics visibilityGraphParameters;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;

   private final VisibilityGraphPathPlanner bodyPathPlanner;
   private final NarrowPassageBodyPathOptimizer narrowPassageBodyPathOptimizer;
   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder = new WaypointDefinedBodyPathPlanHolder();

   private final PlanThenSnapPlanner planThenSnapPlanner;
   private final AStarFootstepPlanner aStarFootstepPlanner;
   private final List<VariableDescriptor> variableDescriptors;

   private final AtomicBoolean isPlanning = new AtomicBoolean();
   private final FootstepPlannerRequest request = new FootstepPlannerRequest();
   private final FootstepPlannerOutput output = new FootstepPlannerOutput();
   private final Stopwatch stopwatch = new Stopwatch();

   private final FramePose3D startMidFootPose = new FramePose3D();
   private final FramePose3D goalMidFootPose = new FramePose3D();

   private final List<Consumer<FootstepPlannerRequest>> requestCallbacks = new ArrayList<>();
   private final List<Consumer<FootstepPlannerOutput>> statusCallbacks = new ArrayList<>();

   private final List<Consumer<SwingPlannerType>> swingReplanRequestCallbacks = new ArrayList<>();
   private final List<Consumer<FootstepPlan>> swingReplanStatusCallbacks = new ArrayList<>();

   public FootstepPlanningModule(String name)
   {
      this(name,
           new DefaultVisibilityGraphParameters(),
           new DefaultFootstepPlannerParameters(),
           new DefaultSwingPlannerParameters(),
           null,
           PlannerTools.createDefaultFootPolygons(),
           null);
   }

   public FootstepPlanningModule(String name,
                                 VisibilityGraphsParametersBasics visibilityGraphParameters,
                                 FootstepPlannerParametersBasics footstepPlannerParameters,
                                 SwingPlannerParametersBasics swingPlannerParameters,
                                 WalkingControllerParameters walkingControllerParameters,
                                 SideDependentList<ConvexPolygon2D> footPolygons,
                                 StepReachabilityData stepReachabilityData)
   {
      this.name = name;
      this.visibilityGraphParameters = visibilityGraphParameters;
      this.footstepPlannerParameters = footstepPlannerParameters;

      BodyPathPostProcessor pathPostProcessor = new ObstacleAvoidanceProcessor(visibilityGraphParameters);
      this.bodyPathPlanner = new VisibilityGraphPathPlanner(visibilityGraphParameters, pathPostProcessor, registry);
      this.narrowPassageBodyPathOptimizer = new NarrowPassageBodyPathOptimizer(footstepPlannerParameters, registry);
      this.planThenSnapPlanner = new PlanThenSnapPlanner(footstepPlannerParameters, footPolygons);
      this.aStarFootstepPlanner = new AStarFootstepPlanner(footstepPlannerParameters,
                                                           footPolygons,
                                                           bodyPathPlanHolder,
                                                           swingPlannerParameters,
                                                           walkingControllerParameters,
                                                           stepReachabilityData);
      this.variableDescriptors = collectVariableDescriptors(aStarFootstepPlanner.getRegistry());

      addStatusCallback(output -> output.getPlannerTimings().setTimePlanningStepsSeconds(stopwatch.lapElapsed()));
      addStatusCallback(output -> output.getPlannerTimings().setTotalElapsedSeconds(stopwatch.totalElapsed()));
   }

   public FootstepPlannerOutput handleRequest(FootstepPlannerRequest request)
   {
      if (isPlanning.getAndSet(true))
      {
         LogTools.info("Received planning request packet but planner is currently running");
         return null;
      }
      else
      {
         LogTools.info("Handling footstep planner request...");
      }

      stopwatch.start();
      output.clear();

      try
      {
         handleRequestInternal(request);
      }
      catch (Exception exception)
      {
         exception.printStackTrace();

         output.clear();
         output.setRequestId(request.getRequestId());
         output.setFootstepPlanningResult(FootstepPlanningResult.EXCEPTION);
         output.setException(exception);
         statusCallbacks.forEach(callback -> callback.accept(output));
      }

      isPlanning.set(false);
      return output;
   }

   private void handleRequestInternal(FootstepPlannerRequest request) throws Exception
   {
      this.request.set(request);
      requestCallbacks.forEach(callback -> callback.accept(request));
      output.setRequestId(request.getRequestId());
      bodyPathPlanHolder.getPlan().clear();

      startMidFootPose.interpolate(request.getStartFootPoses().get(RobotSide.LEFT), request.getStartFootPoses().get(RobotSide.RIGHT), 0.5);
      goalMidFootPose.interpolate(request.getGoalFootPoses().get(RobotSide.LEFT), request.getGoalFootPoses().get(RobotSide.RIGHT), 0.5);

      // Update planar regions
      boolean flatGroundMode = request.getAssumeFlatGround() || request.getPlanarRegionsList() == null || request.getPlanarRegionsList().isEmpty();
      PlanarRegionsList planarRegionsList = flatGroundMode ? null : request.getPlanarRegionsList();
      bodyPathPlanner.setPlanarRegionsList(planarRegionsList);
      narrowPassageBodyPathOptimizer.setPlanarRegionsList(planarRegionsList);

      // record time
      output.getPlannerTimings().setTimeBeforePlanningSeconds(stopwatch.lap());

      if (request.getPlanBodyPath() && !flatGroundMode)
      {
         bodyPathPlanner.setStanceFootPoses(request.getStartFootPoses().get(RobotSide.LEFT), request.getStartFootPoses().get(RobotSide.RIGHT));
         bodyPathPlanner.setGoal(goalMidFootPose);

         BodyPathPlanningResult bodyPathPlannerResult = bodyPathPlanner.planWaypoints();
         List<Pose3DReadOnly> waypoints = bodyPathPlanner.getWaypoints();
         setNominalOrientations(waypoints);

         if (visibilityGraphParameters.getOptimizeForNarrowPassage())
         {
            List<FramePose3D> waypointsList = bodyPathPlanner.getWaypointsAsFramePoseList();

            narrowPassageBodyPathOptimizer.setWaypoints(waypointsList);
            waypoints = narrowPassageBodyPathOptimizer.runNarrowPassageOptimizer();
            bodyPathPlannerResult = narrowPassageBodyPathOptimizer.getBodyPathPlanningResult();
         }

         if (!bodyPathPlannerResult.validForExecution() || (waypoints.size() < 2 && request.getAbortIfBodyPathPlannerFails()))
         {
            reportBodyPathPlan(bodyPathPlannerResult);
            output.setBodyPathPlanningResult(bodyPathPlannerResult);
            statusCallbacks.forEach(callback -> callback.accept(output));
            return;
         }
         else if (waypoints.size() < 2 && !request.getAbortIfBodyPathPlannerFails())
         {
            double horizonLength = Double.POSITIVE_INFINITY;
            bodyPathPlanner.computeBestEffortPlan(horizonLength);
         }

         bodyPathPlanHolder.setPoseWaypoints(waypoints);
         double pathLength = bodyPathPlanHolder.computePathLength(0.0);
         if (MathTools.intervalContains(request.getHorizonLength(), 0.0, pathLength))
         {
            double alphaIntermediateGoal = request.getHorizonLength() / pathLength;
            bodyPathPlanHolder.getPointAlongPath(alphaIntermediateGoal, goalMidFootPose);
         }

         reportBodyPathPlan(BodyPathPlanningResult.FOUND_SOLUTION);
      }
      else if (visibilityGraphParameters.getOptimizeForNarrowPassage())
      {
         narrowPassageBodyPathOptimizer.setWaypointsFromStartAndEndPoses(startMidFootPose, goalMidFootPose);
         List<Pose3DReadOnly> waypoints = narrowPassageBodyPathOptimizer.runNarrowPassageOptimizer();

         bodyPathPlanHolder.setPoseWaypoints(waypoints);
         double pathLength = bodyPathPlanHolder.computePathLength(0.0);
         if (MathTools.intervalContains(request.getHorizonLength(), 0.0, pathLength))
         {
            double alphaIntermediateGoal = request.getHorizonLength() / pathLength;
            bodyPathPlanHolder.getPointAlongPath(alphaIntermediateGoal, goalMidFootPose);
         }

         reportBodyPathPlan(narrowPassageBodyPathOptimizer.getBodyPathPlanningResult());
      }
      else
      {
         List<Pose3DReadOnly> waypoints = new ArrayList<>();
         waypoints.add(startMidFootPose);
         waypoints.addAll(request.getBodyPathWaypoints());
         waypoints.add(goalMidFootPose);
         setNominalOrientations(waypoints);

         bodyPathPlanHolder.setPoseWaypoints(waypoints);
         reportBodyPathPlan(BodyPathPlanningResult.FOUND_SOLUTION);
      }

      if (request.getPerformAStarSearch())
      {
         aStarFootstepPlanner.setStatusCallbacks(statusCallbacks);
         aStarFootstepPlanner.handleRequest(request, output);
      }
      else
      {
         RobotSide initialStanceSide = request.getRequestedInitialStanceSide();
         FramePose3D initialStancePose = new FramePose3D(request.getStartFootPoses().get(initialStanceSide));
         planThenSnapPlanner.setInitialStanceFoot(initialStancePose, initialStanceSide);
         planThenSnapPlanner.setPlanarRegions(planarRegionsList);

         FootstepPlannerGoal goal = new FootstepPlannerGoal();
         goal.setGoalPoseBetweenFeet(goalMidFootPose);
         planThenSnapPlanner.setGoal(goal);

         FootstepPlanningResult result = planThenSnapPlanner.plan();

         FootstepPlan plan = planThenSnapPlanner.getPlan();
         output.setFootstepPlanningResult(result);
         output.getFootstepPlan().clear();
         for (int i = 0; i < plan.getNumberOfSteps(); i++)
         {
            output.getFootstepPlan().addFootstep(plan.getFootstep(i));
         }

         output.getPlannerTimings().setTimePlanningStepsSeconds(stopwatch.lap());

         aStarFootstepPlanner.getSwingPlanningModule()
                             .computeSwingWaypoints(request.getPlanarRegionsList(),
                                                    output.getFootstepPlan(),
                                                    request.getStartFootPoses(),
                                                    request.getSwingPlannerType());
         statusCallbacks.forEach(callback -> callback.accept(output));
      }
   }

   private static void setNominalOrientations(List<Pose3DReadOnly> waypoints)
   {
      for (int i = 0; i < waypoints.size() - 1; i++)
      {
         double dx = waypoints.get(i + 1).getX() - waypoints.get(i).getX();
         double dy = waypoints.get(i + 1).getY() - waypoints.get(i).getY();
         ((Pose3DBasics) waypoints.get(i)).getOrientation().setToYawOrientation(Math.atan2(dy, dx));
      }
   }

   /**
    * Requires that {@link #handleRequest(FootstepPlannerRequest)} has already been called. Replans the swing waypoints for the last planned path.
    */
   public FootstepPlan recomputeSwingTrajectories(SwingPlannerType swingPlannerType)
   {
      if (isPlanning.get())
      {
         LogTools.info("Received swing planning request packet but planner is currently running");
         return null;
      }
      else if (output.getFootstepPlan().isEmpty())
      {
         LogTools.info("Must plan a path before recomputing swing trajectory. Ignoring request.");
         return null;
      }
      else
      {
         LogTools.info("Handling swing planner request...");
      }

      try
      {
         swingReplanRequestCallbacks.forEach(callback -> callback.accept(swingPlannerType));
         aStarFootstepPlanner.getSwingPlanningModule()
                             .computeSwingWaypoints(request.getPlanarRegionsList(), output.getFootstepPlan(), request.getStartFootPoses(), swingPlannerType);
         swingReplanStatusCallbacks.forEach(callback -> callback.accept(output.getFootstepPlan()));
         return output.getFootstepPlan();
      }
      catch (Exception e)
      {
         e.printStackTrace();
         return null;
      }
   }

   private void reportBodyPathPlan(BodyPathPlanningResult bodyPathPlanningResult)
   {
      BodyPathPlan bodyPathPlan = bodyPathPlanHolder.getPlan();
      if (bodyPathPlan.getNumberOfWaypoints() > 0)
      {
         output.getBodyPath().clear();
         for (int i = 0; i < bodyPathPlan.getNumberOfWaypoints(); i++)
         {
            output.getBodyPath().add(new Pose3D(bodyPathPlan.getWaypoint(i)));
         }

         output.getGoalPose().set(goalMidFootPose);
      }

      output.setBodyPathPlanningResult(bodyPathPlanningResult);
      output.setFootstepPlanningResult(FootstepPlanningResult.PLANNING);
      output.getPlannerTimings().setTimePlanningBodyPathSeconds(stopwatch.lap());
      statusCallbacks.forEach(callback -> callback.accept(output));
   }

   public void addRequestCallback(Consumer<FootstepPlannerRequest> callback)
   {
      requestCallbacks.add(callback);
   }

   public void addIterationCallback(Consumer<AStarIterationData<FootstepGraphNode>> callback)
   {
      aStarFootstepPlanner.addIterationCallback(callback);
   }

   public void addStatusCallback(Consumer<FootstepPlannerOutput> callback)
   {
      statusCallbacks.add(callback);
   }

   public void addCustomTerminationCondition(FootstepPlannerTerminationCondition plannerTerminationCondition)
   {
      aStarFootstepPlanner.addCustomTerminationCondition(plannerTerminationCondition);
   }

   public void clearCustomTerminationConditions()
   {
      aStarFootstepPlanner.clearCustomTerminationConditions();
   }

   public boolean registerRosNode(ROS2NodeInterface ros2Node, boolean manageROS2Node)
   {
      this.manageROS2Node = manageROS2Node;
      if (this.ros2Node != null)
         return false;
      this.ros2Node = ros2Node;
      return true;
   }

   public void halt()
   {
      aStarFootstepPlanner.halt();
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

   public void addSwingReplanRequestCallback(Consumer<SwingPlannerType> callback)
   {
      swingReplanRequestCallbacks.add(callback);
   }

   public void addSwingReplanStatusCallback(Consumer<FootstepPlan> callback)
   {
      swingReplanStatusCallbacks.add(callback);
   }

   public FootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return footstepPlannerParameters;
   }

   public VisibilityGraphsParametersBasics getVisibilityGraphParameters()
   {
      return visibilityGraphParameters;
   }

   public SwingPlannerParametersBasics getSwingPlannerParameters()
   {
      return aStarFootstepPlanner.getSwingPlanningModule().getSwingPlannerParameters();
   }

   public SideDependentList<ConvexPolygon2D> getFootPolygons()
   {
      return aStarFootstepPlanner.getFootPolygons();
   }

   public FootstepSnapAndWiggler getSnapper()
   {
      return aStarFootstepPlanner.getSnapper();
   }

   public FootstepChecker getChecker()
   {
      return aStarFootstepPlanner.getChecker();
   }

   public VisibilityGraphPathPlanner getBodyPathPlanner()
   {
      return bodyPathPlanner;
   }

   public BodyPathPlan getBodyPathPlan()
   {
      return bodyPathPlanHolder.getBodyPathPlan();
   }

   public FootstepGraphNode getEndNode()
   {
      return aStarFootstepPlanner.getEndNode();
   }

   public HashMap<GraphEdge<FootstepGraphNode>, FootstepPlannerEdgeData> getEdgeDataMap()
   {
      return aStarFootstepPlanner.getEdgeDataMap();
   }

   public List<FootstepPlannerIterationData> getIterationData()
   {
      return aStarFootstepPlanner.getIterationData();
   }

   public YoRegistry getAStarPlannerRegistry()
   {
      return aStarFootstepPlanner.getRegistry();
   }

   public List<VariableDescriptor> getVariableDescriptors()
   {
      return variableDescriptors;
   }

   private static List<VariableDescriptor> collectVariableDescriptors(YoRegistry registry)
   {
      Function<YoVariable, VariableDescriptor> descriptorFunction = variable ->
      {
         if (variable.getType() == YoVariableType.ENUM)
         {
            return new VariableDescriptor(variable.getName(),
                                          variable.getType(),
                                          variable.getRegistry().getName(),
                                          ((YoEnum<?>) variable).getEnumValuesAsString());
         }
         else
         {
            return new VariableDescriptor(variable.getName(), variable.getType(), variable.getRegistry().getName());
         }
      };

      return registry.collectSubtreeVariables().stream().map(descriptorFunction).collect(Collectors.toList());
   }

   public AdaptiveSwingTrajectoryCalculator getAdaptiveSwingTrajectoryCalculator()
   {
      return aStarFootstepPlanner.getSwingPlanningModule().getAdaptiveSwingTrajectoryCalculator();
   }

   public SwingOverPlanarRegionsTrajectoryExpander getSwingOverPlanarRegionsTrajectoryExpander()
   {
      return aStarFootstepPlanner.getSwingPlanningModule().getSwingOverPlanarRegionsTrajectoryExpander();
   }

   public SwingPlanningModule getSwingPlanningModule()
   {
      return aStarFootstepPlanner.getSwingPlanningModule();
   }

   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public void closeAndDispose()
   {
      if (manageROS2Node && ros2Node != null)
      {
         ((ROS2Node) ros2Node).destroy();
         ros2Node = null;
      }
   }
}
