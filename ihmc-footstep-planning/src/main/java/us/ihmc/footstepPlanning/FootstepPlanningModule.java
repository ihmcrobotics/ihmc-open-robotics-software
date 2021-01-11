package us.ihmc.footstepPlanning;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.trajectories.SwingOverPlanarRegionsTrajectoryExpander;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.graphSearch.AStarIterationData;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepChecker;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.footstepPlanning.log.FootstepPlannerIterationData;
import us.ihmc.footstepPlanning.log.VariableDescriptor;
import us.ihmc.footstepPlanning.simplePlanners.PlanThenSnapPlanner;
import us.ihmc.footstepPlanning.swing.AdaptiveSwingTrajectoryCalculator;
import us.ihmc.footstepPlanning.swing.DefaultSwingPlannerParameters;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
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
import us.ihmc.tools.thread.CloseableAndDisposable;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.yoVariables.variable.YoVariableType;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Collectors;

public class FootstepPlanningModule implements CloseableAndDisposable
{
   private final String name;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private ROS2Node ros2Node;
   private final VisibilityGraphsParametersBasics visibilityGraphParameters;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;

   private final VisibilityGraphPathPlanner bodyPathPlanner;
   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder = new WaypointDefinedBodyPathPlanHolder();

   private final PlanThenSnapPlanner planThenSnapPlanner;
   private final AStarFootstepPlanner aStarFootstepPlanner;
   private final List<VariableDescriptor> variableDescriptors;

   private final FootstepPlanPostProcessHandler postProcessHandler;

   private final AtomicBoolean isPlanning = new AtomicBoolean();
   private final FootstepPlannerRequest request = new FootstepPlannerRequest();
   private final FootstepPlannerOutput output = new FootstepPlannerOutput();
   private final Stopwatch stopwatch = new Stopwatch();

   private final FramePose3D startMidFootPose = new FramePose3D();
   private final FramePose3D goalMidFootPose = new FramePose3D();

   private Consumer<FootstepPlannerRequest> requestCallback = request -> {};
   private Consumer<FootstepPlannerOutput> statusCallback = result -> {};

   public FootstepPlanningModule(String name)
   {
      this(name,
           new DefaultVisibilityGraphParameters(),
           new DefaultFootstepPlannerParameters(),
           new DefaultSwingPlannerParameters(),
           null,
           PlannerTools.createDefaultFootPolygons());
   }

   public FootstepPlanningModule(String name,
                                 VisibilityGraphsParametersBasics visibilityGraphParameters,
                                 FootstepPlannerParametersBasics footstepPlannerParameters,
                                 SwingPlannerParametersBasics swingPlannerParameters,
                                 WalkingControllerParameters walkingControllerParameters,
                                 SideDependentList<ConvexPolygon2D> footPolygons)
   {
      this.name = name;
      this.visibilityGraphParameters = visibilityGraphParameters;
      this.footstepPlannerParameters = footstepPlannerParameters;

      BodyPathPostProcessor pathPostProcessor = new ObstacleAvoidanceProcessor(visibilityGraphParameters);
      this.bodyPathPlanner = new VisibilityGraphPathPlanner(visibilityGraphParameters,
                                                            pathPostProcessor,
                                                            registry);

      this.planThenSnapPlanner = new PlanThenSnapPlanner(footstepPlannerParameters, footPolygons);
      this.aStarFootstepPlanner = new AStarFootstepPlanner(footstepPlannerParameters, footPolygons, bodyPathPlanHolder);
      this.postProcessHandler = new FootstepPlanPostProcessHandler(footstepPlannerParameters,
                                                                   swingPlannerParameters,
                                                                   walkingControllerParameters,
                                                                   footPolygons);
      registry.addChild(postProcessHandler.getYoVariableRegistry());
      aStarFootstepPlanner.setPostProcessorCallback(output -> postProcessHandler.handleRequest(output.getKey(), output.getValue()));
      this.variableDescriptors = collectVariableDescriptors(aStarFootstepPlanner.getRegistry());

      addStatusCallback(output -> output.getPlannerTimings().setTimePlanningStepsSeconds(stopwatch.lapElapsed()));
      addStatusCallback(output -> output.getPlannerTimings().setTotalElapsedSeconds(stopwatch.totalElapsed()));
   }

   public FootstepPlannerOutput handleRequest(FootstepPlannerRequest request)
   {
      if (isPlanning.get())
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
         return output;
      }
      catch (Exception exception)
      {
         exception.printStackTrace();

         output.clear();
         output.setRequestId(request.getRequestId());
         output.setFootstepPlanningResult(FootstepPlanningResult.EXCEPTION);
         output.setException(exception);
         statusCallback.accept(output);
         isPlanning.set(false);
         return output;
      }
   }

   private void handleRequestInternal(FootstepPlannerRequest request) throws Exception
   {
      this.request.set(request);
      requestCallback.accept(request);
      output.setRequestId(request.getRequestId());
      isPlanning.set(true);
      bodyPathPlanHolder.getPlan().clear();

      startMidFootPose.interpolate(request.getStartFootPoses().get(RobotSide.LEFT), request.getStartFootPoses().get(RobotSide.RIGHT), 0.5);
      goalMidFootPose.interpolate(request.getGoalFootPoses().get(RobotSide.LEFT), request.getGoalFootPoses().get(RobotSide.RIGHT), 0.5);

      // Update planar regions
      boolean flatGroundMode = request.getAssumeFlatGround() || request.getPlanarRegionsList() == null || request.getPlanarRegionsList().isEmpty();
      PlanarRegionsList planarRegionsList = flatGroundMode ? null : request.getPlanarRegionsList();
      bodyPathPlanner.setPlanarRegionsList(planarRegionsList);

      // record time
      output.getPlannerTimings().setTimeBeforePlanningSeconds(stopwatch.lap());

      if (request.getPlanBodyPath() && !flatGroundMode)
      {
         bodyPathPlanner.setStanceFootPoses(request.getStartFootPoses().get(RobotSide.LEFT), request.getStartFootPoses().get(RobotSide.RIGHT));
         bodyPathPlanner.setGoal(goalMidFootPose);

         BodyPathPlanningResult bodyPathPlannerResult = bodyPathPlanner.planWaypoints();
         List<Pose3DReadOnly> waypoints = bodyPathPlanner.getWaypoints();

         if (!bodyPathPlannerResult.validForExecution() || (waypoints.size() < 2 && request.getAbortIfBodyPathPlannerFails()))
         {
            reportBodyPathPlan(bodyPathPlannerResult);
            output.setBodyPathPlanningResult(bodyPathPlannerResult);
            statusCallback.accept(output);
            isPlanning.set(false);
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
      else
      {
         List<Pose3DReadOnly> waypoints = new ArrayList<>();
         waypoints.add(startMidFootPose);
         waypoints.addAll(request.getBodyPathWaypoints());
         waypoints.add(goalMidFootPose);

         bodyPathPlanHolder.setPoseWaypoints(waypoints);
         reportBodyPathPlan(BodyPathPlanningResult.FOUND_SOLUTION);
      }

      if (request.getPerformAStarSearch())
      {
         postProcessHandler.setStatusCallback(statusCallback);
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

         postProcessHandler.handleRequest(request, output);
         statusCallback.accept(output);
      }

      isPlanning.set(false);
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
      statusCallback.accept(output);
   }

   public void addRequestCallback(Consumer<FootstepPlannerRequest> callback)
   {
      requestCallback = requestCallback.andThen(callback);
   }

   public void addIterationCallback(Consumer<AStarIterationData<FootstepGraphNode>> callback)
   {
      aStarFootstepPlanner.addIterationCallback(callback);
   }

   public void addStatusCallback(Consumer<FootstepPlannerOutput> callback)
   {
      statusCallback = statusCallback.andThen(callback);
   }

   public void addCustomTerminationCondition(FootstepPlannerTerminationCondition plannerTerminationCondition)
   {
      aStarFootstepPlanner.addCustomTerminationCondition(plannerTerminationCondition);
   }

   public void clearCustomTerminationConditions()
   {
      aStarFootstepPlanner.clearCustomTerminationConditions();
   }

   public boolean registerRosNode(ROS2Node ros2Node)
   {
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
      return postProcessHandler.getSwingPlannerParameters();
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
      return postProcessHandler.getAdaptiveSwingTrajectoryCalculator();
   }

   public SwingOverPlanarRegionsTrajectoryExpander getSwingOverPlanarRegionsTrajectoryExpander()
   {
      return postProcessHandler.getSwingOverPlanarRegionsTrajectoryExpander();
   }

   public FootstepPlanPostProcessHandler getPostProcessHandler()
   {
      return postProcessHandler;
   }

   public YoRegistry getYoVariableRegistry()
   {
      return registry;
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
}
