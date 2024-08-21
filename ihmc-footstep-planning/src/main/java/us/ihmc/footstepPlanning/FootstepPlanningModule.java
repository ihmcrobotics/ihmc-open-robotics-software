package us.ihmc.footstepPlanning;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Collectors;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.staticReachability.StepReachabilityData;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.bodyPath.AStarBodyPathPlanner;
import us.ihmc.footstepPlanning.bodyPath.AStarBodyPathPlannerInterface;
import us.ihmc.footstepPlanning.bodyPath.BodyPathLatticePoint;
import us.ihmc.footstepPlanning.bodyPath.GPUAStarBodyPathPlanner;
import us.ihmc.footstepPlanning.graphSearch.AStarIterationData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.HeightMapFootstepChecker;
import us.ihmc.footstepPlanning.log.*;
import us.ihmc.footstepPlanning.simplePlanners.PlanThenSnapPlanner;
import us.ihmc.footstepPlanning.swing.AdaptiveSwingTrajectoryCalculator;
import us.ihmc.footstepPlanning.swing.DefaultSwingPlannerParameters;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
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
   private ROS2NodeInterface ros2Node;
   private boolean manageROS2Node = false;
   private final AStarBodyPathPlannerParametersBasics aStarBodyPathPlannerParameters;
   private final DefaultFootstepPlannerParametersBasics footstepPlannerParameters;

   private final WaypointDefinedBodyPathPlanHolder bodyPathPlanHolder = new WaypointDefinedBodyPathPlanHolder();
   private final AStarBodyPathPlannerInterface bodyPathPlannerInterface;

   // TODO plan then snap planner needs to work for height maps.
   private final PlanThenSnapPlanner planThenSnapPlanner;
   private final AStarFootstepPlanner aStarFootstepPlanner;
   private final List<VariableDescriptor> footstepPlanVariableDescriptors;

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

   private final boolean useGPU;

   public FootstepPlanningModule()
   {
      this(FootstepPlanningModule.class.getSimpleName());
   }

   public FootstepPlanningModule(String name)
   {
      this(name,
           new AStarBodyPathPlannerParameters(),
           new DefaultFootstepPlannerParameters(),
           new DefaultSwingPlannerParameters(),
           null,
           PlannerTools.createDefaultFootPolygons(),
           null);
   }

   public FootstepPlanningModule(String name, boolean useGPU)
   {
      this(name,
           new AStarBodyPathPlannerParameters(),
           new DefaultFootstepPlannerParameters(),
           new DefaultSwingPlannerParameters(),
           null,
           PlannerTools.createDefaultFootPolygons(),
           null,
           useGPU);
   }

   public FootstepPlanningModule(String name,
                                 AStarBodyPathPlannerParametersBasics aStarBodyPathPlannerParameters,
                                 DefaultFootstepPlannerParametersBasics footstepPlannerParameters,
                                 SwingPlannerParametersBasics swingPlannerParameters,
                                 WalkingControllerParameters walkingControllerParameters,
                                 SideDependentList<ConvexPolygon2D> footPolygons,
                                 StepReachabilityData stepReachabilityData)
   {
      this(name,
           aStarBodyPathPlannerParameters,
           footstepPlannerParameters,
           swingPlannerParameters,
           walkingControllerParameters,
           footPolygons,
           stepReachabilityData,
           true);
   }

   public FootstepPlanningModule(String name,
                                 AStarBodyPathPlannerParametersBasics aStarBodyPathPlannerParameters,
                                 DefaultFootstepPlannerParametersBasics footstepPlannerParameters,
                                 SwingPlannerParametersBasics swingPlannerParameters,
                                 WalkingControllerParameters walkingControllerParameters,
                                 SideDependentList<ConvexPolygon2D> footPolygons,
                                 StepReachabilityData stepReachabilityData,
                                 boolean useGPU)
   {
      this.name = name;
      this.aStarBodyPathPlannerParameters = aStarBodyPathPlannerParameters;
      this.footstepPlannerParameters = footstepPlannerParameters;

      this.useGPU = useGPU;
      if (useGPU)
      {
         this.bodyPathPlannerInterface = new GPUAStarBodyPathPlanner(footstepPlannerParameters, aStarBodyPathPlannerParameters, footPolygons, stopwatch);
      }
      else
      {
         this.bodyPathPlannerInterface = new AStarBodyPathPlanner(footstepPlannerParameters, aStarBodyPathPlannerParameters, footPolygons, stopwatch);
      }
      this.planThenSnapPlanner = new PlanThenSnapPlanner(footstepPlannerParameters, footPolygons);
      this.aStarFootstepPlanner = new AStarFootstepPlanner(footstepPlannerParameters,
                                                           footPolygons,
                                                           bodyPathPlanHolder,
                                                           swingPlannerParameters,
                                                           walkingControllerParameters,
                                                           stepReachabilityData,
                                                           stopwatch,
                                                           statusCallbacks);

      this.footstepPlanVariableDescriptors = collectVariableDescriptors(aStarFootstepPlanner.getRegistry());
   }

   public FootstepPlannerOutput handleRequest(FootstepPlannerRequest request)
   {
      return handleRequest(request, false);
   }

   public FootstepPlannerOutput handleRequest(FootstepPlannerRequest request, boolean quiet)
   {
      if (isPlanning.getAndSet(true))
      {
         LogTools.info("Received planning request packet but planner is currently running");
         return null;
      }
      else
      {
         if (!quiet)
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

   public void destroy()
   {
      if (useGPU)
         ((GPUAStarBodyPathPlanner) bodyPathPlannerInterface).destroyOpenCLStuff();
   }

   private void handleRequestInternal(FootstepPlannerRequest request) throws Exception
   {
      this.request.set(request);
      requestCallbacks.forEach(callback -> callback.accept(request));
      output.setRequestId(request.getRequestId());
      output.setRequestId(request.getRequestId());
      bodyPathPlanHolder.getPlan().clear();

      aStarFootstepPlanner.clearLoggedData();
      bodyPathPlannerInterface.clearLoggedData();

      boolean heightMapAvailable = request.getHeightMapData() != null && !request.getHeightMapData().isEmpty();

      startMidFootPose.interpolate(request.getStartFootPoses().get(RobotSide.LEFT), request.getStartFootPoses().get(RobotSide.RIGHT), 0.5);
      goalMidFootPose.interpolate(request.getGoalFootPoses().get(RobotSide.LEFT), request.getGoalFootPoses().get(RobotSide.RIGHT), 0.5);

      // Update planar regions
      boolean flatGroundMode = request.getAssumeFlatGround() || !heightMapAvailable;

      // record time
      output.getPlannerTimings().setTimeBeforePlanningSeconds(stopwatch.lap());
      output.getPlannerTimings().setTotalElapsedSeconds(stopwatch.totalElapsed());

      if (request.getPlanBodyPath() && !flatGroundMode)
      {
         bodyPathPlannerInterface.handleRequest(request, output);
         List<Pose3D> bodyPathWaypoints = output.getBodyPath();

         if (bodyPathWaypoints.size() < 2 && request.getAbortIfBodyPathPlannerFails())
         {
            reportBodyPathPlan(BodyPathPlanningResult.NO_PATH_EXISTS);
            output.setBodyPathPlanningResult(BodyPathPlanningResult.NO_PATH_EXISTS);
            statusCallbacks.forEach(callback -> callback.accept(output));
            return;
         }
         else if (bodyPathWaypoints.size() < 2 && !request.getAbortIfBodyPathPlannerFails())
         {
            bodyPathWaypoints.clear();
            bodyPathWaypoints.add(new Pose3D(startMidFootPose));
            bodyPathWaypoints.add(new Pose3D(goalMidFootPose));
         }

         // set the start orientation to be the heading
         double dx = bodyPathWaypoints.get(1).getX() - bodyPathWaypoints.get(0).getX();
         double dy = bodyPathWaypoints.get(1).getY() - bodyPathWaypoints.get(0).getY();
         ((Pose3DBasics) bodyPathWaypoints.get(0)).getOrientation().setToYawOrientation(Math.atan2(dy, dx));

         bodyPathPlanHolder.setPoseWaypoints(bodyPathWaypoints);
         double pathLength = bodyPathPlanHolder.computePathLength(0.0);
         if (MathTools.intervalContains(request.getHorizonLength(), 0.0, pathLength))
         {
            double alphaIntermediateGoal = request.getHorizonLength() / pathLength;
            bodyPathPlanHolder.getPointAlongPath(alphaIntermediateGoal, goalMidFootPose);
         }
      }
      else
      {
         List<Pose3DReadOnly> waypoints = new ArrayList<>();

         if (request.getBodyPathWaypoints().isEmpty())
         {
            waypoints.add(startMidFootPose);
            waypoints.add(goalMidFootPose);
            setNominalOrientations(waypoints);
         }
         else
         {
            waypoints.addAll(request.getBodyPathWaypoints());
         }

         bodyPathPlanHolder.setPoseWaypoints(waypoints);
         reportBodyPathPlan(BodyPathPlanningResult.FOUND_SOLUTION);
      }

      if (request.getPlanFootsteps() && request.getPerformAStarSearch())
      {
         aStarFootstepPlanner.handleRequest(request, output);
      }
      else if (request.getPlanFootsteps())
      {
         RobotSide initialStanceSide = request.getRequestedInitialStanceSide();
         FramePose3D initialStancePose = new FramePose3D(request.getStartFootPoses().get(initialStanceSide));
         planThenSnapPlanner.setInitialStanceFoot(initialStancePose, initialStanceSide);
         planThenSnapPlanner.setHeightMapData(request.getHeightMapData());

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
                             .computeSwingWaypoints(request.getHeightMapData(),
                                                    output.getFootstepPlan(),
                                                    request.getStartFootPoses(),
                                                    request.getSwingPlannerType());
         statusCallbacks.forEach(callback -> callback.accept(output));
      }
   }

   private static void setNominalOrientations(List<? extends Pose3DReadOnly> waypoints)
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
         aStarFootstepPlanner.getSwingPlanningModule().computeSwingWaypoints(request.getHeightMapData(),
                                                                             output.getFootstepPlan(),
                                                                             request.getStartFootPoses(),
                                                                             swingPlannerType);
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
      output.setSwingTrajectories(getSwingPlanningModule().getSwingTrajectories());
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
      bodyPathPlannerInterface.halt();
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

   public DefaultFootstepPlannerParametersBasics getFootstepPlannerParameters()
   {
      return footstepPlannerParameters;
   }

   public AStarBodyPathPlannerParametersBasics getAStarBodyPathPlannerParameters()
   {
      return aStarBodyPathPlannerParameters;
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

   public HeightMapFootstepChecker getChecker()
   {
      return aStarFootstepPlanner.getChecker();
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

   public HashMap<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData> getBodyPathEdgeDataMap()
   {
      return bodyPathPlannerInterface.getEdgeDataMap();
   }

   public List<FootstepPlannerIterationData> getIterationData()
   {
      return aStarFootstepPlanner.getIterationData();
   }

   public List<AStarBodyPathIterationData> getBodyPathIterationData()
   {
      return bodyPathPlannerInterface.getIterationData();
   }

   public YoRegistry getBodyPathPlannerRegistry()
   {
      return bodyPathPlannerInterface.getRegistry();
   }

   public YoRegistry getAStarPlannerRegistry()
   {
      return aStarFootstepPlanner.getRegistry();
   }

   public List<VariableDescriptor> getFootstepPlanVariableDescriptors()
   {
      return footstepPlanVariableDescriptors;
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

   public SwingPlanningModule getSwingPlanningModule()
   {
      return aStarFootstepPlanner.getSwingPlanningModule();
   }

   public AStarFootstepPlanner getAStarFootstepPlanner()
   {
      return aStarFootstepPlanner;
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
