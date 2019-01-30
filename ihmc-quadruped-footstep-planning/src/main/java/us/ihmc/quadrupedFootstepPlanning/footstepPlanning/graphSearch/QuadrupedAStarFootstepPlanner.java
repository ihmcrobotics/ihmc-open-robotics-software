package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch;

import controller_msgs.msg.dds.QuadrupedGroundPlaneMessage;
import org.apache.commons.math3.util.Precision;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.*;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnappingTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics.DistanceAndYawBasedHeuristics;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics.NodeComparator;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.listeners.QuadrupedFootstepPlannerListener;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.listeners.StartAndGoalListener;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.FootstepNodeCheckerOfCheckers;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost.FootstepCostBuilder;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettings;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.*;
import java.util.List;

public class QuadrupedAStarFootstepPlanner implements QuadrupedFootstepPlanner
{
   private static final boolean debug = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final RobotQuadrant defaultFirstQuadrant = RobotQuadrant.FRONT_LEFT;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final FootstepPlannerParameters parameters;
   private final QuadrupedXGaitSettings xGaitSettings;

   private HashSet<FootstepNode> expandedNodes;
   private PriorityQueue<FootstepNode> stack;
   private FootstepNode startNode;
   private FootstepNode goalNode;
   private FootstepNode endNode;

   private PlanarRegionsList planarRegionsList;

   private final FramePose2D goalPoseInWorld = new FramePose2D();

   private final FootstepGraph graph;
   private final FootstepNodeChecker nodeChecker;
   private final QuadrupedFootstepPlannerListener listener;
   private final CostToGoHeuristics heuristics;
   private final FootstepNodeExpansion nodeExpansion;
   private final FootstepCost stepCostCalculator;
   private final FootstepNodeSnapper snapper;

   private final ArrayList<StartAndGoalListener> startAndGoalListeners = new ArrayList<>();

   private final YoDouble timeout = new YoDouble("footstepPlannerTimeout", registry);
   private final YoDouble planningTime = new YoDouble("PlanningTime", registry);
   private final YoLong numberOfExpandedNodes = new YoLong("NumberOfExpandedNodes", registry);
   private final YoDouble percentRejectedNodes = new YoDouble("PercentRejectedNodes", registry);
   private final YoLong iterationCount = new YoLong("IterationCount", registry);

   private final YoBoolean initialize = new YoBoolean("initialize", registry);

   private final YoBoolean validGoalNode = new YoBoolean("validGoalNode", registry);
   private final YoBoolean abortPlanning = new YoBoolean("abortPlanning", registry);

   public QuadrupedAStarFootstepPlanner(FootstepPlannerParameters parameters, QuadrupedXGaitSettings xGaitSettings, FootstepNodeChecker nodeChecker,
                                        CostToGoHeuristics heuristics, FootstepNodeExpansion nodeExpansion, FootstepCost stepCostCalculator,
                                        FootstepNodeSnapper snapper, QuadrupedFootstepPlannerListener listener, YoVariableRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.xGaitSettings = xGaitSettings;
      this.nodeChecker = nodeChecker;
      this.heuristics = heuristics;
      this.nodeExpansion = nodeExpansion;
      this.stepCostCalculator = stepCostCalculator;
      this.listener = listener;
      this.snapper = snapper;
      this.graph = new FootstepGraph();
      timeout.set(Double.POSITIVE_INFINITY);
      this.initialize.set(true);

      parentRegistry.addChild(registry);
   }

   public void addStartAndGoalListener(StartAndGoalListener startAndGoalListener)
   {
      startAndGoalListeners.add(startAndGoalListener);
   }

   @Override
   public void setTimeout(double timeoutInSeconds)
   {
      timeout.set(timeoutInSeconds);
   }

   @Override
   public void setStart(QuadrupedFootstepPlannerStart start)
   {
      checkGoalType(start);

      startNode = getNodeFromTarget(start);
      QuadrantDependentList<RigidBodyTransform> startNodeSnapTransforms = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         startNodeSnapTransforms.put(robotQuadrant, FootstepNodeSnappingTools
               .computeSnapTransform(robotQuadrant, startNode, new Point3D(startNode.getX(robotQuadrant), startNode.getY(robotQuadrant), 0.0)));
      }
      snapper.addSnapData(startNode, new FootstepNodeSnapData(startNodeSnapTransforms));
      nodeChecker.addStartNode(startNode, startNodeSnapTransforms);

      FramePose2DReadOnly startPose = new FramePose2D(worldFrame, startNode.getOrComputeMidStancePoint(), startNode.getNominalYaw());
      startAndGoalListeners.parallelStream().forEach(listener -> listener.setInitialPose(startPose));
   }

   @Override
   public void setGoal(QuadrupedFootstepPlannerGoal goal)
   {
      checkGoalType(goal);

      goalNode = getNodeFromTarget(goal);

      goalPoseInWorld.set(goalNode.getOrComputeMidStancePoint(), goalNode.getNominalYaw());
      startAndGoalListeners.parallelStream().forEach(listener -> listener.setGoalPose(goalPoseInWorld));
   }

   private FootstepNode getNodeFromTarget(QuadrupedFootstepPlannerTarget target)
   {
      FootstepNode nodeToReturn = null;

      if (target.getTargetType().equals(FootstepPlannerTargetType.POSE_BETWEEN_FEET))
      {
         FramePose3DReadOnly goalPose = target.getTargetPose();
         ReferenceFrame goalFrame = new PoseReferenceFrame("GoalFrame", goalPose);

         FramePoint2D frontLeftStepPosition = new FramePoint2D(goalFrame, xGaitSettings.getStanceLength() / 2.0, xGaitSettings.getStanceWidth() / 2.0);
         FramePoint2D frontRightStepPosition = new FramePoint2D(goalFrame, xGaitSettings.getStanceLength() / 2.0, -xGaitSettings.getStanceWidth() / 2.0);
         FramePoint2D hindLeftStepPosition = new FramePoint2D(goalFrame, -xGaitSettings.getStanceLength() / 2.0, xGaitSettings.getStanceWidth() / 2.0);
         FramePoint2D hindRightStepPosition = new FramePoint2D(goalFrame, -xGaitSettings.getStanceLength() / 2.0, -xGaitSettings.getStanceWidth() / 2.0);

         frontLeftStepPosition.changeFrame(worldFrame);
         frontRightStepPosition.changeFrame(worldFrame);
         hindLeftStepPosition.changeFrame(worldFrame);
         hindRightStepPosition.changeFrame(worldFrame);

         nodeToReturn = new FootstepNode(defaultFirstQuadrant, frontLeftStepPosition, frontRightStepPosition, hindLeftStepPosition, hindRightStepPosition);
      }
      else if (target.getTargetType().equals(FootstepPlannerTargetType.FOOTSTEPS))
      {
         FramePoint3D frontLeftGoalPosition = new FramePoint3D(target.getFootGoalPosition(RobotQuadrant.FRONT_LEFT));
         FramePoint3D frontRightGoalPosition = new FramePoint3D(target.getFootGoalPosition(RobotQuadrant.FRONT_RIGHT));
         FramePoint3D hindLeftGoalPosition = new FramePoint3D(target.getFootGoalPosition(RobotQuadrant.HIND_LEFT));
         FramePoint3D hindRightGoalPosition = new FramePoint3D(target.getFootGoalPosition(RobotQuadrant.HIND_RIGHT));

         frontLeftGoalPosition.changeFrame(worldFrame);
         frontRightGoalPosition.changeFrame(worldFrame);
         hindLeftGoalPosition.changeFrame(worldFrame);
         hindRightGoalPosition.changeFrame(worldFrame);

         nodeToReturn = new FootstepNode(defaultFirstQuadrant, frontLeftGoalPosition.getX(), frontLeftGoalPosition.getY(), frontRightGoalPosition.getX(),
                                         frontRightGoalPosition.getY(), hindLeftGoalPosition.getX(), hindLeftGoalPosition.getY(), hindRightGoalPosition.getX(),
                                         hindRightGoalPosition.getY());
      }

      return nodeToReturn;
   }

   @Override
   public void setGroundPlane(QuadrupedGroundPlaneMessage message)
   {
   }


   @Override
   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      nodeChecker.setPlanarRegions(planarRegionsList);
      snapper.setPlanarRegions(planarRegionsList);
      this.planarRegionsList = planarRegionsList;
   }

   @Override
   public FootstepPlanningResult plan()
   {
      if (initialize.getBooleanValue())
      {
         boolean success = initialize();
         initialize.set(false);
         if (!success)
            return FootstepPlanningResult.PLANNER_FAILED;
      }

      if (debug)
         PrintTools.info("A* planner has initialized");

      if (!planInternal())
         return FootstepPlanningResult.PLANNER_FAILED;

      FootstepPlanningResult result = checkResult();

      if (result.validForExecution() && listener != null)
         listener.plannerFinished(null);

      if (debug)
      {
         PrintTools.info("A* Footstep planning statistics for " + result);
         System.out.println("   Finished planning after " + Precision.round(planningTime.getDoubleValue(), 2) + " seconds.");
         System.out.println("   Expanded each node to an average of " + numberOfExpandedNodes.getLongValue() + " children nodes.");
         System.out.println("   Planning took a total of " + iterationCount.getLongValue() + " iterations.");
         System.out.println("   During the planning " + percentRejectedNodes.getDoubleValue() + "% of nodes were rejected as invalid.");
         System.out.println("   Goal was : " + goalPoseInWorld);
      }

      initialize.set(true);
      return result;
   }

   @Override
   public List<? extends QuadrupedTimedStep> getSteps()
   {
      if (endNode == null || !graph.doesNodeExist(endNode))
         return null;

      List<QuadrupedTimedOrientedStep> steps = new ArrayList<>();
      List<FootstepNode> path = graph.getPathFromStart(endNode);

      double currentTime = 0;

      for (int i = 0; i < path.size(); i++)
      {
         currentTime += xGaitSettings.getEndDoubleSupportDuration();

         FootstepNode node = path.get(i);

         RobotQuadrant robotQuadrant = node.getMovingQuadrant();

         QuadrupedTimedOrientedStep newStep = new QuadrupedTimedOrientedStep();
         newStep.setRobotQuadrant(robotQuadrant);

         TimeInterval timeInterval = newStep.getTimeInterval();
         timeInterval.setStartTime(currentTime);
         currentTime += xGaitSettings.getStepDuration();
         timeInterval.setEndTime(currentTime);

         Point3D position = new Point3D(node.getX(robotQuadrant), node.getY(robotQuadrant), 0.0);
         FootstepNodeSnapData snapData = snapper.getSnapData(node);
//         position.applyTransform(snapData.getSnapTransform(robotQuadrant));

         newStep.setGoalPosition(position);

         steps.add(newStep);
      }

      return steps;
   }

   @Override
   public double getPlanningDuration()
   {
      return planningTime.getDoubleValue();
   }


   private boolean initialize()
   {
      if (startNode == null)
         throw new NullPointerException("Need to set initial conditions before planning.");
      if (goalNode == null)
         throw new NullPointerException("Need to set goal before planning.");

      abortPlanning.set(false);

      if (planarRegionsList != null)
         checkStartHasPlanarRegion();

      graph.initialize(startNode);
      NodeComparator nodeComparator = new NodeComparator(graph, goalNode, heuristics);
      stack = new PriorityQueue<>(nodeComparator);

      validGoalNode.set(nodeChecker.isNodeValid(goalNode, null));
      if (!validGoalNode.getBooleanValue())// && !parameters.getReturnBestEffortPlan())
      {
         if (debug)
            PrintTools.info("Goal node isn't valid. To plan without a valid goal node, best effort planning must be enabled");
         return false;
      }

      stack.add(startNode);
      expandedNodes = new HashSet<>();
      endNode = null;

      if (listener != null)
      {
         listener.addNode(startNode, null);
         listener.tickAndUpdate();
      }

      return true;
   }

   private void checkStartHasPlanarRegion()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         Point3D startPoint = new Point3D(startNode.getX(robotQuadrant), startNode.getY(robotQuadrant), 0.0);
         Point3DReadOnly startPos = PlanarRegionTools
               .projectPointToPlanesVertically(startPoint, snapper.getOrCreateSteppableRegions(startNode.getRoundedX(robotQuadrant), startNode.getRoundedY(robotQuadrant)));

         if (startPos == null)
         {
            if (debug)
               PrintTools.info("adding plane at start foot");
               addPlanarRegionAtZeroHeight(startNode.getX(robotQuadrant), startNode.getY(robotQuadrant));
         }
      }
   }

   private void addPlanarRegionAtZeroHeight(double xLocation, double yLocation)
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(0.3, 0.3);
      polygon.addVertex(-0.3, 0.3);
      polygon.addVertex(0.3, -0.3);
      polygon.addVertex(-0.3, -0.25);
      polygon.update();

      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(new AxisAngle(), new Vector3D(xLocation, yLocation, 0.0)), polygon);
      planarRegionsList.addPlanarRegion(planarRegion);
   }

   @Override
   public void cancelPlanning()
   {
      if (debug)
         PrintTools.info("Cancel has been requested.");
      abortPlanning.set(true);
   }

   public void requestInitialize()
   {
      initialize.set(true);
   }

   private boolean planInternal()
   {
      long planningStartTime = System.nanoTime();

      long rejectedNodesCount = 0;
      long expandedNodesCount = 0;
      long iterations = 0;

      while (!stack.isEmpty())
      {
         if (initialize.getBooleanValue())
         {
            boolean success = initialize();
            rejectedNodesCount = 0;
            expandedNodesCount = 0;
            iterations = 0;
            initialize.set(false);
            if (!success)
               return false;
         }

         iterations++;

         FootstepNode nodeToExpand = stack.poll();
         if (expandedNodes.contains(nodeToExpand))
            continue;
         expandedNodes.add(nodeToExpand);

         if (checkAndHandleNodeAtGoal(nodeToExpand))
            break;

         //         checkAndHandleBestEffortNode(nodeToExpand);

         HashSet<FootstepNode> neighbors = nodeExpansion.expandNode(nodeToExpand);
         expandedNodesCount += neighbors.size();
         for (FootstepNode neighbor : neighbors)
         {
            if (listener != null)
               listener.addNode(neighbor, nodeToExpand);

            // Checks if the footstep (center of the foot) is on a planar region
            if (!nodeChecker.isNodeValid(neighbor, nodeToExpand))
            {
               rejectedNodesCount++;
               continue;
            }

            double cost = stepCostCalculator.compute(nodeToExpand, neighbor);
            graph.checkAndSetEdge(nodeToExpand, neighbor, cost);

            if (/*!parameters.getReturnBestEffortPlan() || */endNode == null || stack.comparator().compare(neighbor, endNode) < 0)
               stack.add(neighbor);
         }

         if (listener != null)
            listener.tickAndUpdate();

         long timeInNano = System.nanoTime();
         if (Conversions.nanosecondsToSeconds(timeInNano - planningStartTime) > timeout.getDoubleValue() || abortPlanning.getBooleanValue())
         {
            if (abortPlanning.getBooleanValue())
               PrintTools.info("Abort planning requested.");
            abortPlanning.set(false);
            break;
         }
      }

      long timeInNano = System.nanoTime();
      planningTime.set(Conversions.nanosecondsToSeconds(timeInNano - planningStartTime));
      percentRejectedNodes.set(100.0 * rejectedNodesCount / expandedNodesCount);
      iterationCount.set(iterations);
      numberOfExpandedNodes.set(expandedNodesCount / Math.max(iterations, 1));

      return true;
   }

   private boolean checkAndHandleNodeAtGoal(FootstepNode nodeToExpand)
   {
      if (!validGoalNode.getBooleanValue())
         return false;

      if (goalNode.geometricallyEquals(nodeToExpand))
      {
         endNode = goalNode;
         graph.checkAndSetEdge(nodeToExpand, endNode, 0.0);
         return true;
      }

      return false;
   }

   /*
   private void checkAndHandleBestEffortNode(FootstepNode nodeToExpand)
   {
      if (!parameters.getReturnBestEffortPlan())
         return;

      if (graph.getPathFromStart(nodeToExpand).size() - 1 < parameters.getMinimumStepsForBestEffortPlan())
         return;

      if (endNode == null || heuristics.compute(nodeToExpand, goalNode.get(nodeToExpand.getRobotSide())) < heuristics
            .compute(endNode, goalNode.get(endNode.getRobotSide())))
      {
         if (listener != null)
            listener.reportLowestCostNodeList(graph.getPathFromStart(nodeToExpand));
         endNode = nodeToExpand;
      }
   }\
   */

   private FootstepPlanningResult checkResult()
   {
      if (stack.isEmpty() && endNode == null)
         return FootstepPlanningResult.NO_PATH_EXISTS;
      if (!graph.doesNodeExist(endNode))
         return FootstepPlanningResult.TIMED_OUT_BEFORE_SOLUTION;

      if (heuristics.getWeight() <= 1.0)
         return FootstepPlanningResult.OPTIMAL_SOLUTION;

      return FootstepPlanningResult.SUB_OPTIMAL_SOLUTION;
   }

   public static void checkGoalType(QuadrupedFootstepPlannerTarget goal)
   {
      FootstepPlannerTargetType supportedGoalType1 = FootstepPlannerTargetType.POSE_BETWEEN_FEET;
      FootstepPlannerTargetType supportedGoalType2 = FootstepPlannerTargetType.FOOTSTEPS;
      if (!goal.getTargetType().equals(supportedGoalType1) && !goal.getTargetType().equals(supportedGoalType2))
         throw new IllegalArgumentException("Planner does not support goals other than " + supportedGoalType1 + " and " + supportedGoalType2);
   }

   public static QuadrupedAStarFootstepPlanner createPlanner(FootstepPlannerParameters parameters, QuadrupedXGaitSettings xGaitSettings,
                                                             QuadrupedFootstepPlannerListener listener, FootstepNodeExpansion expansion,
                                                             YoVariableRegistry registry)
   {
      SimplePlanarRegionFootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(parameters);
      //      FootstepNodeSnapAndWiggler postProcessingSnapper = new FootstepNodeSnapAndWiggler(footPolygons, parameters);
      SimplePlanarRegionFootstepNodeSnapper postProcessingSnapper = new SimplePlanarRegionFootstepNodeSnapper(parameters);

      SnapBasedNodeChecker snapBasedNodeChecker = new SnapBasedNodeChecker(parameters, snapper);
      //      PlanarRegionBaseOfCliffAvoider cliffAvoider = new PlanarRegionBaseOfCliffAvoider(parameters, snapper, footPolygons);

      DistanceAndYawBasedHeuristics heuristics = new DistanceAndYawBasedHeuristics(parameters);

      FootstepNodeChecker nodeChecker = new FootstepNodeCheckerOfCheckers(Arrays.asList(snapBasedNodeChecker));//, cliffAvoider));
      nodeChecker.addPlannerListener(listener);

      FootstepCostBuilder costBuilder = new FootstepCostBuilder();
      costBuilder.setFootstepPlannerParameters(parameters);
      costBuilder.setSnapper(snapper);
      costBuilder.setIncludeHeightCost(true);
      costBuilder.setIncludeHeightCost(true);
      costBuilder.setIncludePitchAndRollCost(true);

      FootstepCost footstepCost = costBuilder.buildCost();

      QuadrupedAStarFootstepPlanner planner = new QuadrupedAStarFootstepPlanner(parameters, xGaitSettings, nodeChecker, heuristics, expansion, footstepCost,
                                                                                postProcessingSnapper, listener, registry);

      return planner;
   }
}
