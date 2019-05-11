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
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.*;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.footstepSnapping.*;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepGraph;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics.CostToGoHeuristics;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics.NodeComparator;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.heuristics.SpeedAndYawBasedHeuristics;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.listeners.QuadrupedFootstepPlannerListener;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.listeners.StartAndGoalListener;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.FootstepNodeChecker;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.FootstepNodeCheckerOfCheckers;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.PlanarRegionCliffAvoider;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeChecking.SnapBasedNodeChecker;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.graphSearch.stepCost.FootstepCostBuilder;
import us.ihmc.quadrupedFootstepPlanning.pathPlanning.WaypointsForQuadrupedFootstepPlanner;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

import java.util.*;
import java.util.List;

public class QuadrupedAStarFootstepPlanner implements QuadrupedBodyPathAndFootstepPlanner
{
   private static final boolean debug = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final RobotQuadrant defaultFirstQuadrant = RobotQuadrant.FRONT_LEFT;

   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;
   private final PlanarRegionConstraintDataParameters highLevelPlanarRegionConstraintDataParameters = new PlanarRegionConstraintDataParameters();

   private HashSet<FootstepNode> expandedNodes;
   private PriorityQueue<FootstepNode> stack;
   private FramePose3DReadOnly goalPose;
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
   private final FootstepNodeSnapper postProcessingSnapper;

   private final ArrayList<StartAndGoalListener> startAndGoalListeners = new ArrayList<>();

   private final YoDouble timeout = new YoDouble("footstepPlannerTimeout", registry);
   private final YoDouble planningTime = new YoDouble("PlanningTime", registry);
   private final YoLong numberOfExpandedNodes = new YoLong("NumberOfExpandedNodes", registry);
   private final YoDouble percentRejectedNodes = new YoDouble("PercentRejectedNodes", registry);
   private final YoLong iterationCount = new YoLong("IterationCount", registry);

   private final YoBoolean initialize = new YoBoolean("initialize", registry);

   private final YoBoolean validGoalNode = new YoBoolean("validGoalNode", registry);
   private final YoBoolean abortPlanning = new YoBoolean("abortPlanning", registry);

   private final QuadrantDependentList<YoBoolean> footReachedTheGoal = new QuadrantDependentList<>();
   private final YoBoolean centerReachedGoal = new YoBoolean("centerReachedGoal", registry);

   public QuadrupedAStarFootstepPlanner(FootstepPlannerParameters parameters, QuadrupedXGaitSettingsReadOnly xGaitSettings, FootstepNodeChecker nodeChecker,
                                        CostToGoHeuristics heuristics, FootstepNodeExpansion nodeExpansion, FootstepCost stepCostCalculator,
                                        FootstepNodeSnapper snapper, FootstepNodeSnapper postProcessingSnapper, QuadrupedFootstepPlannerListener listener,
                                        YoVariableRegistry parentRegistry)
   {
      this.xGaitSettings = xGaitSettings;
      this.nodeChecker = nodeChecker;
      this.heuristics = heuristics;
      this.nodeExpansion = nodeExpansion;
      this.stepCostCalculator = stepCostCalculator;
      this.listener = listener;
      this.snapper = snapper;
      this.postProcessingSnapper = postProcessingSnapper;
      this.graph = new FootstepGraph();
      timeout.set(Double.POSITIVE_INFINITY);
      this.initialize.set(true);
      highLevelPlanarRegionConstraintDataParameters.enforceTranslationLessThanGridCell = true;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footReachedTheGoal.put(robotQuadrant, new YoBoolean(robotQuadrant.getShortName() + "FootReachedTheGoal", registry));

      parentRegistry.addChild(registry);
   }

   public void addStartAndGoalListener(StartAndGoalListener startAndGoalListener)
   {
      startAndGoalListeners.add(startAndGoalListener);
   }

   public FootstepPlanningResult planPath()
   {
      return FootstepPlanningResult.OPTIMAL_SOLUTION;
   }

   public BodyPathPlan getPathPlan()
   {
      return null;
   }

   @Override
   public WaypointsForQuadrupedFootstepPlanner getWaypointPathPlanner()
   {
      return null;
   }

   @Override
   public QuadrupedFootstepPlanner getFootstepPlanner()
   {
      return this;
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

      startNode = getNodeFromTarget(start.getInitialQuadrant(), start);
      QuadrantDependentList<RigidBodyTransform> startNodeSnapTransforms = new QuadrantDependentList<>();

      if (start.getTargetType() == FootstepPlannerTargetType.FOOTSTEPS)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            int xIndex = startNode.getXIndex(robotQuadrant);
            int yIndex = startNode.getYIndex(robotQuadrant);
            RigidBodyTransform snapTransform = FootstepNodeTools.computeSnapTransform(xIndex, yIndex, start.getFootGoalPosition(robotQuadrant), new Quaternion());
            snapper.addSnapData(xIndex, yIndex, new FootstepNodeSnapData(snapTransform));
            startNodeSnapTransforms.put(robotQuadrant, snapTransform);
         }
      }
      else if (start.getTargetType() == FootstepPlannerTargetType.POSE_BETWEEN_FEET)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            Point3D startPoint = new Point3D(startNode.getX(robotQuadrant), startNode.getY(robotQuadrant), 0.0);
            Point3D projectedPoint = new Point3D(startNode.getX(robotQuadrant), startNode.getY(robotQuadrant), 0.0);

            PlanarRegion planarRegion = null;
            if (planarRegionsList != null)
            {
               planarRegion = PlanarRegionSnapTools.findHighestRegion(startNode.getX(robotQuadrant), startNode.getY(robotQuadrant),
                                                          planarRegionsList.getPlanarRegionsAsList(), highLevelPlanarRegionConstraintDataParameters);

               if (planarRegion == null)
               {
                  planarRegion = addPlanarRegionAtHeight(startPoint.getX(), startPoint.getY(), 0.0, start.getTargetPose().getYaw());
               }
               else
               {
                  projectedPoint.setZ(planarRegion.getPlaneZGivenXY(startPoint.getX(), startPoint.getY()));
               }
            }

            int xIndex = startNode.getXIndex(robotQuadrant);
            int yIndex = startNode.getYIndex(robotQuadrant);
            RigidBodyTransform snapTransform = FootstepNodeTools.computeSnapTransform(xIndex, yIndex, projectedPoint, new Quaternion());
            snapper.addSnapData(xIndex, yIndex, new FootstepNodeSnapData(snapTransform));
            startNodeSnapTransforms.put(robotQuadrant, snapTransform);
         }
      }
      nodeChecker.addStartNode(startNode, startNodeSnapTransforms);

      FramePose2DReadOnly startPose = new FramePose2D(worldFrame, startNode.getOrComputeXGaitCenterPoint(), startNode.getNominalYaw());
      startAndGoalListeners.parallelStream().forEach(listener -> listener.setInitialPose(startPose));
   }

   @Override
   public void setGoal(QuadrupedFootstepPlannerGoal goal)
   {
      checkGoalType(goal);

      goalPose = goal.getTargetPose();

      goalNode = getNodeFromTarget(goal);

      goalPoseInWorld.set(goalNode.getOrComputeXGaitCenterPoint(), goalNode.getNominalYaw());
      startAndGoalListeners.parallelStream().forEach(listener -> listener.setGoalPose(goalPoseInWorld));
   }

   private FootstepNode getNodeFromTarget(QuadrupedFootstepPlannerTarget target)
   {
      return getNodeFromTarget(defaultFirstQuadrant, target);
   }

   private FootstepNode getNodeFromTarget(RobotQuadrant quadrant, QuadrupedFootstepPlannerTarget target)
   {
      if (quadrant == null)
         quadrant = defaultFirstQuadrant;

      FootstepNode nodeToReturn = null;

      if (target.getTargetType().equals(FootstepPlannerTargetType.POSE_BETWEEN_FEET))
      {
         FramePose3DReadOnly goalPose = target.getTargetPose();
         ReferenceFrame goalFrame = new PoseReferenceFrame("GoalFrame", goalPose);
         goalFrame.update();

         FramePoint2D frontLeftStepPosition = new FramePoint2D(goalFrame, xGaitSettings.getStanceLength() / 2.0, xGaitSettings.getStanceWidth() / 2.0);
         FramePoint2D frontRightStepPosition = new FramePoint2D(goalFrame, xGaitSettings.getStanceLength() / 2.0, -xGaitSettings.getStanceWidth() / 2.0);
         FramePoint2D hindLeftStepPosition = new FramePoint2D(goalFrame, -xGaitSettings.getStanceLength() / 2.0, xGaitSettings.getStanceWidth() / 2.0);
         FramePoint2D hindRightStepPosition = new FramePoint2D(goalFrame, -xGaitSettings.getStanceLength() / 2.0, -xGaitSettings.getStanceWidth() / 2.0);

         frontLeftStepPosition.changeFrameAndProjectToXYPlane(worldFrame);
         frontRightStepPosition.changeFrameAndProjectToXYPlane(worldFrame);
         hindLeftStepPosition.changeFrameAndProjectToXYPlane(worldFrame);
         hindRightStepPosition.changeFrameAndProjectToXYPlane(worldFrame);

         nodeToReturn = new FootstepNode(quadrant.getNextReversedRegularGaitSwingQuadrant(), frontLeftStepPosition, frontRightStepPosition, hindLeftStepPosition, hindRightStepPosition,
                                         xGaitSettings.getStanceLength(), xGaitSettings.getStanceWidth());
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

         nodeToReturn = new FootstepNode(quadrant.getNextReversedRegularGaitSwingQuadrant(), frontLeftGoalPosition.getX(), frontLeftGoalPosition.getY(), frontRightGoalPosition.getX(),
                                         frontRightGoalPosition.getY(), hindLeftGoalPosition.getX(), hindLeftGoalPosition.getY(), hindRightGoalPosition.getX(),
                                         hindRightGoalPosition.getY(), xGaitSettings.getStanceLength(), xGaitSettings.getStanceWidth());
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
      highLevelPlanarRegionConstraintDataParameters.projectionInsideDelta = parameters.getProjectInsideDistanceForExpansion();
      highLevelPlanarRegionConstraintDataParameters.projectInsideUsingConvexHull = parameters.getProjectInsideUsingConvexHull();
      nodeChecker.setPlanarRegions(planarRegionsList);
      snapper.setPlanarRegions(planarRegionsList);
      postProcessingSnapper.setPlanarRegions(planarRegionsList);
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
         LogTools.info("A* Footstep planning statistics for " + result);
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
   public FootstepPlan getPlan()
   {
      if (endNode == null || !graph.doesNodeExist(endNode))
         return null;

      FootstepPlan plan = new FootstepPlan();
      plan.setLowLevelPlanGoal(goalPose);

      List<FootstepNode> path = graph.getPathFromStart(endNode);
      addGoalNodesToEnd(path);

      double lastStepStartTime = 0;

      for (int i = 0; i < path.size(); i++)
      {
         FootstepNode node = path.get(i);

         RobotQuadrant robotQuadrant = node.getMovingQuadrant();

         QuadrupedTimedOrientedStep newStep = new QuadrupedTimedOrientedStep();
         newStep.setRobotQuadrant(robotQuadrant);
         newStep.setGroundClearance(xGaitSettings.getStepGroundClearance());

         double endTimeShift;
         if (i == 0)
         {
            endTimeShift = 0.0;
         }
         else
         {
            endTimeShift = QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(robotQuadrant.getNextReversedRegularGaitSwingQuadrant(), xGaitSettings);
         }
         double thisStepStartTime = lastStepStartTime + endTimeShift;
         double thisStepEndTime = thisStepStartTime + xGaitSettings.getStepDuration();

         TimeIntervalBasics timeInterval = newStep.getTimeInterval();
         timeInterval.setInterval(thisStepStartTime, thisStepEndTime);

         Point3D position = new Point3D(node.getX(robotQuadrant), node.getY(robotQuadrant), 0.0);
         FootstepNodeSnapData snapData = postProcessingSnapper.snapFootstepNode(node.getXIndex(robotQuadrant), node.getYIndex(robotQuadrant));
         position.applyTransform(snapData.getSnapTransform());

         newStep.setGoalPosition(position);

         plan.addFootstep(newStep);

         lastStepStartTime = thisStepStartTime;
      }

      return plan;
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

      if (planarRegionsList != null && !planarRegionsList.isEmpty())
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

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footReachedTheGoal.get(robotQuadrant).set(false);
      centerReachedGoal.set(false);

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
         Point3DReadOnly startPos = PlanarRegionTools.projectPointToPlanesVertically(startPoint, snapper
               .getOrCreateSteppableRegions(startNode.getRoundedX(robotQuadrant), startNode.getRoundedY(robotQuadrant)));

         if (startPos == null)
         {
            if (debug)
               PrintTools.info("adding plane at start foot");
            addPlanarRegionAtHeight(startNode.getX(robotQuadrant), startNode.getY(robotQuadrant), 0.0, startNode.getNominalYaw());
         }
      }
   }

   private PlanarRegion addPlanarRegionAtHeight(double xLocation, double yLocation, double height, double yaw)
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(0.3, 0.3);
      polygon.addVertex(-0.3, 0.3);
      polygon.addVertex(0.3, -0.3);
      polygon.addVertex(-0.3, -0.25);
      polygon.update();

      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(new AxisAngle(yaw, 0.0, 0.0), new Vector3D(xLocation, yLocation, height)), polygon);
      planarRegionsList.addPlanarRegion(planarRegion);

      return planarRegion;
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
         {
            continue;
         }

         expandedNodes.add(nodeToExpand);

         if (checkAndHandleNodeAtFinalGoal(nodeToExpand))
            break;

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


   private boolean checkAndHandleNodeAtFinalGoal(FootstepNode nodeToExpand)
   {
      if (!validGoalNode.getBooleanValue())
         return false;

      if (nodeToExpand.xGaitGeometricallyEquals(goalNode))
      {
         endNode = nodeToExpand;
         return true;
      }

      return false;
   }

   private void addGoalNodesToEnd(List<FootstepNode> nodePathToPack)
   {
      FootstepNode endNode = this.endNode;
      RobotQuadrant movingQuadrant = endNode.getMovingQuadrant().getNextRegularGaitSwingQuadrant();

      while (!endNode.geometricallyEquals(goalNode))
      {
         if (!endNode.quadrantGeometricallyEquals(movingQuadrant, goalNode))
         {
            int xFrontLeft = endNode.getXIndex(RobotQuadrant.FRONT_LEFT);
            int yFrontLeft = endNode.getYIndex(RobotQuadrant.FRONT_LEFT);
            int xFrontRight = endNode.getXIndex(RobotQuadrant.FRONT_RIGHT);
            int yFrontRight = endNode.getYIndex(RobotQuadrant.FRONT_RIGHT);
            int xHindLeft = endNode.getXIndex(RobotQuadrant.HIND_LEFT);
            int yHindLeft = endNode.getYIndex(RobotQuadrant.HIND_LEFT);
            int xHindRight = endNode.getXIndex(RobotQuadrant.HIND_RIGHT);
            int yHindRight = endNode.getYIndex(RobotQuadrant.HIND_RIGHT);

            switch (movingQuadrant)
            {
            case FRONT_LEFT:
               xFrontLeft = goalNode.getXIndex(RobotQuadrant.FRONT_LEFT);
               yFrontLeft = goalNode.getYIndex(RobotQuadrant.FRONT_LEFT);
               break;
            case FRONT_RIGHT:
               xFrontRight = goalNode.getXIndex(RobotQuadrant.FRONT_RIGHT);
               yFrontRight = goalNode.getYIndex(RobotQuadrant.FRONT_RIGHT);
               break;
            case HIND_LEFT:
               xHindLeft = goalNode.getXIndex(RobotQuadrant.HIND_LEFT);
               yHindLeft = goalNode.getYIndex(RobotQuadrant.HIND_LEFT);
               break;
            case HIND_RIGHT:
               xHindRight = goalNode.getXIndex(RobotQuadrant.HIND_RIGHT);
               yHindRight = goalNode.getYIndex(RobotQuadrant.HIND_RIGHT);
               break;
            }
            FootstepNode nodeAtGoal = new FootstepNode(movingQuadrant, xFrontLeft, yFrontLeft, xFrontRight, yFrontRight, xHindLeft, yHindLeft, xHindRight,
                                                       yHindRight, endNode.getNominalStanceLength(), endNode.getNominalStanceWidth());

            nodePathToPack.add(nodeAtGoal);
            endNode = nodeAtGoal;
         }

         movingQuadrant = movingQuadrant.getNextRegularGaitSwingQuadrant();
      }
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
   }
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

   public static QuadrupedAStarFootstepPlanner createPlanner(FootstepPlannerParameters parameters, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                                             QuadrupedFootstepPlannerListener listener, FootstepNodeExpansion expansion,
                                                             YoVariableRegistry registry)
   {
      FootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(parameters, parameters::getProjectInsideDistanceForExpansion, true);
      FootstepNodeSnapper postProcessingSnapper = new FootstepNodePlanarRegionSnapAndWiggler(parameters, parameters::getProjectInsideDistanceForPostProcessing,
                                                                                             false);

      SnapBasedNodeChecker snapBasedNodeChecker = new SnapBasedNodeChecker(parameters, snapper);
      PlanarRegionCliffAvoider cliffAvoider = new PlanarRegionCliffAvoider(parameters, snapper);

      CostToGoHeuristics heuristics = new SpeedAndYawBasedHeuristics(snapper, parameters, xGaitSettings);

      FootstepNodeChecker nodeChecker = new FootstepNodeCheckerOfCheckers(Arrays.asList(snapBasedNodeChecker, cliffAvoider));
      nodeChecker.addPlannerListener(listener);

      FootstepCostBuilder costBuilder = new FootstepCostBuilder();
      costBuilder.setFootstepPlannerParameters(parameters);
      costBuilder.setXGaitSettings(xGaitSettings);
      costBuilder.setSnapper(snapper);
      costBuilder.setIncludeHeightCost(true);
      costBuilder.setIncludePitchAndRollCost(true);

      FootstepCost footstepCost = costBuilder.buildCost();

      QuadrupedAStarFootstepPlanner planner = new QuadrupedAStarFootstepPlanner(parameters, xGaitSettings, nodeChecker, heuristics, expansion, footstepCost,
                                                                                snapper, postProcessingSnapper, listener, registry);

      return planner;
   }
}
