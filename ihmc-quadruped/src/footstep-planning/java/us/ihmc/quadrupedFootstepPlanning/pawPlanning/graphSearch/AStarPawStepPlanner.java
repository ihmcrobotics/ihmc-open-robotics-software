package us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;

import org.apache.commons.math3.util.Precision;

import ihmc_common_msgs.msg.dds.GroundPlaneMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedFootstepPlanning.pathPlanning.WaypointsForPawStepPlanner;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.BodyPathAndPawPlanner;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlan;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlanner;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerGoal;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerStart;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerTarget;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerTargetType;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlanningResult;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNode;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawNodeTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.graph.PawStepGraph;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.heuristics.PawNodeComparator;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.heuristics.PawPlaningCostToGoHeuristicsBuilder;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.heuristics.PawPlanningCostToGoHeuristics;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.listeners.PawStepPlannerListener;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.listeners.StartAndGoalPawListener;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking.PawNodeChecker;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking.PawNodeCheckerOfCheckers;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking.PawNodeTransitionChecker;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking.PawNodeTransitionCheckerOfCheckers;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking.SnapBasedPawNodeChecker;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeChecking.SnapBasedPawNodeTransitionChecker;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeExpansion.ParameterBasedPawNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.nodeExpansion.PawNodeExpansion;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.parameters.PawStepPlannerParametersReadOnly;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.CliffAvoidancePlanarRegionFootstepNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapData;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PawNodeSnapper;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PlanarRegionPawConstraintDataHolder;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PlanarRegionPawConstraintDataParameters;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.pawSnapping.PlanarRegionPawSnapTools;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost.PawNodeCost;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.graphSearch.stepCost.PawNodeCostBuilder;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoLong;

public class AStarPawStepPlanner implements BodyPathAndPawPlanner
{
   private static final boolean debug = true;

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final RobotQuadrant defaultFirstQuadrant = RobotQuadrant.FRONT_LEFT;

   private RobotQuadrant startQuadrant;
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);

   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;
   private final PlanarRegionPawConstraintDataHolder highLevelConstraintDataHolder = new PlanarRegionPawConstraintDataHolder();
   private final PlanarRegionPawConstraintDataParameters highLevelPlanarRegionConstraintDataParameters = new PlanarRegionPawConstraintDataParameters();
   private final PawStepPlannerParametersReadOnly parameters;

   private HashSet<PawNode> expandedNodes;
   private PriorityQueue<PawNode> stack;
   private final FramePose3D goalPoseInWorld = new FramePose3D();
   private PawNode startNode;
   private QuadrantDependentList<PawNode> goalNodes;
   private PawNode endNode;

   private PlanarRegionsList planarRegionsList;

   private final PawStepGraph graph;
   private final PawNodeChecker nodeChecker;
   private final PawNodeTransitionChecker nodeTransitionChecker;
   private final PawStepPlannerListener listener;
   private final PawPlanningCostToGoHeuristics heuristics;
   private final PawNodeExpansion nodeExpansion;
   private final PawNodeCost stepCostCalculator;
   private final PawNodeSnapper snapper;

   private final ArrayList<StartAndGoalPawListener> startAndGoalListeners = new ArrayList<>();

   private final YoDouble timeout = new YoDouble("pawPlannerTimeout", registry);
   private final YoDouble bestEffortTimeout = new YoDouble("pawPlannerBestEffortTimeout", registry);
   private final YoDouble planningTime = new YoDouble("PlanningTime", registry);
   private final YoLong numberOfExpandedNodes = new YoLong("NumberOfExpandedNodes", registry);
   private final YoDouble percentRejectedNodes = new YoDouble("PercentRejectedNodes", registry);
   private final YoLong iterationCount = new YoLong("IterationCount", registry);

   private final YoBoolean initialize = new YoBoolean("initialize", registry);

   private final YoBoolean validGoalNode = new YoBoolean("validGoalNode", registry);
   private final YoBoolean abortPlanning = new YoBoolean("abortPlanning", registry);

   private final YoBoolean hasReachedFinalGoal = new YoBoolean("hasReachedFinalGoal", registry);

   private final YoDouble heuristicsInflationWeight = new YoDouble("heuristicsInflationWeight", registry);

   public AStarPawStepPlanner(PawStepPlannerParametersReadOnly parameters, QuadrupedXGaitSettingsReadOnly xGaitSettings, PawNodeChecker nodeChecker,
                              PawNodeTransitionChecker nodeTransitionChecker, PawPlanningCostToGoHeuristics heuristics, PawNodeExpansion nodeExpansion,
                              PawNodeCost stepCostCalculator, PawNodeSnapper snapper,
                              PawStepPlannerListener listener, YoRegistry parentRegistry)
   {
      this.parameters = parameters;
      this.xGaitSettings = xGaitSettings;
      this.nodeChecker = nodeChecker;
      this.nodeTransitionChecker = nodeTransitionChecker;
      this.heuristics = heuristics;
      this.nodeExpansion = nodeExpansion;
      this.stepCostCalculator = stepCostCalculator;
      this.listener = listener;
      this.snapper = snapper;
      this.graph = new PawStepGraph();
      timeout.set(Double.POSITIVE_INFINITY);
      this.initialize.set(true);
      highLevelPlanarRegionConstraintDataParameters.enforceTranslationLessThanGridCell = true;

      heuristics.setHeuristicsInflationWeight(heuristicsInflationWeight);

      parentRegistry.addChild(registry);
   }

   public void addStartAndGoalListener(StartAndGoalPawListener startAndGoalListener)
   {
      startAndGoalListeners.add(startAndGoalListener);
   }

   public PawStepPlanningResult planPath()
   {
      return PawStepPlanningResult.OPTIMAL_SOLUTION;
   }

   public BodyPathPlan getPathPlan()
   {
      return null;
   }

   @Override
   public WaypointsForPawStepPlanner getWaypointPathPlanner()
   {
      return null;
   }

   @Override
   public PawStepPlanner getPawStepPlanner()
   {
      return this;
   }

   @Override
   public void setTimeout(double timeoutInSeconds)
   {
      timeout.set(timeoutInSeconds);
   }

   @Override
   public void setBestEffortTimeout(double timeoutInSeconds)
   {
      bestEffortTimeout.set(timeoutInSeconds);
   }

   @Override
   public void setStart(PawStepPlannerStart start)
   {
      checkGoalType(start);
      requestInitialize();

      if (start.getInitialQuadrant() == null)
         startQuadrant = defaultFirstQuadrant;
      else
         startQuadrant = start.getInitialQuadrant();

      startNode = getNodeFromTarget(start.getInitialQuadrant(), start);
      QuadrantDependentList<RigidBodyTransform> startNodeSnapTransforms = new QuadrantDependentList<>();

      if (start.getTargetType() == PawStepPlannerTargetType.FOOTSTEPS)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            int xIndex = startNode.getXIndex(robotQuadrant);
            int yIndex = startNode.getYIndex(robotQuadrant);
            RigidBodyTransform snapTransform = PawNodeTools.computeSnapTransform(xIndex, yIndex, start.getPawGoalPosition(robotQuadrant), new Quaternion());
            snapper.addSnapData(xIndex, yIndex, new PawNodeSnapData(snapTransform));
            startNodeSnapTransforms.put(robotQuadrant, snapTransform);
         }
      }
      else if (start.getTargetType() == PawStepPlannerTargetType.POSE_BETWEEN_FEET)
      {
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            Point3D startPoint = new Point3D(startNode.getX(robotQuadrant), startNode.getY(robotQuadrant), 0.0);
            Point3D projectedPoint = new Point3D(startNode.getX(robotQuadrant), startNode.getY(robotQuadrant), 0.0);

            PlanarRegion planarRegion = null;
            if (planarRegionsList != null)
            {
               planarRegion = PlanarRegionPawSnapTools.findHighestRegion(startNode.getX(robotQuadrant), startNode.getY(robotQuadrant),
                                                                         planarRegionsList.getPlanarRegionsAsList(), highLevelConstraintDataHolder,
                                                                         highLevelPlanarRegionConstraintDataParameters);

               if (planarRegion == null)
               {
                  planarRegion = addPlanarRegionAtHeight(startPoint.getX(), startPoint.getY(), 0.0, start.getTargetPose().getOrientation());
               }
               else
               {
                  projectedPoint.setZ(planarRegion.getPlaneZGivenXY(startPoint.getX(), startPoint.getY()));
               }
            }

            int xIndex = startNode.getXIndex(robotQuadrant);
            int yIndex = startNode.getYIndex(robotQuadrant);
            RigidBodyTransform snapTransform = PawNodeTools.computeSnapTransform(xIndex, yIndex, projectedPoint, new Quaternion());
            snapper.addSnapData(xIndex, yIndex, new PawNodeSnapData(snapTransform));
            startNodeSnapTransforms.put(robotQuadrant, snapTransform);
         }
      }
      nodeTransitionChecker.addStartNode(startNode, startNodeSnapTransforms);

      FramePose2DReadOnly startPose = new FramePose2D(worldFrame, startNode.getOrComputeXGaitCenterPoint(), startNode.getStepYaw());
      startAndGoalListeners.parallelStream().forEach(listener -> listener.setInitialPose(startPose));
   }

   @Override
   public void setGoal(PawStepPlannerGoal goal)
   {
      checkGoalType(goal);

      goalNodes = new QuadrantDependentList<>();

      if (goal.getTargetType().equals(PawStepPlannerTargetType.POSE_BETWEEN_FEET))
      {
         FramePose3DReadOnly goalPose = goal.getTargetPose();
         ReferenceFrame goalFrame = new PoseReferenceFrame("GoalFrame", goalPose);
         goalFrame.update();

         FramePoint2D frontLeftGoalPosition = new FramePoint2D(goalFrame, xGaitSettings.getStanceLength() / 2.0, xGaitSettings.getStanceWidth() / 2.0);
         FramePoint2D frontRightGoalPosition = new FramePoint2D(goalFrame, xGaitSettings.getStanceLength() / 2.0, -xGaitSettings.getStanceWidth() / 2.0);
         FramePoint2D hindLeftGoalPosition = new FramePoint2D(goalFrame, -xGaitSettings.getStanceLength() / 2.0, xGaitSettings.getStanceWidth() / 2.0);
         FramePoint2D hindRightGoalPosition = new FramePoint2D(goalFrame, -xGaitSettings.getStanceLength() / 2.0, -xGaitSettings.getStanceWidth() / 2.0);

         frontLeftGoalPosition.changeFrameAndProjectToXYPlane(worldFrame);
         frontRightGoalPosition.changeFrameAndProjectToXYPlane(worldFrame);
         hindLeftGoalPosition.changeFrameAndProjectToXYPlane(worldFrame);
         hindRightGoalPosition.changeFrameAndProjectToXYPlane(worldFrame);

         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            goalNodes.put(robotQuadrant, new PawNode(robotQuadrant, frontLeftGoalPosition, frontRightGoalPosition, hindLeftGoalPosition, hindRightGoalPosition,
                                                     goalPose.getYaw(), xGaitSettings.getStanceLength(), xGaitSettings.getStanceWidth()));
         }

         goalPoseInWorld.set(goalPose);
      }
      else if (goal.getTargetType().equals(PawStepPlannerTargetType.FOOTSTEPS))
      {
         goalPoseInWorld.setToZero(worldFrame);

         FramePoint2D frontLeftGoalPosition = new FramePoint2D(goal.getPawGoalPosition(RobotQuadrant.FRONT_LEFT));
         FramePoint2D frontRightGoalPosition = new FramePoint2D(goal.getPawGoalPosition(RobotQuadrant.FRONT_RIGHT));
         FramePoint2D hindLeftGoalPosition = new FramePoint2D(goal.getPawGoalPosition(RobotQuadrant.HIND_LEFT));
         FramePoint2D hindRightGoalPosition = new FramePoint2D(goal.getPawGoalPosition(RobotQuadrant.HIND_RIGHT));


         frontLeftGoalPosition.checkReferenceFrameMatch(worldFrame);
         frontRightGoalPosition.checkReferenceFrameMatch(worldFrame);
         hindLeftGoalPosition.checkReferenceFrameMatch(worldFrame);
         hindRightGoalPosition.checkReferenceFrameMatch(worldFrame);

         double nominalYaw = PawNode.computeNominalYaw(frontLeftGoalPosition.getX(), frontLeftGoalPosition.getY(), frontRightGoalPosition.getX(),
                                                       frontRightGoalPosition.getY(), hindLeftGoalPosition.getX(), hindLeftGoalPosition.getY(),
                                                       hindRightGoalPosition.getX(), hindRightGoalPosition.getY());

         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            goalNodes.put(robotQuadrant, new PawNode(robotQuadrant, frontLeftGoalPosition, frontRightGoalPosition, hindLeftGoalPosition, hindRightGoalPosition,
                                                     nominalYaw, xGaitSettings.getStanceLength(), xGaitSettings.getStanceWidth()));
            goalPoseInWorld.getPosition().add(goal.getPawGoalPosition(robotQuadrant));
         }

         goalPoseInWorld.getPosition().scale(0.25);
         goalPoseInWorld.getOrientation().setToYawOrientation(nominalYaw);
      }

      heuristics.setGoalPose(goalPoseInWorld);
      startAndGoalListeners.parallelStream().forEach(listener -> listener.setGoalPose(goalPoseInWorld));
   }

   private PawNode getNodeFromTarget(RobotQuadrant quadrant, PawStepPlannerTarget target)
   {
      if (quadrant == null)
         quadrant = defaultFirstQuadrant;

      PawNode nodeToReturn = null;

      if (target.getTargetType().equals(PawStepPlannerTargetType.POSE_BETWEEN_FEET))
      {
         FramePose3DReadOnly goalPose = target.getTargetPose();
         ReferenceFrame goalFrame = new PoseReferenceFrame("GoalFrame", goalPose);
         goalFrame.update();

         FramePoint2D frontLeftGoalPosition = new FramePoint2D(goalFrame, xGaitSettings.getStanceLength() / 2.0, xGaitSettings.getStanceWidth() / 2.0);
         FramePoint2D frontRightGoalPosition = new FramePoint2D(goalFrame, xGaitSettings.getStanceLength() / 2.0, -xGaitSettings.getStanceWidth() / 2.0);
         FramePoint2D hindLeftGoalPosition = new FramePoint2D(goalFrame, -xGaitSettings.getStanceLength() / 2.0, xGaitSettings.getStanceWidth() / 2.0);
         FramePoint2D hindRightGoalPosition = new FramePoint2D(goalFrame, -xGaitSettings.getStanceLength() / 2.0, -xGaitSettings.getStanceWidth() / 2.0);

         frontLeftGoalPosition.changeFrameAndProjectToXYPlane(worldFrame);
         frontRightGoalPosition.changeFrameAndProjectToXYPlane(worldFrame);
         hindLeftGoalPosition.changeFrameAndProjectToXYPlane(worldFrame);
         hindRightGoalPosition.changeFrameAndProjectToXYPlane(worldFrame);

         double nominalYaw = PawNode.computeNominalYaw(frontLeftGoalPosition.getX(), frontLeftGoalPosition.getY(), frontRightGoalPosition.getX(),
                                                       frontRightGoalPosition.getY(), hindLeftGoalPosition.getX(), hindLeftGoalPosition.getY(),
                                                       hindRightGoalPosition.getX(), hindRightGoalPosition.getY());

         nodeToReturn = new PawNode(quadrant.getNextReversedRegularGaitSwingQuadrant(), frontLeftGoalPosition, frontRightGoalPosition,
                                    hindLeftGoalPosition, hindRightGoalPosition, nominalYaw, xGaitSettings.getStanceLength(),
                                    xGaitSettings.getStanceWidth());
      }
      else if (target.getTargetType().equals(PawStepPlannerTargetType.FOOTSTEPS))
      {
         FramePoint3D frontLeftGoalPosition = new FramePoint3D(target.getPawGoalPosition(RobotQuadrant.FRONT_LEFT));
         FramePoint3D frontRightGoalPosition = new FramePoint3D(target.getPawGoalPosition(RobotQuadrant.FRONT_RIGHT));
         FramePoint3D hindLeftGoalPosition = new FramePoint3D(target.getPawGoalPosition(RobotQuadrant.HIND_LEFT));
         FramePoint3D hindRightGoalPosition = new FramePoint3D(target.getPawGoalPosition(RobotQuadrant.HIND_RIGHT));

         frontLeftGoalPosition.changeFrame(worldFrame);
         frontRightGoalPosition.changeFrame(worldFrame);
         hindLeftGoalPosition.changeFrame(worldFrame);
         hindRightGoalPosition.changeFrame(worldFrame);

         double nominalYaw = PawNode.computeNominalYaw(frontLeftGoalPosition.getX(), frontLeftGoalPosition.getY(), frontRightGoalPosition.getX(),
                                                       frontRightGoalPosition.getY(), hindLeftGoalPosition.getX(), hindLeftGoalPosition.getY(),
                                                       hindRightGoalPosition.getX(), hindRightGoalPosition.getY());

         nodeToReturn = new PawNode(quadrant.getNextReversedRegularGaitSwingQuadrant(), frontLeftGoalPosition.getX(), frontLeftGoalPosition.getY(),
                                    frontRightGoalPosition.getX(), frontRightGoalPosition.getY(), hindLeftGoalPosition.getX(), hindLeftGoalPosition.getY(),
                                    hindRightGoalPosition.getX(), hindRightGoalPosition.getY(), nominalYaw, xGaitSettings.getStanceLength(),
                                    xGaitSettings.getStanceWidth());
      }

      return nodeToReturn;
   }

   @Override
   public void setGroundPlane(GroundPlaneMessage message)
   {
   }

   @Override
   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      highLevelPlanarRegionConstraintDataParameters.projectionInsideDelta = parameters.getProjectInsideDistance();
      highLevelPlanarRegionConstraintDataParameters.minimumProjectionInsideDelta = parameters.getMinimumProjectInsideDistance();
      highLevelPlanarRegionConstraintDataParameters.projectInsideUsingConvexHull = parameters.getProjectInsideUsingConvexHull();
      nodeTransitionChecker.setPlanarRegions(planarRegionsList);
      snapper.setPlanarRegions(planarRegionsList);
      this.planarRegionsList = planarRegionsList;
   }

   @Override
   public PawStepPlanningResult plan()
   {
      if (initialize.getBooleanValue())
      {
         boolean success = initialize();
         initialize.set(false);
         if (!success)
            return PawStepPlanningResult.PLANNER_FAILED;
      }

      if (debug)
         PrintTools.info("A* planner has initialized");

      if (!planInternal())
         return PawStepPlanningResult.PLANNER_FAILED;

      PawStepPlanningResult result = checkResult();

      if (result.validForExecution())
      {
         if (listener != null)
         listener.plannerFinished(null);

         List<PawNode> path = graph.getPathFromStart(endNode);

         // checking path
         if (debug)
         {
            PawNode previousNode = path.get(0);
            for (int i = 0; i < path.size(); i++)
            {
               PawNode node = path.get(i);
               RobotQuadrant robotQuadrant = node.getMovingQuadrant();

               PawNodeSnapData snapData = snapper.getSnapData(node.getXIndex(robotQuadrant), node.getYIndex(robotQuadrant));

               Point3D position = new Point3D(node.getX(robotQuadrant), node.getY(robotQuadrant), 0.0);
               if (snapData != null)
               {
                  RigidBodyTransform snapTransform = snapData.getSnapTransform();
                  position.applyTransform(snapTransform);

                  if (snapTransform.containsNaN())
                  {
                     System.out.println("Failed to snap in post processing.");
                     result = PawStepPlanningResult.PLANNER_FAILED;
                     break;
                  }
               }

               Point3D previousPosition = new Point3D(previousNode.getX(robotQuadrant), previousNode.getY(robotQuadrant), 0.0);
               PawNodeSnapData previousSnapData = snapper.getSnapData(previousNode.getXIndex(robotQuadrant), previousNode.getYIndex(robotQuadrant));
               if (previousSnapData != null)
               {
                  RigidBodyTransform previousSnapTransform = previousSnapData.getSnapTransform();
                  previousPosition.applyTransform(previousSnapTransform);
               }
               if (Math.abs(position.getZ() - previousPosition.getZ()) > parameters.getMaximumStepChangeZ())
               {
                  LogTools.error("height change error.");
               }

               previousNode = node;
            }

            if (path.get(1).getMovingQuadrant() != startQuadrant)
            {
               LogTools.error("The plan is starting with the wrong quadrant.");
               //            result = PawStepPlanningResult.PLANNER_FAILED;
            }
         }
      }


      if (debug)
      {
         LogTools.info("A* Paw planning statistics for " + result);
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
   public PawStepPlan getPlan()
   {
      if (endNode == null || !graph.doesNodeExist(endNode))
         return null;

      PawStepPlan plan = new PawStepPlan();
      plan.setLowLevelPlanGoal(goalPoseInWorld);

      List<PawNode> path = graph.getPathFromStart(endNode);

      double lastStepStartTime = 0;

      for (int i = 1; i < path.size(); i++)
      {
         PawNode node = path.get(i);

         RobotQuadrant robotQuadrant = node.getMovingQuadrant();

         QuadrupedTimedOrientedStep newStep = new QuadrupedTimedOrientedStep();
         newStep.setRobotQuadrant(robotQuadrant);
         newStep.setGroundClearance(xGaitSettings.getStepGroundClearance());

         double endTimeShift;
         if (i == 1)
         {
            endTimeShift = 0.0;
         }
         else
         {
            endTimeShift = QuadrupedXGaitTools.computeTimeDeltaBetweenSteps(robotQuadrant.getNextReversedRegularGaitSwingQuadrant(), xGaitSettings);
         }
         double thisStepStartTime = lastStepStartTime + endTimeShift;
         double thisStepEndTime = thisStepStartTime + xGaitSettings.getStepDuration();

         newStep.getTimeInterval().setInterval(thisStepStartTime, thisStepEndTime);

         Point3D position = new Point3D(node.getX(robotQuadrant), node.getY(robotQuadrant), 0.0);
         PawNodeSnapData snapData = snapper.getSnapData(node.getXIndex(robotQuadrant), node.getYIndex(robotQuadrant));
         if (snapData != null)
            position.applyTransform(snapData.getSnapTransform());

         if (debug)
         {
            PawNode previousNode = path.get(i - 1);
            Point3D previousPosition = new Point3D(previousNode.getX(robotQuadrant), previousNode.getY(robotQuadrant), 0.0);
            PawNodeSnapData previousSnapData = snapper.getSnapData(previousNode.getXIndex(robotQuadrant), previousNode.getYIndex(robotQuadrant));
            if (previousSnapData != null)
            {
               RigidBodyTransform previousSnapTransform = previousSnapData.getSnapTransform();
               previousPosition.applyTransform(previousSnapTransform);
            }
            if (Math.abs(position.getZ() - previousPosition.getZ()) > parameters.getMaximumStepChangeZ())
            {
               LogTools.error("height change error.");
            }
         }

         newStep.setGoalPosition(position);

         plan.addPawStep(newStep);

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
      if (goalNodes == null)
         throw new NullPointerException("Need to set goal before planning.");

      abortPlanning.set(false);

      if (planarRegionsList != null && !planarRegionsList.isEmpty())
         checkStartHasPlanarRegion();

      graph.initialize(startNode);
      PawNodeComparator nodeComparator = new PawNodeComparator(graph, heuristics);
      stack = new PriorityQueue<>(nodeComparator);

      validGoalNode.set(true);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (!nodeTransitionChecker.isNodeValid(goalNodes.get(robotQuadrant), null))
         {
            validGoalNode.set(false);
            if (debug)
               PrintTools.info("Goal node isn't valid. To plan without a valid goal node, best effort planning must be enabled");
            return false;
         }
      }


      stack.add(startNode);
      expandedNodes = new HashSet<>();
      endNode = null;

      heuristicsInflationWeight.set(parameters.getHeuristicsInflationWeight());
      hasReachedFinalGoal.set(false);

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
         Point3DReadOnly startPos = PlanarRegionTools.projectPointToPlanesVertically(startPoint, planarRegionsList.getPlanarRegionsAsList());

         if (startPos == null)
         {
            if (debug)
               PrintTools.info("adding plane at start paw");
            addPlanarRegionAtHeight(startNode.getX(robotQuadrant), startNode.getY(robotQuadrant), 0.0, startNode.getStepOrientation());
         }
      }
   }

   private PlanarRegion addPlanarRegionAtHeight(double xLocation, double yLocation, double height, Orientation3DReadOnly orientation)
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(0.3, 0.3);
      polygon.addVertex(-0.3, 0.3);
      polygon.addVertex(0.3, -0.3);
      polygon.addVertex(-0.3, -0.25);
      polygon.update();

      PlanarRegion planarRegion = new PlanarRegion(new RigidBodyTransform(orientation, new Vector3D(xLocation, yLocation, height)), polygon);
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

         PawNode nodeToExpand = stack.poll();
         if (expandedNodes.contains(nodeToExpand))
         {
            continue;
         }

         expandedNodes.add(nodeToExpand);

         hasReachedFinalGoal.set(checkAndHandleNodeAtFinalGoal(nodeToExpand));
         if (hasReachedFinalGoal.getBooleanValue())
         {
            if (parameters.returnBestEffortPlan())
            {
               if (!performRepairingStep())
                  break;
            }
            else
            {
               break;
            }
         }

         checkAndHandleBestEffortNode(nodeToExpand);

         HashSet<PawNode> neighbors = nodeExpansion.expandNode(nodeToExpand);
         expandedNodesCount += neighbors.size();
         for (PawNode neighbor : neighbors)
         {
            if (listener != null)
               listener.addNode(neighbor, nodeToExpand);

            // Checks if the paw is valid
            if (!nodeChecker.isNodeValid(neighbor))
            {
               rejectedNodesCount++;
               continue;
            }

            // Checks if the paw transition is valid
            if (!nodeTransitionChecker.isNodeValid(neighbor, nodeToExpand))
            {
               rejectedNodesCount++;
               continue;
            }

            double transitionCost = stepCostCalculator.compute(nodeToExpand, neighbor);
            graph.checkAndSetEdge(nodeToExpand, neighbor, transitionCost);

//            if (!parameters.returnBestEffortPlan() || endNode == null || stack.comparator().compare(neighbor, endNode) < 0)
               stack.add(neighbor);
         }

         if (listener != null)
            listener.tickAndUpdate();

         long timeInNano = System.nanoTime();
         double planningTime = Conversions.nanosecondsToSeconds(timeInNano - planningStartTime);
         boolean hardTimeout =  planningTime > timeout.getDoubleValue();
         boolean bestEffortTimedOut = parameters.returnBestEffortPlan() && planningTime > bestEffortTimeout.getDoubleValue() && endNode != null && graph.getPathFromStart(endNode).size() > parameters.getMinimumStepsForBestEffortPlan();
         if (hardTimeout || bestEffortTimedOut || abortPlanning.getBooleanValue())
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


   private boolean checkAndHandleNodeAtFinalGoal(PawNode nodeToExpand)
   {
      if (!validGoalNode.getBooleanValue())
         return false;

      RobotQuadrant nodeQuadrant = nodeToExpand.getMovingQuadrant();
      if (goalNodes.get(nodeQuadrant).equals(nodeToExpand))
      {
         PawNode parentNode = nodeToExpand;
         nodeQuadrant = nodeQuadrant.getNextRegularGaitSwingQuadrant();
         while (nodeQuadrant != nodeToExpand.getMovingQuadrant())
         {
            PawNode nodeAtGoal = PawNode.constructNodeFromOtherNode(nodeQuadrant, goalNodes.get(nodeQuadrant).getXIndex(nodeQuadrant),
                                                         goalNodes.get(nodeQuadrant).getYIndex(nodeQuadrant), goalNodes.get(nodeQuadrant).getYawIndex(),
                                                         parentNode);

            if (!nodeChecker.isNodeValid(nodeAtGoal) || !nodeTransitionChecker.isNodeValid(nodeAtGoal, parentNode))
            {
               break;
            }

            endNode = nodeAtGoal;
            graph.checkAndSetEdge(parentNode, endNode, 0.0);

            parentNode = endNode;
            nodeQuadrant = nodeQuadrant.getNextRegularGaitSwingQuadrant();
         }

         return endNode != null;
      }

      return false;
   }

   private boolean performRepairingStep()
   {
      if (!parameters.performGraphRepairingStep())
         return false;

      if (heuristicsInflationWeight.getDoubleValue() <= 1.0)
         return false;


      double currentInflationWeight = heuristicsInflationWeight.getDoubleValue();
      double inflationReduction =(1.0 - parameters.getRepairingHeuristicWeightScaling()) * heuristicsInflationWeight.getDoubleValue();
      inflationReduction = Math.max(inflationReduction, parameters.getMinimumHeuristicWeightReduction());
      double newInflationWeight = Math.max(currentInflationWeight - inflationReduction, 1.0);

      if (debug)
      {
         System.out.println("Reducing the inflation weight from " + currentInflationWeight + " to " + newInflationWeight);
      }
      heuristicsInflationWeight.set(newInflationWeight);

      return true;
   }


   private void checkAndHandleBestEffortNode(PawNode nodeToExpand)
   {
      if (!parameters.returnBestEffortPlan())
         return;

      if (graph.getPathFromStart(nodeToExpand).size() - 1 < parameters.getMinimumStepsForBestEffortPlan())
         return;

      if (endNode == null || heuristics.compute(nodeToExpand) < heuristics.compute(endNode))
      {
         if (listener != null)
            listener.reportLowestCostNodeList(graph.getPathFromStart(nodeToExpand));
         endNode = nodeToExpand;
      }
   }

   private PawStepPlanningResult checkResult()
   {
      if (stack.isEmpty() && endNode == null)
         return PawStepPlanningResult.NO_PATH_EXISTS;
      if (!graph.doesNodeExist(endNode))
         return PawStepPlanningResult.TIMED_OUT_BEFORE_SOLUTION;

      if (heuristicsInflationWeight.getDoubleValue() <= 1.0)
         return PawStepPlanningResult.OPTIMAL_SOLUTION;

      return PawStepPlanningResult.SUB_OPTIMAL_SOLUTION;
   }

   public static void checkGoalType(PawStepPlannerTarget goal)
   {
      PawStepPlannerTargetType supportedGoalType1 = PawStepPlannerTargetType.POSE_BETWEEN_FEET;
      PawStepPlannerTargetType supportedGoalType2 = PawStepPlannerTargetType.FOOTSTEPS;
      if (!goal.getTargetType().equals(supportedGoalType1) && !goal.getTargetType().equals(supportedGoalType2))
         throw new IllegalArgumentException("Planner does not support goals other than " + supportedGoalType1 + " and " + supportedGoalType2);
   }

   public static AStarPawStepPlanner createPlanner(PawStepPlannerParametersReadOnly parameters, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                                   PawStepPlannerListener listener, YoRegistry registry)
   {
      PawNodeSnapper snapper = new CliffAvoidancePlanarRegionFootstepNodeSnapper(parameters, true);
      PawNodeExpansion expansion = new ParameterBasedPawNodeExpansion(parameters, xGaitSettings);

      SnapBasedPawNodeTransitionChecker snapBasedNodeTransitionChecker = new SnapBasedPawNodeTransitionChecker(parameters, snapper);
      SnapBasedPawNodeChecker snapBasedNodeChecker = new SnapBasedPawNodeChecker(parameters, snapper);
//      PawPlanarRegionCliffAvoider cliffAvoider = new PawPlanarRegionCliffAvoider(parameters, snapper);

      PawPlaningCostToGoHeuristicsBuilder heuristicsBuilder = new PawPlaningCostToGoHeuristicsBuilder();
      heuristicsBuilder.setPawPlannerParameters(parameters);
      heuristicsBuilder.setXGaitSettings(xGaitSettings);
      heuristicsBuilder.setSnapper(snapper);
      heuristicsBuilder.setUseDistanceBasedHeuristics(true);

      PawPlanningCostToGoHeuristics heuristics = heuristicsBuilder.buildHeuristics();

      PawNodeTransitionChecker nodeTransitionChecker = new PawNodeTransitionCheckerOfCheckers(Arrays.asList(snapBasedNodeTransitionChecker));
      PawNodeChecker nodeChecker = new PawNodeCheckerOfCheckers(Arrays.asList(snapBasedNodeChecker));//, cliffAvoider));
      nodeTransitionChecker.addPlannerListener(listener);
      nodeChecker.addPlannerListener(listener);

      PawNodeCostBuilder costBuilder = new PawNodeCostBuilder();
      costBuilder.setPawPlannerParameters(parameters);
      costBuilder.setXGaitSettings(xGaitSettings);
      costBuilder.setSnapper(snapper);
      costBuilder.setIncludeHeightCost(true);

      PawNodeCost pawNodeCost = costBuilder.buildCost();

      AStarPawStepPlanner planner = new AStarPawStepPlanner(parameters, xGaitSettings, nodeChecker, nodeTransitionChecker, heuristics,
                                                            expansion, pawNodeCost, snapper, listener, registry);

      return planner;
   }
}
