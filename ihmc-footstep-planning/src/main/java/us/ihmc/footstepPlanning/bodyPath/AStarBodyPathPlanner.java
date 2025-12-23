package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.AStarBodyPathPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.BodyPathPlanningResult;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.log.AStarBodyPathEdgeData;
import us.ihmc.footstepPlanning.log.AStarBodyPathIterationData;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.graph.structure.NodeComparator;
import us.ihmc.perception.gpuMapping.HeightMapTools;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

public class AStarBodyPathPlanner
{
   private static final boolean debug = false;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final AStarBodyPathPlannerParametersReadOnly plannerParameters;
   private final AStarBodyPathEdgeData edgeData;
   private TerrainMapData terrainMapData;
   private final HashSet<BodyPathLatticePoint> expandedNodeSet = new HashSet<>();
   private final DirectedGraph<BodyPathLatticePoint> graph = new DirectedGraph<>();
   private final List<BodyPathLatticePoint> neighbors = new ArrayList<>();

   private final YoBoolean containsCollision = new YoBoolean("containsCollision", registry);
   private final YoDouble edgeCost = new YoDouble("edgeCost", registry);
   private final YoDouble deltaHeight = new YoDouble("deltaHeight", registry);
   private final YoDouble snapHeight = new YoDouble("snapHeight", registry);
   private final YoDouble incline = new YoDouble("incline", registry);
   private final YoDouble inclineCost = new YoDouble("inclineCost", registry);
   private final YoDouble roll = new YoDouble("roll", registry);
   private final YoDouble nominalIncline = new YoDouble("nominalIncline", registry);
   private final YoDouble heuristicCost = new YoDouble("heuristicCost", registry);
   private final YoDouble yoloTraversabilityCost = new YoDouble("traversabilityCost", registry);
   private final YoDouble totalCost = new YoDouble("totalCost", registry);

   private final PriorityQueue<BodyPathLatticePoint> stack;
   private BodyPathLatticePoint startNode, goalNode;
   private final HashMap<BodyPathLatticePoint, Double> gridHeightMap = new HashMap<>();
   private BodyPathLatticePoint leastCostNode = null;
   private double leastCost = Double.POSITIVE_INFINITY;
   private final YoEnum<RejectionReason> rejectionReason = new YoEnum<>("rejectionReason", registry, RejectionReason.class, true);

   /* Performs box collision check */
   private final BodyPathCollisionDetector collisionDetector = new BodyPathCollisionDetector();

   private final TIntArrayList xSnapOffsets = new TIntArrayList();
   private final TIntArrayList ySnapOffsets = new TIntArrayList();

   private final List<AStarBodyPathIterationData> iterationData = new ArrayList<>();
   private final HashMap<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData> edgeDataMap = new HashMap<>();

   private final List<Consumer<FootstepPlannerOutput>> statusCallbacks;
   private final Stopwatch stopwatch;
   private double planningStartTime;
   private int iterations = 0;
   private BodyPathPlanningResult result = null;
   private boolean reachedGoal = false;
   private final AtomicBoolean haltRequested = new AtomicBoolean();
   private static final int maxIterations = 3000;

   private final AStarBodyPathSmoother smoother;

   public AStarBodyPathPlanner(AStarBodyPathPlannerParametersReadOnly plannerParameters)
   {
      this(plannerParameters, new Stopwatch());
   }

   public AStarBodyPathPlanner(AStarBodyPathPlannerParametersReadOnly plannerParameters,
                               Stopwatch stopwatch)
   {
      this(plannerParameters, new ArrayList<>(), stopwatch);
   }

   public AStarBodyPathPlanner(AStarBodyPathPlannerParametersReadOnly plannerParameters,
                               List<Consumer<FootstepPlannerOutput>> statusCallbacks,
                               Stopwatch stopwatch)
   {
      this.plannerParameters = plannerParameters;
      this.statusCallbacks = statusCallbacks;
      this.stopwatch = stopwatch;
      stack = new PriorityQueue<>(new NodeComparator<>(graph, this::heuristics));

      List<YoVariable> allVariables = registry.collectSubtreeVariables();
      this.edgeData = new AStarBodyPathEdgeData(allVariables.size());
      graph.setGraphExpansionCallback(edge ->
                                      {
                                         for (int i = 0; i < allVariables.size(); i++)
                                         {
                                            edgeData.setData(i, allVariables.get(i).getValueAsLongBits());
                                         }

                                         edgeData.setParentNode(edge.getStartNode());
                                         edgeData.setChildNode(edge.getEndNode());
                                         edgeData.setChildSnapHeight(gridHeightMap.get(edge.getEndNode()));

                                         edgeDataMap.put(edge, edgeData.getCopyAndClear());

                                         containsCollision.set(false);
                                         deltaHeight.set(Double.NaN);
                                         edgeCost.set(Double.NaN);
                                         snapHeight.set(Double.NaN);
                                         rejectionReason.set(null);
                                         roll.set(0.0);
                                         incline.set(0.0);
                                         heuristicCost.setToNaN();
                                         yoloTraversabilityCost.setToNaN();
                                         totalCost.setToNaN();
                                      });

      smoother = new AStarBodyPathSmoother(plannerParameters);
   }

   public void setTerrainMapData(TerrainMapData terrainMapData)
   {
      if (this.terrainMapData == null || !EuclidCoreTools.epsilonEquals(this.terrainMapData.getCellSize(), terrainMapData.getCellSize(), 1e-3))
      {
         collisionDetector.initialize(terrainMapData.getCellSize(), plannerParameters.getCollisionBoxSizeX(), plannerParameters.getCollisionBoxSizeY());
      }

      this.terrainMapData = terrainMapData;
   }

   static void packRadialOffsets(TerrainMapData terrainMapData, double radius, TIntArrayList xOffsets, TIntArrayList yOffsets)
   {
      int minMaxOffsetXY = (int) Math.round(radius / terrainMapData.getCellSize());

      xOffsets.clear();
      yOffsets.clear();

      for (int i = -minMaxOffsetXY; i <= minMaxOffsetXY; i++)
      {
         for (int j = -minMaxOffsetXY; j <= minMaxOffsetXY; j++)
         {
            double x = i * terrainMapData.getCellSize();
            double y = j * terrainMapData.getCellSize();
            if (EuclidCoreTools.norm(x, y) < radius && !(i == 0 && j == 0))
            {
               xOffsets.add(i);
               yOffsets.add(j);
            }
         }
      }
   }

   private enum RejectionReason
   {
      INVALID_SNAP,
      TOO_STEEP,
      STEP_TOO_HIGH,
      COLLISION
   }

   public void handleRequest(FootstepPlannerRequest request, FootstepPlannerOutput outputToPack)
   {
      haltRequested.set(false);
      iterations = 0;
      reachedGoal = false;
      stopwatch.start();
      result = BodyPathPlanningResult.PLANNING;
      planningStartTime = stopwatch.totalElapsed();
      stopwatch.lap();

      iterationData.clear();
      edgeDataMap.clear();
      gridHeightMap.clear();
      setTerrainMapData(request.getEnvironmentHandler().getTerrainMapData());

      packRadialOffsets(terrainMapData, plannerParameters.getSnapRadius(), xSnapOffsets, ySnapOffsets);

      Pose3D startPose = new Pose3D();
      Pose3D goalPose = new Pose3D();

      startPose.interpolate(request.getStartFootPoses().get(RobotSide.LEFT), request.getStartFootPoses().get(RobotSide.RIGHT), 0.5);
      goalPose.interpolate(request.getGoalFootPoses().get(RobotSide.LEFT), request.getGoalFootPoses().get(RobotSide.RIGHT), 0.5);

      startNode = new BodyPathLatticePoint(startPose.getX(), startPose.getY());
      goalNode = new BodyPathLatticePoint(goalPose.getX(), goalPose.getY());
      stack.clear();
      stack.add(startNode);
      graph.initialize(startNode);
      expandedNodeSet.clear();
      gridHeightMap.put(startNode, startPose.getZ());
      leastCostNode = startNode;
      leastCost = heuristics(leastCostNode);
      nominalIncline.set(Math.atan2(goalPose.getZ() - startPose.getZ(), goalPose.getPosition().distanceXY(startPose.getPosition())));

      planningLoop:
      while (true)
      {
         iterations++;
         outputToPack.getPlannerTimings().setPathPlanningIterations(iterations);

         if (stopwatch.totalElapsed() >= request.getTimeout())
         {
            result = BodyPathPlanningResult.TIMED_OUT_BEFORE_SOLUTION;
            break;
         }
         if (haltRequested.get())
         {
            result = BodyPathPlanningResult.HALTED;
            break;
         }
         if (iterations > maxIterations)
         {
            result = BodyPathPlanningResult.MAXIMUM_ITERATIONS_REACHED;
            break;
         }

         BodyPathLatticePoint node = getNextNode();
         if (node == null)
         {
            result = BodyPathPlanningResult.NO_PATH_EXISTS;
            if (debug)
            {
               LogTools.info("Stack is empty, no path exists...");
            }
            break;
         }

         populateNeighbors(node);

         double parentSnapHeight = gridHeightMap.get(node);
         for (int neighborIndex = 0; neighborIndex < neighbors.size(); neighborIndex++)
         {
            BodyPathLatticePoint neighbor = neighbors.get(neighborIndex);
            if (node.getXIndex() == 8 && node.getYIndex() == 2 && neighbor.getXIndex() == 10 && neighbor.getY() == 3)
               LogTools.info("break");

            this.snapHeight.set(snap(neighbor));
            heuristicCost.set(xyDistance(neighbor, goalNode));

            if (Double.isNaN(snapHeight.getDoubleValue()))
            {
               rejectionReason.set(RejectionReason.INVALID_SNAP);
               graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
               continue;
            }

            double xyDistance = xyDistance(node, neighbor);
            deltaHeight.set(Math.abs(snapHeight.getDoubleValue() - parentSnapHeight));
            incline.set(Math.atan2(deltaHeight.getValue(), xyDistance));

            if (deltaHeight.getValue() > 0.1)
            {
               rejectionReason.set(RejectionReason.TOO_STEEP);
               graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
               continue;
            }

            if (Math.abs(incline.getValue()) > Math.toRadians(plannerParameters.getMaxIncline()))
            {
               rejectionReason.set(RejectionReason.TOO_STEEP);
               graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
               continue;
            }

            double distanceFromStart = EuclidCoreTools.norm(startPose.getX() - neighbor.getX(), startPose.getY() - neighbor.getY());
            if (plannerParameters.getCheckForCollisions() && distanceFromStart > plannerParameters.getCollisionStartTolerance())
            {
               this.containsCollision.set(collisionDetector.collisionDetected(terrainMapData,
                                                                              neighbor,
                                                                              neighborIndex,
                                                                              snapHeight.getDoubleValue(),
                                                                              plannerParameters.getCollisionBoxGroundClearance()));
               if (containsCollision.getValue())
               {
                  rejectionReason.set(RejectionReason.COLLISION);
                  graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
                  continue;
               }
            }

            edgeCost.set(xyDistance);

            if (incline.getValue() < nominalIncline.getValue())
            {
               inclineCost.set(0.0);
            }
            else
            {
               double inclineDelta = Math.abs(incline.getValue() - nominalIncline.getValue());
               inclineCost.set(plannerParameters.getInclineCostWeight() * Math.max(0.0, inclineDelta - plannerParameters.getInclineCostDeadband()));
               this.edgeCost.add(inclineCost);
            }

            if (edgeCost.getValue() < 0.0)
            {
               throw new RuntimeException("Negative edge cost!");
            }

            // YOLO obstacle clearance terrain cost term
            double obstacleClearance = terrainMapData.getObstacleClearanceScore(neighbor.getX(), neighbor.getY());
            if (!Double.isNaN(obstacleClearance))
            {
               double obstacleClearanceCost = plannerParameters.getObstacleClearanceWeight() * (1.0f - obstacleClearance);
               yoloTraversabilityCost.set(obstacleClearanceCost);
            }
            else
            {
               yoloTraversabilityCost.setToNaN();
            }

            if (!Double.isNaN(yoloTraversabilityCost.getValue()))
               edgeCost.add(yoloTraversabilityCost.getValue());

            totalCost.set(heuristicCost.getValue() + edgeCost.getValue());
            graph.checkAndSetEdge(node, neighbor, edgeCost.getValue());
            stack.add(neighbor);

            if (neighbor.equals(goalNode))
            {
               reachedGoal = true;
               result = BodyPathPlanningResult.FOUND_SOLUTION;
               break planningLoop;
            }
            else if (heuristicCost.getValue() < leastCost)
            {
               leastCost = heuristicCost.getDoubleValue();
               leastCostNode = neighbor;
            }
         }

         expandedNodeSet.add(node);

         AStarBodyPathIterationData iterationData = new AStarBodyPathIterationData();
         iterationData.setParentNode(node);
         iterationData.getChildNodes().addAll(neighbors);
         iterationData.setParentNodeHeight(parentSnapHeight);
         this.iterationData.add(iterationData);

         if (publishStatus(request))
         {
            reportStatus(request, outputToPack);
            stopwatch.lap();
         }
      }

      reportStatus(request, outputToPack);
   }

   private void reportStatus(FootstepPlannerRequest request, FootstepPlannerOutput outputToPack)
   {
      if (debug)
      {
         LogTools.info("Reporting status");
      }

      boolean performSmoothing = plannerParameters.getPerformSmoothing() && result == BodyPathPlanningResult.FOUND_SOLUTION;

      outputToPack.setBodyPathPlanningResult(result);
      outputToPack.getBodyPath().clear();
      outputToPack.getBodyPathUnsmoothed().clear();

      BodyPathLatticePoint terminalNode = reachedGoal ? goalNode : leastCostNode;
      List<BodyPathLatticePoint> path = graph.getPathFromStart(terminalNode);
      List<Point3D> bodyPath = new ArrayList<>();

      for (int i = 0; i < path.size(); i++)
      {
         Point3D waypoint = new Point3D(path.get(i).getX(), path.get(i).getY(), gridHeightMap.get(path.get(i)));
         bodyPath.add(waypoint);
         outputToPack.getBodyPathUnsmoothed().add(waypoint);

         if (!performSmoothing)
         {
            outputToPack.getBodyPath().add(new Pose3D(waypoint, new Quaternion()));
         }
      }

      if (performSmoothing)
      {
         List<Pose3D> smoothedPath = smoother.doSmoothing(bodyPath, terrainMapData);
         for (int i = 0; i < bodyPath.size(); i++)
         {
            Pose3D waypoint = new Pose3D(smoothedPath.get(i));

            if (i == 0)
            {
               double yaw = AngleTools.interpolateAngle(request.getStartFootPoses().get(RobotSide.LEFT).getYaw(), request.getStartFootPoses().get(RobotSide.RIGHT).getYaw(), 0.5);
               waypoint.getOrientation().setYawPitchRoll(yaw, 0.0, 0.0);
            }
            else if (i == bodyPath.size() - 1)
            {
               double yaw = AngleTools.interpolateAngle(request.getGoalFootPoses().get(RobotSide.LEFT).getYaw(), request.getGoalFootPoses().get(RobotSide.RIGHT).getYaw(), 0.5);
               waypoint.getOrientation().setYawPitchRoll(yaw, 0.0, 0.0);
            }

            outputToPack.getBodyPath().add(waypoint);
         }
      }

      outputToPack.getPlannerTimings().setTimePlanningBodyPathSeconds(stopwatch.totalElapsed() - planningStartTime);
      outputToPack.getPlannerTimings().setTotalElapsedSeconds(stopwatch.totalElapsed());

      if (reachedGoal)
      {
         outputToPack.setFootstepPlanningResult(FootstepPlanningResult.PLANNING);
      }

      markSolutionEdges(terminalNode);
      statusCallbacks.forEach(callback -> callback.accept(outputToPack));
   }

   public void clearLoggedData()
   {
      edgeDataMap.clear();
      iterationData.clear();
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

   private void markSolutionEdges(BodyPathLatticePoint terminalNode)
   {
      edgeDataMap.values().forEach(data -> data.setSolutionEdge(false));
      List<BodyPathLatticePoint> path = graph.getPathFromStart(terminalNode);
      for (int i = 1; i < path.size(); i++)
      {
         edgeDataMap.get(new GraphEdge<>(path.get(i - 1), path.get(i))).setSolutionEdge(true);
      }
   }

   public BodyPathLatticePoint getNextNode()
   {
      while (!stack.isEmpty())
      {
         BodyPathLatticePoint nextNode = stack.poll();
         if (!expandedNodeSet.contains(nextNode))
         {
            return nextNode;
         }
      }

      return null;
   }

   /**
    * Populates a 16-connected grid starting along +x and moving clockwise
    */
   private void populateNeighbors(BodyPathLatticePoint latticePoint)
   {
      neighbors.clear();

      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + 1, latticePoint.getYIndex()));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + 2, latticePoint.getYIndex() + 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + 1, latticePoint.getYIndex() + 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + 1, latticePoint.getYIndex() + 2));

      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex(), latticePoint.getYIndex() + 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() - 1, latticePoint.getYIndex() + 2));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() - 1, latticePoint.getYIndex() + 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() - 2, latticePoint.getYIndex() + 1));

      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() - 1, latticePoint.getYIndex()));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() - 2, latticePoint.getYIndex() - 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() - 1, latticePoint.getYIndex() - 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() - 1, latticePoint.getYIndex() - 2));

      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex(), latticePoint.getYIndex() - 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + 1, latticePoint.getYIndex() - 2));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + 1, latticePoint.getYIndex() - 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + 2, latticePoint.getYIndex() - 1));
   }

   private double snap(BodyPathLatticePoint latticePoint)
   {
      if (gridHeightMap.containsKey(latticePoint))
      {
         return gridHeightMap.get(latticePoint);
      }

      int centerIndex = terrainMapData.getCenterIndex();
      int xIndex = HeightMapTools.coordinateToIndex(latticePoint.getX(), terrainMapData.getGridCenterX(), terrainMapData.getCellSize(), centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(latticePoint.getY(), terrainMapData.getGridCenterY(), terrainMapData.getCellSize(), centerIndex);

      TDoubleArrayList heights = new TDoubleArrayList();
      for (int i = 0; i < xSnapOffsets.size(); i++)
      {
         int xQuery = xIndex + xSnapOffsets.get(i);
         int yQuery = yIndex + ySnapOffsets.get(i);
         double heightQuery = terrainMapData.getHeight(xQuery, yQuery);
         if (!Double.isNaN(heightQuery))
         {
            heights.add(heightQuery);
         }
      }

      if (heights.isEmpty())
      {
         gridHeightMap.put(latticePoint, Double.NaN);
         return Double.NaN;
      }

      double maxHeight = heights.max();
      double minHeight = maxHeight - plannerParameters.getMinSnapHeightThreshold();

      double runningSum = 0.0;
      int numberOfSamples = 0;

      for (int i = 0; i < xSnapOffsets.size(); i++)
      {
         int xQuery = xIndex + xSnapOffsets.get(i);
         int yQuery = yIndex + ySnapOffsets.get(i);
         double heightQuery = terrainMapData.getHeight(xQuery, yQuery);
         if (!Double.isNaN(heightQuery) && heightQuery > minHeight)
         {
            runningSum += heightQuery;
            numberOfSamples++;
         }
      }

      double averageHeight = runningSum / numberOfSamples;
      gridHeightMap.put(latticePoint, averageHeight);
      return averageHeight;
   }

   static double xyDistance(BodyPathLatticePoint startNode, BodyPathLatticePoint endNode)
   {
      return EuclidCoreTools.norm(startNode.getX() - endNode.getX(), startNode.getY() - endNode.getY());
   }

   private double heuristics(BodyPathLatticePoint node)
   {
      return xyDistance(node, goalNode);
   }

   public void halt()
   {
      haltRequested.set(true);
   }

   public List<AStarBodyPathIterationData> getIterationData()
   {
      return iterationData;
   }

   public HashMap<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData> getEdgeDataMap()
   {
      return edgeDataMap;
   }

   public YoRegistry getRegistry()
   {
      return registry;
   }
}
