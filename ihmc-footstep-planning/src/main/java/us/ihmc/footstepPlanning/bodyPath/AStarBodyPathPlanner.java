package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.footstepPlanning.BodyPathPlanningResult;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.log.AStarBodyPathEdgeData;
import us.ihmc.footstepPlanning.log.AStarBodyPathIterationData;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.graph.structure.NodeComparator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Consumer;

public class AStarBodyPathPlanner
{
   private static final double traversibilityCostScale = 0.25;
   private static final boolean computeSurfaceNormalCost = true;
   private static final boolean useEdgeDetectorCost = false;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FootstepPlannerParametersReadOnly parameters;
   private final AStarBodyPathEdgeData edgeData;
   private HeightMapData heightMapData;
   private final HashSet<BodyPathLatticePoint> expandedNodeSet = new HashSet<>();
   private final DirectedGraph<BodyPathLatticePoint> graph = new DirectedGraph<>();
   private final List<BodyPathLatticePoint> neighbors = new ArrayList<>();

   private final YoBoolean containsCollision = new YoBoolean("containsCollision", registry);
   private final YoDouble edgeCost = new YoDouble("edgeCost", registry);
   private final YoDouble deltaHeight = new YoDouble("deltaHeight", registry);
   private final YoDouble snapHeight = new YoDouble("snapHeight", registry);
   private final SideDependentList<YoDouble> inclineCost = new SideDependentList<>(side -> new YoDouble(side.getCamelCaseNameForStartOfExpression() + "InclineCost", registry));

   private final PriorityQueue<BodyPathLatticePoint> stack;
   private BodyPathLatticePoint startNode, goalNode;
   private final HashMap<BodyPathLatticePoint, Double> gridHeightMap = new HashMap<>();
   private BodyPathLatticePoint leastCostNode = null;
   private final YoEnum<RejectionReason> rejectionReason = new YoEnum<>("rejectionReason", registry, RejectionReason.class, true);

   /* Indicator of how flat and planar and available footholds are */
   private final BodyPathTraversibilityCalculator traversibilityCalculator;
   /* Uses edge detection as a heuristic for good areas to walk */
   private final HeightMapObstacleDetector obstacleDetector = new HeightMapObstacleDetector();
   /* Performs box collision check */
   private final BodyPathCollisionDetector collisionDetector = new BodyPathCollisionDetector();
   /* Computes surface normals and penalizes pitch and roll */
   private final HeightMapSurfaceNormalCalculator surfaceNormalCalculator = new HeightMapSurfaceNormalCalculator();

   private final TIntArrayList xSnapOffsets = new TIntArrayList();
   private final TIntArrayList ySnapOffsets = new TIntArrayList();

   private final List<AStarBodyPathIterationData> iterationData = new ArrayList<>();
   private final HashMap<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData> edgeDataMap = new HashMap<>();

   private final List<Consumer<FootstepPlannerOutput>> statusCallbacks = new ArrayList<>();
   private final Stopwatch stopwatch = new Stopwatch();
   private int iterations = 0;
   private BodyPathPlanningResult result = null;
   private boolean reachedGoal = false;
   private final AtomicBoolean haltRequested = new AtomicBoolean();
   private static final int maxIterations = 3000;

   /* Parameters to extract */
   static final double groundClearance = 0.2;
   static final double maxStepUpDown = 0.2;
   static final double snapRadius = 0.1;
   static final double boxSizeY = 0.6;
   static final double boxSizeX = 0.25;

   public AStarBodyPathPlanner(FootstepPlannerParametersReadOnly parameters, ConvexPolygon2D footPolygon)
   {
      this.parameters = parameters;
      stack = new PriorityQueue<>(new NodeComparator<>(graph, this::heuristics));
      traversibilityCalculator = new BodyPathTraversibilityCalculator(parameters, footPolygon, gridHeightMap, registry);

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
                                         deltaHeight.set(Double.NaN);
                                         rejectionReason.set(null);
                                      });
   }

   public void setHeightMapData(HeightMapData heightMapData)
   {
      if (this.heightMapData == null || !EuclidCoreTools.epsilonEquals(this.heightMapData.getGridResolutionXY(), heightMapData.getGridResolutionXY(), 1e-3))
      {
         collisionDetector.initialize(heightMapData.getGridResolutionXY(), boxSizeX, boxSizeY);
      }

      this.heightMapData = heightMapData;
      traversibilityCalculator.setHeightMap(heightMapData);
   }

   private static void packRadialOffsets(HeightMapData heightMapData, int minMaxOffsetXY, double radius, TIntArrayList xOffsets, TIntArrayList yOffsets)
   {
      xOffsets.clear();
      yOffsets.clear();

      for (int i = -minMaxOffsetXY; i <= minMaxOffsetXY; i++)
      {
         for (int j = -minMaxOffsetXY; j <= minMaxOffsetXY; j++)
         {
            double x = i * heightMapData.getGridResolutionXY();
            double y = j * heightMapData.getGridResolutionXY();
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
      TOO_HIGH_OR_LOW,
      COLLISION,
      NON_TRAVERSIBLE
   }

   public void handleRequest(FootstepPlannerRequest request, FootstepPlannerOutput outputToPack)
   {
      haltRequested.set(false);
      iterations = 0;
      reachedGoal = false;
      stopwatch.start();
      result = BodyPathPlanningResult.PLANNING;

      iterationData.clear();
      edgeDataMap.clear();
      gridHeightMap.clear();

      int minMaxOffsetXY = (int) Math.round(snapRadius / heightMapData.getGridResolutionXY());
      packRadialOffsets(heightMapData, minMaxOffsetXY, snapRadius, xSnapOffsets, ySnapOffsets);

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

      if (useEdgeDetectorCost)
      {
         obstacleDetector.compute(heightMapData);
      }
      if (computeSurfaceNormalCost)
      {
         double patchWidth = 0.25;
         surfaceNormalCalculator.computeSurfaceNormals(heightMapData, patchWidth);
      }

      planningLoop:
      while (true)
      {
         iterations++;
         outputToPack.getPlannerTimings().setStepPlanningIterations(iterations);

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
            LogTools.info("Stack is empty, no path exists...");
            break;
         }

         populateNeighbors(node);

         for (int i = 0; i < neighbors.size(); i++)
         {
            BodyPathLatticePoint neighbor = neighbors.get(i);

            this.snapHeight.set(snap(neighbor));
            if (Double.isNaN(snapHeight.getDoubleValue()))
            {
               rejectionReason.set(RejectionReason.INVALID_SNAP);
               graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
               continue;
            }

            deltaHeight.set(Math.abs(snapHeight.getDoubleValue() - gridHeightMap.get(node)));
            if (deltaHeight.getValue() > maxStepUpDown)
            {
               rejectionReason.set(RejectionReason.TOO_HIGH_OR_LOW);
               graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
               continue;
            }

            this.containsCollision.set(collisionDetector.collisionDetected(heightMapData, neighbor, i, snapHeight.getDoubleValue(), groundClearance));
            if (containsCollision.getValue())
            {
               rejectionReason.set(RejectionReason.COLLISION);
               graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
               continue;
            }

            double distanceCost = xyDistance(node, neighbor);
            double traversibilityCost = traversibilityCostScale * traversibilityCalculator.computeTraversibilityIndicator(neighbor, node);
            edgeCost.set(distanceCost + traversibilityCost);

            if (useEdgeDetectorCost)
            {
               int xIndex = HeightMapTools.coordinateToIndex(node.getX(), heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());
               int yIndex = HeightMapTools.coordinateToIndex(node.getY(), heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());
               double obstacleCost = obstacleDetector.getTerrainCost().get(xIndex, yIndex);
               edgeCost.add(obstacleCost);
            }
            if (computeSurfaceNormalCost)
            {
               double yaw = Math.atan2(neighbor.getY() - node.getY(), neighbor.getX() - node.getX());
               Pose2D bodyPose = new Pose2D();
               bodyPose.set(node.getX(), node.getY(), yaw);

               for (RobotSide side : RobotSide.values)
               {
                  Pose2D stepPose = new Pose2D();
                  stepPose.set(bodyPose);
                  stepPose.appendTranslation(0.0, side.negateIfRightSide(0.5 * parameters.getIdealFootstepWidth()));
                  UnitVector3DBasics surfaceNormal = surfaceNormalCalculator.getSurfaceNormal(HeightMapTools.coordinateToKey(stepPose.getX(),
                                                                                                                             stepPose.getY(),
                                                                                                                             heightMapData.getGridCenter().getX(),
                                                                                                                             heightMapData.getGridCenter().getY(),
                                                                                                                             heightMapData.getGridResolutionXY(),
                                                                                                                             heightMapData.getCenterIndex()));

                  if (surfaceNormal == null)
                     continue;

                  Vector2D edge = new Vector2D(neighbor.getX() - node.getX(), neighbor.getY() - node.getY());
                  edge.normalize();

                  /* Roll is the amount of incline orthogonal to the direction of motion */
                  double roll = Math.asin(Math.abs(edge.getY() * surfaceNormal.getX() - edge.getX() * surfaceNormal.getY()));
                  inclineCost.get(side).set(0.8 * Math.max(0.0, roll - Math.toRadians(5.0)));
                  edgeCost.add(inclineCost.get(side));
               }
            }

            if (!traversibilityCalculator.isTraversible())
            {
               rejectionReason.set(RejectionReason.NON_TRAVERSIBLE);
               graph.checkAndSetEdge(node, neighbor, edgeCost.getValue());
               continue;
            }

            graph.checkAndSetEdge(node, neighbor, edgeCost.getValue());
            stack.add(neighbor);

            if (node.equals(goalNode))
            {
               reachedGoal = true;
               result = BodyPathPlanningResult.FOUND_SOLUTION;
               break planningLoop;
            }
            else if (heuristics(node) < heuristics(leastCostNode))
            {
               leastCostNode = node;
            }
         }

         expandedNodeSet.add(node);

         AStarBodyPathIterationData iterationData = new AStarBodyPathIterationData();
         iterationData.setParentNode(node);
         iterationData.getChildNodes().addAll(neighbors);
         iterationData.setParentNodeHeight(gridHeightMap.get(node));
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
      LogTools.info("reporting status");

      outputToPack.setRequestId(request.getRequestId());
      outputToPack.setBodyPathPlanningResult(result);

      outputToPack.getBodyPath().clear();
      BodyPathLatticePoint terminalNode = reachedGoal ? goalNode : leastCostNode;
      List<BodyPathLatticePoint> path = graph.getPathFromStart(terminalNode);
      for (int i = 0; i < path.size(); i++)
      {
         Pose3D waypoint = new Pose3D();
         waypoint.getPosition().set(path.get(i).getX(), path.get(i).getY(), gridHeightMap.get(path.get(i)));
         outputToPack.getBodyPath().add(waypoint);
      }

      markSolutionEdges(terminalNode);
      statusCallbacks.forEach(callback -> callback.accept(outputToPack));
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
    * Populates an 8-connected grid starting along +x and moving clockwise
    */
   private void populateNeighbors(BodyPathLatticePoint latticePoint)
   {
      neighbors.clear();

      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + 1, latticePoint.getYIndex()));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + 1, latticePoint.getYIndex() + 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex(), latticePoint.getYIndex() + 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() - 1, latticePoint.getYIndex() + 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() - 1, latticePoint.getYIndex()));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() - 1, latticePoint.getYIndex() - 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex(), latticePoint.getYIndex() - 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + 1, latticePoint.getYIndex() - 1));
   }

   private static Pair<Integer, Integer> rotate(int xOff, int yOff, int i)
   {
      if (i == 0)
         return Pair.of(xOff, yOff);
      else if (i == 1)
         return Pair.of(-yOff, xOff);
      else if (i == 2)
         return Pair.of(-xOff, -yOff);
      else
         return Pair.of(yOff, -xOff);
   }

   private double snap(BodyPathLatticePoint latticePoint)
   {
      if (gridHeightMap.containsKey(latticePoint))
      {
         return gridHeightMap.get(latticePoint);
      }

      int centerIndex = heightMapData.getCenterIndex();
      int xIndex = HeightMapTools.coordinateToIndex(latticePoint.getX(), heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(latticePoint.getY(), heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), centerIndex);

      TDoubleArrayList heights = new TDoubleArrayList();
      for (int i = 0; i < xSnapOffsets.size(); i++)
      {
         int xQuery = xIndex + xSnapOffsets.get(i);
         int yQuery = yIndex + ySnapOffsets.get(i);
         double heightQuery = heightMapData.getHeightAt(xQuery, yQuery);
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
      gridHeightMap.put(latticePoint, maxHeight);
      return maxHeight;
   }

   static double xyDistance(BodyPathLatticePoint startNode, BodyPathLatticePoint endNode)
   {
      return EuclidCoreTools.norm(startNode.getX() - endNode.getX(), startNode.getY() - endNode.getY());
   }

   private double heuristics(BodyPathLatticePoint node)
   {
      return xyDistance(node, goalNode);
   }

   /**
    * For debugging
    */
   private static List<Pose3DReadOnly> straightLine(Pose3D startPose, Pose3D goalPose)
   {
      List<Pose3DReadOnly> poses = new ArrayList<>();

      poses.add(startPose);

      double angleEpsilon = 0.3;
      if (!EuclidCoreTools.angleGeometricallyEquals(startPose.getYaw(), goalPose.getYaw(), angleEpsilon))
      {
         poses.add(new Pose3D(startPose.getPosition(), goalPose.getOrientation()));
      }

      boolean addFinal = !EuclidCoreTools.angleGeometricallyEquals(Math.atan2(goalPose.getY() - startPose.getY(), goalPose.getX() - startPose.getX()),
                                                                   goalPose.getYaw(),
                                                                   angleEpsilon);
      int nInterpolate = 10;

      for (int i = 1; i < nInterpolate; i++)
      {
         if (i != nInterpolate - 1 || addFinal)
         {
            Pose3D pose = new Pose3D(goalPose);
            pose.getPosition().interpolate(startPose.getPosition(), goalPose.getPosition(), ((double) i) / (nInterpolate - 1));
            poses.add(pose);
         }
      }

      poses.add(goalPose);
      return poses;
   }

   public void setStatusCallbacks(List<Consumer<FootstepPlannerOutput>> statusCallbacks)
   {
      this.statusCallbacks.clear();
      this.statusCallbacks.addAll(statusCallbacks);
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
