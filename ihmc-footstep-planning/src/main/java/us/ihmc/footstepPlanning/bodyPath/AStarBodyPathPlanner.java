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
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
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
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
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
   private static final boolean checkForCollisions = true;
   private static final boolean computeSurfaceNormalCost = true;
   private static final boolean useRANSACTraversibility = true;
   private static final double rollCostWeight = 5.25;

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
   private final YoDouble incline = new YoDouble("incline", registry);
   private final YoDouble roll = new YoDouble("roll", registry);
   private final YoFrameVector3D leastSqNormal = new YoFrameVector3D("leastSqNormal", ReferenceFrame.getWorldFrame(), registry);

   private final PriorityQueue<BodyPathLatticePoint> stack;
   private BodyPathLatticePoint startNode, goalNode;
   private final HashMap<BodyPathLatticePoint, Double> gridHeightMap = new HashMap<>();
   private BodyPathLatticePoint leastCostNode = null;
   private final YoEnum<RejectionReason> rejectionReason = new YoEnum<>("rejectionReason", registry, RejectionReason.class, true);

   /* Indicator of how flat and planar and available footholds are, using least squares */
   private final BodyPathLSTraversibilityCalculator leastSqTraversibilityCalculator;
   /* Indicator of how flat and planar and available footholds are */
   private final BodyPathRANSACTraversibilityCalculator ransacTraversibilityCalculator;
   /* Performs box collision check */
   private final BodyPathCollisionDetector collisionDetector = new BodyPathCollisionDetector();
   /* Computes surface normals with least-squares and penalizing roll when changing elevation */
   private final HeightMapLeastSquaresNormalCalculator surfaceNormalCalculator = new HeightMapLeastSquaresNormalCalculator();
   /* Computes surface normals with RANSAC, used for traversibility */
   private final HeightMapRANSACNormalCalculator ransacNormalCalculator = new HeightMapRANSACNormalCalculator();

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

   private final AStarBodyPathSmoother smoother = new AStarBodyPathSmoother();

   /* Parameters to extract */
   static final double groundClearance = 0.3;
   static final double maxStepUpDown = 0.2;
   static final double maxIncline = Math.toRadians(55.0);
   static final double snapRadius = 0.1;
   static final double boxSizeY = 0.85;
   static final double boxSizeX = 0.3;

   public AStarBodyPathPlanner(FootstepPlannerParametersReadOnly parameters, ConvexPolygon2D footPolygon)
   {
      this.parameters = parameters;
      stack = new PriorityQueue<>(new NodeComparator<>(graph, this::heuristics));

      if (useRANSACTraversibility)
      {
         ransacTraversibilityCalculator = new BodyPathRANSACTraversibilityCalculator(gridHeightMap::get, ransacNormalCalculator, registry);
         leastSqTraversibilityCalculator = null;
      }
      else
      {
         leastSqTraversibilityCalculator = new BodyPathLSTraversibilityCalculator(parameters, footPolygon, gridHeightMap, registry);
         ransacTraversibilityCalculator = null;
      }

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
                                         leastSqNormal.setToZero();
                                         roll.set(0.0);
                                         incline.set(0.0);
                                         ransacTraversibilityCalculator.clearVariables();
                                      });

      smoother.setRansacNormalCalculator(ransacNormalCalculator);
      smoother.setLeastSquaresNormalCalculator(surfaceNormalCalculator);
   }

   public void setHeightMapData(HeightMapData heightMapData)
   {
      if (this.heightMapData == null || !EuclidCoreTools.epsilonEquals(this.heightMapData.getGridResolutionXY(), heightMapData.getGridResolutionXY(), 1e-3))
      {
         collisionDetector.initialize(heightMapData.getGridResolutionXY(), boxSizeX, boxSizeY);
      }

      this.heightMapData = heightMapData;
      ransacNormalCalculator.initialize(heightMapData);

      if (useRANSACTraversibility)
      {
         ransacTraversibilityCalculator.setHeightMap(heightMapData);
      }
      else
      {
         leastSqTraversibilityCalculator.setHeightMap(heightMapData);
      }
   }

   static void packRadialOffsets(HeightMapData heightMapData, double radius, TIntArrayList xOffsets, TIntArrayList yOffsets)
   {
      int minMaxOffsetXY = (int) Math.round(radius / heightMapData.getGridResolutionXY());

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
      TOO_STEEP,
      STEP_TOO_HIGH,
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

      packRadialOffsets(heightMapData, snapRadius, xSnapOffsets, ySnapOffsets);

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

      if (computeSurfaceNormalCost)
      {
         double patchWidth = 0.25;
         surfaceNormalCalculator.computeSurfaceNormals(heightMapData, patchWidth);
      }

      if (useRANSACTraversibility)
      {
         ransacNormalCalculator.initialize(heightMapData);
         ransacTraversibilityCalculator.initialize(startNode);
      }
      else
      {
         leastSqTraversibilityCalculator.initialize(startNode);
      }

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
            LogTools.info("Stack is empty, no path exists...");
            break;
         }

         populateNeighbors(node);

         double parentSnapHeight = gridHeightMap.get(node);
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

            double xyDistance = xyDistance(node, neighbor);
            deltaHeight.set(Math.abs(snapHeight.getDoubleValue() - parentSnapHeight));
            if (deltaHeight.getValue() > maxStepUpDown)
            {
               rejectionReason.set(RejectionReason.TOO_STEEP);
               graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
               continue;
            }

            incline.set(Math.atan2(deltaHeight.getValue(), xyDistance));

            if (Math.abs(incline.getValue()) > maxIncline)
            {
               rejectionReason.set(RejectionReason.TOO_STEEP);
               graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
               continue;
            }

            if (checkForCollisions)
            {
               this.containsCollision.set(collisionDetector.collisionDetected(heightMapData, neighbor, i, snapHeight.getDoubleValue(), groundClearance));
               if (containsCollision.getValue())
               {
                  rejectionReason.set(RejectionReason.COLLISION);
                  graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
                  continue;
               }
            }

            edgeCost.set(xyDistance);

            if (useRANSACTraversibility)
            {
               double traversibilityIndicator = ransacTraversibilityCalculator.computeTraversibility(neighbor, node, i);
               if (ransacTraversibilityCalculator.isTraversible())
               {
                  edgeCost.add(traversibilityIndicator);
               }
               else
               {
                  rejectionReason.set(RejectionReason.NON_TRAVERSIBLE);
                  graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
                  continue;
               }
            }
            else
            {
               double traversibilityIndicator = leastSqTraversibilityCalculator.computeTraversibilityIndicator(neighbor, node);
               if (leastSqTraversibilityCalculator.isTraversible())
               {
                  edgeCost.add(traversibilityIndicator);
               }
               else
               {
                  rejectionReason.set(RejectionReason.NON_TRAVERSIBLE);
                  graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
                  continue;
               }
            }

            if (computeSurfaceNormalCost)
            {
               double yaw = Math.atan2(neighbor.getY() - node.getY(), neighbor.getX() - node.getX());
               Pose2D bodyPose = new Pose2D();
//               bodyPose.set(0.5 * (node.getX() + neighbor.getX()), 0.5 * (node.getY() + neighbor.getY()), yaw);
               bodyPose.set(neighbor.getX(), neighbor.getY(), yaw);

               UnitVector3DBasics surfaceNormal = surfaceNormalCalculator.getSurfaceNormal(HeightMapTools.coordinateToKey(bodyPose.getX(),
                                                                                                                          bodyPose.getY(),
                                                                                                                          heightMapData.getGridCenter().getX(),
                                                                                                                          heightMapData.getGridCenter().getY(),
                                                                                                                          heightMapData.getGridResolutionXY(),
                                                                                                                          heightMapData.getCenterIndex()));

               if (surfaceNormal != null)
               {
                  Vector2D edge = new Vector2D(neighbor.getX() - node.getX(), neighbor.getY() - node.getY());
                  edge.normalize();

                  /* Roll is the amount of incline orthogonal to the direction of motion */
                  leastSqNormal.set(surfaceNormal);
                  roll.set(Math.asin(Math.abs(edge.getY() * surfaceNormal.getX() - edge.getX() * surfaceNormal.getY())));
                  double inclineScale = EuclidCoreTools.clamp(Math.abs(incline.getValue()) / Math.toRadians(22.0), 0.0, 1.0);
                  double rollAngleDeadbanded = Math.max(0.0, Math.abs(roll.getValue()) - Math.toRadians(5.0));
                  edgeCost.add(rollCostWeight * inclineScale * rollAngleDeadbanded);
               }
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
      LogTools.info("Reporting status");
      boolean performSmoothing = result == BodyPathPlanningResult.FOUND_SOLUTION;

      outputToPack.setRequestId(request.getRequestId());
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
         List<Pose3D> smoothedPath = smoother.doSmoothing(bodyPath, heightMapData);
         for (int i = 0; i < bodyPath.size(); i++)
         {
            Pose3D waypoint = new Pose3D(smoothedPath.get(i));
            outputToPack.getBodyPath().add(waypoint);
         }
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

//      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + 2, latticePoint.getYIndex() + 1));
//      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + 1, latticePoint.getYIndex() + 2));
//      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() - 1, latticePoint.getYIndex() + 2));
//      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() - 2, latticePoint.getYIndex() + 1));
//      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() - 2, latticePoint.getYIndex() - 1));
//      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() - 1, latticePoint.getYIndex() - 2));
//      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + 1, latticePoint.getYIndex() - 2));
//      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + 2, latticePoint.getYIndex() - 1));
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
