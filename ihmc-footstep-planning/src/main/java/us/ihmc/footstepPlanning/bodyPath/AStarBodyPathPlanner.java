package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.log.AStarBodyPathEdgeData;
import us.ihmc.footstepPlanning.log.AStarBodyPathIterationData;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.graph.structure.NodeComparator;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.*;

public class AStarBodyPathPlanner
{
   private static final double traversibilityCostScale = 0.5;
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final AStarBodyPathEdgeData edgeData;
   private HeightMapData heightMapData;
   private final HashSet<BodyPathLatticePoint> expandedNodeSet = new HashSet<>();
   private final DirectedGraph<BodyPathLatticePoint> graph = new DirectedGraph<>();
   private final List<BodyPathLatticePoint> neighbors = new ArrayList<>();

   /* squared up offsets */
   private final TIntArrayList xOffsetsSq = new TIntArrayList();
   private final TIntArrayList yOffsetsSq = new TIntArrayList();
   private final TIntArrayList yawOffsetsSq = new TIntArrayList();

   /* diagonal offsets */
   private final TIntArrayList xOffsetsDiag = new TIntArrayList();
   private final TIntArrayList yOffsetsDiag = new TIntArrayList();
   private final TIntArrayList yawOffsetsDiag = new TIntArrayList();

   private final YoBoolean containsCollision = new YoBoolean("containsCollision", registry);
   private final YoDouble edgeCost = new YoDouble("edgeCost", registry);
   private final YoDouble deltaHeight = new YoDouble("deltaHeight", registry);
   private final YoDouble snapHeight = new YoDouble("snapHeight", registry);

   private final BodyPathTraversibilityCalculator traversibilityCalculator;
   private final HeightMapObstacleDetector obstacleDetector = new HeightMapObstacleDetector();

   private final PriorityQueue<BodyPathLatticePoint> stack;
   private BodyPathLatticePoint startNode, goalNode;
   private final HashMap<BodyPathLatticePoint, Double> gridHeightMap = new HashMap<>();
   private BodyPathLatticePoint leastCostNode = null;
   private YoEnum<RejectionReason> rejectionReason = new YoEnum<>("rejectionReason", registry, RejectionReason.class, true);

   private final TIntArrayList xCollisionOffsets = new TIntArrayList();
   private final TIntArrayList yCollisionOffsets = new TIntArrayList();
   private final TIntArrayList xSnapOffsets = new TIntArrayList();
   private final TIntArrayList ySnapOffsets = new TIntArrayList();

   private final List<AStarBodyPathIterationData> iterationData = new ArrayList<>();
   private final HashMap<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData> edgeDataMap = new HashMap<>();

   /* Parameters to extract */
   static final double groundClearance = 0.25;
   static final double collisionRadius = 0.3;
   static final double snapRadius = 0.1;
   static final double maxStepUpDown = 0.2;

   public AStarBodyPathPlanner(FootstepPlannerParametersReadOnly parameters, ConvexPolygon2D footPolygon)
   {
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

      int[] xOffsets = new int[]{-1, 0, 1, 2};
      int[] yOffsets = new int[]{-1, 0, 1};
      int[] yawOffsets = new int[]{-1, 0, 1};

      for (int i = 0; i < xOffsets.length; i++)
      {
         for (int j = 0; j < yOffsets.length; j++)
         {
            for (int k = 0; k < yawOffsets.length; k++)
            {
               if (i == 0 && j == 0 && k == 0)
                  continue;

               xOffsetsSq.add(xOffsets[i]);
               yOffsetsSq.add(yOffsets[j]);
               yawOffsetsSq.add(yawOffsets[k]);
            }
         }
      }

      xOffsets = new int[] {-1, 0, 1};
      for (int k = 0; k < yawOffsets.length; k++)
      {
         for (int i = 0; i < xOffsets.length; i++)
         {
            for (int j = 0; j < yOffsets.length; j++)
            {
               if (i == 0 && j == 0 && k == 0)
                  continue;

               xOffsetsDiag.add(xOffsets[i]);
               yOffsetsDiag.add(yOffsets[j]);
               yawOffsetsDiag.add(yawOffsets[k]);
            }
         }

         xOffsetsDiag.add(2);
         yOffsetsDiag.add(1);
         yawOffsetsDiag.add(yawOffsets[k]);

         xOffsetsDiag.add(1);
         yOffsetsDiag.add(2);
         yawOffsetsDiag.add(yawOffsets[k]);
      }
   }

   public void setHeightMapData(HeightMapData heightMapData)
   {
      if (this.heightMapData == null || !EuclidCoreTools.epsilonEquals(this.heightMapData.getGridResolutionXY(), heightMapData.getGridResolutionXY(), 1e-3))
      {
         int minMaxOffsetXY = (int) Math.round(collisionRadius / heightMapData.getGridResolutionXY());
         packRadialOffsets(heightMapData, minMaxOffsetXY, collisionRadius, xCollisionOffsets, yCollisionOffsets);
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

   public List<Pose3DReadOnly> planPath(Pose3DReadOnly startPose, Pose3DReadOnly goalPose)
   {
      iterationData.clear();
      edgeDataMap.clear();
      gridHeightMap.clear();

      int minMaxOffsetXY = (int) Math.round(snapRadius / heightMapData.getGridResolutionXY());
      packRadialOffsets(heightMapData, minMaxOffsetXY, snapRadius, xSnapOffsets, ySnapOffsets);

      startNode = new BodyPathLatticePoint(startPose.getX(), startPose.getY(), startPose.getYaw());
      goalNode = new BodyPathLatticePoint(goalPose.getX(), goalPose.getY(), goalPose.getYaw());
      stack.clear();
      stack.add(startNode);
      graph.initialize(startNode);
      expandedNodeSet.clear();
      gridHeightMap.put(startNode, startPose.getZ());
      leastCostNode = startNode;
      obstacleDetector.compute(heightMapData);

      int maxIterations = 3000;
      int iterationCount = 0;
      boolean reachedGoal = false;

      planningLoop:
      while (iterationCount++ < maxIterations)
      {
         BodyPathLatticePoint node = getNextNode();

         if (node == null)
         {
            LogTools.info("Stack is empty, no path exists...");
            break;
         }

         populateNeighbors(node);

         for (BodyPathLatticePoint neighbor : neighbors)
         {
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

            this.containsCollision.set(collisionDetected(neighbor, snapHeight.getDoubleValue()));
            if (containsCollision.getValue())
            {
               rejectionReason.set(RejectionReason.COLLISION);
               graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
               continue;
            }

            double distanceCost = xyDistance(node, neighbor);
            double traversibilityCost = traversibilityCostScale * traversibilityCalculator.computeTraversibilityIndicator(neighbor, node);

            int xIndex = HeightMapTools.coordinateToIndex(node.getX(), heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());
            int yIndex = HeightMapTools.coordinateToIndex(node.getY(), heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), heightMapData.getCenterIndex());
            double obstacleCost = obstacleDetector.getTerrainCost().get(xIndex, yIndex);

//            edgeCost.set(distanceCost + traversibilityCost);
            edgeCost.set(distanceCost + traversibilityCost + obstacleCost);

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
      }

      BodyPathLatticePoint terminalNode = reachedGoal ? goalNode : leastCostNode;
      List<BodyPathLatticePoint> path = graph.getPathFromStart(terminalNode);
      List<Pose3DReadOnly> waypoints = new ArrayList<>();
      for (int i = 0; i < path.size(); i++)
      {
         Pose3D waypoint = new Pose3D();
         waypoint.getPosition().set(path.get(i).getX(), path.get(i).getY(), gridHeightMap.get(path.get(i)));
         waypoints.add(waypoint);
      }

      markSolutionEdges(terminalNode);
      return waypoints;
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

   private void populateNeighbors(BodyPathLatticePoint latticePoint)
   {
      neighbors.clear();

      if (latticePoint.getYawIndex() % 2 == 0)
      {
         // squared up
         for (int i = 0; i < xOffsetsSq.size(); i++)
         {
            Pair<Integer, Integer> offset = rotate(xOffsetsSq.get(i), yOffsetsSq.get(i), latticePoint.getYawIndex() / 2);
            neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + offset.getLeft(),
                                                   latticePoint.getYIndex() + offset.getRight(),
                                                   latticePoint.getYawIndex() + yawOffsetsSq.get(i)));
         }
      }
      else
      {
         // diagonal
         for (int i = 0; i < xOffsetsDiag.size(); i++)
         {
            Pair<Integer, Integer> offset = rotate(xOffsetsDiag.get(i), yOffsetsDiag.get(i), latticePoint.getYawIndex() / 2);
            neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + offset.getLeft(),
                                                   latticePoint.getYIndex() + offset.getRight(),
                                                   latticePoint.getYawIndex() + yawOffsetsDiag.get(i)));
         }
      }
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

   private boolean collisionDetected(BodyPathLatticePoint latticePoint, double height)
   {
      int centerIndex = heightMapData.getCenterIndex();
      int xIndex = HeightMapTools.coordinateToIndex(latticePoint.getX(), heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), centerIndex);
      int yIndex = HeightMapTools.coordinateToIndex(latticePoint.getY(), heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), centerIndex);
      double heightThreshold = height + groundClearance;

      for (int i = 0; i < xCollisionOffsets.size(); i++)
      {
         int xQuery = xIndex + xCollisionOffsets.get(i);
         int yQuery = yIndex + yCollisionOffsets.get(i);
         double heightQuery = heightMapData.getHeightAt(xQuery, yQuery);
         if (Double.isNaN(heightQuery))
         {
            continue;
         }

         if (heightQuery >= heightThreshold)
         {
            return true;
         }
      }

      return false;
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
