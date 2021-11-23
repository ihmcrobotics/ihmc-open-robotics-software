package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
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
import us.ihmc.yoVariables.variable.YoVariable;

import java.util.*;

public class AStarBodyPathPlanner
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final AStarBodyPathEdgeData edgeData;
   private HeightMapData heightMapData;
   private final HashSet<BodyPathLatticePoint> expandedNodeSet = new HashSet<>();
   private final DirectedGraph<BodyPathLatticePoint> graph = new DirectedGraph<>();
   private final List<BodyPathLatticePoint> neighbors = new ArrayList<>();

   private final YoBoolean containsCollision = new YoBoolean("containsCollision", registry);
   private final YoDouble edgeCost = new YoDouble("edgeCost", registry);
   private final YoDouble deltaHeight = new YoDouble("deltaHeight", registry);

   private final PriorityQueue<BodyPathLatticePoint> stack;
   private BodyPathLatticePoint startNode, goalNode;
   private final HashMap<BodyPathLatticePoint, Double> gridHeightMap = new HashMap<>();
   private BodyPathLatticePoint leastCostNode = null;

   private final TIntArrayList xCollisionOffsets = new TIntArrayList();
   private final TIntArrayList yCollisionOffsets = new TIntArrayList();

   private final List<AStarBodyPathIterationData> iterationData = new ArrayList<>();
   private final HashMap<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData> edgeDataMap = new HashMap<>();

   /* Parameters to extract */
   private static final double groundClearance = 0.25;
   private static final double collisionRadius = 0.3;
   private static final double maxStepUpDown = 0.2;

   public AStarBodyPathPlanner()
   {
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
                                         deltaHeight.set(Double.NaN);
                                      });
   }

   public void setHeightMapData(HeightMapData heightMapData)
   {
      if (this.heightMapData == null || !EuclidCoreTools.epsilonEquals(this.heightMapData.getGridResolutionXY(), heightMapData.getGridResolutionXY(), 1e-3))
      {
         int minMaxOffsetXY = (int) Math.round(collisionRadius / heightMapData.getGridResolutionXY());
         for (int i = -minMaxOffsetXY; i <= minMaxOffsetXY; i++)
         {
            for (int j = -minMaxOffsetXY; j <= minMaxOffsetXY; j++)
            {
               double x = i * heightMapData.getGridResolutionXY();
               double y = j * heightMapData.getGridResolutionXY();
               if (EuclidCoreTools.norm(x, y) < collisionRadius && !(i == 0 && j == 0))
               {
                  xCollisionOffsets.add(i);
                  yCollisionOffsets.add(j);
               }
            }
         }

      }

      this.heightMapData = heightMapData;
   }

   public List<Pose3DReadOnly> planPath(Pose3DReadOnly startPose, Pose3DReadOnly goalPose)
   {
      iterationData.clear();
      edgeDataMap.clear();

      startNode = new BodyPathLatticePoint(startPose.getX(), startPose.getY());
      goalNode = new BodyPathLatticePoint(goalPose.getX(), goalPose.getY());
      stack.clear();
      stack.add(startNode);
      graph.initialize(startNode);
      expandedNodeSet.clear();
      snap(startNode);
      leastCostNode = startNode;

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
            double height = snap(neighbor);
            if (Double.isNaN(height))
            {
               graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
               continue;
            }

            deltaHeight.set(Math.abs(height - gridHeightMap.get(node)));
            if (deltaHeight.getValue() > maxStepUpDown)
            {
               graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
               continue;
            }

            this.containsCollision.set(collisionDetected(neighbor, height));
            if (containsCollision.getValue())
            {
               graph.checkAndSetEdge(node, neighbor, Double.POSITIVE_INFINITY);
               continue;
            }

            edgeCost.set(xyDistance(node, neighbor));
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
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() - 1, latticePoint.getYIndex() - 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() - 1, latticePoint.getYIndex()));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() - 1, latticePoint.getYIndex() + 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex(), latticePoint.getYIndex() + 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + 1, latticePoint.getYIndex() + 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + 1, latticePoint.getYIndex()));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex() + 1, latticePoint.getYIndex() - 1));
      neighbors.add(new BodyPathLatticePoint(latticePoint.getXIndex(), latticePoint.getYIndex() - 1));
   }

   private double snap(BodyPathLatticePoint latticePoint)
   {
      double height = heightMapData.getHeightAt(latticePoint.getX(), latticePoint.getY());
      gridHeightMap.put(latticePoint, height);
      return height;
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
