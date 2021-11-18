package us.ihmc.footstepPlanning.bodyPath;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.pathPlanning.graph.structure.NodeComparator;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

import java.util.*;

public class AStarBodyPathPlanner
{
   private HeightMapData heightMapData;
   private final HashSet<BodyPathLatticePoint> expandedNodeSet = new HashSet<>();
   private final DirectedGraph<BodyPathLatticePoint> graph = new DirectedGraph<>();
   private final List<BodyPathLatticePoint> neighbors = new ArrayList<>();

   private final PriorityQueue<BodyPathLatticePoint> stack;
   private BodyPathLatticePoint startNode, goalNode;
   private final HashMap<BodyPathLatticePoint, Double> gridHeightMap = new HashMap<>();

   private final TIntArrayList xCollisionOffsets = new TIntArrayList();
   private final TIntArrayList yCollisionOffsets = new TIntArrayList();
   
   /* Parameters to extract */
   private static final double groundClearance = 0.25;
   private static final double collisionRadius = 0.3;
   private static final double maxStepUpDown = 0.2;

   public AStarBodyPathPlanner()
   {
      stack = new PriorityQueue<>(new NodeComparator<>(graph, this::heuristics));
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
               double y = i * heightMapData.getGridResolutionXY();
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

   public List<Pose3DReadOnly> planPath(Pose3D startPose, Pose3D goalPose)
   {
      startNode = new BodyPathLatticePoint(startPose.getX(), startPose.getY());
      goalNode = new BodyPathLatticePoint(goalPose.getX(), goalPose.getY());
      stack.clear();
      stack.add(startNode);
      graph.initialize(startNode);
      expandedNodeSet.clear();
      snap(startNode);

      int maxIterations = 3000;
      int iterationCount = 0;
      boolean reachedGoal = false;

      planningLoop:
      while (iterationCount++ < maxIterations)
      {
         BodyPathLatticePoint node = getNextNode();
         System.out.println("Expanding " + node);

         if (node == null)
         {
            break;
         }

         populateNeighbors(node);

         for (BodyPathLatticePoint neighbor : neighbors)
         {
            double height = snap(neighbor);
            if (Double.isNaN(height))
               continue;

            if (Math.abs(height - gridHeightMap.get(node)) > maxStepUpDown)
            {
               continue;
            }

            boolean containsCollision = collisionDetected(neighbor, height);
            if (containsCollision)
            {
               continue;
            }

            double cost = computeCost(node, neighbor);
            graph.checkAndSetEdge(node, neighbor, cost);
            stack.add(neighbor);

            if (node.equals(goalNode))
            {
               reachedGoal = true;
               break planningLoop;
            }
         }

         expandedNodeSet.add(node);
      }

      if (reachedGoal)
      {
         List<BodyPathLatticePoint> path = graph.getPathFromStart(goalNode);
         List<Pose3DReadOnly> waypoints = new ArrayList<>();
         for (int i = 0; i < path.size(); i++)
         {
            Pose3D waypoint = new Pose3D();
            waypoint.getPosition().set(path.get(i).getX(), path.get(i).getY(), gridHeightMap.get(path.get(i)));
            waypoints.add(waypoint);
         }

         return waypoints;
      }
      else
      {
         return null;
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
      int minMaxIndexXY = heightMapData.getMinMaxIndexXY();
      int xIndex = HeightMapTools.toIndex(latticePoint.getX(), heightMapData.getGridCenter().getX(), heightMapData.getGridResolutionXY(), minMaxIndexXY);
      int yIndex = HeightMapTools.toIndex(latticePoint.getY(), heightMapData.getGridCenter().getY(), heightMapData.getGridResolutionXY(), minMaxIndexXY);
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

   private double computeCost(BodyPathLatticePoint startNode, BodyPathLatticePoint endNode)
   {
      return Math.sqrt(MathTools.square(startNode.getX() - endNode.getX()) + MathTools.square(startNode.getY() - endNode.getY()));
   }

   private double heuristics(BodyPathLatticePoint node)
   {
      return Math.sqrt(MathTools.square(node.getX() - goalNode.getX()) + MathTools.square(node.getY() - goalNode.getY()));
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
}
