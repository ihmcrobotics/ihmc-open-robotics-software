package us.ihmc.humanoidBehaviors.exploreArea;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.AStarIterationData;
import us.ihmc.pathPlanning.PlannerTestEnvironments;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.pathPlanning.graph.structure.NodeComparator;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;

public class ExploreAreaLatticePlanner
{
   static class LatticeCell
   {
      final int x, y;

      public LatticeCell(int x, int y)
      {
         this.x = x;
         this.y = y;
      }

      public LatticeCell(double x, double y)
      {
         this.x = ExploredAreaLattice.toIndex(x);
         this.y = ExploredAreaLattice.toIndex(y);
      }

      @Override
      public int hashCode()
      {
         return 13 * x + 17 * y;
      }

      @Override
      public boolean equals(Object other)
      {
         if (!(other instanceof LatticeCell))
         {
            return false;
         }

         LatticeCell otherCell = (LatticeCell) other;
         return (otherCell.x == x) && (otherCell.y == y);
      }
   }

   private final HashSet<LatticeCell> expandedNodeSet = new HashSet<>();
   private final DirectedGraph<LatticeCell> graph = new DirectedGraph<>();
   private final AStarIterationData<LatticeCell> iterationData = new AStarIterationData<>();
   private final List<LatticeCell> neighbors = new ArrayList<>();
   private final NodeComparator<LatticeCell> nodeComparator = new NodeComparator<>(graph, this::getDistanceToGoal);
   private final PriorityQueue<LatticeCell> stack = new PriorityQueue<>(nodeComparator);

   private final ExploredAreaLattice exploredAreaLattice;

   private LatticeCell goalCell;
   private boolean foundGoal;

   public ExploreAreaLatticePlanner(BoundingBox3DReadOnly areaToExplore)
   {
      this.exploredAreaLattice = new ExploredAreaLattice(areaToExplore);
   }

   public List<Point3D> doPlan(double startX, double startY, double goalX, double goalY, boolean printState)
   {
      LatticeCell startCell = new LatticeCell(startX, startY);
      goalCell = new LatticeCell(goalX, goalY);
      initialize(startCell);

      while (true)
      {
         LatticeCell nodeToExpand = getNextNode();
         if (nodeToExpand == null)
            break;

         doPlanningIteration(nodeToExpand,
                             exploredAreaLattice.getLattice(),
                             exploredAreaLattice.getMinX(),
                             exploredAreaLattice.getMaxX(),
                             exploredAreaLattice.getMinY(),
                             exploredAreaLattice.getMaxY());
         if (foundGoal)
            break;
      }

      List<LatticeCell> path = foundGoal ? graph.getPathFromStart(goalCell) : new ArrayList<>();

      if (printState)
      {
         exploredAreaLattice.printState(path);
      }

      List<Point3D> waypoints = new ArrayList<>();
      for (int i = 0; i < path.size(); i++)
      {
         waypoints.add(new Point3D(ExploredAreaLattice.toDouble(path.get(i).x), ExploredAreaLattice.toDouble(path.get(i).y), 0.0));
      }

      return waypoints;
   }

   public void processRegions(PlanarRegionsList planarRegionsList)
   {
      for (int i = 0; i < planarRegionsList.getNumberOfPlanarRegions(); i++)
      {
         exploredAreaLattice.processRegion(planarRegionsList.getPlanarRegion(i));
      }
   }

   private void initialize(LatticeCell startNode)
   {
      graph.initialize(startNode);
      stack.clear();
      stack.add(startNode);
      expandedNodeSet.clear();

      foundGoal = false;
   }

   public void doPlanningIteration(LatticeCell cellToExpand, ExploredAreaLattice.CellStatus[][] exploredAreaLattice, int minX, int maxX, int minY, int maxY)
   {
      iterationData.clear();
      iterationData.setParentNode(cellToExpand);

      neighbors.clear();
      neighbors.add(new LatticeCell(cellToExpand.x - 1, cellToExpand.y));
      neighbors.add(new LatticeCell(cellToExpand.x + 1, cellToExpand.y));
      neighbors.add(new LatticeCell(cellToExpand.x, cellToExpand.y - 1));
      neighbors.add(new LatticeCell(cellToExpand.x, cellToExpand.y + 1));
      neighbors.add(new LatticeCell(cellToExpand.x - 1, cellToExpand.y - 1));
      neighbors.add(new LatticeCell(cellToExpand.x - 1, cellToExpand.y + 1));
      neighbors.add(new LatticeCell(cellToExpand.x + 1, cellToExpand.y - 1));
      neighbors.add(new LatticeCell(cellToExpand.x + 1, cellToExpand.y + 1));

      for (LatticeCell cell : neighbors)
      {
         boolean isValidRange = isValidRange(minX, maxX, minY, maxY, cell);
         if (!isValidRange)
            continue;

         ExploredAreaLattice.CellStatus cellType = exploredAreaLattice[cell.x - minX][cell.y - minY];
         boolean isObstacle = cellType == ExploredAreaLattice.CellStatus.OBSTACLE || cellType == ExploredAreaLattice.CellStatus.NEXT_TO_OBSTACLE;
         if (isObstacle)
            continue;

         double edgeCost = nextToObstacle(cell, exploredAreaLattice, minX, maxX, minY, maxY) ? 5.0 : 1.0;
         graph.checkAndSetEdge(cellToExpand, cell, edgeCost);
         stack.add(cell);

         if (cell.equals(goalCell))
            foundGoal = true;
      }

      expandedNodeSet.add(cellToExpand);
   }

   private static boolean isValidRange(int minX, int maxX, int minY, int maxY, LatticeCell cell)
   {
      return cell.x >= minX && cell.x <= maxX && cell.y >= minY && cell.y <= maxY;
   }

   private boolean nextToObstacle(LatticeCell cell, ExploredAreaLattice.CellStatus[][] exploredAreaLattice, int minX, int maxX, int minY, int maxY)
   {
      LatticeCell cell0 = new LatticeCell(cell.x - 1, cell.y);
      LatticeCell cell1 = new LatticeCell(cell.x + 1, cell.y);
      LatticeCell cell2 = new LatticeCell(cell.x, cell.y - 1);
      LatticeCell cell3 = new LatticeCell(cell.x, cell.y + 1);
      LatticeCell cell4 = new LatticeCell(cell.x - 1, cell.y + 1);
      LatticeCell cell5 = new LatticeCell(cell.x + 1, cell.y + 1);
      LatticeCell cell6 = new LatticeCell(cell.x - 1, cell.y - 1);
      LatticeCell cell7 = new LatticeCell(cell.x + 1, cell.y - 1);
      if (isValidRange(minX, maxX, minY, maxY, cell0) && isObstacle(cell0, exploredAreaLattice, minX, minY))
         return true;
      if (isValidRange(minX, maxX, minY, maxY, cell1) && isObstacle(cell1, exploredAreaLattice, minX, minY))
         return true;
      if (isValidRange(minX, maxX, minY, maxY, cell2) && isObstacle(cell2, exploredAreaLattice, minX, minY))
         return true;
      if (isValidRange(minX, maxX, minY, maxY, cell3) && isObstacle(cell3, exploredAreaLattice, minX, minY))
         return true;
      if (isValidRange(minX, maxX, minY, maxY, cell4) && isObstacle(cell4, exploredAreaLattice, minX, minY))
         return true;
      if (isValidRange(minX, maxX, minY, maxY, cell5) && isObstacle(cell5, exploredAreaLattice, minX, minY))
         return true;
      if (isValidRange(minX, maxX, minY, maxY, cell6) && isObstacle(cell6, exploredAreaLattice, minX, minY))
         return true;
      if (isValidRange(minX, maxX, minY, maxY, cell7) && isObstacle(cell7, exploredAreaLattice, minX, minY))
         return true;
      return false;
   }

   private boolean isObstacle(LatticeCell cell, ExploredAreaLattice.CellStatus[][] exploredAreaLattice, int minX, int minY)
   {
      ExploredAreaLattice.CellStatus cellStatus = exploredAreaLattice[cell.x - minX][cell.y - minY];
      return cellStatus == ExploredAreaLattice.CellStatus.OBSTACLE || cellStatus == ExploredAreaLattice.CellStatus.NEXT_TO_OBSTACLE;
   }

   private double getDistanceToGoal(LatticeCell cell)
   {
      return Math.abs(cell.x - goalCell.x) + Math.abs(cell.y - goalCell.y);
   }

   public LatticeCell getNextNode()
   {
      while (!stack.isEmpty())
      {
         LatticeCell nextNode = stack.poll();
         if (!expandedNodeSet.contains(nextNode))
         {
            return nextNode;
         }
      }

      return null;
   }

   public static void main(String[] args)
   {
//      BoundingBox3D areaToExplore = new BoundingBox3D(new Point3D(0.0, 0.0, -1.0), new Point3D(10.0, 10.0, 2.0));
      //      PlanarRegionsList regions = PlannerTestEnvironments.getMazeCorridor();

      BoundingBox3D areaToExplore = new BoundingBox3D(new Point3D(-4.0, -7.0, -1.0), new Point3D(8.0, 5.0, 2.0));
      PlanarRegionsList regions = PlannerTestEnvironments.getTrickCorridor();

      ExploreAreaLatticePlanner planner = new ExploreAreaLatticePlanner(areaToExplore);

      //      PlanarRegionsList regions = PlanarRegionsList.flatGround(20.0);
      planner.processRegions(regions);

      double startX = 0.5;
      double startY = 0.5;
      double goalX = 5.0;
      double goalY = 1.0;

      planner.doPlan(startX, startY, goalX, goalY, true);
   }
}
