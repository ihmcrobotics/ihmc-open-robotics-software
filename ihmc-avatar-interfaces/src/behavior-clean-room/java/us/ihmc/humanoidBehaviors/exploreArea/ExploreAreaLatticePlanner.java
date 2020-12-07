package us.ihmc.humanoidBehaviors.exploreArea;

import us.ihmc.euclid.geometry.BoundingBox3D;
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

   public List<LatticeCell> doPlan(double startX, double startY, double goalX, double goalY, boolean printState)
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

      return path;
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

      for (LatticeCell cell : neighbors)
      {
         boolean isValid = cell.x >= minX && cell.x <= maxX && cell.y >= minY && cell.y <= maxY;
         isValid = isValid && exploredAreaLattice[cell.x][cell.y] != ExploredAreaLattice.CellStatus.OBSTACLE;

         if (isValid)
         {
            double edgeCost = 1.0;
            graph.checkAndSetEdge(cellToExpand, cell, edgeCost);
            stack.add(cell);
         }

         if (cell.equals(goalCell))
            foundGoal = true;
      }

      expandedNodeSet.add(cellToExpand);
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
      BoundingBox3D areaToExplore = new BoundingBox3D(new Point3D(0.0, 0.0, -1.0), new Point3D(10.0, 10.0, 2.0));
      ExploreAreaLatticePlanner planner = new ExploreAreaLatticePlanner(areaToExplore);

            PlanarRegionsList regions = PlannerTestEnvironments.getTrickCorridor();
//      PlanarRegionsList regions = PlannerTestEnvironments.getMazeCorridor();
      //      PlanarRegionsList regions = PlanarRegionsList.flatGround(20.0);
      planner.processRegions(regions);

      double startX = 0.5;
      double startY = 0.5;
      double goalX = 9.0;
      double goalY = 6.0;

      List<ExploreAreaLatticePlanner.LatticeCell> latticeCells = planner.doPlan(startX, startY, goalX, goalY, true);
   }
}
