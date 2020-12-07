package us.ihmc.humanoidBehaviors.exploreArea;

import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.pathPlanning.PlannerTestEnvironments;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.ArrayList;
import java.util.List;

public class ExploredAreaLattice
{
  static final double cellWidth = 0.5;
  static final double obstacleInclineThreshold = Math.toRadians(30.0);
  static final double obstacleNormalZThreshold = Math.cos(obstacleInclineThreshold);

   enum CellStatus
   {
      WALKABLE,
      OBSTACLE
   }

   private final CellStatus[][] lattice;
   private final int minX;
   private final int maxX;
   private final int minY;
   private final int maxY;

   public ExploredAreaLattice(BoundingBox3DReadOnly areaToExplore)
   {
      this.minX = toIndex(areaToExplore.getMinX());
      this.maxX = toIndex(areaToExplore.getMaxX());
      this.minY = toIndex(areaToExplore.getMinY());
      this.maxY = toIndex(areaToExplore.getMaxY());

      int numX = maxX - minX + 1;
      int numY = maxY - minY + 1;
      lattice = new CellStatus[numX][numY];
   }

   private final ConvexPolygon2D projectedPolygon = new ConvexPolygon2D();
   private final ConvexPolygon2D latticeSquare = new ConvexPolygon2D();

   public void processRegion(PlanarRegion planarRegion)
   {
      CellStatus cellStatus = planarRegion.getNormalZ() > obstacleNormalZThreshold ? CellStatus.WALKABLE : CellStatus.OBSTACLE;
      for (int i = 0; i < planarRegion.getConvexPolygons().size(); i++)
      {
         ConvexPolygon2D convexPolygon = planarRegion.getConvexPolygons().get(i);
         projectedPolygon.clear();
         RigidBodyTransformReadOnly transformToWorld = planarRegion.getTransformToWorld();
         for (int j = 0; j < convexPolygon.getNumberOfVertices(); j++)
         {
            Point3D vertex = new Point3D(convexPolygon.getVertex(j).getX(), convexPolygon.getVertex(j).getY(), 0.0);
            transformToWorld.transform(vertex);
            projectedPolygon.addVertex(vertex.getX(), vertex.getY());
         }
         projectedPolygon.update();

         int minX = Math.max(this.minX, toIndex(projectedPolygon.getMinX()));
         int maxX = Math.min(this.maxX, toIndex(projectedPolygon.getMaxX()));
         int minY = Math.max(this.minY, toIndex(projectedPolygon.getMinY()));
         int maxY = Math.min(this.maxY, toIndex(projectedPolygon.getMaxY()));

         for (int xCoordinate = minX; xCoordinate <= maxX; xCoordinate++)
         {
            for (int yCoordinate = minY; yCoordinate <= maxY; yCoordinate++)
            {
               setLatticeSquare(toDouble(xCoordinate), toDouble(yCoordinate));
               boolean intersection = DiscreteFootstepTools.arePolygonsIntersecting(projectedPolygon, latticeSquare);
               if (intersection)
               {
                  int xIndex = xCoordinate - this.minX;
                  int yIndex = yCoordinate - this.minY;

                  if (lattice[xIndex][yIndex] == null)
                     lattice[xIndex][yIndex] = cellStatus;
                  else if (lattice[xIndex][yIndex] == CellStatus.WALKABLE)
                     lattice[xIndex][yIndex] = cellStatus;
                  // If it's currently OBSTACLE then leave as is
               }
            }
         }
      }
   }

   public CellStatus[][] getLattice()
   {
      return lattice;
   }

   public int getMinX()
   {
      return minX;
   }

   public int getMaxX()
   {
      return maxX;
   }

   public int getMinY()
   {
      return minY;
   }

   public int getMaxY()
   {
      return maxY;
   }

   private List<Point2D> planPath(double goalX, double goalY)
   {
      return null;
   }

   private void setLatticeSquare(double dx, double dy)
   {
      latticeSquare.clear();
      latticeSquare.addVertex(-0.5 * cellWidth + dx, -0.5 * cellWidth + dy);
      latticeSquare.addVertex(-0.5 * cellWidth + dx, 0.5 * cellWidth + dy);
      latticeSquare.addVertex(0.5 * cellWidth + dx, -0.5 * cellWidth + dy);
      latticeSquare.addVertex(0.5 * cellWidth + dx, 0.5 * cellWidth + dy);
      latticeSquare.update();
   }

   static int toIndex(double value)
   {
      return (int) (Math.round(value / cellWidth));
   }

   static double toDouble(double index)
   {
      return cellWidth * index;
   }

   public void printState(List<ExploreAreaLatticePlanner.LatticeCell> path)
   {
      for (int i = 0; i < lattice.length; i++)
      {
         for (int j = 0; j < lattice[i].length; j++)
         {
            CellStatus cellStatus = lattice[i][j];
            if (path != null && path.contains(new ExploreAreaLatticePlanner.LatticeCell(minX + i, minY + j)))
               System.out.print("oo");
            else if (cellStatus == null)
               System.out.print(",,");
            else if (cellStatus == CellStatus.WALKABLE)
               System.out.print("..");
            else if (cellStatus == CellStatus.OBSTACLE)
               System.out.print("XX");
         }
         System.out.println();
      }
   }

   public static void main(String[] args)
   {
      ExploredAreaLattice exploreAreaLattice = new ExploredAreaLattice(new BoundingBox3D(new Point3D(0.0, 0.0, -1.0), new Point3D(10.0, 10.0, 2.0)));

//      PlanarRegionsList regions = PlannerTestEnvironments.getTrickCorridor();
      PlanarRegionsList regions = PlannerTestEnvironments.getMazeCorridor();
//      PlanarRegionsList regions = PlanarRegionsList.flatGround(20.0);

      for (int i = 0; i < regions.getNumberOfPlanarRegions(); i++)
      {
         exploreAreaLattice.processRegion(regions.getPlanarRegion(i));
      }

      double startX = 0.5;
      double startY = 0.5;
      double goalX = 9.0;
      double goalY = 6.0;

      ExploreAreaLatticePlanner planner = new ExploreAreaLatticePlanner();
      List<ExploreAreaLatticePlanner.LatticeCell> latticeCells = planner.doPlan(startX, startY, goalX, goalY, exploreAreaLattice);
      exploreAreaLattice.printState(latticeCells);
   }
}
