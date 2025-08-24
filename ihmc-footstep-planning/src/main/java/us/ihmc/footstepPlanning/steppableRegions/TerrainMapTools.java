package us.ihmc.footstepPlanning.steppableRegions;

import perception_msgs.msg.dds.TerrainMapMessage;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.robotics.geometry.LeastSquaresZPlaneFitter;

import java.util.ArrayList;

public class TerrainMapTools
{
   /**
    * Computes and returns the index along the desired axis of the grid that matches this point. It is assumed that the coordinate and center values are
    * expressed in the same dimension of the same frame.
    *
    * @param cellsPerMeter this is the number of cells found in one meter of the grid, which maps to the resolution.
    * @param cellsPerSide  this is the cells per side of the grid.
    * @param coordinate    the coordinate in question.
    * @param center        the center of the grid along the defined axis.
    * @return index that contains the cell along the axis in question.
    */
   public static int getLocalIndex(int cellsPerMeter, int cellsPerSide, double coordinate, double center)
   {
      // TODO probably a height map tools method for this.
      return (int) ((coordinate - center) * cellsPerMeter + (double) cellsPerSide / 2);
   }

   public static boolean isOutOfBounds(int cellsPerSide, int rIndex, int cIndex)
   {
      return rIndex < 0 || rIndex >= cellsPerSide || cIndex < 0 || cIndex >= cellsPerSide;
   }

   public static UnitVector3DReadOnly computeSurfaceNormalInWorld(TerrainMapData terrainMapData, double x, double y, int patchSize)
   {
      int cellsPerMeter = terrainMapData.getCenterIndex();
      int localGridSize = terrainMapData.getLocalGridSize();
      double centerX = terrainMapData.getTerrainMapCenter().getX();
      double centerY = terrainMapData.getTerrainMapCenter().getY();
      int rIndex = getLocalIndex(cellsPerMeter, localGridSize, x, centerX);
      int cIndex = getLocalIndex(cellsPerMeter, localGridSize, y, centerY);
      int halfGridSize = localGridSize / 2;

      if (terrainMapData.hasSnapNormal())
      {
         return terrainMapData.getNormalLocal(rIndex, cIndex);
      }
      else
      {
         Plane3D bestFitPlane = new Plane3D();
         ArrayList<Point3D> points = new ArrayList<>();

         //LogTools.info("rIndex: {}, cIndex: {}, origin: {}", rIndex, cIndex, sensorOrigin);

         for (int i = -patchSize; i <= patchSize; i++)
         {
            for (int j = -patchSize; j <= patchSize; j++)
            {
               int r = rIndex + i;
               int c = cIndex + j;
               if (isOutOfBounds(localGridSize, r, c))
                  continue;

               float height = terrainMapData.getHeightLocal(r, c);

               // compute full 3d point
               Point3D point = new Point3D();
               point.setX(centerX + (double) (r - halfGridSize) / cellsPerMeter);
               point.setY(centerY + (double) (c - halfGridSize) / cellsPerMeter);
               point.setZ(height);

               //LogTools.info("Point: {}", point);

               points.add(point);
            }
         }

         LeastSquaresZPlaneFitter planeFitter = new LeastSquaresZPlaneFitter();
         planeFitter.fitPlaneToPoints(points, bestFitPlane);
         return bestFitPlane.getNormal();
      }
   }

   public static TerrainMapMessage toMessage(TerrainMapData terrainMapData)
   {
      TerrainMapMessage message = new TerrainMapMessage();

      message.setLocalGridSize(terrainMapData.getLocalGridSize());
      message.setCellsPerMeter((byte) terrainMapData.getCenterIndex());

      message.setMapCenterX(terrainMapData.getTerrainMapCenter().getX());
      message.setMapCenterY(terrainMapData.getTerrainMapCenter().getY());

      if (terrainMapData.hasTerrainCost())
      {
         message.setHasTerrainCostData(true);
         PerceptionMessageTools.packDataArray(message.getTerrainCostData(), terrainMapData.getTerrainCostMap());
      }
      if (terrainMapData.hasContactMap())
      {
         message.setHasContactMapData(true);
         PerceptionMessageTools.packDataArray(message.getContactMapData(), terrainMapData.getContactMap());
      }

      if (terrainMapData.hasHeightMap())
      {
         message.setHasHeightMapData(true);
         PerceptionMessageTools.packDataArray(message.getHeightMapData(), terrainMapData.getHeightMap());
      }
      if (terrainMapData.hasSnapNormal())
      {
         message.setHasSnappedNormalData(true);
         PerceptionMessageTools.packDataArray(message.getSnappedNormalXData(), terrainMapData.getSnapNormalXMat());
         PerceptionMessageTools.packDataArray(message.getSnappedNormalYData(), terrainMapData.getSnapNormalYMat());
         PerceptionMessageTools.packDataArray(message.getSnappedNormalZData(), terrainMapData.getSnapNormalZMat());
      }
      if (terrainMapData.hasSnappedArea())
      {
         message.setHasSnappedAreaData(true);
         PerceptionMessageTools.packDataArray(message.getSnappedAreaData(), terrainMapData.getSnappedAreaFractionMat());
      }
      if (terrainMapData.hasSteppability())
      {
         message.setHasSteppabilityData(true);
         PerceptionMessageTools.packDataArray(message.getSteppabilityData(), terrainMapData.getSteppabilityMat());
      }
      if (terrainMapData.hasSteppableConnections())
      {
         message.setHasSteppableConnectionsData(true);
         PerceptionMessageTools.packDataArray(message.getSteppableConnectionsData(), terrainMapData.getSteppabilityConnectionsMat());
      }

      return message;
   }

   public static boolean isEmpty(TerrainMapMessage message)
   {
      if (message.getHasHeightMapData())
         return false;
      return !message.getHasSteppabilityData();
   }
}
