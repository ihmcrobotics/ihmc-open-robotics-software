package us.ihmc.footstepPlanning.tools;

import static us.ihmc.perception.gpuMapping.TerrainMapData.*;

import perception_msgs.TerrainMapMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.perception.gpuMapping.HeightMapTools;
import us.ihmc.perception.gpuMapping.SnapResult;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public class PlanarRegionToHeightMapConverter
{
   public static final double defaultResolution = 0.02;

   public static TerrainMapMessage convertFromPlanarRegionsToHeightMap(PlanarRegionsList planarRegionsList)
   {
      return convertFromPlanarRegionsToHeightMap(planarRegionsList, defaultResolution);
   }

   public static TerrainMapMessage convertFromPlanarRegionsToHeightMap(PlanarRegionsList planarRegionsList, double resolutionXY)
   {
      return convertFromPlanarRegionsToHeightMap(planarRegionsList, resolutionXY, true);
   }
   public static TerrainMapMessage convertFromPlanarRegionsToHeightMap(PlanarRegionsList planarRegionsList, double resolutionXY, boolean includeNormal)
   {
      return convertFromPlanarRegionsToHeightMap(planarRegionsList.getPlanarRegionsAsList(), resolutionXY, includeNormal);
   }

   public static TerrainMapMessage convertFromPlanarRegionsToHeightMap(List<PlanarRegion> planarRegionList, double resolutionXY)
   {
      return convertFromPlanarRegionsToHeightMap(planarRegionList, resolutionXY, true);
   }
   public static TerrainMapMessage convertFromPlanarRegionsToHeightMap(List<PlanarRegion> planarRegionList, double resolutionXY, boolean includeNormal)
   {
      BoundingBox2D occupiedArea = new BoundingBox2D();
      planarRegionList.forEach(planarRegion ->
                               {
                                  planarRegion.getConcaveHull().forEach(point ->
                                                                        {
                                                                           Point3D point3D = new Point3D(point);
                                                                           planarRegion.transformFromLocalToWorld(point3D);
                                                                           occupiedArea.updateToIncludePoint(point3D.getX(), point3D.getY());
                                                                        });
                               });

      double width = occupiedArea.getMaxX() - occupiedArea.getMinX();
      double height = occupiedArea.getMaxY() - occupiedArea.getMinY();

      double gridCenterX = 0.5 * (occupiedArea.getMaxX() + occupiedArea.getMinX());
      double gridCenterY = 0.5 * (occupiedArea.getMaxY() + occupiedArea.getMinY());
      double sideLength = Math.max(width, height);

      TerrainMapMessage message = new TerrainMapMessage();
      message.setWidthInMeters(sideLength);
      message.setGridCenterX(gridCenterX);
      message.setGridCenterY(gridCenterY);
      message.setCellSizeInMeters(resolutionXY);

      int centerIndex = HeightMapTools.computeCenterIndex(sideLength, resolutionXY);
      int cellsPerAxis = 2 * centerIndex + 1;

      for (int xIndex = 0; xIndex < cellsPerAxis; xIndex++)
      {
         for (int yIndex = 0; yIndex < cellsPerAxis; yIndex++)
         {
            int key = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
            double xPosition = HeightMapTools.keyToXCoordinate(key, gridCenterX, resolutionXY, centerIndex);
            double yPosition = HeightMapTools.keyToYCoordinate(key, gridCenterY, resolutionXY, centerIndex);

            Point3D pointToProject = new Point3D(xPosition, yPosition, 0.0);
            Point3DReadOnly projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(pointToProject, planarRegionList);

            if (projectedPoint != null && Double.isFinite(projectedPoint.getZ()))
            {
               List<PlanarRegion> intersectingRegions = PlanarRegionTools.findPlanarRegionsContainingPoint(planarRegionList, projectedPoint, 1e-3);
               message.getHeightMap().add((float) projectedPoint.getZ());
               message.getTraversabilityScore().add(1.0f);
               message.getTraversabilityClass().add(SnapResult.VALID.toByte());

               if (includeNormal)
               {
                  UnitVector3DReadOnly normal = intersectingRegions.get(0).getNormal();
                  if (normal.getZ() < 0.0)
                  {
                     UnitVector3D flippedNormal = new UnitVector3D(normal);
                     flippedNormal.negate();
                     normal = flippedNormal;
                  }

                  message.getSnappedNormalXData().add(TerrainMapData.packFloatAsByte(normal.getX32(), -NORMAL_MIN_MAX_XY, NORMAL_MIN_MAX_XY));
                  message.getSnappedNormalYData().add(TerrainMapData.packFloatAsByte(normal.getY32(), -NORMAL_MIN_MAX_XY, NORMAL_MIN_MAX_XY));
                  message.getSnappedNormalZData().add(TerrainMapData.packFloatAsByte(normal.getZ32(), NORMAL_MIN_Z, NORMAL_MAX_Z));
               }
            }
            else
            {
               message.getHeightMap().add(Float.NaN);
               message.getTraversabilityScore().add(0.0f);
               message.getTraversabilityClass().add(SnapResult.SNAP_FAILED.toByte());
               if (includeNormal)
               {
                  message.getSnappedNormalXData().add((byte) 0);
                  message.getSnappedNormalYData().add((byte) 0);
                  message.getSnappedNormalZData().add((byte) 0);
               }
            }
         }
      }

      return message;
   }
}
