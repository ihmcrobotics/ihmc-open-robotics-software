package us.ihmc.footstepPlanning.tools;

import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.perception.gpuHeightMap.HeightMapTools;

import java.util.List;

public class PlanarRegionToHeightMapConverter
{
   public static final double defaultResolution = 0.02;
   public static final double defaultEstimatedGroundHeight = 0.0;

   public static HeightMapMessage convertFromPlanarRegionsToHeightMap(PlanarRegionsList planarRegionsList)
   {
      return convertFromPlanarRegionsToHeightMap(planarRegionsList, defaultResolution);
   }

   public static HeightMapMessage convertFromPlanarRegionsToHeightMap(PlanarRegionsList planarRegionsList, double resolutionXY)
   {
      return convertFromPlanarRegionsToHeightMap(planarRegionsList.getPlanarRegionsAsList(), resolutionXY);
   }

   public static HeightMapMessage convertFromPlanarRegionsToHeightMap(List<PlanarRegion> planarRegionList, double resolutionXY)
   {
      return convertFromPlanarRegionsToHeightMap(planarRegionList, resolutionXY, defaultEstimatedGroundHeight);
   }

   public static HeightMapMessage convertFromPlanarRegionsToHeightMap(PlanarRegionsList planarRegionsList, double resolutionXY, double estimatedGroundHeight)
   {
      return convertFromPlanarRegionsToHeightMap(planarRegionsList.getPlanarRegionsAsList(), resolutionXY, estimatedGroundHeight);
   }

   public static HeightMapMessage convertFromPlanarRegionsToHeightMap(List<PlanarRegion> planarRegionList, double resolutionXY, double estimatedGroundHeight)
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

      HeightMapMessage message = new HeightMapMessage();
      message.setGridSizeXy(sideLength);
      message.setXyResolution(resolutionXY);
      message.setGridCenterX(gridCenterX);
      message.setGridCenterY(gridCenterY);
      message.setEstimatedGroundHeight(estimatedGroundHeight);

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

            if (projectedPoint != null && Double.isFinite(projectedPoint.getZ()) && !MathTools.epsilonEquals(projectedPoint.getZ(), estimatedGroundHeight, 1e-2))
            {
               message.getKeys().add(key);
               message.getHeights().add((float) projectedPoint.getZ());
            }
         }
      }

      return message;
   }
}
