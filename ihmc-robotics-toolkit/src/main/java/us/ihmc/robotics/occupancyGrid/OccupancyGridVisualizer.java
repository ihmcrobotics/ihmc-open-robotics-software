package us.ihmc.robotics.occupancyGrid;

import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public class OccupancyGridVisualizer
{
   private final YoFramePoint3D[][] cellViz;
   private final int maxLengthDivisions;
   private final int maxWidthDivisions;

   private final OccupancyGrid gridToVisualize;
   private final FramePoint3D positionInGrid = new FramePoint3D();

   public OccupancyGridVisualizer(String namePrefix, OccupancyGrid gridToVisualize, int maxLengthDivisions, int maxWidthDivisions,
                                  YoVariableRegistry registry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.gridToVisualize = gridToVisualize;
      this.maxLengthDivisions = maxLengthDivisions;
      this.maxWidthDivisions = maxWidthDivisions;

      cellViz = new YoFramePoint3D[maxLengthDivisions][maxWidthDivisions];
      for (int i = 0; i < cellViz.length; i++)
      {
         for (int j = 0; j < cellViz[0].length; j++)
         {
            String namePrefix2 = "CellViz_X" + String.valueOf(i) + "Y" + String.valueOf(j);
            YoFramePoint3D pointForViz = new YoFramePoint3D(namePrefix + namePrefix2, ReferenceFrame.getWorldFrame(), registry);
            pointForViz.setToNaN();
            cellViz[i][j] = pointForViz;
            YoGraphicPosition yoGraphicPosition = new YoGraphicPosition(namePrefix + namePrefix2, pointForViz, 0.004, YoAppearance.Orange());
            yoGraphicsListRegistry.registerArtifact(getClass().getSimpleName(), yoGraphicPosition.createArtifact());
            yoGraphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), yoGraphicPosition);
         }
      }
   }

   public void update()
   {
      for (int xIndex = 0; xIndex < maxLengthDivisions; xIndex++)
      {
         for (int yIndex = 0; yIndex < maxWidthDivisions; yIndex++)
         {
            boolean isOccupied = gridToVisualize.isCellOccupied(xIndex, yIndex);
            if (isOccupied)
            {
               positionInGrid.set(gridToVisualize.getGridFrame(), gridToVisualize.getXLocation(xIndex), gridToVisualize.getYLocation(yIndex), 0.0);
               cellViz[xIndex][yIndex].setMatchingFrame(positionInGrid);
            }
            else
            {
               cellViz[xIndex][yIndex].setToNaN();
            }

         }
      }
   }
}
