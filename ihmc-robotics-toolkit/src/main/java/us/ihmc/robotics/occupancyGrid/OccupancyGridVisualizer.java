package us.ihmc.robotics.occupancyGrid;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class OccupancyGridVisualizer
{
   private final List<YoFramePoint3D> cellViz = new ArrayList<>();

   private final OccupancyGrid gridToVisualize;
   private final FramePoint3D positionInGrid = new FramePoint3D();

   public OccupancyGridVisualizer(String namePrefix,
                                  OccupancyGrid gridToVisualize,
                                  int maxPoints,
                                  YoRegistry registry,
                                  YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(namePrefix, gridToVisualize, maxPoints, YoAppearance.Orange(), registry, yoGraphicsListRegistry);
   }

   public OccupancyGridVisualizer(String namePrefix,
                                  OccupancyGrid gridToVisualize,
                                  int maxPoints,
                                  AppearanceDefinition color,
                                  YoRegistry registry,
                                  YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.gridToVisualize = gridToVisualize;

      for (int i = 0; i < maxPoints; i++)
      {
         String namePrefix2 = "CellViz" + i;
         YoFramePoint3D pointForViz = new YoFramePoint3D(namePrefix + namePrefix2, ReferenceFrame.getWorldFrame(), registry);
         pointForViz.setToNaN();
         cellViz.add(pointForViz);
         YoGraphicPosition yoGraphicPosition = new YoGraphicPosition(namePrefix + namePrefix2, pointForViz, 0.004, color);
         yoGraphicsListRegistry.registerArtifact(namePrefix + "Visualizer", yoGraphicPosition.createArtifact());
      }
   }

   public void update()
   {
      int maxSize = gridToVisualize.getAllActiveCells().size();
      int cellCounter = 0;
      for (int i = 0; i < maxSize && cellCounter < cellViz.size(); i++)
      {
         OccupancyGridCell cell = gridToVisualize.getAllActiveCells().get(i);
         if (cell.getIsOccupied())
         {
            positionInGrid.setIncludingFrame(gridToVisualize.getGridFrame(),
                                             gridToVisualize.getXLocation(cell.getXIndex()),
                                             gridToVisualize.getYLocation(cell.getYIndex()),
                                             0.0);
            cellViz.get(cellCounter).setMatchingFrame(positionInGrid);
            cellCounter++;
         }
      }

      for (; cellCounter < cellViz.size(); cellCounter++)
         cellViz.get(cellCounter).setToNaN();
   }
}
