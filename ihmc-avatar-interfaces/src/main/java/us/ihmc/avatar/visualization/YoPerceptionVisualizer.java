package us.ihmc.avatar.visualization;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.TerrainMapCommand;
import us.ihmc.perception.gpuMapping.HeightMapTools;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.SCS2YoGraphicHolder;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameConvexPolygon2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;

public class YoPerceptionVisualizer implements SCS2YoGraphicHolder
{
   public static boolean VISUALIZE_HEIGHT_MAP = false;
   public static boolean VISUALIZE_PLANAR_REGIONS = true;

   private static final int NUMBER_OF_HEIGHT_MAP_POINTS_TO_VISUALIZE = 5000;
   private static final int NUMBER_OF_PLANAR_REGIONS_TO_VISUALIZE = 2;
   private static final int NUMBER_OF_PLANAR_REGION_VERTICES = 40;

   private final List<YoFramePoint3D> heights = new ArrayList<>();
   private final List<YoFramePose3D> planarRegionPoses = new ArrayList<>();
   private final List<YoFrameConvexPolygon2D> planarRegionPolygons = new ArrayList<>();

   public YoPerceptionVisualizer(YoRegistry registry)
   {
      if (VISUALIZE_HEIGHT_MAP)
      {
         for (int i = 0; i < NUMBER_OF_HEIGHT_MAP_POINTS_TO_VISUALIZE; i++)
         {
            heights.add(new YoFramePoint3D("height" + i, ReferenceFrame.getWorldFrame(), registry));
         }
      }

      if (VISUALIZE_PLANAR_REGIONS)
      {
         for (int i = 0; i < NUMBER_OF_PLANAR_REGIONS_TO_VISUALIZE; i++)
         {
            planarRegionPoses.add(new YoFramePose3D("pose" + i, ReferenceFrame.getWorldFrame(), registry));
            planarRegionPolygons.add(new YoFrameConvexPolygon2D("region" + i, ReferenceFrame.getWorldFrame(), NUMBER_OF_PLANAR_REGION_VERTICES, registry));
         }
      }
   }

   public void visualizeHeightMap(TerrainMapCommand terrainMapCommand)
   {
      if (!VISUALIZE_HEIGHT_MAP)
         return;

      Point2DReadOnly gridCenter = terrainMapCommand.getGridCenter();
      double cellSize = terrainMapCommand.getCellSize();
      double gridWidth = terrainMapCommand.getGridWidth();
      int centerIndex = HeightMapTools.computeCenterIndex(gridWidth, cellSize);

      int nominalCellsPerSide = 201;
      int multiplier = nominalCellsPerSide * nominalCellsPerSide / heights.size();

      for (int i = 0; i < heights.size(); i++)
      {
         int key = multiplier * i;
         double x = HeightMapTools.keyToXCoordinate(key, gridCenter.getX(), cellSize, centerIndex);
         double y = HeightMapTools.keyToYCoordinate(key, gridCenter.getY(), cellSize, centerIndex);
         double z = terrainMapCommand.getHeightAt(key);
         heights.get(i).set(x, y, z);
      }
   }

   public void visualizePlanarRegions(PlanarRegionsListCommand planarRegionsListCommand)
   {
      if (!VISUALIZE_PLANAR_REGIONS)
         return;

      for (int i = 0; i < NUMBER_OF_PLANAR_REGIONS_TO_VISUALIZE; i++)
      {
         planarRegionPoses.get(i).setToNaN();
         planarRegionPolygons.get(i).clear();
      }
      for (int i = 0; i < Math.min(planarRegionsListCommand.getNumberOfPlanarRegions(), NUMBER_OF_PLANAR_REGIONS_TO_VISUALIZE); i++)
      {
         planarRegionPoses.get(i).set(planarRegionsListCommand.getPlanarRegionCommand(i).getTransformToWorld());

         ConvexPolygon2D convexHull = planarRegionsListCommand.getPlanarRegionCommand(i).getConvexHull();
         YoFrameConvexPolygon2D yoConvexHull = planarRegionPolygons.get(i);

         for (int j = 0; j < Math.min(convexHull.getNumberOfVertices(), NUMBER_OF_PLANAR_REGION_VERTICES); j++)
         {
            yoConvexHull.addVertex(convexHull.getVertex(j));
         }

         yoConvexHull.update();
      }
   }

   @Override
   public YoGraphicGroupDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());

      if (VISUALIZE_HEIGHT_MAP)
      {
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicPointcloud3D("HeightMap", heights, 0.006, ColorDefinitions.DarkBlue()));
      }

      if (VISUALIZE_PLANAR_REGIONS)
      {
         for (int i = 0; i < NUMBER_OF_PLANAR_REGIONS_TO_VISUALIZE; i++)
         {
            group.addChild(YoGraphicDefinitionFactory.newYoGraphicPolygonExtruded3DDefinition("region" + i,
                                                                                              planarRegionPoses.get(i),
                                                                                              planarRegionPolygons.get(i),
                                                                                              0.01,
                                                                                              REGIONS_COLORS[i % REGIONS_COLORS.length]));
         }
      }

      return group;
   }

   private static final ColorDefinition[] REGIONS_COLORS = new ColorDefinition[] {new ColorDefinition(1.0, 0.0, 0.0, 0.5),
                                                                                  new ColorDefinition(0.0, 1.0, 0.0, 0.5),
                                                                                  new ColorDefinition(0.0, 0.0, 1.0, 0.5),
                                                                                  new ColorDefinition(1.0, 1.0, 0.0, 0.5),
                                                                                  new ColorDefinition(1.0, 0.0, 1.0, 0.5)};
}
