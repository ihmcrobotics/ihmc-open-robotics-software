package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import perception_msgs.msg.dds.GlobalMapCellEntry;
import perception_msgs.msg.dds.GlobalMapCellMap;
import us.ihmc.sensorProcessing.globalHeightMap.GlobalHeightMap;
import us.ihmc.sensorProcessing.globalHeightMap.GlobalMapTile;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;

/**
 * Creates a graphic for the GPU Height Map to be visualized in the RDX UI. Each height value from the height map is turned into a 2cm polygon that is then
 * visualized on the UI. The height value location will be in the center of the 2cm polygon that is visualized.
 */
public class RDXGlobalHeightMapGraphic implements RenderableProvider
{
   private final GlobalHeightMap globalHeightMap = new GlobalHeightMap();

   private final HashMap<GlobalMapTile, RDXHeightMapGraphicNew> globalMapRenderables = new HashMap<>();

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   public void update()
   {
      for (RDXHeightMapGraphicNew mapRenderables : globalMapRenderables.values())
         mapRenderables.update();
   }

   public void generateMeshesAsync(GlobalMapCellMap globalMapCellMap)
   {
      executorService.clearQueueAndExecute(() -> generateMeshes(globalMapCellMap));
   }

   public void generateMeshes(GlobalMapCellMap globalMapCellMap)
   {
      Collection<GlobalMapCellEntry> globalOccupiedMapCells = new ArrayList<>();
      globalOccupiedMapCells = globalMapCellMap.getGlobalMapCells();

      for (GlobalMapCellEntry globalMapCellEntry : globalOccupiedMapCells)
      {
         GlobalMapTile globalMapTile = new GlobalMapTile(globalMapCellEntry.getResolution(), globalMapCellEntry.getXIndex(), globalMapCellEntry.getYIndex());
         globalMapTile.setHeightAt(0, globalMapCellEntry.getCellHeight());

         RDXHeightMapGraphicNew graphic =  getOrCreateHeightMapGraphic(globalMapTile);

         graphic.generateMeshesGlobal(globalMapTile);
      }
   }

   //   public void generateMeshesAsync(HeightMapMessage heightMapMessage)
   //   {
   //      executorService.clearQueueAndExecute(() -> generateMeshes(heightMapMessage));
   //   }

   //   public void generateMeshes(HeightMapMessage heightMapMessage)
   //   {
   //      HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(heightMapMessage);
   //      globalHeightMap.addHeightMap(heightMapData);
   //
   //      for (GlobalMapCell globalMapCell : globalHeightMap.getModifiedMapCells())
   //      {
   //         RDXHeightMapGraphicNew graphic = getOrCreateHeightMapGraphic(globalMapCell);
   //         graphic.generateMeshesAsync(HeightMapMessageTools.toMessage(heightMapData));
   //      }
   //   }

   private RDXHeightMapGraphicNew getOrCreateHeightMapGraphic(GlobalMapTile globalMapTile)
   {
      RDXHeightMapGraphicNew graphic = globalMapRenderables.get(globalMapTile);
      if (graphic == null)
      {
         graphic = new RDXHeightMapGraphicNew();
         globalMapRenderables.put(globalMapTile, graphic);
      }

      return graphic;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RDXHeightMapGraphicNew mapRenderables : globalMapRenderables.values())
         mapRenderables.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      executorService.destroy();
      for (RDXHeightMapGraphicNew mapRenderables : globalMapRenderables.values())
         mapRenderables.destroy();
   }
}

