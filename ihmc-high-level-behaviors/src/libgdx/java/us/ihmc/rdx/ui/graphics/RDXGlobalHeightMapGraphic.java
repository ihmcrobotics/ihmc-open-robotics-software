package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.IntMap;
import com.badlogic.gdx.utils.Pool;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.sensorProcessing.globalHeightMap.GlobalHeightMap;
import us.ihmc.sensorProcessing.globalHeightMap.GlobalMapTile;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
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
   // FIXME turn into an int map using the lattice from the global height map tile structure
   private final IntMap<RDXHeightMapGraphicNew> globalMapRenderables = new IntMap<>();

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   public void update()
   {
      for (RDXHeightMapGraphicNew mapRenderables : globalMapRenderables.values())
         mapRenderables.update();
   }

   //   public void generateMeshesAsync(GlobalMapCellMap globalMapCellMap)
   //   {
   //      executorService.clearQueueAndExecute(() -> generateMeshes(globalMapCellMap));
   //   }
   //
   //   public void generateMeshes(GlobalMapCellMap globalMapCellMap)
   //   {
   //      Collection<GlobalMapCellEntry> globalOccupiedMapCells = new ArrayList<>();
   //      globalOccupiedMapCells = globalMapCellMap.getGlobalMapCells();
   //
   //      for (GlobalMapCellEntry globalMapCellEntry : globalOccupiedMapCells)
   //      {
   //         GlobalMapTile globalMapTile = new GlobalMapTile(globalMapCellEntry.getResolution(), globalMapCellEntry.getXIndex(), globalMapCellEntry.getYIndex());
   //         globalMapTile.setHeightAt(0, globalMapCellEntry.getCellHeight());
   //
   //         RDXHeightMapGraphicNew graphic =  getOrCreateHeightMapGraphic(globalMapTile);
   //
   //         graphic.generateMeshesGlobal(globalMapTile);
   //      }
   //   }

   public void generateMeshesAsync(HeightMapMessage heightMapMessage)
   {
//      RDXHeightMapGraphicNew heightMapGraphicNew = new RDXHeightMapGraphicNew();
//      heightMapGraphicNew.generateMeshesAsync(heightMapMessage);


      // TODO figure out how to calculate the tile key from the ehight map message
      HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(heightMapMessage);
      RDXHeightMapGraphicNew graphic = getOrCreateHeightMapGraphic(heightMapData);
      graphic.generateMeshesAsync(heightMapMessage);
   }

   private RDXHeightMapGraphicNew getOrCreateHeightMapGraphic(int tileKey)
   {
      RDXHeightMapGraphicNew graphic = globalMapRenderables.get(tileKey);
      if (graphic == null)
      {
         graphic = new RDXHeightMapGraphicNew();
         globalMapRenderables.put(tileKey, graphic);
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

