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
   private final IntMap<RDXHeightMapGraphicNew> globalMapRenderables = new IntMap<>();

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   public void update()
   {
      for (RDXHeightMapGraphicNew mapRenderables : globalMapRenderables.values())
         mapRenderables.update();
   }

   public void generateMeshesAsync(HeightMapMessage heightMapMessage, int tileKey)
   {

      RDXHeightMapGraphicNew graphic = getOrCreateHeightMapGraphic(tileKey);
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

