package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.IntMap;
import com.badlogic.gdx.utils.Pool;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

/**
 * Creates a graphic for the GPU Height Map to be visualized in the RDX UI. Each height value from the height map is turned into a 2cm polygon that is then
 * visualized on the UI. The height value location will be in the center of the 2cm polygon that is visualized.
 */
public class RDXGlobalHeightMapGraphic implements RenderableProvider
{
   private final IntMap<RDXHeightMapGraphic> globalMapRenderables = new IntMap<>();

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   public void update()
   {
      for (RDXHeightMapGraphic mapRenderables : globalMapRenderables.values())
         mapRenderables.update();
   }

   public void generateMeshesAsync(HeightMapMessage heightMapMessage, int tileKey)
   {

      RDXHeightMapGraphic graphic = getOrCreateHeightMapGraphic(tileKey);
      graphic.generateMeshesAsync(heightMapMessage);
   }

   private RDXHeightMapGraphic getOrCreateHeightMapGraphic(int tileKey)
   {
      RDXHeightMapGraphic graphic = globalMapRenderables.get(tileKey);
      if (graphic == null)
      {
         graphic = new RDXHeightMapGraphic();
         globalMapRenderables.put(tileKey, graphic);
      }

      return graphic;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RDXHeightMapGraphic mapRenderables : globalMapRenderables.values())
         mapRenderables.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      executorService.destroy();
      for (RDXHeightMapGraphic mapRenderables : globalMapRenderables.values())
         mapRenderables.destroy();
   }
}

