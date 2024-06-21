package us.ihmc.rdx.ui.graphics;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Mesh;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.model.MeshPart;
import com.badlogic.gdx.graphics.g3d.utils.MeshBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import org.lwjgl.opengl.GL41;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.rdx.mesh.RDXMultiColorMeshBuilder;
import us.ihmc.sensorProcessing.globalHeightMap.GlobalHeightMap;
import us.ihmc.sensorProcessing.globalHeightMap.GlobalMapCell;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.IntFunction;
import java.util.function.IntToDoubleFunction;

/**
 * Creates a graphic for the GPU Height Map to be visualized in the RDX UI. Each height value from the height map is turned into a 2cm polygon that is then
 * visualized on the UI. The height value location will be in the center of the 2cm polygon that is visualized.
 */
public class RDXGlobalHeightMapGraphic implements RenderableProvider
{
   private final GlobalHeightMap globalHeightMap = new GlobalHeightMap();

   private final HashMap<GlobalMapCell, RDXHeightMapGraphicNew> globalMapRenderables = new HashMap<>();

   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);


   public void update()
   {
      for (RDXHeightMapGraphicNew mapRenderables : globalMapRenderables.values())
         mapRenderables.update();
   }

   public void generateMeshesAsync(HeightMapMessage heightMapMessage)
   {
      executorService.clearQueueAndExecute(() -> generateMeshes(heightMapMessage));
   }

   public void generateMeshes(HeightMapMessage heightMapMessage)
   {
      HeightMapData heightMapData = HeightMapMessageTools.unpackMessage(heightMapMessage);
      globalHeightMap.addHeightMap(heightMapData);

      for (GlobalMapCell globalMapCell : globalHeightMap.getModifiedMapCells())
      {
         RDXHeightMapGraphicNew graphic = getOrCreateHeightMapGraphic(globalMapCell);
         graphic.generateMeshesAsync(HeightMapMessageTools.toMessage(heightMapData));
      }
   }

   private RDXHeightMapGraphicNew getOrCreateHeightMapGraphic(GlobalMapCell globalMapCell)
   {
      RDXHeightMapGraphicNew graphic = globalMapRenderables.get(globalMapCell);
      if (graphic == null)
      {
         graphic = new RDXHeightMapGraphicNew();
         globalMapRenderables.put(globalMapCell, graphic);
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