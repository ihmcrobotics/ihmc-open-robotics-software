package us.ihmc.rdx.visualizers;

import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.SteppableRegionsListCollectionMessage;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.ihmcPerception.steppableRegions.SteppableRegion;
import us.ihmc.ihmcPerception.steppableRegions.SteppableRegionMessageConverter;
import us.ihmc.ihmcPerception.steppableRegions.SteppableRegionsListCollection;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.List;

public class RDXSteppableRegionGraphic implements RenderableProvider
{
   private final ResettableExceptionHandlingExecutorService executorService = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

   private boolean inPaintHeight = true;
   private boolean renderGroundCells = false;
   private boolean renderGroundPlane = false;

   private final RecyclingArrayList<RDXGridMapGraphic> gridMapGraphics = new RecyclingArrayList<>(RDXGridMapGraphic::new);


   public void clear()
   {
      gridMapGraphics.clear();
   }

   public void update()
   {
      for (RDXGridMapGraphic gridMapGraphic : gridMapGraphics)
         gridMapGraphic.update();
   }

   public void setInPaintHeight(boolean inPaintHeight)
   {
      this.inPaintHeight = inPaintHeight;
   }

   public void setRenderGroundPlane(boolean renderGroundPlane)
   {
      this.renderGroundPlane = renderGroundPlane;
   }

   public void setRenderGroundCells(boolean renderGroundCells)
   {
      this.renderGroundCells = renderGroundCells;
   }

   public void generateMeshesAsync(SteppableRegionsListCollectionMessage steppableRegions, int yawToShow)
   {
      executorService.clearQueueAndExecute(() -> generateMeshes(steppableRegions, yawToShow));
   }

   private void generateMeshes(SteppableRegionsListCollectionMessage message, int yawToShow)
   {
      SteppableRegionsListCollection steppableRegionsCollection = SteppableRegionMessageConverter.convertToSteppableRegionsListCollection(message);
      List<SteppableRegion> steppableRegionsToView = steppableRegionsCollection.getSteppableRegions(yawToShow).getSteppableRegionsAsList();

      for (int i = 0; i < gridMapGraphics.size(); i++)
         gridMapGraphics.get(i).clear();
      gridMapGraphics.clear();
      steppableRegionsToView.forEach(this::generateMeshes);
   }

   private void generateMeshes(SteppableRegion steppableRegion)
   {
      RDXGridMapGraphic gridMapGraphic = gridMapGraphics.add();
      HeightMapMessage heightMapMessage = HeightMapMessageTools.toMessage(steppableRegion.getLocalHeightMap());
      heightMapMessage.setSequenceId(steppableRegion.getRegionId());
      gridMapGraphic.setInPaintHeight(inPaintHeight);
      gridMapGraphic.colorFromHeight(false);
      gridMapGraphic.setRenderGroundCells(renderGroundCells);
      gridMapGraphic.setRenderGroundPlane(renderGroundPlane);
      gridMapGraphic.generateMeshes(heightMapMessage);
   }


   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (RDXGridMapGraphic gridMapGraphic : gridMapGraphics)
      {
         if (gridMapGraphic == null)
            continue;

         gridMapGraphic.getRenderables(renderables, pool);
      }
   }

   public void destroy()
   {
      executorService.destroy();
   }

}
